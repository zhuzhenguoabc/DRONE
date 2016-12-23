/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <cmath>
#include <cstdint>

// define a constraining macro for ease of use
#define MIN(var_min,var1,var2) {if ((var1) <= (var2)) var_min = var1;else if ((var1) > (var2)) var_min = var2;}

#define __DEBUG

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif


// States used to control mission
enum class GoHomeState
{
  HOME_NOT_OK,
  ON_GROUND,
  HOME_OK,
  TURN_HOME,
  FLY_HOME,
  LANDING,
  EMERGENCY_LANDING
};

// Define a structure with controllable variables for ease of use
struct FlatVars
{
  float x;
  float y;
  float z;
  float yaw;
};

void print_usage()
{
  printf("-d  DFT for Position Estimate\n");
  printf("-g  GPS for Position Estimate\n");
  printf("-h  Print this message\n");
}
float constrain_min_max(float input, float min, float max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  else
    return input;
}

static float max_lin_velocity = .5; // maximum linear velocity in m/s
static float max_lin_acceleration = .25; // maximum linear acceleration in m/s/s

static float max_ang_velocity = 1.0; // desired velocity in rad/s
static float max_ang_acceleration = 0.5; // desired acceleration in rad/s/s

int set_max_lin_velocity(float max_lin_vel_in)
{
  max_lin_velocity = max_lin_vel_in;
  return 0;
}
int set_max_lin_acceleration(float max_lin_accel_in)
{
  max_lin_acceleration = max_lin_accel_in;
  return 0;
}
int set_max_ang_velocity(float max_ang_vel_in)
{
  max_ang_velocity = max_ang_vel_in;
  return 0;
}
int set_max_ang_acceleration(float max_ang_accel_in)
{
  max_ang_acceleration = max_ang_accel_in;
  return 0;
}

int goto_waypoint(FlatVars input_state, FlatVars goal_state, FlatVars last_commanded_world_velocity, bool stop_at_waypoint, FlatVars * cmd_out, uint8_t * reached_goal)
{
  // This function computes the desired velocity to reach a goal state (location+yaw) from a starting state while respecting dynamics and obeying a maximum velocity and acceleration
  // This function requires as input the previously specified velocity to ensure continuity in velocity.
  // All coordinates are in the world frame. This means that body commanded input velocities will need to be transformed to the world frame and outputs that should be in the body frame will need to be transformed.
  // The output variable reached_goal [xp yp zp yawp xv yv zv yawv] is set to 0s once the position and velocity have converged within a specified threshhold
  FlatVars current_cmd_world_velocity;

  // directly copy the current velocity command as the output command. This ensures continuity even if function does not successfully compute the velocity.
  memcpy(cmd_out,&last_commanded_world_velocity,sizeof(FlatVars));

  // initialize persistent variables
  static uint64_t t_prev = 0;
  static int initialized = 0;
  static FlatVars previous_state;
  static FlatVars previous_goal_state;
  static FlatVars previous_desired_velocity;

  // initialize flags to determine if goal state has been reached
  *reached_goal = 0b11111111;

  // get current time
  struct timeval tv; gettimeofday(&tv, NULL);
  uint64_t t_now = (uint64_t) tv.tv_sec*1000000 + tv.tv_usec;
  // compute change in time from last function call. This function assumes relatively uniform dt
  float dt = (t_now-t_prev)/1000000.0;
  // bound dt
  if (dt<.001){dt=.001;}
  else if (dt>.1){initialized = 0;dt = 0.1;}
  // save persistent time for next function call
  t_prev = t_now;

  // initialize the system and return -1. This ensures 2 samples to compute dt before it is used
  if(initialized == 0)
  {
    memcpy(&previous_goal_state,&goal_state,sizeof(FlatVars));
    memcpy(&previous_desired_velocity,&last_commanded_world_velocity,sizeof(FlatVars));
    previous_state=input_state;
    initialized=1;
    return -1;
  }


  // *****************************************************************************
  // ***** Bound linear components of velocity separately from yaw component *****
  // *****************************************************************************

  // compute vector to goal state from current state and it's magnitude
  float goal_vec_x = goal_state.x - input_state.x;
  float goal_vec_y = goal_state.y - input_state.y;
  float goal_vec_z = goal_state.z - input_state.z;
  float goal_vec_mag = sqrtf(goal_vec_x*goal_vec_x+goal_vec_y*goal_vec_y+goal_vec_z*goal_vec_z);

  float last_vel_mag = sqrtf(last_commanded_world_velocity.x*last_commanded_world_velocity.x +last_commanded_world_velocity.y*last_commanded_world_velocity.y +last_commanded_world_velocity.z*last_commanded_world_velocity.z);

  // compute unit vector to goal. If magnitude is below threshhold, unit vector is zeros
  float unit_vec_to_goal_x,unit_vec_to_goal_y,unit_vec_to_goal_z;
  if(goal_vec_mag<.0001){unit_vec_to_goal_x = 0;unit_vec_to_goal_y = 0;unit_vec_to_goal_z = 0;}
  else{unit_vec_to_goal_x = goal_vec_x/goal_vec_mag;unit_vec_to_goal_y = goal_vec_y/goal_vec_mag;unit_vec_to_goal_z = goal_vec_z/goal_vec_mag;}

  // compute maximum possible velocity capable of stopping under maximum acceleration
  float max_vel_close_mag = sqrtf(2*max_lin_acceleration*goal_vec_mag);

  // now compute the desired magnitude of velocity as the smaller of the stopping velocity and max allowed velocity
  float des_velocity = max_lin_velocity;

  if(stop_at_waypoint == true)
  {
    MIN(des_velocity,max_vel_close_mag,max_lin_velocity);
  }

  // compute the desired change vector to achieve the desired velocity and its magnitude
  float des_vel_change_x = unit_vec_to_goal_x*des_velocity-last_commanded_world_velocity.x;
  float des_vel_change_y = unit_vec_to_goal_y*des_velocity-last_commanded_world_velocity.y;
  float des_vel_change_z = unit_vec_to_goal_z*des_velocity-last_commanded_world_velocity.z;
  float des_vel_change_mag = sqrtf(des_vel_change_x*des_vel_change_x + des_vel_change_y*des_vel_change_y + des_vel_change_z*des_vel_change_z);

  float unit_des_vel_change_x;float unit_des_vel_change_y;float unit_des_vel_change_z;

  // Compute the unit vector in the direction of required acceleration. Limit if magnitude below thresh
  if (des_vel_change_mag < .00001){unit_des_vel_change_x = 0;unit_des_vel_change_y = 0;unit_des_vel_change_z = 0;}
  else{unit_des_vel_change_x = des_vel_change_x/des_vel_change_mag;unit_des_vel_change_y = des_vel_change_y/des_vel_change_mag;unit_des_vel_change_z = des_vel_change_z/des_vel_change_mag;}

  // compute applied acceleration to influence velocity in correct direction
  float accel_applied_x = unit_des_vel_change_x*max_lin_acceleration;
  float accel_applied_y = unit_des_vel_change_y*max_lin_acceleration;
  float accel_applied_z = unit_des_vel_change_z*max_lin_acceleration;

  // apply this acceleration to influence velocity in correct direction
  current_cmd_world_velocity.x = last_commanded_world_velocity.x + accel_applied_x*dt;
  current_cmd_world_velocity.y = last_commanded_world_velocity.y + accel_applied_y*dt;
  current_cmd_world_velocity.z = last_commanded_world_velocity.z + accel_applied_z*dt;

  // bound current velocity by specified limit
  float current_vel_mag = sqrtf(current_cmd_world_velocity.x*current_cmd_world_velocity.x+current_cmd_world_velocity.y*current_cmd_world_velocity.y+current_cmd_world_velocity.z*current_cmd_world_velocity.z);
  if (current_vel_mag > max_lin_velocity)
  {
    current_cmd_world_velocity.x = current_cmd_world_velocity.x/current_vel_mag*max_lin_velocity;
    current_cmd_world_velocity.y = current_cmd_world_velocity.y/current_vel_mag*max_lin_velocity;
    current_cmd_world_velocity.z = current_cmd_world_velocity.z/current_vel_mag*max_lin_velocity;
  }

  // *****************************************************************************
  // ***** For goal_reached condition, use closest point from line segment   *****
  // ***** between last 2 positions, and compare to goal condition           *****
  // *****************************************************************************

  //Vector from current pos to last pos
  float moved_vec_x = previous_state.x - input_state.x;
  float moved_vec_y = previous_state.y - input_state.y;
  float moved_vec_z = previous_state.z - input_state.z;

  //Vector dot products to get projection of point onto line
  float goal_vec_dotprod = (goal_vec_x*moved_vec_x)+(goal_vec_y*moved_vec_y)+(goal_vec_z*moved_vec_z);
  float moved_vec_dotprod = (moved_vec_x*moved_vec_x)+(moved_vec_y*moved_vec_y)+(moved_vec_z*moved_vec_z);
  float param = -1; // shows percentage of closest point on moved vector (For checking endpoints of line segment)

  if (moved_vec_dotprod != 0)
  {
    param = goal_vec_dotprod / moved_vec_dotprod;
  }

  float goal_closest_point_x;
  float goal_closest_point_y;
  float goal_closest_point_z;

    // if param < 0 (closest point is closest to input state)
  if (param < 0)
  {
    goal_closest_point_x = input_state.x;
    goal_closest_point_y = input_state.y;
    goal_closest_point_z = input_state.z;
  }

    // if param > 1 (closest point is closest to previous state)
  else if (param > 1)
  {
    goal_closest_point_x = previous_state.x;
    goal_closest_point_y = previous_state.y;
    goal_closest_point_z = previous_state.z;
  }
    // param is % of distance moved (closest point to goal is orthogonal and on line segment)
  else
  {
    goal_closest_point_x = input_state.x + param * moved_vec_x;
    goal_closest_point_y = input_state.y + param * moved_vec_y;
    goal_closest_point_z = input_state.z + param * moved_vec_z;
  }

  float goal_closest_point_mag = sqrtf((goal_state.x-goal_closest_point_x)*(goal_state.x-goal_closest_point_x) +
                                       (goal_state.y-goal_closest_point_y)*(goal_state.y-goal_closest_point_y) +
                                       (goal_state.z-goal_closest_point_z)*(goal_state.z-goal_closest_point_z));

  // if close to goal (x,y, and z position), set position flags to 0
  if (goal_closest_point_mag<.05)
  {
    *reached_goal=*reached_goal&0b00011111;
  }
  // if low veloctiy (x,y, and z velocity), set velocity flags to 0
  if ((last_vel_mag<.01) || (stop_at_waypoint == false))
  {
    *reached_goal=*reached_goal&0b11110001;
  }
  // if close to goal and low velocity, command 0 lateral velocity
  if (((*reached_goal&0b11101110) == 0) && (stop_at_waypoint == true))
  {
    current_cmd_world_velocity.x = 0;
    current_cmd_world_velocity.y = 0;
    current_cmd_world_velocity.z = 0;
  }

  // Store current state as last state
  previous_state = input_state;

  // *****************************************************************************
  // ***** Bound yaw component of velocity separately from linear components *****
  // *****************************************************************************

  // compute yaw error, being careful to wrap correctly
  float yaw_error;
  float unwrapped_yaw_error_plus_pi = goal_state.yaw-input_state.yaw + M_PI;
  if(unwrapped_yaw_error_plus_pi<0)
  {
    yaw_error = (2*M_PI-fmod(-unwrapped_yaw_error_plus_pi, (2*M_PI))) -M_PI;
  }
  else
  {
    yaw_error = fmod(unwrapped_yaw_error_plus_pi, (2*M_PI)) -M_PI;
  }
  float yaw_error_abs = fabs(yaw_error);

  // determine which direction to turn
  float yaw_rate_direction = 0;
  if(yaw_error<-0.0001){yaw_rate_direction = -1;}
  if(yaw_error>0.0001){yaw_rate_direction = 1;}

  // determine max anglular velocity that supports stopping at given angle
  float max_vel_yaw_close_mag = sqrtf(2*max_ang_acceleration*yaw_error_abs);

  // compute desired angular velocity to reach goal location
  float des_ang_velocity = max_ang_velocity;
  MIN(des_ang_velocity,max_vel_yaw_close_mag,max_ang_velocity);

  // compute change in velocity required to reach desired yaw rate and it's magnitude
  float des_vel_change_yaw = yaw_rate_direction*des_ang_velocity-last_commanded_world_velocity.yaw;

  // compute direction to acclerate in in yaw
  float yaw_acceleration_direction = 0;
  if(des_vel_change_yaw<-0.0001){yaw_acceleration_direction = -1;}
  if(des_vel_change_yaw>0.0001){yaw_acceleration_direction = 1;}

  // compute yaw acceleration to use
  float accel_applied_yaw = yaw_acceleration_direction*max_ang_acceleration;
  // increment commanded angular velocity by desired accleration*dt
  current_cmd_world_velocity.yaw = last_commanded_world_velocity.yaw + accel_applied_yaw*dt;

  // bound maximum yaw rate
  current_cmd_world_velocity.yaw = constrain_min_max(current_cmd_world_velocity.yaw,-max_ang_velocity,max_ang_velocity);

  // if yaw error within threshhold of zero, set set yaw position flag to 0
  if((yaw_error_abs<.008) || (stop_at_waypoint == false))
  {
    *reached_goal=*reached_goal&0b11101111;
  }
  // if yaw velocity within threshhold of zero, set set yaw velocity flag to 0
  if((fabs(last_commanded_world_velocity.yaw) < 0.02) || (stop_at_waypoint == false))
  {
    *reached_goal=*reached_goal&0b11111110;
  }
  // if yaw error and velocity within threshhold of zero, command 0 yaw velocity
  if(((*reached_goal&0b00010001)==0) && (stop_at_waypoint == true))
  {
    current_cmd_world_velocity.yaw = 0;
  }

  // *******************************
  // ***** Set output velocity *****
  // *******************************

  cmd_out->x = current_cmd_world_velocity.x;
  cmd_out->y = current_cmd_world_velocity.y;
  cmd_out->z = current_cmd_world_velocity.z;
  cmd_out->yaw = current_cmd_world_velocity.yaw;

  // Save previous velocity to check for reset
  memcpy(&previous_desired_velocity,&current_cmd_world_velocity,sizeof(FlatVars));

  return 0;
}
