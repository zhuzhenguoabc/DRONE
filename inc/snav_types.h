/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef SNAV_TYPES_H_
#define SNAV_TYPES_H_

/** @addtogroup sn_datatypes
@{ */
/**
 * Mode ID codes returned by querying the flight system mode.
 */
typedef enum
{
  SN_SENSOR_ERROR_MODE = -2,
  /**< Error -- flight is not possible in current state. */
  SN_UNDEFINED_MODE = -1,
  /**< Mode is not defined in this list. */
  SN_WAITING_FOR_DEVICE_TO_CONNECT,
  /**< Waiting for an RC or DroneController to connect. */
  SN_EMERGENCY_KILL_MODE,
  /**< Propellers were stopped -- Most likely due to a crash. */
  SN_EMERGENCY_LANDING_MODE,
  /**< Low fixed-thrust emergency descent. */
  SN_THERMAL_IMU_CALIBRATION_MODE,
  /**< Thermal accel/gyro calibration. */
  SN_STATIC_ACCEL_CALIBRATION_MODE,
  /**< Static accel offset calibration. */
  SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE,
  /**< Optic flow camera yaw calibration. */
  SN_MAGNETOMETER_CALIBRATION_MODE,
  /**< Compass (magnetometer) calibration. */
  SN_CALIBRATION_SUCCESS,
  /**< Last active calibration was successful. */
  SN_CALIBRATION_FAILURE,
  /**< Last active calibration was not successful. */
  SN_ESC_RPM_MODE,
  /**< API controls the ESC RPMs. */
  SN_ESC_PWM_MODE,
  /**< API controls the ESC PWMs. */
  SN_RATE_MODE,
  /**< Thrust, roll rate, pitch rate, yaw rate; does not auto-stabilize. */
  SN_THRUST_ANGLE_MODE,
  /**< Thrust, roll angle, pitch angle, yaw rate. */
  SN_ALT_HOLD_MODE,
  /**< Vertical velocity, roll angle, pitch angle, yaw rate. */
  SN_THRUST_GPS_HOVER_MODE,
  /**< Thrust control with lateral position hold using GPS.  */
  SN_GPS_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using GPS. */
  SN_OPTIC_FLOW_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using optic flow. */
  SN_VIO_POS_HOLD_MODE,
  /**< Body-relative 3D velocity and yaw rate using VIO. */
  SN_THRUST_ATT_ANG_VEL_MODE,
  /**< Thrust, attitude, and angular velocity. */
  SN_PRESSURE_LANDING_MODE,
  /**< Vertical velocity-controlled descent with zero roll/pitch. */
  SN_PRESSURE_GPS_LANDING_MODE,
  /**< 3D velocity-controlled descent. */
  SN_GPS_GO_HOME_MODE,
  /**< 3D velocity-controlled return to home position. @newpage */
} SnMode;

/**
 * Input command types.
 */
typedef enum
{
  SN_INPUT_CMD_TYPE_NONE = -1,
  /**< No input. */
  SN_INPUT_CMD_TYPE_RC,
  /**< RC-style input commands. */
  SN_INPUT_CMD_TYPE_API_THRUST_ATT_ANG_VEL,
  /**< Thrust attitude angular velocity input commands from the API. */
  SN_INPUT_CMD_TYPE_API_ESC,
  /**< ESC input commands from the API. */
} SnInputCommandType;

/**
 * RC command input source.
 */
typedef enum
{
  SN_RC_CMD_NO_INPUT = -1,
  /**< No input. */
  SN_RC_CMD_SPEKTRUM_INPUT,
  /**< Spektrum. */
  SN_RC_CMD_API_INPUT,
  /**< RC commands from API. */
} SnRcCommandSource;

/**
 * RC command. This enum specifies how the dimensionless commands sent by the
 * sn_send_rc_command() function are interpreted and indirectly selects
 * the desired operation mode. The actual mode can be verified with the
 * sn_get_mode() function.
 */
typedef enum
{
  SN_RC_RATES_CMD,
  /**< Command pitch rate, negative roll rate, thrust magnitude, and yaw rate. */
  SN_RC_THRUST_ANGLE_CMD,
  /**< Command pitch angle, negative roll angle, thrust magnitude, and yaw rate. */
  SN_RC_ALT_HOLD_CMD,
  /**< Command pitch angle, negative roll angle, Z speed, and yaw rate. */
  SN_RC_THRUST_ANGLE_GPS_HOVER_CMD,
  /**< Command pitch angle, negative roll angle, thrust magnitude, and yaw rate;
       holds lateral position using GPS when roll and pitch commands are zero. */
  SN_RC_GPS_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using GPS. */
  SN_RC_OPTIC_FLOW_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using optic
       flow. */
  SN_RC_VIO_POS_HOLD_CMD,
  /**< Command vehicle-relative X and Y speeds, Z speed, and yaw rate using
       visual inertial odometry (VIO). */
  SN_RC_NUM_CMD_TYPES
  /**< Do not use -- Reserved to hold the number of RC command types. @newpage */
} SnRcCommandType;

/**
 * RC command options.
 * The options can be OR-ed to form hybrid options
 * (e.g., RC_OPT_ENABLE_DEADBAND|RC_OPT_NON_LINEAR_TRANSFORMS).
 */
typedef enum
{
  RC_OPT_LINEAR_MAPPING          =0,
  /**< Linear control (default). */
  RC_OPT_ENABLE_DEADBAND         =1,
  /**< Enable deadband. */
  RC_OPT_NON_LINEAR_TRANSFORMS   =2,
  /**< Non-linear transforms. */
  RC_OPT_LOW_THRUST_YAW_DISABLE  =4,
  /**< Disable yaw control at very low thrust. */
  RC_OPT_DEFAULT_RC              =7,
  /**< Default RC.  */
  RC_OPT_TRIGGER_LANDING         =8,
  /**< Trigger landing. The vehicle determines which landing mode is appropriate
       based on which sensors are available and what mode is active. */
} SnRcCommandOptions;

/**
 * Collective state of all of the propellers -- Identification code returned by
 * querying the flight system propeller state.
 */
typedef enum
{
  SN_PROPS_STATE_UNKNOWN = -1,   /**< State of propellers is unknown. */
  SN_PROPS_STATE_NOT_SPINNING,   /**< All propellers are not spinning. */
  SN_PROPS_STATE_STARTING,       /**< Propellers are starting to spin. */
  SN_PROPS_STATE_SPINNING        /**< All propellers are spinning. */
} SnPropsState;

/**
 * Identification code returned by querying the sensor data status.
 */
typedef enum
{
  SN_DATA_INVALID = -1,
  /**< Sensor data is invalid. */
  SN_DATA_VALID = 0,
  /**< Sensor data is valid. */
  SN_DATA_NOT_INITIALIZED,
  /**< Sensor data has not been initialized. */
  SN_DATA_STUCK,
  /**< Sensor data is unchanging. */
  SN_DATA_TIMEOUT,
  /**< Sensor data has not been updated past the data timeout threshold. */
  SN_DATA_UNCALIBRATED,
  /**< Sensor data has not been calibrated. */
  SN_DATA_OFFSET_UNCALIBRATED,
  /**< Sensor data is missing offset calibration. */
  SN_DATA_TEMP_UNCALIBRATED,
  /**< Sensor data is missing temperature calibration. */
  SN_DATA_STARTING,
  /**< Sensor is acquiring additional samples. */
  SN_DATA_STATUS_UNAVAILABLE,
  /**< Sensor data status unavailable. */
  SN_DATA_NOT_ORIENTED,
  /**< Sensor data missing the orientation parameter. */
  SN_DATA_NO_LOCK,
  /**< Sensor data unable to lock on. */
  SN_DATA_WARNING,
  /**< Sensor is in a warning state. */
  SN_DATA_TRANSITIONING
  /**< Sensor data is transitioning. @newpage */
} SnDataStatus;

/**
 * Individual state of a motor -- Identification code returned by querying the
 * state feedback from ESCs.
 */
typedef enum
{
  SN_MOTOR_STATE_UNKNOWN = -1,      /**< State of motor is unknown. */
  SN_MOTOR_STATE_NOT_SPINNING,      /**< Motor is not spinning. */
  SN_MOTOR_STATE_STARTING,          /**< Motor is starting to spin. */
  SN_MOTOR_STATE_SPINNING_FORWARD,
  /**< Motor is spinning in the forward direction. */
  SN_MOTOR_STATE_SPINNING_BACKWARD
  /**< Motor is spinning in the backward direction. */
} SnMotorState;

/**
 * Identification code returned by querying the sensor calibration status.
 */
typedef enum
{
  SN_CALIB_STATUS_NOT_CALIBRATED,
  /**< Calibration data does not exist. */
  SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS,
  /**< Calibration procedure is in progress. */
  SN_CALIB_STATUS_CALIBRATED
  /**< Calibration data exists. */
} SnCalibStatus;

/**
 * Spektrum data transmission mode. This value is used to request binding and display the current mode.
 */
typedef enum
{
  SN_RC_RECEIVER_MODE_UNKNOWN,
  /**< Unknown DSM mode. */
  SN_SPEKTRUM_MODE_DSM2_22,
  /**< DSM2 22 ms (6-channel maximum, every 22 ms). */
  SN_SPEKTRUM_MODE_DSM2_11,
  /**< DSM2 11 ms (9-channel maximum, complete packet every 22 ms).  */
  SN_SPEKTRUM_MODE_DSMX_22,
  /**< DSMX 22 ms (6-channel maximum, every 22 ms). */
  SN_SPEKTRUM_MODE_DSMX_11
  /**< DSMX 11 ms (9-channel maximum, complete packet every 22 ms). */
} SnRcReceiverMode;


/**
 * Supported GNSS receiver types.
 */
typedef enum
{
  SN_GNSS_RECEIVER_TYPE_UNKNOWN,
  /**< Unknown receiver. */
  SN_GNSS_RECEIVER_TYPE_CSR,
  /**< CSR receiver. */
  SN_GNSS_RECEIVER_TYPE_UBLOX,
  /**< U-blox receiver.  */
} SnGnssReceiverType;

/** @} */ /* end_addtogroup sn_datatypes */


#endif //SNAV_TYPES_H_
