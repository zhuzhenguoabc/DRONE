/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef SNAV_CACHED_DATA_H_
#define SNAV_CACHED_DATA_H_

#include <stdint.h>

#include "snav_types.h"

/**
 * Version information required to uniquely identify the software and device
 */
typedef struct
{
  char compile_date[16];
  /**< Null terminated string containing compilation date */
  char compile_time[16];
  /**< Null terminated string containing compilation time */
  char library_version[18];
  /**< Null terminated string representing version information */
  char library_hash[41];
  /**< Null terminated string with unique build identifier */
  char mac_address[18];
  /**< Null terminated string containing wlan0 mac address if it was successfully polled */
  int32_t esc_hw_version[8];
  /**< Hardware revision of the ESCs */
  int32_t esc_sw_version[8];
  /**< Software version of the ESCs */
} VersionInfo;

/**
 * General information about the state of of the system. This can be very useful for debugging system issues.
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run */
  int32_t desired_mode;
  /**< Cast to enum: SnMode. Desired mode that the flight controller will attempt to transition to if possible */
  int32_t current_mode;
  /**< Cast to enum: SnMode. Current mode of the flight controller */
  float voltage;
  /**< Estimated input system voltage (Units: V) */
  float current;
  /**< Estimated electrical current being used by system (if available) (Units: A) */
  int32_t props_state;
  /**< Cast to enum: SnPropsState. State of the propellers */
  uint8_t on_ground;
  /**< Flag representing flight controller's belief of being on the ground or not; 1 corresponds to being on the ground, 0 corresponds to not being on the ground */
  int32_t input_cmd_type;
  /**< Cast to enum: SnInputCommandType.  */
  char last_error_code[32];
  /**< Null terminated string to represent the last error code detected. This persists even if error code clears to allow detection of infrequent errors. */
} GeneralStatus;

/**
 * Status of various sensors and estimators
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: ms) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run */
  int32_t imu_0_status;
  /**< Cast to enum: SnDataStatus. Status of imu0 sensor */
  int32_t baro_0_status;
  /**< Cast to enum: SnDataStatus. Status of barometer0 sensor */
  int32_t mag_0_status;
  /**< Cast to enum: SnDataStatus. Status of magnetometer0 sensor */
  int32_t gps_0_status;
  /**< Cast to enum: SnDataStatus. Status of global positioning gps0 sensor */
  int32_t sonar_0_status;
  /**< Cast to enum: SnDataStatus. Status of sonar0 sensor */
  int32_t optic_flow_0_status;
  /**< Cast to enum: SnDataStatus. Status of optic flow0 (dft) sensor */
  int32_t spektrum_rc_0_status;
  /**< Cast to enum: SnDataStatus. Status of Spektrun RC0 sensor */
  int32_t api_rc_status;
  /**< Cast to enum: SnDataStatus. Status of API RC command */
  int32_t rc_active_status;
  /**< Cast to enum: SnDataStatus. Status of currently active RC command */
  int32_t height_estimator_status;
  /**< Cast to enum: SnDataStatus. Status of height estimator */
  int32_t attitude_estimator_status;
  /**< Cast to enum: SnDataStatus. Status of attitude estimator */
} DataStatus;

/**
 * System and sensor update rates
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run */
  float control_loop_freq;
  /**< Main control loop update frequency. (Units: Hz) */
} UpdateRates;

/**
 * Estimate of the orientation of the vehicle
 */
typedef struct
{
  uint64_t time;
  /**< Timestamp (Units: us) */
  uint32_t cntr;
  /**< Counter incremented on successful computation of attitude estimate */
  float roll;
  /**< Roll angle using Tait-Bryan ZYX (Units: rad) */
  float pitch;
  /**< Pitch angle using Tait-Bryan ZYX (Units: rad) */
  float yaw;
  /**< Yaw angle using Tait-Bryan ZYX (Units: rad) */
  float rotation_matrix[9];
  /**< Rotation matrix from vehicle body to world written in row-major order */
} AttitudeEstimate;

/**
 * Apps proc CPU stats
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  /**< Number of times data was updated */
  uint64_t time_apps_us;
  /**< Timestamp from apps proc (Units: us) */
  float cur_freq[4];
  /**< Current apps proc CPU frequency; will read NaN if CPU is not online (Units: GHz) */
  float max_freq[4];
  /**< Max apps proc CPU frequency -- may be throttled due to temperature; will read NaN if CPU is not online (Units: GHz) */
  float temp[13];
  /**< Temperature measurements from thermal zones; will read NaN if thermal zone is disabled (Units: deg C) */
} CpuStats;

/**
 * Inertial Measurement Unit raw data
 */
typedef struct
{
  uint64_t time;
  /**< Time data was received (Units: us) */
  uint32_t cntr;
  /**< Number of measurements received */
  float temp;
  /**< Temperature of IMU (Units: deg C) */
  float lin_acc[3];
  float ang_vel[3];
} Imu0Raw;

/**
 * Inertial Measurement Unit data after compensation
 */
typedef struct
{
  uint64_t time;
  /**< Time data was received (Units: us) */
  uint32_t cntr;
  /**< Number of compensated measurements recorded */
  float temp;
  /**< Temperature of IMU */
  float lin_acc[3];
  float ang_vel[3];
} Imu0Compensated;


typedef struct
{
  float accel_slope[3];
  /**< xyz slopes for accelerometer temperature calibration (Units: gravity/deg C) */
  float accel_offset[3];
  /**< xyz offsets for accelerometer temperature calibration (Units: gravity) */
  float accel_residual[3];
  /**< Average squared residual for accelerometer temperature calibration (Units: gravity^2) */
  float gyro_slope[3];
  /**< xyz slopes for gyroscope temperature calibration (Units: (rad/s)/deg C) */
  float gyro_offset[3];
  /**< xyz offsets for gyroscope temperature calibration (Units: (rad/s)) */
  float gyro_residual[3];
  /**< Average squared residual for gyroscope temperature calibration (Units: (rad/s)^2) */
} Imu0CalibrationThermal;


typedef struct
{
  char name[16];
  /**< Type of offset calibration. For example, static or dynamic. */
  float accel_offset[3];
  /**< xyz accelerometer offsets from accelerometer offset calibration (Units: gravity (~9.81 m/s/s)) */
  float avg_thrust;
  /**< Average thrust found from in flight accelerometer calibration (Units: g) */
  float roll_trim_offset;
  /**< Roll trim offset from in flight accelerometer calibration (Units: g) */
  float pitch_trim_offset;
  /**< Pitch trim offset from in flight accelerometer calibration (Units: g) */
} Imu0CalibrationOffset;

/**
 * Raw barometer data
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  /**< Number of times data was read */
  float pressure;
  /**< Atmospheric pressure measurement (Units: Pa) */
  float temp;
  /**< Temperature of sensor (Units: deg C) */
} Barometer0Raw;

/**
 * Raw sonar data
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  /**< Number of times data was read */
  float range;
  /**< Range measurement (Units: m) */
} Sonar0Raw;


typedef struct
{
  uint64_t time;
  /**< Time when data was received (Units: us) */
  uint32_t cntr;
  /**< Number of data packets received */
  uint8_t identifier;
  /**< Type of compass sensor */
  float field[3];
  /**< xyz components of the magnetic field in sensor frame (Units:   ) */
} Mag0Raw;


typedef struct
{
  uint64_t time;
  /**< Time when data was received (Units: us) */
  uint32_t cntr;
  /**< Number of data packets received */
  uint8_t identifier;
  /**< Type of compass sensor */
  float field[3];
  /**< xyz components of the magnetic field in sensor frame */
} Mag1Raw;


typedef struct
{
  uint64_t time;
  /**< Time when data was received (Units: us) */
  uint32_t cntr;
  /**< Number of data packets received */
  uint8_t identifier;
  /**< Type of compass sensor */
  float field[3];
  /**< xyz components of the magnetic field in sensor frame */
} Mag0Compensated;


typedef struct
{
  uint64_t time;
  /**< Time when data was received (Units: us) */
  uint32_t cntr;
  /**< Number of data packets received */
  uint8_t identifier;
  /**< Type of compass sensor */
  float field[3];
  /**< xyz components of the magnetic field in sensor frame */
} Mag1Compensated;


typedef struct
{
  float matrix[9];
  /**< Scale parameters of mapping */
  float offset[3];
  /**< xyz offset of mapping */
} Mag0Calibration3D;

/**
 * Raw Spektrum RC data
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  /**< Number of times data was read */
  uint8_t protocol;
  /**< RC protocol identifier */
  uint8_t num_channels;
  /**< Number of RC channels being populated */
  uint16_t vals[32];
  /**< Raw Spektrum channel values */
} SpektrumRc0Raw;

/**
 * RC commands sent through API
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  int32_t cmd_type;
  /**< Cast to enum: SnRcCommandType. How the commands should be interpreted if possible */
  int32_t cmd_options;
  /**< Cast to enum: SnRcCommandOptions. Options used to deviate from linear mapping */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1] */
} ApiRcRaw;

/**
 * RC commands being used for control
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  int32_t source;
  /**< Cast to enum: SnRcCommandSource. Source of the active RC commands */
  int32_t cmd_type;
  /**< Cast to enum: SnRcCommandType. How the commands should be interpreted if possible; not relevant if source is Spektrum RC */
  int32_t cmd_options;
  /**< Cast to enum: SnRcCommandOptions. Options used to deviate from linear mapping */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1] */
} RcActive;

/**
 * Information regarding captured camera frames
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  uint32_t frame_num;
  /**< Frame number received, starting from 0 */
  int64_t frame_timestamp;
  /**< Timestamp of frame (Units: us) */
  float exposure;
  /**< Normalized exposure setting used to take the frame */
  float gain;
  /**< Normalized gain setting used to take the frame */
  float average_luminance;
  /**< Normalized average luminance of frame */
} Camera0FrameInfo;

/**
 * Downward Facing Tracker (DFT) data
 */
typedef struct
{
  uint64_t time;
  /**< Time struct was logged (Units: us) */
  uint32_t cntr;
  float pixel_flow[2];
  /**< Pixel displacement between subsequent image frames (Units: pixels) */
  int32_t sample_size;
  /**< Number of inliers after calculation of displacement */
  float error_sum;
  /**< Error metric, sum of squared error over sample size points */
} OpticFlow0Raw;

/**
 * Downfacing camera calibration for tilt angle (optic flow camera yaw calibration)
 */
typedef struct
{
  float x_factor;
  /**< Tilt factor in x (Units: pixels/rad) */
  float y_factor;
  /**< Tilt factor in y (Units: pixels/rad) */
} OpticFlow0CalibrationTilt;


typedef struct
{
  uint64_t time;
  /**< Time when data was received (Units: us) */
  uint32_t cntr;
  /**< Number of complete nav messages */
  uint8_t identifier;
  /**< Type of GNSS receiver */
  uint32_t num_errors;
  /**< Number of CRC errors */
  uint32_t gps_time_sec;
  /**< Time of Week (Units: s) */
  uint32_t gps_time_nsec;
  /**< Time of Week (Units: ns) */
  int32_t latitude;
  /**< Position latitude (Units: deg*10e7) */
  int32_t longitude;
  /**< Position longitude (Units: deg*10e7) */
  float altitude;
  /**< Altitude MSL (Units: m) */
  float lin_vel[3];
  /**< Velocity NEU (Units: m/s) */
  uint8_t fix_type;
  /**< Fix type / quality */
  uint8_t num_satellites;
  /**< Number of satellites used in solution */
  float horizontal_acc;
  /**< Horizontal accuracy of position estimate (Units: m) */
  float speed_acc;
  /**< Horizontal speed accuracy (Units: m/s) */
  uint8_t agc[2];
  /**< Value of automatic gain controller */
  uint8_t sv_ids[20];
  /**< Satellite identification number */
  uint8_t sv_cno[20];
  /**< Satellite signal strength (Units: C/N0) */
} Gps0Raw;


typedef struct
{
  uint64_t time;
  /**< Timestamp (Units: us) */
  uint32_t cntr;
  float position_estimated[3];
  /**< Estimated xyz position, note that this data is subject to change in future releases (Units: m) */
  float yaw_estimated;
  /**< Estimated yaw angle, note that this data is subject to change in future releases (Units: rad) */
  float position_desired[3];
  /**< Desired xyz position, note that this data is subject to change in future releases (Units: m) */
  float yaw_desired;
  /**< Desired yaw angle, note that this data is subject to change in future releases (Units: rad) */
} HighLevelControlData;


typedef struct
{
  uint64_t time;
  /**< Time when this data was published (Units: us) */
  uint32_t cntr;
  /**< Number of times ESC feedback was updated */
  uint8_t packet_cntr[8];
  /**< Number of packets received by each ESC */
  int16_t rpm[8];
  /**< RPM of the motors (Units: RPM) */
  int8_t power[8];
  /**< Power applied by the ESC (Units: %) */
  float voltage[8];
  /**< Voltage measured by each ESC (Units: V) */
} EscRaw;


typedef struct
{
  VersionInfo version_info;
  GeneralStatus general_status;
  DataStatus data_status;
  UpdateRates update_rates;
  AttitudeEstimate attitude_estimate;
  CpuStats cpu_stats;
  Imu0Raw imu_0_raw;
  Imu0Compensated imu_0_compensated;
  Imu0CalibrationThermal imu_0_calibration_thermal;
  Imu0CalibrationOffset imu_0_calibration_offset;
  Barometer0Raw barometer_0_raw;
  Sonar0Raw sonar_0_raw;
  Mag0Raw mag_0_raw;
  Mag1Raw mag_1_raw;
  Mag0Compensated mag_0_compensated;
  Mag1Compensated mag_1_compensated;
  Mag0Calibration3D mag_0_calibration_3d;
  SpektrumRc0Raw spektrum_rc_0_raw;
  ApiRcRaw api_rc_raw;
  RcActive rc_active;
  Camera0FrameInfo camera_0_frame_info;
  OpticFlow0Raw optic_flow_0_raw;
  OpticFlow0CalibrationTilt optic_flow_0_calibration_tilt;
  Gps0Raw gps_0_raw;
  HighLevelControlData high_level_control_data;
  EscRaw esc_raw;
} SnavCachedData;

#endif // SNAV_CACHED_DATA_H_
