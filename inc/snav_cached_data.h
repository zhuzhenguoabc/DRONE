/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef SNAV_CACHED_DATA_H_
#define SNAV_CACHED_DATA_H_

#include <stdint.h>

#include "snav_types.h"

/** @addtogroup sn_datatypes
@{ */
/**
 * Version information required to uniquely identify the software and device.
 */
typedef struct
{
  char compile_date[16];
  /**< Null terminated string containing the compilation date. */
  char compile_time[16];
  /**< Null terminated string containing the compilation time. */
  char library_version[18];
  /**< Null terminated string representing version information. */
  char library_hash[41];
  /**< Null terminated string with a unique build identifier. */
  char mac_address[18];
  /**< Null terminated string containing wlan0 mac address if it was successfully polled. */
  char dspal_version[40];
  /**< Version of DSPaL library used. */
  int32_t esc_hw_version[8];
  /**< Hardware revision of the ESCs. */
  int32_t esc_sw_version[8];
  /**< Software version of the ESCs. */
} VersionInfo;

/**
 * Machine vision (MV) version information.
 */
typedef struct
{
  char version_recommended[18];
  /**< Null-terminated string containing the recommended MV SDK version. */
  char version_found[18];
  /**< Null-terminated string containing the MV SDK version found on the system. */
  uint8_t strict_checking;
  /**< Specifies if the recommended MV SDK version is required. @newpagetable */
} MvSdkVersionInfo;

/**
 * Version of the sensor_imu API.
 */
typedef struct
{
  char version_recommended[18];
  /**< Null-terminated string containing the recommended sensor_imu API version. */
  char version_found[18];
  /**< Null-terminated string containing the sensor_imu API version found on the system. */
  uint8_t strict_checking;
  /**< Specifies if the recommended sensor_imu API version is required. */
} SensorImuApiVersionInfo;

/**
 * General system state information for debugging system issues.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the most recent iteration of the main flight loop started.  (Units: @us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run. */
  int32_t desired_mode;
  /**< Cast to enum: #SnMode. Desired mode that the flight controller attempts to transition to. */
  int32_t current_mode;
  /**< Cast to enum: #SnMode. Current flight controller mode. */
  float voltage;
  /**< Estimated input system voltage.  (Units: V) */
  float current;
  /**< If available, the estimated electrical current being used by the system.  (Units: A) */
  int32_t is_using_external_voltage;
  /**< Cast to enum: #SnavBool. 1 -- Voltage is being measured by the external voltage driver.  0 -- Voltage is measured using ESCs. */
  int32_t props_state;
  /**< Cast to enum: #SnPropsState. Propeller state. */
  uint8_t on_ground;
  /**< Flag representing flight controller's detection of the device being on the ground. 1 -- Device on the ground, 0 otherwise. */
  int32_t input_cmd_type;
  /**< Cast to enum: #SnInputCommandType. Input command type. */
  char last_error_code[32];
  /**< Null-terminated string to represent the last error code detected. To allow detection of infrequent errors, this string persists even if cleared by an error code. */
} GeneralStatus;

/**
 * Status of various sensors and estimators.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged.  (Units: ms) */
  uint32_t loop_cntr;
  /**< Number of times at which the control loop has run. */
  int32_t imu_0_status;
  /**< Cast to enum: #SnDataStatus. IMU sensor status. */
  int32_t baro_0_status;
  /**< Cast to enum: #SnDataStatus. Barometer sensor status. */
  int32_t esc_feedback_status;
  /**< Cast to enum: #SnDataStatus. ESC feedback status. */
  int32_t mag_0_status;
  /**< Cast to enum: #SnDataStatus. Magnetometer sensor. */
  int32_t gps_0_status;
  /**< Cast to enum: #SnDataStatus. Global positioning GPS sensor status. */
  int32_t sonar_0_status;
  /**< Cast to enum: #SnDataStatus. Sonar sensor status. */
  int32_t optic_flow_0_status;
  /**< Cast to enum: #SnDataStatus. Optic flow (DFT) sensor status. */
  int32_t spektrum_rc_0_status;
  /**< Cast to enum: #SnDataStatus. Spektrum RC sensor status. */
  int32_t api_rc_status;
  /**< Cast to enum: #SnDataStatus. API RC command status. */
  int32_t rc_active_status;
  /**< Cast to enum: #SnDataStatus. Active RC command status. */
  int32_t height_estimator_status;
  /**< Cast to enum: #SnDataStatus. Height estimator status. */
  int32_t attitude_estimator_status;
  /**< Cast to enum: #SnDataStatus. Attitude estimator status. */
  int32_t vio_0_status;
  /**< Cast to enum: #SnDataStatus. VIO status. */
  int32_t voa_status;
  /**< Cast to enum: #SnDataStatus. VOA status. */
} DataStatus;

/**
 * System and sensor update rates.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t loop_cntr;
  /**< Number of times the control loop has run. */
  float control_loop_freq;
  /**< Main control loop update frequency.  (Units: Hz) */
} UpdateRates;

/**
 * Estimate of the vehicle orientation.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter incremented upon successful computation of the attitude estimate. */
  float roll;
  /**< Roll angle using Tait-Bryan ZYX.  (Units: rad) */
  float pitch;
  /**< Pitch angle using Tait-Bryan ZYX.  (Units: rad) */
  float yaw;
  /**< Yaw angle using Tait-Bryan ZYX.  (Units: rad) */
  float rotation_matrix[9];
  /**< Rotation matrix from vehicle body to world in row-major order. */
  float magnetic_yaw_offset;
  /**< Yaw angle with respect to magnetic east = yaw + magnetic_yaw_offset. (Units: rad) */
  float magnetic_declination;
  /**< Yaw angle with respect to true east = yaw + magnetic_yaw_offset + magnetic_declination. (Units: rad) */
} AttitudeEstimate;

/**
 * Apps processor CPU status.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times data was updated */
  uint64_t time_apps_us;
  /**< Timestamp from the Apps processor.  (Units: @us) */
  float cur_freq[4];
  /**< Current Apps processor CPU frequency. If the CPU is not online, Snapdragon Navigator reads NaN.  (Units: GHz) */
  float max_freq[4];
  /**< Maximum Apps processor CPU frequency -- Can be throttled due to temperature. If the CPU is not online, Snapdragon Navigator reads NaN.  (Units: GHz) */
  float temp[13];
  /**< Temperature measurements from thermal zones. Reads NaN if the thermal zone is disabled.  (Units: @degc) */
} CpuStats;

/**
 * Inertial measurement unit 0 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu0Raw;

/**
 * Inertial measurement unit 1 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu1Raw;

/**
 * Inertial measurement unit 2 raw data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of measurements received. */
  float temp;
  /**< IMU temperature.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu2Raw;

/**
 * Inertial measurement unit (IMU) data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of compensated measurements recorded. */
  float temp;
  /**< Temperature of IMU.  (Units: @degc) */
  float lin_acc[3];
  /**< Linear acceleration.  (Units: gravity (~9.81 m/s/s)) */
  float ang_vel[3];
  /**< Angular velocity.  (Units: rad/s) */
} Imu0Compensated;

/**
 * Data results from IMU temperature calibration.
 */
typedef struct
{
  float accel_slope[3];
  /**< XYZ slopes for accelerometer temperature calibration. \n  (Units: gravity/@degc) */
  float accel_offset[3];
  /**< XYZ offsets for accelerometer temperature calibration. \n  (Units: gravity) */
  float accel_residual[3];
  /**< Average squared residual for accelerometer temperature calibration.  (Units: gravity^2) */
  float gyro_slope[3];
  /**< XYZ slopes for gyroscope temperature calibration. \n  (Units: (rad/s)/@degc) */
  float gyro_offset[3];
  /**< XYZ offsets for gyroscope temperature calibration.  (Units: (rad/s))) */
  float gyro_residual[3];
  /**< Average squared residual for gyroscope temperature calibration.  (Units: (rad/s)^2) */
} Imu0CalibrationThermal;

/**
 * IMU sensor offset values.
 */
typedef struct
{
  char name[16];
  /**< Type of offset calibration (e.g., static or dynamic). */
  float accel_offset[3];
  /**< XYZ accelerometer offsets from accelerometer offset calibration.  (Units: gravity (~9.81 m/s/s)) */
  float avg_thrust;
  /**< Average thrust found from in flight accelerometer calibration.  (Units: g) */
  float roll_trim_offset;
  /**< Roll trim offset from in flight accelerometer calibration. \n  (Units: g) */
  float pitch_trim_offset;
  /**< Pitch trim offset from in flight accelerometer calibration.  (Units: g) */
} Imu0CalibrationOffset;

/**
 * Raw barometer data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times data was read. */
  float pressure;
  /**< Atmospheric pressure measurement.  (Units: Pa) */
  float temp;
  /**< Temperature of the sensor.  (Units: @degc) */
} Barometer0Raw;

/**
 * Raw sonar data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float range;
  /**< Range measurement.  (Units: m) */
} Sonar0Raw;

/**
 * Raw magnetometer data from the mag0 sensor.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame.  (Units: @uT) */
} Mag0Raw;

/**
 * Raw magnetometer data from the mag1 sensor.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. (Units: @uT) */
} Mag1Raw;

/**
 * Mag0 sensor data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. */
} Mag0Compensated;

/**
 * Mag1 sensor data after compensation.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of data packets received. */
  uint8_t identifier;
  /**< Type of compass sensor. */
  float field[3];
  /**< XYZ components of the magnetic field in the sensor frame. */
} Mag1Compensated;

/**
 * Data results from Mag0 3D calibration.
 */
typedef struct
{
  float matrix[9];
  /**< Scale parameters of mapping. */
  float offset[3];
  /**< XYZ offset of mapping. */
} Mag0Calibration3D;

/**
 * Raw Spektrum RC data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t protocol;
  /**< Cast to enum: #SnRcReceiverMode. RC protocol identifier */
  uint8_t num_channels;
  /**< Number of RC channels being populated. */
  uint16_t vals[16];
  /**< Raw Spektrum channel values. */
} SpektrumRc0Raw;

/**
 * RC commands sent through the API.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t cmd_type;
  /**< Cast to enum: #SnRcCommandType. How the command is interpreted (if possible). */
  int32_t cmd_options;
  /**< Cast to enum: #SnRcCommandOptions. Options used to deviate from linear mapping. */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1]. */
} ApiRcRaw;

/**
 * RC commands for control.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  int32_t source;
  /**< Cast to enum: #SnRcCommandSource. Source of the active RC commands. */
  int32_t cmd_type;
  /**< Cast to enum: #SnRcCommandType. Specifies how the commands are interpreted (if possible). Irrelevant if the source is Spektrum RC. */
  int32_t cmd_options;
  /**< Cast to enum: #SnRcCommandOptions. Options used to deviate from linear mapping. */
  float cmd[4];
  /**< Unitless RC-type command in range [-1, 1]. */
} RcActive;

/**
 * Captured camera frame information from the downward camera.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  uint32_t frame_num;
  /**< Frame number received, starting at 0. */
  int64_t frame_timestamp;
  /**< Timestamp of frame.  (Units: @us) */
  float exposure;
  /**< Normalized exposure setting used to take the frame. */
  float gain;
  /**< Normalized gain setting used to take the frame. */
  float average_luminance;
  /**< Normalized average luminance of the frame. */
} Camera0FrameInfo;

/**
 * Downward facing tracker (DFT) data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the struct was logged.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float pixel_flow[2];
  /**< Pixel displacement between subsequent image frames. Pixel flow is in the opposite direction of camera (and therefore vehicle) movement.  (Units: pixels) */
  int32_t sample_size;
  /**< Number of inliers after calculation of displacement. */
  float error_sum;
  /**< Error metric, sum of squared error over sample size points. */
} OpticFlow0Raw;

/**
 * Downfacing camera calibration for tilt angle (optic flow camera yaw calibration).
 */
typedef struct
{
  float x_factor;
  /**< Tilt factor in X.  (Units: pixels/rad) */
  float y_factor;
  /**< Tilt factor in Y.  (Units: pixels/rad) */
} OpticFlow0CalibrationTilt;

/**
 * Raw GPS data.
 */
typedef struct
{
  uint32_t iter;
  /**< Loop iteration in which data was logged. */
  int64_t time;
  /**< Time at which the data was received.  (Units: @us) */
  uint32_t cntr;
  /**< Number of complete messages received. */
  uint8_t identifier;
  /**< Type of GNSS receiver. */
  uint32_t num_errors;
  /**< Number of CRC errors. */
  uint32_t gps_week;
  /**< GPS week number.  (Units: weeks) */
  uint32_t gps_time_sec;
  /**< Time of week.  (Units: sec) */
  uint32_t gps_time_nsec;
  /**< Time of week.  (Units: ns) */
  int32_t latitude;
  /**< Position latitude.  (Units: deg*10e7) */
  int32_t longitude;
  /**< Position longitude.  (Units: deg*10e7) */
  float altitude;
  /**< Altitude at mean sea level (MSL).  (Units: m) */
  float lin_vel[3];
  /**< Velocity of the north-east-up (NEU) frame.  (Units: m/s) */
  uint8_t fix_type;
  /**< Fix type/quality. */
  uint8_t num_satellites;
  /**< Number of satellites used in the solution. */
  float horizontal_acc;
  /**< Horizontal accuracy of the position estimate.  (Units: m) */
  float speed_acc;
  /**< Horizontal speed accuracy.  (Units: m/s) */
  uint8_t agc[2];
  /**< Value of the automatic gain controller. */
  uint8_t sv_ids[20];
  /**< Satellite identification number. */
  uint8_t sv_cno[20];
  /**< Satellite signal strength.  (Units: C/N0) */
} Gps0Raw;

/**
 * High-level control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter that is incremented with each control loop. */
  float position_estimated[3];
  /**< Estimated XYZ position. This data is subject to change in future releases.  (Units: m) */
  float yaw_estimated;
  /**< Estimated yaw angle. This data is subject to change in future releases.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position. This data is subject to change in future releases.  (Units: m) */
  float yaw_desired;
  /**< Desired yaw angle. This data is subject to change in future releases.  (Units: rad) */
} HighLevelControlData;

/**
 * VIO position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter that is incremented with each control loop. */
  float position_estimated[3];
  /**< Estimated XYZ position. (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity. (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position. (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity. (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle.  (Units: rad) */
  uint8_t is_valid;
  /**< Is 1 if the VIO position and velocity data is valid. */
} VioPosVel;

/**
 * GPS position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Counter that is incremented with each control loop. */
  float position_estimated[3];
  /**< Estimated XYZ position. +X is east, +Y is north, +Z is vertically up  (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity.  (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle of the vehicle’s body-fixed frame with respect to the East North Up (ENU) frame.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position.  (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity.  (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle of the vehicle’s body-fixed frame with respect to the ENU frame.  (Units: rad) */
  uint8_t is_enabled;
  /**< If enabled (set to 1), this data is populated when the vehicle gets a GPS lock. */
} GpsPosVel;

/**
 * Optic flow position and velocity control data.
 */
typedef struct
{
  int64_t time;
  /**< Timestamp.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times the data was read. */
  float position_estimated[3];
  /**< Estimated XYZ position.  (Units: m) */
  float velocity_estimated[3];
  /**< Estimated XYZ velocity.  (Units: m/s) */
  float yaw_estimated;
  /**< Estimated yaw angle.  (Units: rad) */
  float position_desired[3];
  /**< Desired XYZ position.  (Units: m) */
  float velocity_desired[3];
  /**< Desired XYZ velocity.  (Units: m/s) */
  float yaw_desired;
  /**< Desired yaw angle.  (Units: rad) */
  uint8_t is_valid;
  /**< 1 if the optic flow position and velocity data are valid, otherwise 0.  */
} OpticFlowPosVel;

/**
 * Raw ESC data.
 */
typedef struct
{
  int64_t time;
  /**< Time at which this data was published.  (Units: @us) */
  uint32_t cntr;
  /**< Number of times ESC feedback was updated. */
  uint32_t bytes_tx;
  /**< Number of bytes received from ESCs. (Units: bytes) */
  uint32_t bytes_rx;
  /**< Number of bytes received from all ESCs. (Units: bytes) */
  uint32_t errors_rx;
  /**< Number of feedback read errors. */
  uint8_t packet_cntr[8];
  /**< Number of control packets received by each ESC. */
  uint32_t packets_rx[8];
  /**< Number of packets received from each ESC. (Units: packets) */
  int16_t rpm[8];
  /**< Motor RPM. */
  int8_t power[8];
  /**< Power applied by the ESC.  (Units: %) */
  float voltage[8];
  /**< Voltage measured by each ESC.  (Units: V) */
} EscRaw;

/**
 * Additional detailed power and temperature information.
 */
typedef struct
{
  int64_t time;
  /**< Time at which the packet was logged. */
  uint32_t cntr;
  /**< Number of packets logged. */
  float voltage;
  /**< Total battery voltage. (Units: V) */
  float voltage_cells[6];
  /**< Voltage of the individual battery cells. (Units: V) */
  float voltage_aux;
  /**< Voltage at AUX input. (Units: V) */
  float current;
  /**< Total battery current. (Units: A) */
  float current_fc;
  /**< Current going to the flight controller. (Units: A) */
  float temperatures[4];
  /**< Board-specific temperature measurements. (Units: @degc) */
} PowerStatus;

/**
 * Snapdragon Navigator cached data for the sn_get_flight_data_ptr() function.
 */
typedef struct
{
  VersionInfo version_info;
  /**< Version information required to uniquely identify the software and device. */
  MvSdkVersionInfo mv_sdk_version_info;
  /**< Machine vision (MV) version information. */
  SensorImuApiVersionInfo sensor_imu_api_version_info;
  /**< Version of the sensor_imu API. */
  GeneralStatus general_status;
  /**< General system state information for debugging system issues. */
  DataStatus data_status;
  /**< Status of various sensors and estimators. */
  UpdateRates update_rates;
  /**< System and sensor update rates. */
  AttitudeEstimate attitude_estimate;
  /**< Estimate of the vehicle orientation. */
  CpuStats cpu_stats;
  /**< Apps processor CPU status. */
  Imu0Raw imu_0_raw;
  /**< Inertial measurement unit 0 raw data. */
  Imu1Raw imu_1_raw;
  /**< Inertial measurement unit 1 raw data. */
  Imu2Raw imu_2_raw;
  /**< Inertial measurement unit 2 raw data. */
  Imu0Compensated imu_0_compensated;
  /**< Inertial measurement unit (IMU) data after compensation. */
  Imu0CalibrationThermal imu_0_calibration_thermal;
  /**< Data results from IMU temperature calibration. */
  Imu0CalibrationOffset imu_0_calibration_offset;
  /**< IMU sensor offset values. */
  Barometer0Raw barometer_0_raw;
  /**< Raw barometer data. */
  Sonar0Raw sonar_0_raw;
  /**< Raw sonar data. */
  Mag0Raw mag_0_raw;
  /**< Raw magnetometer data from the mag0 sensor. */
  Mag1Raw mag_1_raw;
  /**< Raw magnetometer data from the mag1 sensor. */
  Mag0Compensated mag_0_compensated;
  /**< Mag0 sensor data after compensation. */
  Mag1Compensated mag_1_compensated;
  /**< Mag1 sensor data after compensation. */
  Mag0Calibration3D mag_0_calibration_3d;
  /**< Data results from Mag0 3D calibration. */
  SpektrumRc0Raw spektrum_rc_0_raw;
  /**< Raw Spektrum RC data. */
  ApiRcRaw api_rc_raw;
  /**< RC commands sent through the API. */
  RcActive rc_active;
  /**< RC commands for control. */
  Camera0FrameInfo camera_0_frame_info;
  /**< Captured camera frame information from the downward camera. */
  OpticFlow0Raw optic_flow_0_raw;
  /**< Downward facing tracker (DFT) data. */
  OpticFlow0CalibrationTilt optic_flow_0_calibration_tilt;
  /**< Downfacing camera calibration for tilt angle (optic flow camera yaw calibration). */
  Gps0Raw gps_0_raw;
  /**< Raw GPS data. */
  HighLevelControlData high_level_control_data;
  /**< High-level control data. */
  VioPosVel vio_pos_vel;
  /**< VIO position and velocity control data. */
  GpsPosVel gps_pos_vel;
  /**< GPS position and velocity control data. */
  OpticFlowPosVel optic_flow_pos_vel;
  /**< Optic flow position and velocity control data. */
  EscRaw esc_raw;
  /**< Raw ESC data. */
  PowerStatus power_status;
  /**< Additional detailed power and temperature information. */
} SnavCachedData;

/** @} */ /* end_addtogroup sn_datatypes */

#endif // SNAV_CACHED_DATA_H_
