/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef SN_INTERFACE_H_
#define SN_INTERFACE_H_

#include "snav_cached_data.h"
#include "snav_types.h"

#define VERSION "1.2.0"
#define DEPRECATED __attribute__((deprecated))

/** @file */

#ifdef __cplusplus
extern "C"{
#endif


/** @addtogroup sn_interface
@{ */
/**
 * Updates the internal cache of flight control data.
 *
 * @detdesc
 * This function caches the current state of all other components that can be
 * queried. This function must be called once per control loop before querying
 * the flight software information.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_update_data();

/**
 * Gets the current mode from the internal cache of flight control data.
 *
 * @datatypes
 * #SnMode
 *
 * @param[out] mode_id
 * Pointer to the value to be set to the current mode.
 *
 * @detdesc
 * This mode represents the current state of the flight control software.
 * See the #SnMode enum for an interpretation.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_mode(SnMode *mode_id);

/**
 * Gets the current state of propellers from the internal cache of flight control
 * data.
 *
 * @datatypes
 * #SnPropsState
 *
 * @param[out] props_state Pointer to the value (SnPropsState) to be set to the
 *                         current state of the propellers.
 *
 * @detdesc
 * This function can be used to determine whether propellers are currently stopped,
 * starting, or spinning.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_props_state(SnPropsState *props_state);

/**
 * Gets the on ground flag from the internal cache of flight control data.
 *
 * @param[out] on_ground_flag Pointer to the flag to be set to 0 if flight control
 * determines that vehicle is not on ground or 1 if flight control determines that
 * the vehicle is on the ground.
 *
 * @detdesc
 * This flag helps determine if it is safe to call the sn_stop_props() function.
 * There is a delay between the vehicle landing and the on_ground_flag parameter
 * changing to 1.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_on_ground_flag(int *on_ground_flag);

/**
 * Gets the latest battery voltage from the internal cache of flight control
 * data.
 *
 * @param[out] voltage Pointer to be set to the latest battery voltage (Volts).
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_voltage(float *voltage);


/**
 * Detects whether GPS is enabled.
 *
 * @param[out] gps_enabled
 * Pointer to GPS enabled/disabled -- 1 if GPS is enabled; 0 otherwise.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 */
int sn_is_gps_enabled(int * gps_enabled);

/**
 * Gets the latest IMU data status from the internal cache of flight control
 * data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status Pointer to the value to be set to the latest status of
 *                    the IMU data.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_imu_status(SnDataStatus *status);

/**
 * Gets the latest IMU temperature reading from the internal cache of flight
 * control data.
 *
 * @param[out] temp Pointer to the float to be set to the latest IMU temperature
 *                  reading in degrees Celsius.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_imu_temp(float *temp);

/**
 * Gets the bias compensated linear acceleration vector from the internal cache
 * of flight control data. The vector is represented with respect to the
 * vehicle's body frame.
 *
 * @param[out] ax Pointer to the float to be set to the latest X component of the
 *                bias compensated linear acceleration vector in g's.
 * @param[out] ay Pointer to the float to be set to the latest Y component of the
 *                bias compensated linear acceleration vector in g's.
 * @param[out] az Pointer to the float to be set to the latest Z component of the
 *                bias compensated linear acceleration vector in g's.
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_bias_compensated_lin_acc(float *ax, float *ay, float *az);

/**
 * Gets the bias compensated angular velocity vector from the internal cache of
 * flight control data. The vector is represented with respect to the vehicle's
 * body frame.
 *
 * @param[out] wx Pointer to the float to be set to the latest X component of the
 *                bias compensated angular velocity vector in rad/s.
 * @param[out] wy Pointer to the float to be set to the latest Y component of the
 *                bias compensated angular velocity vector in rad/s.
 * @param[out] wz Pointer to the float to be set to the latest Z component of the
 *                bias compensated angular velocity vector in rad/s.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_bias_compensated_ang_vel(float *wx, float *wy, float *wz);

/**
 * Gets the latest barometer data status from the internal cache of flight
 * control data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status Pointer to the value to be set to
 *                    the latest barometer data status.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_baro_status(SnDataStatus *status);

/**
 * Gets the latest barometer data from the internal cache of flight control data.
 *
 * @param[out] pressure Pointer to the float to be set to the latest barometer
 *                      pressure reading in Pascals.
 * @param[out] temp     Pointer to the float to be set to the latest barometer
 *                      temperature reading in degrees Celsius.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_baro_data(float *pressure, float *temp);

/**
 * Gets the latest RC data status from the internal cache of
 * flight control data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status Pointer to the value to be set to the latest status
 *                    of the RC data.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_rc_status(SnDataStatus *status);

/**
 * Gets the latest RC data from the internal cache of flight
 * control data.
 *
 * @param[out] channel_vals Pointer to array to be filled with RC channel values.
 *
 * @param[in] size          Number of channel_vals array elements.
 * @param[out] used         Pointer to be set to the number of elements of the
 *                          channel_vals array used.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is too small for all of the RC data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_rc_data(unsigned int *channel_vals, unsigned int size,
    unsigned int *used);

/**
 * Gets the latest magnetometer data status from the internal cache of flight
 * control data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the latest status of the magnetometer data.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_mag_status(SnDataStatus *status);

/**
 * Gets the latest GPS data status from the internal cache of flight control data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status Pointer to the value to be set to the latest GPS data status.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_gps_status(SnDataStatus *status);

/**
 * Gets the latest sonar data status from the internal cache of flight control
 * data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status Pointer to the value to be set to the latest sonar data status.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_sonar_status(SnDataStatus *status);

/**
 * Gets the latest sonar data from the internal cache of flight control data.
 *
 * @param[out] range Pointer to the float to be set to the latest sonar range
 *                   measurement in meters.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_sonar_data(float *range);

/**
 * Gets the latest optic flow data status from the internal cache of flight
 * control data.
 *
 * @datatypes
 * #SnDataStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the latest status of the optic flow data.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_optic_flow_status(SnDataStatus *status);

/**
 * Gets the latest optic flow sample size from the internal cache of flight
 * control data. Sample size refers to the number of inlier features being used by the
 * optic flow algorithm.
 *
 * @param[out] sample_size
 * Pointer to the value to be set to the latest optic flow sample size.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_optic_flow_sample_size(int *sample_size);

/**
 * Gets the control loop frequency from the internal cache of flight control data.
 *
 * @param[out] frequency
 * Pointer to the float to be set to the control loop
 * frequency in Hz.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_control_loop_frequency(float *frequency);

/**
 * Gets the estimated position vector from the internal cache of flight control
 * data.
 *
 * @param[out] x Pointer to the float to be set to estimated X position (m).
 * @param[out] y Pointer to the float to be set to estimated Y position (m).
 * @param[out] z Pointer to the float to be set to estimated Z position (m).
 *
 * @detdesc
 * This function returns the fused position generated by the flight estimator.
 * These components are in a Z-up (relative to gravity) coordinate frame.
 * These values are relative to the initialization location and direction.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_position_est(float *x, float *y, float *z);

/**
 * Gets the estimated velocity vector from the internal cache of flight control
 * data.
 *
 * @param[out] vx Pointer to the float to be set to estimated X velocity (m/s).
 * @param[out] vy Pointer to the float to be set to estimated Y velocity (m/s).
 * @param[out] vz Pointer to the float to be set to estimated Z velocity (m/s).
 *
 * @detdesc
 * This function returns the fused velocity generated by the flight estimator.
 * These components are in a Z-up (relative to gravity) coordinate frame.
 * These values are relative to the initialization location and direction.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 */
int sn_get_velocity_est(float *vx, float *vy, float *vz);

/**
 * Gets the estimated yaw angle from the internal cache of flight control data.
 *
 * @param[out] yaw Pointer to the float to be set to estimated yaw angle (radians).
 *
 * @detdesc
 * This angle is the relative estimated rotation about a gravity-oriented
 * vector.
 * @par
 * This angle is initialized to zero.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_yaw_est(float *yaw);

/**
 * Gets the desired position vector from the internal cache of flight control
 * data.
 *
 * @param[out] x Pointer to the float to be set to desired X position (m).
 * @param[out] y Pointer to the float to be set to desired Y position (m).
 * @param[out] z Pointer to the float to be set to desired Z position (m).
 *
 * @detdesc
 * This function allows access to the position components that the internal controller
 * is attempting to attain. These components are in a Z-up (relative to gravity)
 * coordinate frame. These values are relative to the initialization location
 * and direction.
 * @par
 * @note1hang It is best to perform simple external control using this vector as
 *            the current state. The internal controller then controls the higher
 *            order dynamics to attain this position.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_position_des(float *x, float *y, float *z);

/**
 * Gets the desired yaw angle from the internal cache of flight control data.
 *
 * @param[out] yaw
 * Pointer to the float to be set to desired yaw angle (radians).
 *
 * @detdesc
 * This angle is the relative desired rotation about a gravity-oriented vector.
 * @par
 * @note1hang Similar to the desired position components, it is best to control
 *            the yaw using this value as the current yaw.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_yaw_des(float *yaw);

/**
 * Gets the estimated quaternion from the body frame to world frame from
 * the internal cache of flight control data.
 *
 * @param[out] w Pointer to the float to be set to W component of quaternion.
 * @param[out] x Pointer to the float to be set to X component of quaternion.
 * @param[out] y Pointer to the float to be set to Y component of quaternion.
 * @param[out] z Pointer to the float to be set to Z component of quaternion.
 *
 * @detdesc
 * This unit quaternion represents an estimate of the rotation from the vehicle
 * body frame into the world frame. This quaternion represents the same rotation
 * in SO(3) as the sn_get_rot_est() and sn_get_euler_angles_est() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 * @newpage
 */
int sn_get_quat_est(float *w,float *x,float *y,float *z);

/**
 * Gets the estimated rotation matrix from body frame to world frame from
 * internal cache of flight control data.
 *
 * @param[out] R[][]
 * Pointer to the array to be filled with the direction cosine matrix.
 *
 * @detdesc
 * This matrix represents an estimate of the rotation from the vehicle body
 * frame into the world frame. This matrix provides the same rotation in SO(3)
 * as the sn_get_quat_est() and sn_get_euler_angles_est() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (most flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_rot_est(float R[3][3]);

/**
 * Gets the estimated ZYX Euler angles from body frame to world frame from
 * internal cache of flight control data.
 *
 * @param[out] roll  Pointer to the float to be set to the roll angle in radians.
 * @param[out] pitch Pointer to the float to be set to the pitch angle in radians.
 * @param[out] yaw   Pointer to the float to be set to the yaw angle in radians.
 *
 * @detdesc
 * These ZYX Euler angles represent the same rotation in SO(3) as the
 * sn_get_quat_est() and sn_get_rot_est() functions.
 * @par
 * @note1hang Euler angles have limitations -- Using the sn_get_rot_est() or
 * sn_get_quat_est() function is preferred.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_get_rot_est() \n
 * sn_get_quat_est()
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_euler_angles_est(float *roll, float *pitch, float *yaw);

/**
 * Gets the ESC hardware versions from the internal cache of flight control
 * data.
 *
 * @param[out] hw_versions
 * Pointer to the array to be filled with hardware versions from the ESCs.
 * The array is filled in ascending order of ESC IDs, e.g., [hw_v_0, hw_v_1,
 * ..., hw_v_n].
 *
 * @param[in] size    Number of hw_versions array elements.
 * @param[out] used   Pointer to the value to be set to the number of elements
 *                    used in the hw_versions array.
 *
 * @detdesc
 * @par
 * If the given array is too small to hold all of the feedback data, no data
 * copies into the array and the function returns an error code.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the array size is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data. @newpage
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_esc_hw_versions(int *hw_versions, unsigned int size,
    unsigned int *used);

/**
 * Gets the ESC software versions from the internal cache of flight control
 * data.
 *
 * @param[out] sw_versions
 * Pointer to the array to be filled with software versions from the ESCs.
 * The array is filled in ascending order of ESC IDs, e.g., [sw_v_0, sw_v_1,
 * ..., sw_v_n].
 *
 * @param[in] size    Number of sw_versions array elements.
 * @param[out] used   Pointer to the value to be set to the number of elements
 *                    used in the sw_versions array.
 *
 * @detdesc
 * @par
 * If the given array is too small to hold all of the feedback data, no data
 * copies into the array and the function returns an error code.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the array size is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data. @newpage
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_esc_sw_versions(int *sw_versions, unsigned int size,
    unsigned int *used);

/**
 * Gets the packet counter feedback data from the ESCs from the internal cache of
 * flight control data.
 *
 * @param[out] cntr_feedback
 * Pointer to the array to be filled with counters from ESCs. The array is filled
 * in ascending order of ESC IDs, e.g., [cntr_0, cntr_1, ..., cntr_n]
 *
 * @param[in] size    Number of cntr_feedback array elements.
 * @param[out] used   Pointer to the value to be set to the number of elements
 *                    used in the cntr_feedback array.
 *
 * @detdesc
 * The counter refers to the number of command packets received by the ESC.
 * The counter is in the range [0, 255] and wraps back to 0 after reaching 255.
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller.
 * @par
 * If the given array is too small to hold all of the feedback data, no data
 * copies into the array and the function returns an error code.
 * @par
 * @note1hang ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the array size is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 */
DEPRECATED int sn_get_esc_packet_cntr_feedback(unsigned int *cntr_feedback,
    unsigned int size, unsigned int *used);

/**
 * Gets the ESC-generated RPM feedback data from the internal cache of flight
 * control data.
 *
 * @param[out] rpm_feedback Pointer to the array to be filled with RPMs from ESCs.
 *                          The array is filled in ascending order of ESC IDs,
 *                          e.g., [rpm_0, rpm_1, ..., rpm_n].
 * @param[in] size          Number of rpm_feedback array elements.
 * @param[out] used         Pointer to to the value to be set to the number of
 *                          elements used in the rpm_feedback array.
 *
 * @detdesc
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller. If the given array is too small to
 * hold all of the feedback data, no data copies into the array and the function
 * returns an error code.
 * @par
 * @note1hang
 * ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_esc_rpm_feedback(int *rpm_feedback, unsigned int size,
    unsigned int *used);

/**
 * Gets the power feedback data from the ESCs
 * from the internal cache of flight control data.
 *
 * @param[out] power_feedback Pointer to int array to be filled with power data
 * from ESCs. Power lies in the [-100, 100] range in which 100 corresponds to 100%
 * duty cycle and negative implies reversed direction. Array is filled in ascending
 * order of ESC IDs, e.g., [power_0, power_1, ..., power_n]
 *
 * @param[in] size            Number of power_feedback array elements.
 * @param[out] used           Pointer to the value to be set to the number of
 *                            elements used in the power_feedback array.
 *
 * @detdesc
 * Power returns in the range [-100, 100], where 100 corresponds to
 * 100% duty cycle and negative implies reversed direction.
 * @par
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller.If the given array is too small to hold
 * all of the feedback data, no data copies into the array and the function
 * returns an error code.
 * @par
 * @note1hang ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_esc_power_feedback(int *power_feedback, unsigned int size,
    unsigned int *used);

/**
 * Gets the voltage feedback data from the ESCs from the internal cache of flight
 * control data.
 *
 * @param[out] voltage_feedback
 * Pointer to the float array to be filled with voltages from ESCs. Array
 * is filled in ascending order of ESC IDs, e.g., [volt_0, volt_1, ..., volt_n].
 *
 * @param[in] size  Number of voltage_feedback array elements.
 * @param[out] used Pointer to the value to be set to the number of elements used
 *                  of the voltage_feedback array.
 *
 * @detdesc
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller. If the given array is too small to
 * hold all of the feedback data, no data copies into the array and the function
 * returns an error code.
 * @par
 * @note1hang ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 *
 * @deprecated
 * This function will be removed in a future release (TBD). Use
 * sn_get_flight_data_ptr() instead.
 *
 * @newpage
 */
DEPRECATED int sn_get_esc_voltage_feedback(float *voltage_feedback, unsigned int size,
    unsigned int *used);

/**
 * Gets the ESC state feedback data from from the internal cache of flight control
 * data.
 *
 * @datatypes
 * #SnMotorState
 *
 * @param[out] state_feedback Pointer to the array to be filled with states from
 *                            ESCs. The array is filled in ascending order of ESC
 *                            IDs, e.g. [state_0, state_1, ..., state_n]
 * @param[in] size            Number of state_feedback array elements.
 * @param[out] used           Pointer to the value to be set to the number of
 *                            elements used of the state_feedback array
 *
 * @detdesc
 * The given array must have a number of elements equal to the number of
 * ESCs connected to the flight controller.
 * @par
 * If the given array is too small to hold all of the feedback data, no data
 * copies into the array and the function returns an error code.
 * @par
 * @note1hang ESC feedback data is only updated if feedback is requested. See the
 * sn_send_esc_rpm() and sn_send_esc_pwm() functions.
 *
 * @return
 * - 0 for success
 * - -1 for failure (flight software non-functional)
 * - -2 if the size of the array is not big enough to hold all of the feedback data
 *
 * @dependencies
 * The sn_update_data() function must be called to refresh the internal cache of
 * flight control data.
 *
 * @sa
 * sn_send_esc_rpm() \n
 * sn_send_esc_pwm()
 *
 */
int sn_get_esc_state_feedback(SnMotorState *state_feedback, unsigned int size,
    unsigned int *used);

/**
 * Non-blocking attempt to spin propellers.
 *
 * @detdesc
 * This function does not guarantee that propellers start spinning.
 * Instead, safety checks are performed and then propellers started if deemed
 * safe.
 * @par
 * This function introduces a time delay before the propellers spin.
 * @par
 * @note1hang For this function to have effect, the following conditions must
 *            be met:
 *            - Propellers must not be spinning -- Verify using the
 *              sn_get_props_state() function
 *            - Vehicle must be in a flight mode
 * @par
 * Check SnPropsState using the sn_get_props_state() function to verify that the
 * command executed.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_spin_props();

/**
 * Non-blocking attempt to stop propellers.
 *
 * @detdesc
 * This function does not guarantee that propellers stop spinning. Safety checks
 * are performed internally and propellers stop if deemed safe.
 * @par
 * @note1hang For this function to have effect, the following conditions must
 *            be met:
 *            - Propellers must be spinning or starting -- Verify using the
 *              sn_get_props_state() function
 *            - Vehicle must be in a flight mode
 * @par
 * Check SnPropsState using the sn_get_props_state() function to verify that the
 * command executed.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_stop_props();

/**
 * Non-blocking attempt to start static accelerometer calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-1) for instructions.
 * @par
 * During this calibration, ensure vehicle is completely stationary on a level
 * surface. Use the sn_get_mode() function to determine whether the calibration
 * succeeds or fails.
 *
 * @par
 * @note1hang Vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_start_static_accel_calibration();

/**
 * Gets the static accelerometer calibration status from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to value to be set to the status of the static accelerometer calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_get_static_accel_calibration_status(SnCalibStatus *status);

/**
 * Non-blocking attempt to start dynamic accelerometer calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so.
 * @par
 * Refer to <em>Qualcomm Snapdragon Navigator User Guide</em> (80-P4698-1) for
 * instructions.
 * @par
 * Use the sn_get_mode() function to determine whether the calibration succeeds or
 * fails.
 * @par
 * @note1hang The vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_start_dynamic_accel_calibration();

/**
 * Gets the status of dynamic accelerometer calibration from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status Pointer to the value to be set to the status of the dynamic
 *                    accelerometer calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_get_dynamic_accel_calibration_status(SnCalibStatus *status);

/**
 * Non-blocking attempt to start thermal IMU calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for vehicle
 * to do so. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-1) for instructions.
 * @par
 * During this calibration, ensure vehicle is completely stationary on a level
 * surface.
 * @par
 * Increase the vehicle temperature during this test to ensure that a large
 * temperature range observed.
 * @par
 * Use the sn_get_mode() function to determine whether the calibration succeeds
 * or fails.
 * @par
 * @note1hang The vehicle must be rebooted after calibration to enable flight.
 * @par
 * @note1hang A static calibration is required immediately after thermal
 *            calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_start_imu_thermal_calibration();

/**
 * Gets the status of thermal IMU calibration from the internal cache of flight
 * control data. This function can be used to determine if calibration data exists
 * or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the
 * status of the IMU thermal calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_get_imu_thermal_calibration_status(SnCalibStatus *status);

/**
 * Non-blocking attempt to start optic flow camera yaw calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for the
 * vehicle. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-1) for instructions on how to run this calibration.
 * @par
 * @note1hang
 * Vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_start_optic_flow_camera_yaw_calibration();

/**
 * Gets the status of optic flow camera yaw calibration from the internal cache
 * of flight control data. This function can be used to determine if calibration
 * data exists or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the status of the optic flow camera
 * yaw calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_get_optic_flow_camera_yaw_calibration_status(SnCalibStatus *status);

/**
 * Non-blocking attempt to start magnetometer (compass) calibration.
 *
 * @detdesc
 * Calibration only starts if it is deemed safe and appropriate for the
 * vehicle. Refer to <em>Qualcomm Snapdragon Navigator User Guide</em>
 * (80-P4698-1) for instructions on how to run this calibration.
 * @par
 * @note1hang
 * The vehicle must be rebooted after calibration to enable flight.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 */
int sn_start_magnetometer_calibration();

/**
 * Gets the magnetometer calibration status from the internal cache of flight
 * control data. This function can be used to determine if calibration data exists
 * or if the calibration procedure is in progress.
 *
 * @datatypes
 * #SnCalibStatus
 *
 * @param[out] status
 * Pointer to the value to be set to the status of the magnetometer
 * calibration.
 *
 * @return
 * - 0 if attempt was received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None. @newpage
 */
int sn_get_magnetometer_calibration_status(SnCalibStatus *status);

/**
 * Sends RPM commands to the ESCs and requests feedback.
 *
 * @param[in] rpm_data Pointer to the array containing RPM data to be sent to ESCs.
 *                     RPMs are ordered in ascending order of ESC ID, e.g.,
 *                     [rpm_0, rpm_1, ..., rpm_n]
 * @param[in] size     Number of rpm_data array elements.
 * @param[in] fb_id    ID of the ESC from which feedback is desired.
 *                     If fb_id = -1, no feedback is requested.
 *
 * @detdesc
 * Sending this command does not guarantee that the ESCs spin the motors.
 * If the vehicle is not in flight, the flight controller forwards the
 * RPM commands to the ESCs. The ESC identified by fb_id requests feedback.
 * @par
 * @note1hang RPM commands must be sent at a rate between 100 Hz and 500 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_send_esc_rpm(int *rpm_data, unsigned int size, int fb_id);

/**
 * Sends PWM commands to the ESCs and requests feedback.
 *
 * @param[in] pwm_data Pointer to int array containing PWM data in the range
 *                     [-800, 800] to be sent to ESCs. PWMs are ordered in
 *                     ascending order of ESC ID, e.g. [pwm_0, pwm_1, ..., pwm_n].
 * @param[in] size     Number of pwm_data array elements.
 * @param[in] fb_id    ID of the ESC from which feedback is desired.
 *                     If fb_id = -1, no feedback is requested.
 *
 * @detdesc
 * The pwm_data array contains ESC PWMs in the range of [-800, 800] in which
 * 800 corresponds to 100% duty cycle and negative implies reversed direction.
 * @par
 * Sending this command does not guarantee that the ESCs spin the motors.
 * If the vehicle is not in flight, the flight controller forwards the
 * PWM commands to the ESCs. The ESC identified by fb_id requests feedback.
 * @par
 * @note1hang PWM commands must be sent at a rate between 100 Hz and 500 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_send_esc_pwm(int *pwm_data, unsigned int size, int fb_id);

/**
 * Sends an "RC-like" command to the flight controller.
 *
 * @datatypes
 * #SnRcCommandType \n
 * #SnRcCommandOptions
 *
 * @param[in] type    Specification of the desired input interpretation.
 * @param[in] options Options for interpreting the command.
 * @param[in] cmd0    Value in the range [-1.0, 1.0] specifying a
 *                    "forward/backward" type command in which "forward" is
 *                     positive.
 * @param[in] cmd1    Value in the range [-1.0, 1.0] specifying a "left/right"
 *                    type command in which "left" is positive.
 * @param[in] cmd2    Value in the range [-1.0, 1.0] specifying an "up/down"
 *                    type command in which "up" is positive.
 * @param[in] cmd3    Value in the range [-1.0, 1.0] specifying a "rotate"
 *                    type command in which rotating counter-clockwise is positive.
 *
 * @detdesc
 * This function sends four dimensionless control commands to the flight controller.
 * The interpretation of the four commands depends on the mode, which
 * can be obtained with the sn_get_mode() function. The desired meaning of the
 * four commands is specified with the type parameter.
 * @par
 * See Section @xref{sec:Understand_RCCI} for the meaning of the four
 * commands in different contexts.
 * @par
 * Suggested #SnRcCommandOptions option usage:
 * - RC_OPT_DEFAULT_RC for intuitive joystick control,
 *   including a small deadband to prevent drift and more intuitive mapping
 * - RC_OPT_LINEAR_MAPPING for absolute control of outputs -- Useful with the
 *   sn_apply_cmd_mapping() function
 * @par
 * @note1hang RC commands must be sent at a rate of at least 50 Hz.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 * @newpage
 */
int sn_send_rc_command(SnRcCommandType type, SnRcCommandOptions options,
    float cmd0, float cmd1, float cmd2, float cmd3);

/**
 * Converts dimensioned commands into dimensionless commands.
 *
 * @datatypes
 * #SnRcCommandType \n
 * #SnRcCommandOptions
 *
 * @param[in] type    Command type remapped to match the type used subsequently in
 *                    the sn_send_rc_command() function.
 *
 * @param[in] options Options to apply during mapping. See sn_send_rc_command().
 * @param[in] input0  Dimensioned command to be mapped into cmd0.
 * @param[in] input1  Dimensioned command to be mapped into cmd1.
 * @param[in] input2  Dimensioned command to be mapped into cmd2.
 * @param[in] input3  Dimensioned command to be mapped into cmd3.
 * @param[out] cmd0   Mapped unitless command 0 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd1   Mapped unitless command 1 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd2   Mapped unitless command 2 to be sent with the
 *                    sn_send_rc_command() function.
 * @param[out] cmd3   Mapped unitless command 3 to be sent with the
 *                    sn_send_rc_command() function.
 *
 * @detdesc
 * This function remaps commands with real units into the appropriate
 * dimensionless commands to be sent with sn_send_rc_command() function. Mapping
 * is based on on the type and options parameters.
 * See Section @xref{sec:Understand_RCCI} for more information.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 *
 * @sa
 * sn_send_rc_command()
 */
int sn_apply_cmd_mapping(SnRcCommandType type, SnRcCommandOptions options,
    float input0, float input1, float input2, float input3,
    float *cmd0, float *cmd1, float *cmd2, float *cmd3);

/**
 * Gets the the name of an RC command from the type.
 *
 * @param[in] type RC command type of interest.
 *
 * @return
 * Pointer to the command name.
 *
 * @dependencies
 * None.
 */
const char * sn_get_cmd_name(SnRcCommandType type);

/**
 * Gets the the dimensioned units of an RC command from the type and index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in range [0, 3] corresponding to cmd0 through cmd3.
 *
 * @return
 * Pointer to the units of a particular command.
 *
 * @dependencies
 * None.
 * @newpage
 */
const char * sn_get_dimensioned_units(SnRcCommandType type, int index);

/**
 * Gets the the minimum value in real units for an RC command from the type and
 * index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in range [0,3] corresponding to cmd0 through cmd3.
 *
 * @detdesc
 * This function returns the smallest possible command to be applied to the system. The
 * returned value maps to a dimensionless value of -1.0.
 * @par
 * Use sn_get_dimensioned_units() to get a string descripton of the units.
 *
 * @return
 * Minimum-allowed value in real units.
 *
 * @dependencies
 * None.
 */
float sn_get_min_value(SnRcCommandType type, int index);

/**
 * Gets the the maximum value in real units for an RC command from the type and
 * index.
 *
 * @datatypes
 * #SnRcCommandType
 *
 * @param[in] type  RC command type of interest.
 * @param[in] index Index in the range [0,3] corresponding to cmd0 through cmd3.
 *
 * @detdesc
 * This function returns the largest possible command to be applied to the system. This
 * value maps to a dimensionless value of 1.0.
 * @par
 * Use the sn_get_dimensioned_units() function to get a string descripton of the
 * units.
 *
 * @return
 * Maximum-allowed value in real units.
 *
 * @dependencies
 * None.
 */
float sn_get_max_value(SnRcCommandType type, int index);

/**
 * Sends thrust, attitude, and angular velocity.
 *
 * @param[in] thrust Commanded thrust in grams.
 * @param[in] qw     Scalar component of quaternion.
 * @param[in] qx     X component of vector part of the quaternion.
 * @param[in] qy     Y component of vector part of the quaternion.
 * @param[in] qz     Z component of vector part of the quaternion.
 * @param[in] wx     X component of angular velocity in rad/s.
 * @param[in] wy     Y component of angular velocity in rad/s.
 * @param[in] wz     Z component of angular velocity in rad/s.
 *
 * @detdesc
 * This function sends the desired thrust in grams, desired attitude represented
 * as a quaternion, and the desired angular velocity vector in rad/s to the flight
 * controller.
 * @par
 * The quaternion is in the following form:
 * @par
 * q = qw + qx*i + qy*j + qz*k
 * @par
 * @note1hang Be cautious -- This function is for advanced users.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None. @newpage
 */
int sn_send_thrust_att_ang_vel_command(float thrust, float qw, float qx,
    float qy, float qz, float wx, float wy, float wz);

/**
 * Sets the battery voltage.
 *
 * @param[in] voltage Battery voltage (V).
 *
 * @detdesc
 * This function overrides the internal battery voltage estimate. This function
 * must be called at a rate faster than 5 Hz for the value to be considered valid,
 * otherwise the flight controller defaults to the internal estimate of
 * battery voltage.
 *
 * @return
 * - 0 if command received
 * - -1 for failure (flight software non-functional)
 *
 * @dependencies
 * None.
 */
int sn_set_battery_voltage(float voltage);


/**
 * Get pointer to cached snav data structure.
 *
 * @param[in] size_cached_struct size of structure. This is used to ensure the header
 * file stays in sync. This argument should be sizeof(SnavCachedData)
 * @param[out] snav_cached_data_struct pointer to be filled with cached data structure
 * pointe. This structure will be udpated duing a call to sn_update_data().
 *
 * @return
 * - 0 if flight data ptr returned successfully
 * - -1 for failure to get pointer to flight data
 */
int sn_get_flight_data_ptr(int size_cached_struct, SnavCachedData **snav_cached_data_struct);


/** @} */ /* end_addtogroup sn_interface */

#ifdef __cplusplus
}
#endif

#endif //SN_INTERFACE_H_

