#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/* Set only after physical verification. Zero means unavailable, never a pose. */
#ifdef ROBOT_CALIBRATION_HEADER
#include ROBOT_CALIBRATION_HEADER
#endif
#ifndef DRIVE_CALIBRATED
#define DRIVE_CALIBRATED 0
#endif
#ifndef ENCODER_LEFT_SIGN
#define ENCODER_LEFT_SIGN 0
#endif
#ifndef ENCODER_RIGHT_SIGN
#define ENCODER_RIGHT_SIGN 0
#endif
#ifndef SERVO_WIRING_VERIFIED
#define SERVO_WIRING_VERIFIED 0
#endif
#ifndef MAX_DUTY
#define MAX_DUTY 25
#endif
#ifndef LINK_TIMEOUT_MS
#define LINK_TIMEOUT_MS 500u
#endif
#ifndef FEEDBACK_WIRING_VERIFIED
#define FEEDBACK_WIRING_VERIFIED 0
#endif
/* Per channel: lift P2.4, tilt P3.5, gripper P5.7.
 * min_us, max_us, safe_start_us, slew_us_per_20ms */
#ifndef SERVO_CALIBRATION
#define SERVO_CALIBRATION {{0,0,0,0},{0,0,0,0},{0,0,0,0}}
#endif
/* home, approach, grasp, carry, release. All must be measured on assembled arm. */
#ifndef ARM_POSES
#define ARM_POSES {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
#endif
#endif
