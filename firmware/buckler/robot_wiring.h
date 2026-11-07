#pragma once
/*
 * HARDWARE SAFETY INTERLOCK.
 * Do not turn on until motor, encoder, and servo signals have been mapped,
 * checked with a multimeter, and verified with wheels off the floor.
 * The base TI-RSLK pin assignments for MSP432 do NOT apply to nRF52.
 */
#define ROBOT_WIRING_VERIFIED 0
#define ROBOT_ARM_CALIBRATED 0
#define ROBOT_ALLOW_BUTTON_DEMO 0

#if ROBOT_WIRING_VERIFIED
/* Fill these with verified nRF52 P0.xx GPIO values ONLY after continuity test.
 * DO NOT use placeholder values. nRF GPIO pins and servo powers are distinct.
 * Example of correct syntax once verified: NRF_GPIO_PIN_MAP(0, 9)
 */
#ifndef ROBOT_LEFT_DIR
#error "Define ROBOT_LEFT_DIR after hardware pin verification"
#endif
#ifndef ROBOT_LEFT_PWM
#error "Define ROBOT_LEFT_PWM after hardware pin verification"
#endif
#ifndef ROBOT_LEFT_SLEEP
#error "Define ROBOT_LEFT_SLEEP after hardware pin verification"
#endif
#ifndef ROBOT_RIGHT_DIR
#error "Define ROBOT_RIGHT_DIR after hardware pin verification"
#endif
#ifndef ROBOT_RIGHT_PWM
#error "Define ROBOT_RIGHT_PWM after hardware pin verification"
#endif
#ifndef ROBOT_RIGHT_SLEEP
#error "Define ROBOT_RIGHT_SLEEP after hardware pin verification"
#endif
#if ROBOT_ARM_CALIBRATED
#ifndef ROBOT_SERVO_LIFT
#error "Define ROBOT_SERVO_LIFT after hardware pin verification"
#endif
#ifndef ROBOT_SERVO_TILT
#error "Define ROBOT_SERVO_TILT after hardware pin verification"
#endif
#ifndef ROBOT_SERVO_GRIP
#error "Define ROBOT_SERVO_GRIP after hardware pin verification"
#endif
#endif /* ROBOT_ARM_CALIBRATED */
#endif /* ROBOT_WIRING_VERIFIED */

/* These are safe *software* limits, not calibrated mechanical servo limits. */
#define ROBOT_MAX_TEST_DUTY_PERCENT 15
#define ROBOT_SERVO_MIN_US 1000
#define ROBOT_SERVO_MAX_US 2000
