#pragma once

// Change to 0 for the car/receiver.
#define RC_TRANSMITTER          1

// ESP-NOW
#define RC_WIFI_CHANNEL         1
#define RC_MAGIC                0x52434331UL   // "RCC1"
#define RC_TX_PERIOD_MS         10             // 100 Hz
#define RC_FAILSAFE_MS          100

// Joystick
#define JOY_X_GPIO              0
#define JOY_Y_GPIO              1
#define SPEED_GPIO              3
#define BUTTON_GPIO             4

// Set to 0 if speed pot isn't fitted yet.
#define USE_SPEED_POT           0

// Joystick deadband as fraction of half travel.
#define JOYSTICK_DEADBAND       0.05f

// Speed pot:
// At minimum, full stick gives 20% motor demand.
// At maximum, full stick gives 100%.
#define SPEED_MIN               0.50f

// Receiver -> SA8302
#define MOTOR_L_FWD_GPIO        3   // INA
#define MOTOR_L_REV_GPIO        4   // INB
#define MOTOR_R_FWD_GPIO        5   // INC
#define MOTOR_R_REV_GPIO        6   // IND

// Motor PWM
#define MOTOR_PWM_FREQ_HZ       1000
#define MOTOR_PWM_BITS          10
#define MOTOR_PWM_MAX           ((1 << MOTOR_PWM_BITS) - 1)

// Set these after confirming physical direction.
#define LEFT_INVERT             0
#define RIGHT_INVERT            0