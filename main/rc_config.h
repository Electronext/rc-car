#pragma once

// Change to 0 for the car/receiver.
#define RC_TRANSMITTER          1

// ESP-NOW
#define RC_WIFI_CHANNEL         1
#define RC_MAGIC                0x52434331UL   // "RCC1"
#define RC_TX_PERIOD_MS         20             // 50 Hz
#define RC_FAILSAFE_MS          100

// Legacy analogue joystick pins.
// These remain until the AS5048B path replaces the analogue
// joystick handling after bring-up.
#define JOY_X_GPIO              0
#define JOY_Y_GPIO              1

// Final transmitter analogue / control allocation.
#define BATTERY_GPIO            0
#define SPEED_GPIO              1
#define BUTTON_GPIO             4

// Battery monitor divider: LiPo+ -> 100k -> ADC -> 100k -> GND.
#define BATTERY_DIVIDER_TOP_OHMS       100000
#define BATTERY_DIVIDER_BOTTOM_OHMS    100000

// Speed pot wiring: 3.3V -> 3.3k -> 10k linear pot -> GND,
// with the wiper connected to SPEED_GPIO.
#define SPEED_POT_SERIES_OHMS          3300
#define SPEED_POT_OHMS                 10000

// AS5048B magnetic joystick sensors.
#define AS5048B_SDA_GPIO        6
#define AS5048B_SCL_GPIO        7
#define AS5048B_I2C_HZ          400000

// Sensor 1: A2=LOW,  A1=LOW  -> 0x40
// Sensor 2: A2=HIGH, A1=LOW  -> 0x42
#define AS5048B_ADDR_1          0x40
#define AS5048B_ADDR_2          0x42

// Temporary bench mode: read the two AS5048Bs only.
// ESP-NOW transmitter control is not started while this is 1.
#define AS5048B_SANITY_TEST     1

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