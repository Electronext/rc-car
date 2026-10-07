#pragma once

// Change to 0 for the car/receiver.
#define RC_TRANSMITTER          1

// ESP-NOW
#define RC_WIFI_CHANNEL         1
#define RC_MAGIC                0x52434331UL   // "RCC1"
#define RC_TX_PERIOD_MS         40             // 25 Hz
#define RC_FAILSAFE_MS          160            // four nominal control periods
#define RC_ACK_PERIOD_MS        100            // 10 Hz RX application heartbeat
#define RC_MAC_LINK_TIMEOUT_MS  250
#define RC_APP_LINK_TIMEOUT_MS  500

// Fixed STA MACs for the two ESP32-C3 boards.
#define RC_TX_MAC_INIT          {0x88, 0x56, 0xA6, 0x58, 0x57, 0xF8}
#define RC_RX_MAC_INIT          {0x48, 0xCA, 0x43, 0xD4, 0x13, 0xB4}

// Legacy analogue joystick pins.
// These remain until the AS5048B path replaces the analogue
// joystick handling after bring-up.
#define JOY_X_GPIO              0
#define JOY_Y_GPIO              1

// Role-specific controls / indicators.
#if RC_TRANSMITTER
#define BATTERY_GPIO            0
#define SPEED_GPIO              1   // speed pot now controls expo
#define STATUS_LED_GPIO         3
#define MODE_SWITCH_GPIO        4   // 3-way speed selector
#define CHARGE_STATUS_GPIO      5
#else
// RX:
//   GPIO3 VBAT: LiPo+ -> 100k -> GPIO3 -> 100k -> GND
//   GPIO1 VUSB: VUSB -> 100k -> GPIO1 -> 150k -> GND
//   GPIO4 !CHG: active-low charger open-drain status
#define BATTERY_GPIO            3
#define VUSB_PRESENT_GPIO       1
#define CHARGE_STATUS_GPIO      4
#define STATUS_LED_GPIO         8
#endif

// Charger status is active LOW (!CHG). On RX it is only meaningful
// while VUSB_PRESENT is HIGH.
#define CHARGE_STATUS_ACTIVE_LEVEL      0
#define VUSB_PRESENT_ACTIVE_LEVEL       1

// Battery monitor divider: LiPo+ -> 100k -> ADC -> 100k -> GND.
#define BATTERY_DIVIDER_TOP_OHMS       100000
#define BATTERY_DIVIDER_BOTTOM_OHMS    100000

// Expo pot wiring: 3.3V -> 330R -> 1k linear pot -> GND,
// with the wiper connected to SPEED_GPIO. The existing direction
// convention is retained: full CCW = maximum expo, full CW = zero expo.
#define SPEED_POT_SERIES_OHMS          330
#define SPEED_POT_OHMS                 1000

// Measured ADC endpoints from full mechanical travel.
#define SPEED_POT_ADC_MIN               4
#define SPEED_POT_ADC_MAX               3385
#define SPEED_POT_INVERT                1

// Maximum centre-softening at full CCW. Expo is a linear/cubic blend:
// y = (1-e)*x + e*x^3. 0 = linear, 1 = pure cubic.
#define CONTROL_EXPO_MAX               0.70f

// Three-way switch speed ceilings: LOW / CENTER / HIGH.
#define SPEED_LEVEL_LOW                0.30f
#define SPEED_LEVEL_MEDIUM             0.60f
#define SPEED_LEVEL_HIGH               1.00f

// AS5048B magnetic joystick sensors.
#define AS5048B_SDA_GPIO        6
#define AS5048B_SCL_GPIO        7
#define AS5048B_I2C_HZ          400000

// Sensor 1: A2=LOW,  A1=LOW  -> 0x40
// Sensor 2: A2=HIGH, A1=LOW  -> 0x42
#define AS5048B_ADDR_1          0x40
#define AS5048B_ADDR_2          0x42

// Steering calibration (AS5048B at 0x40).
// Full-output endpoints are set slightly inside the mechanical
// hard stops so normal steering reaches +/-1.0 without requiring
// pressure against the stops. The deadband covers linkage free
// play before the centring springs provide meaningful resistance.
#define STEERING_RAW_MIN              3000
#define STEERING_DEADBAND_LOW         4950
#define STEERING_CENTER_RAW           5232
#define STEERING_DEADBAND_HIGH        5500
#define STEERING_RAW_MAX              7800

// Set to 1 later if the physical steering direction is reversed.
#define STEERING_INVERT               1

// Throttle calibration (AS5048B at 0x42).
// Forward decreases the raw angle; reverse increases it.
// Full-output points are set slightly inside the measured hard stops.
#define THROTTLE_RAW_FORWARD          12765
#define THROTTLE_NEUTRAL_LOW          13829
#define THROTTLE_CENTER_RAW           13975
#define THROTTLE_NEUTRAL_HIGH         14121
#define THROTTLE_RAW_REVERSE          15165

// Preserve identical counts-per-unit sensitivity in both directions.
// Measured forward usable travel is ~1175 counts; reverse is ~1155,
// so reverse intentionally tops out slightly below 100%.
#define THROTTLE_COUNTS_PER_UNIT      1175

// Temporary bench mode: read the two AS5048Bs only.
// ESP-NOW transmitter control is not started while this is 1.
#define AS5048B_SANITY_TEST     0

// Expo pot is fitted.
#define USE_SPEED_POT           1

// Three-position ON-OFF-ON selector on GPIO4. Measured positions are
// approximately 3 / 2219 / 4095 raw for LOW / CENTER / HIGH.
// Thresholds intentionally leave very large guard bands.
#define MODE_SWITCH_ADC_LOW_MAX       1000
#define MODE_SWITCH_ADC_HIGH_MIN      3000

// Status / power policy. Low-battery handling must run before Wi-Fi.
#define BATTERY_LOW_CUTOFF_MV         3300
#define BATTERY_LOW_RECOVER_MV        3500
#define BATTERY_LED_RED_MV            3400
#define BATTERY_LED_YELLOW_MV         3700
#define BATTERY_LED_GREEN_MV          4200
#define LOW_BATTERY_WARNING_MS       10000
#define LOW_BATTERY_RECHECK_MS       60000

// Link indication: solid battery colour while RX heartbeat is current;
// blink 300 ms on / 700 ms off while disconnected.
// Overall link indication requires both recent TX->RX MAC delivery and
// a recent RX->TX application heartbeat.
#define RC_LINK_TIMEOUT_MS             RC_APP_LINK_TIMEOUT_MS
#define STATUS_LED_ON_MS               300
#define STATUS_LED_OFF_MS              700
#define STATUS_LED_BRIGHTNESS           48
#define STATUS_LED_SELF_TEST_MS         150
#define CHARGE_BREATHE_PERIOD_MS       2000
#define DISCONNECTED_SLEEP_MS        120000
#define RX_SLEEP_POLL_MS               5000
#define RX_POLL_LISTEN_MS                750

// RX reset diagnostics. NVS is written only for cold/unexpected boots;
// runtime minima are retained in RTC RAM without flash wear.
#define RX_RESET_LOG_CAPACITY              32
#define RX_RESET_RECENT_WINDOW_MS        1000
#define RX_DIAG_BATTERY_SAMPLE_MS          50
#define RX_DIAG_BATTERY_SAMPLE_COUNT        8

// Persist compact link-health summaries only when anomalies occurred.
// This avoids writing NVS on every packet or every brief RF glitch.
#define RX_LINK_LOG_CAPACITY               32
#define RX_LINK_DIAG_INTERVAL_MS          5000

// Inactivity is based on user-control state, not packet traffic.
// While asleep, wake briefly at this interval and sample controls
// without starting Wi-Fi; stay asleep if nothing moved.
#define TX_INACTIVITY_SLEEP_MS       300000
#define TX_SLEEP_POLL_MS               1000
#define TX_WAKE_STEERING_COUNTS          80
#define TX_WAKE_THROTTLE_COUNTS          80
#define TX_WAKE_SPEED_COUNTS              20

// ADC oneshot can transiently return ESP_ERR_TIMEOUT when the ADC
// hardware is busy. Retry rather than treating that as a fatal error.
#define ADC_READ_RETRY_COUNT                8

// Joystick deadband as fraction of half travel.
#define JOYSTICK_DEADBAND       0.05f

// Speed selection is handled by the three-way switch; the pot now
// controls expo only.

// Throttle-dependent skid steering.
// At zero throttle, full steering counter-rotates both motors at the
// speed-pot limit. At full throttle, full steering keeps the outer
// motor at full demand and reduces the inner motor by this fraction.
// 0.40 => inner motor runs at 60% of outer motor at full throttle.
#define TURN_INNER_REDUCTION_FULL_THROTTLE  0.40f

// Steering convention is latched forward/reverse while the throttle
// lever crosses its existing neutral deadband. Forward decreases the
// AS5048B raw angle; reverse increases it.
#define THROTTLE_STEER_FRAME_FORWARD_RAW    13960
#define THROTTLE_STEER_FRAME_REVERSE_RAW    13990

// Receiver -> SA8302
// Fit external pulldowns (recommended 4.7k) from all four SA8302
// logic inputs to GND so they stay deterministically LOW throughout
// ROM/bootloader startup before application firmware takes control.
#define MOTOR_L_FWD_GPIO       10   // INA
#define MOTOR_L_REV_GPIO        5   // INB
#define MOTOR_R_FWD_GPIO        0   // INC
#define MOTOR_R_REV_GPIO        7   // IND

// Motor PWM. Logical demand is remapped over the usable motor range:
// a stopped/reversing motor gets a short 25% start boost, then running
// demand is scaled over 20%..100%.
#define MOTOR_PWM_FREQ_HZ       1000
#define MOTOR_PWM_BITS          10
#define MOTOR_PWM_MAX           ((1 << MOTOR_PWM_BITS) - 1)
#define MOTOR_PWM_START_MIN     0.25f
#define MOTOR_PWM_RUN_MIN       0.20f
#define MOTOR_START_BOOST_MS    100

// Set these after confirming physical direction.
#define LEFT_INVERT             0
#define RIGHT_INVERT            0