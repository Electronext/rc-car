#pragma once
#include <stdint.h>

// Existing on-air packet layout; keep unchanged during component extraction.
enum {
    RC_MSG_CONTROL = 1,
    RC_MSG_HEARTBEAT = 2
};

enum {
    RC_CONTROL_FLAG_INVALID = 0x01U,
    RC_CONTROL_FLAG_PIVOT = 0x02U,
    RC_CONTROL_FLAG_STUPID = 0x04U
};

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t sequence;
    uint8_t type;
    int16_t left;       // -1000 ... +1000
    int16_t right;      // -1000 ... +1000
    uint16_t speed;     // 0 ... 1000
    uint8_t flags;
} rc_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t last_control_sequence;
    uint8_t type;
    uint16_t battery_mv;
    int8_t control_rssi;
    uint32_t failsafe_count;
    uint32_t sequence_skips;
    uint32_t max_control_gap_ms;
    int16_t left_pwm_permille;
    int16_t right_pwm_permille;
    uint8_t flags;      // bit 0 charging, bit 1 external power present
} rc_heartbeat_t;

