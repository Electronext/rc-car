#pragma once

#include <stdbool.h>
#include <stdint.h>

void reset_diag_record_boot(int boot_battery_mv,
                            bool charging,
                            uint8_t prior_sleep_reason);

void reset_diag_runtime_sample(int battery_mv,
                               bool linked,
                               bool charging,
                               uint32_t packet_count);


void reset_diag_record_link_summary(int battery_mv,
                                    int rssi_last,
                                    int rssi_min,
                                    bool rssi_valid,
                                    uint32_t max_gap_ms,
                                    uint16_t last_sequence,
                                    uint32_t packet_count,
                                    uint32_t sequence_skips,
                                    uint32_t sequence_skips_delta,
                                    uint32_t failsafe_count,
                                    uint32_t failsafe_delta,
                                    uint32_t ack_mac_ok,
                                    uint32_t ack_mac_fail,
                                    uint32_t ack_mac_fail_delta,
                                    uint32_t ack_submit_err,
                                    uint32_t ack_submit_err_delta);

void reset_diag_start_console_task(void);
