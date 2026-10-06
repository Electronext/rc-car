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

void reset_diag_start_console_task(void);
