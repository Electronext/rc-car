#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "nvs.h"

#include "rc_config.h"
#include "reset_diag.h"

#if !RC_TRANSMITTER

#define DIAG_RING_MAGIC      0x52444731UL  /* RDG1 */
#define DIAG_RING_VERSION    1
#define LINK_RING_MAGIC      0x4C444731UL  /* LDG1 */
#define LINK_RING_VERSION    1
#define DIAG_RTC_MAGIC       0x52544344UL  /* RTCD */

static const char *TAG = "RXDIAG";

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t reset_reason;
    uint32_t wake_cause;
    uint16_t boot_battery_mv;
    uint8_t charging;
    uint8_t prior_sleep_reason;
    uint8_t snapshot_valid;
    uint8_t prev_linked;
    uint8_t prev_charging;
    uint8_t reserved;
    uint16_t prev_min_battery_boot_mv;
    uint16_t prev_min_battery_recent_mv;
    uint32_t prev_uptime_ms;
    uint32_t prev_packet_count;
} reset_diag_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint16_t next;
    uint16_t reserved;
    uint32_t next_sequence;
    reset_diag_record_t records[RX_RESET_LOG_CAPACITY];
} reset_diag_ring_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t uptime_ms;
    uint16_t battery_mv;
    int8_t rssi_last;
    int8_t rssi_min;
    uint8_t rssi_valid;
    uint8_t reserved;
    uint32_t max_gap_ms;
    uint16_t last_sequence;
    uint16_t reserved2;
    uint32_t packet_count;
    uint32_t sequence_skips;
    uint32_t sequence_skips_delta;
    uint32_t failsafe_count;
    uint32_t failsafe_delta;
    uint32_t ack_mac_ok;
    uint32_t ack_mac_fail;
    uint32_t ack_mac_fail_delta;
    uint32_t ack_submit_err;
    uint32_t ack_submit_err_delta;
} link_diag_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint16_t next;
    uint16_t reserved;
    uint32_t next_sequence;
    link_diag_record_t records[RX_LINK_LOG_CAPACITY];
} link_diag_ring_t;

typedef struct {
    uint32_t magic;
    uint32_t uptime_ms;
    uint32_t packet_count;
    uint32_t recent_window_start_ms;
    uint16_t min_battery_boot_mv;
    uint16_t min_battery_recent_mv;
    uint8_t linked;
    uint8_t charging;
} reset_diag_rtc_snapshot_t;

RTC_NOINIT_ATTR static reset_diag_rtc_snapshot_t rtc_snapshot;

#define LINK_DIAG_QUEUE_DEPTH 4

static QueueHandle_t link_diag_queue = NULL;
static volatile uint32_t link_diag_queue_drops = 0;

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    default:                return "OTHER";
    }
}

static const char *wake_cause_name(esp_sleep_wakeup_cause_t cause)
{
    switch (cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "UNDEFINED";
    case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
    case ESP_SLEEP_WAKEUP_GPIO:      return "GPIO";
    default:                         return "OTHER";
    }
}

static void ring_init(reset_diag_ring_t *ring)
{
    memset(ring, 0, sizeof(*ring));
    ring->magic = DIAG_RING_MAGIC;
    ring->version = DIAG_RING_VERSION;
    ring->next_sequence = 1;
}

static esp_err_t ring_load(nvs_handle_t nvs,
                           reset_diag_ring_t *ring)
{
    size_t size = sizeof(*ring);
    esp_err_t err = nvs_get_blob(nvs, "ring", ring, &size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ring_init(ring);
        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }

    if (size != sizeof(*ring) ||
        ring->magic != DIAG_RING_MAGIC ||
        ring->version != DIAG_RING_VERSION ||
        ring->count > RX_RESET_LOG_CAPACITY ||
        ring->next >= RX_RESET_LOG_CAPACITY) {

        ESP_LOGW(TAG, "Reset log invalid; reinitialising");
        ring_init(ring);
    }

    return ESP_OK;
}

static esp_err_t ring_save(nvs_handle_t nvs,
                           const reset_diag_ring_t *ring)
{
    esp_err_t err = nvs_set_blob(
        nvs,
        "ring",
        ring,
        sizeof(*ring)
    );

    if (err != ESP_OK) {
        return err;
    }

    return nvs_commit(nvs);
}

static void link_ring_init(link_diag_ring_t *ring)
{
    memset(ring, 0, sizeof(*ring));
    ring->magic = LINK_RING_MAGIC;
    ring->version = LINK_RING_VERSION;
    ring->next_sequence = 1;
}

static esp_err_t link_ring_load(nvs_handle_t nvs,
                                link_diag_ring_t *ring)
{
    size_t size = sizeof(*ring);
    esp_err_t err = nvs_get_blob(nvs, "linkring", ring, &size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        link_ring_init(ring);
        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }

    if (size != sizeof(*ring) ||
        ring->magic != LINK_RING_MAGIC ||
        ring->version != LINK_RING_VERSION ||
        ring->count > RX_LINK_LOG_CAPACITY ||
        ring->next >= RX_LINK_LOG_CAPACITY) {

        ESP_LOGW(TAG, "Link log invalid; reinitialising");
        link_ring_init(ring);
    }

    return ESP_OK;
}

static esp_err_t link_ring_save(nvs_handle_t nvs,
                                const link_diag_ring_t *ring)
{
    esp_err_t err = nvs_set_blob(
        nvs,
        "linkring",
        ring,
        sizeof(*ring)
    );

    if (err != ESP_OK) {
        return err;
    }

    return nvs_commit(nvs);
}

static void print_link_record(const link_diag_record_t *r)
{
    printf(
        "L#%lu t=%lu ms VBAT=%.3f V "
        "rssi=%s%d min=%s%d maxGap=%lu ms seq=%u packets=%lu | "
        "skips=%lu (+%lu) failsafe=%lu (+%lu) | "
        "ackMAC=%lu/%lu (+%lu fail) submitErr=%lu (+%lu)\n",
        (unsigned long)r->sequence,
        (unsigned long)r->uptime_ms,
        (double)r->battery_mv / 1000.0,
        r->rssi_valid ? "" : "?",
        r->rssi_valid ? (int)r->rssi_last : 0,
        r->rssi_valid ? "" : "?",
        r->rssi_valid ? (int)r->rssi_min : 0,
        (unsigned long)r->max_gap_ms,
        (unsigned)r->last_sequence,
        (unsigned long)r->packet_count,
        (unsigned long)r->sequence_skips,
        (unsigned long)r->sequence_skips_delta,
        (unsigned long)r->failsafe_count,
        (unsigned long)r->failsafe_delta,
        (unsigned long)r->ack_mac_ok,
        (unsigned long)r->ack_mac_fail,
        (unsigned long)r->ack_mac_fail_delta,
        (unsigned long)r->ack_submit_err,
        (unsigned long)r->ack_submit_err_delta
    );
}

static void print_record(const reset_diag_record_t *r)
{
    printf(
        "#%lu reset=%s(%lu) wake=%s(%lu) bootVBAT=%.3f V"
        " charge=%u sleep=%u",
        (unsigned long)r->sequence,
        reset_reason_name((esp_reset_reason_t)r->reset_reason),
        (unsigned long)r->reset_reason,
        wake_cause_name((esp_sleep_wakeup_cause_t)r->wake_cause),
        (unsigned long)r->wake_cause,
        (double)r->boot_battery_mv / 1000.0,
        (unsigned)r->charging,
        (unsigned)r->prior_sleep_reason
    );

    if (r->snapshot_valid) {
        printf(
            " | prev uptime=%lu ms minVBAT_boot=%.3f V"
            " minVBAT_recent=%.3f V linked=%u charge=%u packets=%lu",
            (unsigned long)r->prev_uptime_ms,
            (double)r->prev_min_battery_boot_mv / 1000.0,
            (double)r->prev_min_battery_recent_mv / 1000.0,
            (unsigned)r->prev_linked,
            (unsigned)r->prev_charging,
            (unsigned long)r->prev_packet_count
        );
    } else {
        printf(" | prev=<not retained>");
    }

    printf("\n");
}

static void dump_ring(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("rxdiag", NVS_READONLY, &nvs);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("RXDIAG: no retained diagnostic records\n");
        return;
    }

    if (err != ESP_OK) {
        printf("RXDIAG: nvs_open failed: %s\n", esp_err_to_name(err));
        return;
    }

    reset_diag_ring_t reset_ring;
    err = ring_load(nvs, &reset_ring);

    if (err != ESP_OK) {
        printf("RXDIAG: reset ring load failed: %s\n", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    link_diag_ring_t link_ring;
    err = link_ring_load(nvs, &link_ring);
    nvs_close(nvs);

    if (err != ESP_OK) {
        printf("RXDIAG: link ring load failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf(
        "RXDIAG: %u retained reset record(s), capacity %u\n",
        (unsigned)reset_ring.count,
        (unsigned)RX_RESET_LOG_CAPACITY
    );

    uint16_t first =
        (uint16_t)((reset_ring.next + RX_RESET_LOG_CAPACITY -
                    reset_ring.count) %
                   RX_RESET_LOG_CAPACITY);

    for (uint16_t i = 0; i < reset_ring.count; i++) {
        uint16_t index =
            (uint16_t)((first + i) % RX_RESET_LOG_CAPACITY);
        print_record(&reset_ring.records[index]);
    }

    printf(
        "RXDIAG: %u retained link anomaly summary record(s), capacity %u, queueDrops=%lu\n",
        (unsigned)link_ring.count,
        (unsigned)RX_LINK_LOG_CAPACITY,
        (unsigned long)link_diag_queue_drops
    );

    first =
        (uint16_t)((link_ring.next + RX_LINK_LOG_CAPACITY -
                    link_ring.count) %
                   RX_LINK_LOG_CAPACITY);

    for (uint16_t i = 0; i < link_ring.count; i++) {
        uint16_t index =
            (uint16_t)((first + i) % RX_LINK_LOG_CAPACITY);
        print_link_record(&link_ring.records[index]);
    }
}


static void clear_ring(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("rxdiag", NVS_READWRITE, &nvs);

    if (err != ESP_OK) {
        printf("RXDIAG: nvs_open failed: %s\n", esp_err_to_name(err));
        return;
    }

    reset_diag_ring_t reset_ring;
    ring_init(&reset_ring);

    err = ring_save(nvs, &reset_ring);

    if (err == ESP_OK) {
        link_diag_ring_t link_ring;
        link_ring_init(&link_ring);
        err = link_ring_save(nvs, &link_ring);
    }

    nvs_close(nvs);

    if (err == ESP_OK) {
        printf("RXDIAG: retained reset and link records cleared\n");
    } else {
        printf("RXDIAG: clear failed: %s\n", esp_err_to_name(err));
    }
}


void reset_diag_record_boot(int boot_battery_mv,
                            bool charging,
                            uint8_t prior_sleep_reason)
{
    /*
     * Normal timer/GPIO wakeups from an intentional deep sleep are
     * expected operation, especially the RX 5-second poll cycle.
     * Do not burn NVS endurance recording those.
     */
    if (esp_reset_reason() == ESP_RST_DEEPSLEEP &&
        prior_sleep_reason != 0) {

        uint16_t initial_mv =
            (uint16_t)(boot_battery_mv < 0 ? 0 : boot_battery_mv);

        rtc_snapshot.magic = DIAG_RTC_MAGIC;
        rtc_snapshot.uptime_ms = 0;
        rtc_snapshot.packet_count = 0;
        rtc_snapshot.recent_window_start_ms = 0;
        rtc_snapshot.min_battery_boot_mv = initial_mv;
        rtc_snapshot.min_battery_recent_mv = initial_mv;
        rtc_snapshot.linked = 0;
        rtc_snapshot.charging = charging ? 1 : 0;
        return;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("rxdiag", NVS_READWRITE, &nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    reset_diag_ring_t ring;
    err = ring_load(nvs, &ring);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ring load failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    reset_diag_record_t r = {
        .sequence = ring.next_sequence++,
        .reset_reason = (uint32_t)esp_reset_reason(),
        .wake_cause = (uint32_t)esp_sleep_get_wakeup_cause(),
        .boot_battery_mv =
            (uint16_t)(boot_battery_mv < 0 ? 0 : boot_battery_mv),
        .charging = charging ? 1 : 0,
        .prior_sleep_reason = prior_sleep_reason
    };

    if (rtc_snapshot.magic == DIAG_RTC_MAGIC) {
        r.snapshot_valid = 1;
        r.prev_linked = rtc_snapshot.linked;
        r.prev_charging = rtc_snapshot.charging;
        r.prev_min_battery_boot_mv =
            rtc_snapshot.min_battery_boot_mv;
        r.prev_min_battery_recent_mv =
            rtc_snapshot.min_battery_recent_mv;
        r.prev_uptime_ms = rtc_snapshot.uptime_ms;
        r.prev_packet_count = rtc_snapshot.packet_count;
    }

    ring.records[ring.next] = r;
    ring.next =
        (uint16_t)((ring.next + 1U) % RX_RESET_LOG_CAPACITY);

    if (ring.count < RX_RESET_LOG_CAPACITY) {
        ring.count++;
    }

    err = ring_save(nvs, &ring);
    nvs_close(nvs);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ring save failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGW(TAG, "Recorded RX boot/reset:");
    print_record(&r);

    uint16_t initial_mv =
        (uint16_t)(boot_battery_mv < 0 ? 0 : boot_battery_mv);

    rtc_snapshot.magic = DIAG_RTC_MAGIC;
    rtc_snapshot.uptime_ms = 0;
    rtc_snapshot.packet_count = 0;
    rtc_snapshot.recent_window_start_ms = 0;
    rtc_snapshot.min_battery_boot_mv = initial_mv;
    rtc_snapshot.min_battery_recent_mv = initial_mv;
    rtc_snapshot.linked = 0;
    rtc_snapshot.charging = charging ? 1 : 0;
}

void reset_diag_runtime_sample(int battery_mv,
                               bool linked,
                               bool charging,
                               uint32_t packet_count)
{
    uint32_t now_ms =
        (uint32_t)(esp_timer_get_time() / 1000LL);

    if (rtc_snapshot.magic != DIAG_RTC_MAGIC) {
        memset(&rtc_snapshot, 0, sizeof(rtc_snapshot));
        rtc_snapshot.magic = DIAG_RTC_MAGIC;
        rtc_snapshot.recent_window_start_ms = now_ms;
    }

    uint16_t mv =
        (uint16_t)(battery_mv < 0 ? 0 : battery_mv);

    if (mv > 0 &&
        (rtc_snapshot.min_battery_boot_mv == 0 ||
         mv < rtc_snapshot.min_battery_boot_mv)) {

        rtc_snapshot.min_battery_boot_mv = mv;
    }

    if ((now_ms - rtc_snapshot.recent_window_start_ms) >=
        RX_RESET_RECENT_WINDOW_MS) {

        rtc_snapshot.recent_window_start_ms = now_ms;
        rtc_snapshot.min_battery_recent_mv = mv;
    } else if (mv > 0 &&
               (rtc_snapshot.min_battery_recent_mv == 0 ||
                mv < rtc_snapshot.min_battery_recent_mv)) {

        rtc_snapshot.min_battery_recent_mv = mv;
    }

    rtc_snapshot.uptime_ms = now_ms;
    rtc_snapshot.packet_count = packet_count;
    rtc_snapshot.linked = linked ? 1 : 0;
    rtc_snapshot.charging = charging ? 1 : 0;
}

static void link_diag_writer_task(void *arg)
{
    (void)arg;

    link_diag_record_t pending;

    link_diag_ring_t *ring =
        (link_diag_ring_t *)malloc(sizeof(*ring));

    if (ring == NULL) {
        ESP_LOGE(TAG, "Could not allocate retained link log buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(
                link_diag_queue,
                &pending,
                portMAX_DELAY) != pdTRUE) {

            continue;
        }

        nvs_handle_t nvs;
        esp_err_t err =
            nvs_open("rxdiag", NVS_READWRITE, &nvs);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "link nvs_open failed: %s",
                esp_err_to_name(err)
            );
            continue;
        }

        err = link_ring_load(nvs, ring);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "link ring load failed: %s",
                esp_err_to_name(err)
            );
            nvs_close(nvs);
            continue;
        }

        pending.sequence = ring->next_sequence++;

        ring->records[ring->next] = pending;
        ring->next =
            (uint16_t)((ring->next + 1U) % RX_LINK_LOG_CAPACITY);

        if (ring->count < RX_LINK_LOG_CAPACITY) {
            ring->count++;
        }

        err = link_ring_save(nvs, ring);
        nvs_close(nvs);

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "link ring save failed: %s",
                esp_err_to_name(err)
            );
            continue;
        }

        ESP_LOGW(TAG, "Retained RX link anomaly summary:");
        print_link_record(&pending);
    }
}


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
                                    uint32_t ack_submit_err_delta)
{
    if (link_diag_queue == NULL) {
        link_diag_queue_drops++;
        return;
    }

    link_diag_record_t r = {
        .sequence = 0,
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
        .battery_mv = (uint16_t)(battery_mv < 0 ? 0 : battery_mv),
        .rssi_last = (int8_t)rssi_last,
        .rssi_min = (int8_t)rssi_min,
        .rssi_valid = rssi_valid ? 1 : 0,
        .max_gap_ms = max_gap_ms,
        .last_sequence = last_sequence,
        .packet_count = packet_count,
        .sequence_skips = sequence_skips,
        .sequence_skips_delta = sequence_skips_delta,
        .failsafe_count = failsafe_count,
        .failsafe_delta = failsafe_delta,
        .ack_mac_ok = ack_mac_ok,
        .ack_mac_fail = ack_mac_fail,
        .ack_mac_fail_delta = ack_mac_fail_delta,
        .ack_submit_err = ack_submit_err,
        .ack_submit_err_delta = ack_submit_err_delta
    };

    if (xQueueSend(link_diag_queue, &r, 0) != pdTRUE) {
        link_diag_queue_drops++;
    }
}


static void console_task(void *arg)
{
    (void)arg;

    printf(
        "RXDIAG serial commands: d=dump, c=clear, x=dump+clear, ?=help\n"
    );

    while (1) {
        int ch = getchar();

        if (ch == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /*
         * Process commands one byte at a time so ESP-IDF monitor's
         * "send message" action does not need to append Enter/newline.
         * Ignore CR/LF in case a terminal does send them.
         */
        switch (ch) {
        case 'd':
        case 'D':
            dump_ring();
            break;

        case 'c':
        case 'C':
            clear_ring();
            break;

        case 'x':
        case 'X':
            dump_ring();
            clear_ring();
            break;

        case '?':
        case 'h':
        case 'H':
            printf(
                "RXDIAG: d=dump, c=clear, x=dump+clear, ?=help\n"
            );
            break;

        case '\r':
        case '\n':
            break;

        default:
            printf(
                "RXDIAG: unknown command '%c'; enter ? for help\n",
                (char)ch
            );
            break;
        }
    }
}

void reset_diag_start_console_task(void)
{
    if (link_diag_queue == NULL) {
        link_diag_queue =
            xQueueCreate(
                LINK_DIAG_QUEUE_DEPTH,
                sizeof(link_diag_record_t)
            );

        if (link_diag_queue == NULL) {
            ESP_LOGE(TAG, "Could not create link diagnostic queue");
        } else {
            BaseType_t created =
                xTaskCreate(
                    link_diag_writer_task,
                    "rxdiag_writer",
                    6144,
                    NULL,
                    2,
                    NULL
                );

            if (created != pdPASS) {
                ESP_LOGE(TAG, "Could not create link diagnostic writer");
                vQueueDelete(link_diag_queue);
                link_diag_queue = NULL;
            }
        }
    }

    xTaskCreate(
        console_task,
        "rxdiag_console",
        3072,
        NULL,
        2,
        NULL
    );
}

#else

void reset_diag_record_boot(int boot_battery_mv,
                            bool charging,
                            uint8_t prior_sleep_reason)
{
    (void)boot_battery_mv;
    (void)charging;
    (void)prior_sleep_reason;
}

void reset_diag_runtime_sample(int battery_mv,
                               bool linked,
                               bool charging,
                               uint32_t packet_count)
{
    (void)battery_mv;
    (void)linked;
    (void)charging;
    (void)packet_count;
}

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
                                    uint32_t ack_submit_err_delta)
{
    (void)battery_mv;
    (void)rssi_last;
    (void)rssi_min;
    (void)rssi_valid;
    (void)max_gap_ms;
    (void)last_sequence;
    (void)packet_count;
    (void)sequence_skips;
    (void)sequence_skips_delta;
    (void)failsafe_count;
    (void)failsafe_delta;
    (void)ack_mac_ok;
    (void)ack_mac_fail;
    (void)ack_mac_fail_delta;
    (void)ack_submit_err;
    (void)ack_submit_err_delta;
}

void reset_diag_start_console_task(void)
{
}

#endif
