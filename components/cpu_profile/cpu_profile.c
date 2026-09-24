#include "cpu_profile.h"

#ifdef CPU_PROFILE_ENABLED
#include <inttypes.h>
#include <stdint.h>

#include "cpu_profile_metrics.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#define PROFILE_CAPACITY 96
#define PROFILE_TOP 12
#define HINT_COUNT (sizeof(s_names) / sizeof(s_names[0]))

typedef struct {
    const char *lookup;
    const char *escaped;
} name_hint_t;

static const name_hint_t s_names[] = {
    {"IDLE0", "IDLE0"}, {"IDLE1", "IDLE1"}, {"main", "main"},
    {"audio_capture", "audio_capture"}, {"audio_playout", "audio_playout"},
    {"lc3_owner", "lc3_owner"}, {"mesh", "mesh"}, {"wifi", "wifi"},
    {"btController", "btController"}, {"BTC_TASK", "BTC_TASK"},
    {"BTU_TASK", "BTU_TASK"}, {"hciT", "hciT"}, {"sys_evt", "sys_evt"},
    {"tcpip", "tcpip"}, {"tiT", "tiT"}, {"phone_ctrl", "phone_ctrl"},
    {"button", "button"}, {"esp_timer", "esp_timer"},
    {"ipc0", "ipc0"}, {"ipc1", "ipc1"}, {"Tmr Svc", "Tmr%20Svc"},
};
static const char *const TAG = "cpu_profile";

typedef struct {
    uint64_t count;
    uint64_t wall_us_sum;
    uint64_t wall_us_max;
} span_stats_t;

static const char *const s_span_names[CPU_PROFILE_SPAN_COUNT] = {
    "cap_convert", "cap_hpf", "cap_cleanup", "play_remote",
    "play_voice_convert", "play_far_reference",
};
static portMUX_TYPE s_span_lock = portMUX_INITIALIZER_UNLOCKED;
static span_stats_t s_spans[CPU_PROFILE_SPAN_COUNT];

typedef struct {
    cpu_profile_sample_t sample;
    uint32_t priority;
    BaseType_t affinity;
    uint64_t delta_us;
    const char *hint;
    bool valid;
} profile_record_t;

static TaskStatus_t *s_status;
static bool s_init_attempted;
static profile_record_t *s_current;
static cpu_profile_sample_t *s_previous;
static size_t s_previous_count;
static uint64_t s_previous_total;
static bool s_baseline;
static uint32_t s_epoch;
static uint8_t s_mac[6];

int64_t cpu_profile_span_begin(void)
{
    return s_status ? esp_timer_get_time() : 0;
}

void cpu_profile_span_end(cpu_profile_span_t span, int64_t start_us)
{
    if ((unsigned)span >= CPU_PROFILE_SPAN_COUNT || start_us <= 0) return;
    int64_t end_us = esp_timer_get_time();
    if (end_us < start_us) return;
    uint64_t elapsed = (uint64_t)(end_us - start_us);
    portENTER_CRITICAL(&s_span_lock);
    span_stats_t *stats = &s_spans[span];
    if (stats->count < UINT64_MAX) stats->count++;
    stats->wall_us_sum = UINT64_MAX - stats->wall_us_sum < elapsed
                             ? UINT64_MAX : stats->wall_us_sum + elapsed;
    if (stats->wall_us_max < elapsed) stats->wall_us_max = elapsed;
    portEXIT_CRITICAL(&s_span_lock);
}

void cpu_profile_init(void)
{
    if (s_init_attempted) return;
    s_init_attempted = true;
    s_status = heap_caps_malloc(PROFILE_CAPACITY * sizeof(*s_status),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_current = heap_caps_malloc(PROFILE_CAPACITY * sizeof(*s_current),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_previous = heap_caps_malloc(PROFILE_CAPACITY * sizeof(*s_previous),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_status || !s_current || !s_previous) {
        ESP_LOGE(TAG, "PSRAM allocation failed (%u + %u + %u bytes); profiling disabled",
                 (unsigned)(PROFILE_CAPACITY * sizeof(*s_status)),
                 (unsigned)(PROFILE_CAPACITY * sizeof(*s_current)),
                 (unsigned)(PROFILE_CAPACITY * sizeof(*s_previous)));
        heap_caps_free(s_status);
        heap_caps_free(s_current);
        heap_caps_free(s_previous);
        s_status = NULL;
        s_current = NULL;
        s_previous = NULL;
        return;
    }
    s_epoch = esp_random();
    if (esp_read_mac(s_mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        ESP_LOGW(TAG, "WiFi STA MAC unavailable; node_mac will be zero");
    }
    ESP_LOGI(TAG, "CPU profile PSRAM buffers: status=%u records=%u previous=%u bytes",
             (unsigned)(PROFILE_CAPACITY * sizeof(*s_status)),
             (unsigned)(PROFILE_CAPACITY * sizeof(*s_current)),
             (unsigned)(PROFILE_CAPACITY * sizeof(*s_previous)));
}

void cpu_profile_log(void)
{
    if (!s_status || !s_current || !s_previous) return;
    const int64_t start = esp_timer_get_time();
    TaskHandle_t before[HINT_COUNT], after[HINT_COUNT];
    for (size_t i = 0; i < HINT_COUNT; i++) before[i] = xTaskGetHandle(s_names[i].lookup);
    TaskHandle_t idle_before[CONFIG_FREERTOS_NUMBER_OF_CORES];
    TaskHandle_t idle_after[CONFIG_FREERTOS_NUMBER_OF_CORES];
    for (int core = 0; core < CONFIG_FREERTOS_NUMBER_OF_CORES; core++) {
        idle_before[core] = xTaskGetIdleTaskHandleForCore(core);
    }
    uint64_t total = 0;
    const int64_t snap_start = esp_timer_get_time();
    UBaseType_t count = uxTaskGetSystemState(s_status, PROFILE_CAPACITY, &total);
    const uint64_t snapshot_us = (uint64_t)(esp_timer_get_time() - snap_start);
    for (size_t i = 0; i < HINT_COUNT; i++) after[i] = xTaskGetHandle(s_names[i].lookup);
    for (int core = 0; core < CONFIG_FREERTOS_NUMBER_OF_CORES; core++) {
        idle_after[core] = xTaskGetIdleTaskHandleForCore(core);
    }
    const bool snapshot_fail = count == 0;
    const UBaseType_t task_count_hint = snapshot_fail ? uxTaskGetNumberOfTasks() : 0;
    const uint64_t uptime_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const uint64_t collect_us = (uint64_t)(esp_timer_get_time() - start);
    uint64_t interval = 0, accounted = 0;
    uint64_t idle_us[CONFIG_FREERTOS_NUMBER_OF_CORES] = {0};
    bool idle_valid[CONFIG_FREERTOS_NUMBER_OF_CORES] = {false};
    unsigned matched = 0, added = 0, gone = 0, reset = 0;
    bool valid = !snapshot_fail && s_baseline && total > s_previous_total;
    if (valid) interval = total - s_previous_total;
    if (!interval) valid = false;

    if (!snapshot_fail) {
        for (UBaseType_t i = 0; i < count; i++) {
            const TaskStatus_t *status = &s_status[i];
            profile_record_t *r = &s_current[i];
            *r = (profile_record_t){
                .sample = {(uintptr_t)status->xHandle, (uint32_t)status->xTaskNumber,
                           (uint64_t)status->ulRunTimeCounter},
                .priority = (uint32_t)status->uxCurrentPriority,
                .affinity = status->xCoreID,
                .hint = "unknown",
            };
            /* pcTaskName points into a TCB which may be freed after the snapshot. */
            for (size_t h = 0; h < HINT_COUNT; h++) {
                if (before[h] && before[h] == after[h] && before[h] == status->xHandle) {
                    r->hint = s_names[h].escaped;
                    break;
                }
            }
            if (s_baseline) {
                size_t p = 0;
                for (; p < s_previous_count; p++) {
                    if (cpu_profile_same_task(&s_previous[p], &r->sample)) break;
                }
                if (p == s_previous_count) {
                    added++;
                } else if (valid && cpu_profile_delta(&s_previous[p], &r->sample,
                                                      interval, &r->delta_us)) {
                    r->valid = true;
                    matched++;
                    accounted += r->delta_us;
                    for (int core = 0; core < CONFIG_FREERTOS_NUMBER_OF_CORES; core++) {
                        if (idle_before[core] && idle_before[core] == idle_after[core] &&
                            idle_before[core] == status->xHandle) {
                            idle_us[core] += r->delta_us;
                            idle_valid[core] = true;
                        }
                    }
                } else if (r->sample.runtime_us < s_previous[p].runtime_us) {
                    reset++;
                }
            }
        }
        if (s_baseline) {
            for (size_t p = 0; p < s_previous_count; p++) {
                bool found = false;
                for (UBaseType_t i = 0; i < count; i++) {
                    if (cpu_profile_same_task(&s_previous[p], &s_current[i].sample)) {
                        found = true;
                        break;
                    }
                }
                if (!found) gone++;
            }
        }
    }

    ESP_LOGI(TAG, "PIPE v=1 dev=esp stage=cpu part=summary epoch_id=0x%08" PRIx32
             " node_mac=%02x:%02x:%02x:%02x:%02x:%02x uptime_ms=%" PRIu64
             " valid=%u total_us=%" PRIu64 " interval_us=%" PRIu64
             " task_count=%u task_count_hint=%u capacity=%u"
             " matched=%u new=%u gone=%u reset=%u coverage_permille=%" PRIu64
             " snapshot_fail=%u",
             s_epoch, s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
             uptime_ms, valid ? 1u : 0u, total, interval,
              (unsigned)count, (unsigned)task_count_hint, PROFILE_CAPACITY,
              matched, added, gone, reset,
              count ? (uint64_t)matched * 1000 / count : 0, snapshot_fail ? 1u : 0u);
    ESP_LOGI(TAG, "PIPE v=1 dev=esp stage=cpu part=idle epoch_id=0x%08" PRIx32
             " node_mac=%02x:%02x:%02x:%02x:%02x:%02x uptime_ms=%" PRIu64
             " valid=%u interval_us=%" PRIu64
             " accounted_permille=%" PRIu64 " chip_permille=%" PRIu64
             " idle0_valid=%u idle0_us=%" PRIu64 " idle0_permille=%" PRIu64
             " idle1_valid=%u idle1_us=%" PRIu64 " idle1_permille=%" PRIu64,
              s_epoch, s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
              uptime_ms, valid ? 1u : 0u, interval,
              cpu_profile_permille(accounted, interval, 1),
              cpu_profile_permille(accounted, interval, CONFIG_FREERTOS_NUMBER_OF_CORES),
              idle_valid[0] ? 1u : 0u, idle_us[0], cpu_profile_permille(idle_us[0], interval, 1),
              CONFIG_FREERTOS_NUMBER_OF_CORES > 1 && idle_valid[1] ? 1u : 0u,
              CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? idle_us[1] : 0,
              CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ?
                  cpu_profile_permille(idle_us[1], interval, 1) : 0);
    ESP_LOGI(TAG, "PIPE v=1 dev=esp stage=cpu part=overhead epoch_id=0x%08" PRIx32
             " node_mac=%02x:%02x:%02x:%02x:%02x:%02x uptime_ms=%" PRIu64
             " snapshot_us=%" PRIu64 " collect_us=%" PRIu64,
             s_epoch, s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
             uptime_ms, snapshot_us, collect_us);

    /* Top-N selection only rearranges compact records, outside the kernel snapshot. */
    if (valid) {
        unsigned limit = matched < PROFILE_TOP ? matched : PROFILE_TOP;
        for (unsigned rank = 0; rank < limit; rank++) {
            UBaseType_t best = count;
            for (UBaseType_t i = rank; i < count; i++) {
                if (s_current[i].valid &&
                    (best == count || s_current[i].delta_us > s_current[best].delta_us)) best = i;
            }
            if (best == count) break;
            profile_record_t tmp = s_current[rank];
            s_current[rank] = s_current[best];
            s_current[best] = tmp;
            const profile_record_t *r = &s_current[rank];
            ESP_LOGI(TAG, "PIPE v=1 dev=esp stage=cpu part=task epoch_id=0x%08" PRIx32
                     " node_mac=%02x:%02x:%02x:%02x:%02x:%02x uptime_ms=%" PRIu64
                     " task_id=0x%08" PRIx32 " task_handle=0x%" PRIxPTR
                     " name_hint=%s priority=%" PRIu32 " affinity=%ld runtime_us=%" PRIu64
                      " delta_us=%" PRIu64 " interval_us=%" PRIu64
                     " cpu_permille=%" PRIu64 " flags=valid",
                     s_epoch, s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
                     uptime_ms, r->sample.creation_id, r->sample.handle, r->hint,
                      r->priority, (long)r->affinity, r->sample.runtime_us, r->delta_us, interval,
                     cpu_profile_permille(r->delta_us, interval, 1));
        }
    }
    if (!snapshot_fail) {
        for (UBaseType_t i = 0; i < count; i++) {
            s_previous[i] = (cpu_profile_sample_t){
                (uintptr_t)s_status[i].xHandle, (uint32_t)s_status[i].xTaskNumber,
                (uint64_t)s_status[i].ulRunTimeCounter};
        }
        s_previous_count = count;
        s_previous_total = total;
        s_baseline = true;
    }
    for (unsigned i = 0; i < CPU_PROFILE_SPAN_COUNT; i++) {
        span_stats_t stats;
        portENTER_CRITICAL(&s_span_lock);
        stats = s_spans[i];
        portEXIT_CRITICAL(&s_span_lock);
        if (stats.count == 0) continue;
        ESP_LOGI(TAG, "PIPE v=1 dev=esp stage=cpu_span part=%s epoch_id=0x%08" PRIx32
                 " node_mac=%02x:%02x:%02x:%02x:%02x:%02x uptime_ms=%" PRIu64
                 " count=%" PRIu64 " wall_us_sum=%" PRIu64 " wall_us_max=%" PRIu64,
                 s_span_names[i], s_epoch, s_mac[0], s_mac[1], s_mac[2], s_mac[3],
                 s_mac[4], s_mac[5], uptime_ms, stats.count, stats.wall_us_sum,
                 stats.wall_us_max);
    }
}
#endif
