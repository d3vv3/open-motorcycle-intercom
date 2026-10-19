/* Real bridge_status.c against a fake clock: a status committed after the
 * reader sampled its clock is newer, not expired. Build from the repo root:
 * cc -std=c11 -Wall -Wextra -Werror -pedantic -pthread -D_GNU_SOURCE \
 *    -Itests/c/host_sdk -Icomponents/uart_bridge \
 *    -Icomponents/uart_bridge/include -Icomponents/board/include -Ishared -I. \
 *    tests/c/bridge_status_fresh_test.c components/uart_bridge/bridge_status.c */
#include <assert.h>
#include <stdatomic.h>
#include "bridge_internal.h"
#include "esp_timer.h"

bridge_context_t g_bridge;
portMUX_TYPE g_bridge_status_lock = portMUX_INITIALIZER_UNLOCKED;
static _Atomic int64_t clock_us = 1000;
static unsigned expired_discards, pending_discards;

int64_t esp_timer_get_time(void) { return atomic_load(&clock_us); }
void test_critical_enter(portMUX_TYPE *lock) { assert(pthread_mutex_lock(lock) == 0); }
void test_critical_exit(portMUX_TYPE *lock) { assert(pthread_mutex_unlock(lock) == 0); }
void test_sdk_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
void bridge_tx_discard_pending_audio(void) { pending_discards++; }
void bridge_tx_discard_for_expired_status(uint32_t generation) { (void)generation; expired_discards++; }

int main(void)
{
    bridge_status_payload_t payload = {
        .version = BRIDGE_PROTOCOL_VERSION, .marker = BRIDGE_STATUS_V2_MARKER, .mesh_state = BRIDGE_MESH_STATE_ACTIVE, .role = 1,
        .node_id = 1, .slot_index = 0, .coordinator_id = 1, .peer_count = 1,
        .audio_codec = MESH_AUDIO_CODEC_LC3, .audio_frame_ms = MESH_AUDIO_V2_FRAME_MS,
    };
    uart_bridge_status_t status;
    bridge_status_apply_v2(&payload);
    assert(uart_bridge_get_status(&status) == ESP_OK && status.generation == 1);

    /* Reader sampled now_us = 1000, a status landed at 1500 before it locked. */
    atomic_store(&clock_us, 1500);
    bridge_status_apply_v2(&payload);
    atomic_store(&clock_us, 1000);
    assert(uart_bridge_get_status(&status) == ESP_OK && status.generation == 2);
    assert(g_bridge.status_expiration_count == 0 && expired_discards == 0);
    assert(bridge_status_is_fresh_locked(1000));

    /* Genuine staleness still expires exactly once. */
    atomic_store(&clock_us, 1500 + BRIDGE_STATUS_STALE_TIMEOUT_US);
    assert(uart_bridge_get_status(&status) == ESP_OK);
    atomic_store(&clock_us, 1501 + BRIDGE_STATUS_STALE_TIMEOUT_US);
    assert(uart_bridge_get_status(&status) == ESP_ERR_NOT_FOUND);
    assert(uart_bridge_get_status(&status) == ESP_ERR_NOT_FOUND);
    assert(g_bridge.status_expiration_count == 1 && expired_discards == 1);
    assert(g_bridge.status_expired_generation == 2);
    return 0;
}
