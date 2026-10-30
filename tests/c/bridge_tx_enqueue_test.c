/* Native FreeRTOS shim; the included producer body is extracted from bridge_tx.c:
 *   mkdir -p /tmp/bridge_tx_enqueue && awk '/^static esp_err_t send_audio_packet\(/,/^}/' \
 *     components/uart_bridge/bridge_tx.c > /tmp/bridge_tx_enqueue/bridge_tx_enqueue.inc
 *   cc -std=c11 -Wall -Wextra -Werror -pedantic -I/tmp/bridge_tx_enqueue tests/c/bridge_tx_enqueue_test.c
 * The tx_mutex wait must be one tick and the producer must not log. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_SIZE, ESP_ERR_TIMEOUT };
enum { pdTRUE = 1, BRIDGE_SPI_MAX_PAYLOAD = 64, BRIDGE_SPI_MAX_XFER = 80,
       BRIDGE_TX_AUDIO_QUEUE_SIZE = 4 };
typedef struct { uint8_t buf[80]; int64_t queued_at_us; } bridge_tx_entry_t;
static struct {
    bool initialized;
    int audio_slots_free, audio_slots_used, tx_mutex;
    uint32_t audio_tx_enqueue_timeout, audio_tx_enqueue_full, audio_tx_enqueue_mutex;
    uint32_t audio_tx_queued;
    uint8_t audio_head, tx_seq;
    bridge_tx_entry_t audio_q[4];
} g_bridge;
static bool ready, fail_mutex, fail_encode;
static unsigned mutex_takes, free_takes, mutex_gives, free_gives, used_gives;
static int xSemaphoreTake(int semaphore, int ticks)
{
    if (semaphore == 1) { assert(ticks == 0); free_takes++; return g_bridge.audio_head != 3; }
    assert(semaphore == 2 && ticks == 1); mutex_takes++;
    return !fail_mutex;
}
static void xSemaphoreGive(int semaphore)
{
    if (semaphore == 1) free_gives++;
    else if (semaphore == 2) mutex_gives++;
    else { assert(semaphore == 3); used_gives++; }
}
static bool uart_bridge_is_mesh_ready(void) { return ready; }
static int64_t esp_timer_get_time(void) { return 123; }
/* No logging shim: any producer logging call must fail this host build. */
static size_t bridge_frame_encode(uint8_t *buf, size_t cap, size_t max,
                                 uint8_t seq, uint8_t type, const uint8_t *data, uint16_t len)
{
    assert(cap == 80 && max == 64 && seq == 0 && type == 7);
    if (len) memcpy(buf, data, len);
    return fail_encode ? 0 : len + 1;
}
#include "bridge_tx_enqueue.inc"
static void reset(void)
{
    memset(&g_bridge, 0, sizeof(g_bridge));
    g_bridge.initialized = ready = true; fail_mutex = fail_encode = false;
    g_bridge.audio_slots_free = 1; g_bridge.tx_mutex = 2; g_bridge.audio_slots_used = 3;
    mutex_takes = free_takes = mutex_gives = free_gives = used_gives = 0;
}
int main(void)
{
    uint8_t data[] = {17, 29};
    reset(); g_bridge.audio_head = 3;
    assert(send_audio_packet(7, data, 2) == ESP_ERR_TIMEOUT);
    assert(g_bridge.audio_tx_enqueue_full == 1 && g_bridge.audio_tx_enqueue_mutex == 0);
    assert(mutex_takes == 0 && free_takes == 1 && free_gives == 0);
    reset(); fail_mutex = true;
    assert(send_audio_packet(7, data, 2) == ESP_ERR_TIMEOUT);
    assert(g_bridge.audio_tx_enqueue_full == 0 && g_bridge.audio_tx_enqueue_mutex == 1);
    assert(mutex_takes == 1 && free_gives == 1 && mutex_gives == 0 && used_gives == 0);
    assert(g_bridge.audio_tx_enqueue_timeout ==
           g_bridge.audio_tx_enqueue_full + g_bridge.audio_tx_enqueue_mutex);
    reset();
    assert(send_audio_packet(7, data, 2) == ESP_OK);
    assert(g_bridge.audio_tx_queued == 1 && g_bridge.audio_head == 1 && g_bridge.tx_seq == 1);
    assert(g_bridge.audio_q[0].queued_at_us == 123 && g_bridge.audio_q[0].buf[0] == 17);
    assert(mutex_takes == 1 && mutex_gives == 1 && used_gives == 1 && free_gives == 0);
    reset(); fail_encode = true;
    assert(send_audio_packet(7, data, 2) == ESP_ERR_INVALID_SIZE);
    assert(free_gives == 1 && mutex_gives == 1 && used_gives == 0);
    reset();
    assert(send_audio_packet(7, NULL, 2) == ESP_ERR_INVALID_SIZE);
    assert(send_audio_packet(7, data, 65) == ESP_ERR_INVALID_SIZE && free_takes == 0);
    ready = false;
    assert(send_audio_packet(7, data, 2) == ESP_ERR_INVALID_STATE && free_takes == 0);
    return 0;
}
