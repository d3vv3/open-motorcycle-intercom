/**
 * @file mesh_protocol_rx.c
 * @brief ESB receive handoff: ISR ring publication and thread-context dispatch
 */

#include "mesh_protocol_internal.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "esb_radio.h"

LOG_MODULE_DECLARE(mesh);

#define C mesh_protocol_context_get()
#define s_rx_ring C->rx_ring
#define s_rx_ring_head C->rx_ring_head
#define s_rx_ring_tail C->rx_ring_tail
#define s_coordinator_id C->coordinator_id
#define s_stat_rx_drop C->stat_rx_drop
#define s_stat_rx_count C->stat_rx_count
#define s_stat_rf_rx_malformed C->stat_rf_rx_malformed
#define s_stat_rf_rx_version_drop C->stat_rf_rx_version_drop

/* NOTE: The ISR only publishes into this spinlock-protected ring; all other
 * protocol state stays owned by the system workqueue. */
static struct k_spinlock s_rx_ring_lock;
static struct k_work s_rx_work;

/* Thread-context packet processing: validate structure, then dispatch by family. */
static void process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi,
                              int64_t timestamp_us)
{
    if (len < sizeof(mesh_header_t)) {
        s_stat_rf_rx_malformed++;
        return;
    }

    const mesh_header_t *hdr = (const mesh_header_t *)data;
    const uint8_t *payload = data + sizeof(mesh_header_t);

    if (hdr->payload_len != (uint16_t)(len - sizeof(mesh_header_t))) {
        s_stat_rf_rx_malformed++;
        return;
    }
    if (hdr->payload_len > MESH_PACKET_PAYLOAD_MAX) {
        s_stat_rf_rx_malformed++;
        return;
    }

    if (hdr->version != MESH_PROTOCOL_VERSION) {
        s_stat_rf_rx_version_drop++;
        return;
    }

    LOG_DBG("RX type=0x%02X src=%d seq=%d (RSSI=%d)", hdr->type, hdr->src_id, hdr->seq, rssi);

    s_stat_rx_count++;

    if ((hdr->type == MESH_PKT_AUDIO || hdr->type == MESH_PKT_AUDIO_V2) &&
        mesh_protocol_audio_process_rx_packet(data, len, rssi)) {
        return;
    }

    if (mesh_protocol_membership_process_rx_packet(hdr, payload, rssi, timestamp_us)) {
        return;
    }

    switch (hdr->type) {
    case MESH_PKT_SPEAKER_GRANT:
        if (hdr->src_id == s_coordinator_id &&
            hdr->payload_len == sizeof(mesh_speaker_grant_payload_t)) {
            mesh_protocol_audio_apply_speaker_grant((const mesh_speaker_grant_payload_t *)payload);
        }
        break;

    case MESH_PKT_SPEAKER_RELEASE:
        if (hdr->src_id == s_coordinator_id &&
            hdr->payload_len == sizeof(mesh_speaker_release_payload_t)) {
            mesh_protocol_audio_apply_speaker_release(
                (const mesh_speaker_release_payload_t *)payload);
        }
        break;

    default:
        break;
    }
}

static void rx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    while (true) {
        struct rx_ring_entry entry;
        k_spinlock_key_t key = k_spin_lock(&s_rx_ring_lock);
        if (s_rx_ring_tail == s_rx_ring_head) {
            k_spin_unlock(&s_rx_ring_lock, key);
            break;
        }
        entry = s_rx_ring[s_rx_ring_tail];
        s_rx_ring_tail = (s_rx_ring_tail + 1) % RX_RING_SIZE;
        k_spin_unlock(&s_rx_ring_lock, key);
        process_rx_packet(entry.data, entry.len, entry.rssi, entry.timestamp_us);
    }
}

static void esb_rx_callback(const uint8_t *data, uint8_t len, const uint8_t *src_addr, int8_t rssi)
{
    ARG_UNUSED(src_addr);

    /* NOTE: Don't printk here - ISR context, can deadlock with USB/UART */

    k_spinlock_key_t key = k_spin_lock(&s_rx_ring_lock);
    uint8_t next_head = (s_rx_ring_head + 1) % RX_RING_SIZE;
    if (next_head == s_rx_ring_tail) {
        atomic_inc(&s_stat_rx_drop);
        k_spin_unlock(&s_rx_ring_lock, key);
        return;
    }

    struct rx_ring_entry *entry = &s_rx_ring[s_rx_ring_head];
    memcpy(entry->data, data, len);
    entry->len = len;
    entry->rssi = rssi;
    entry->timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
    s_rx_ring_head = next_head;
    k_spin_unlock(&s_rx_ring_lock, key);

    /* Submit work to process in thread context */
    k_work_submit(&s_rx_work);
}

void mesh_protocol_rx_init(void)
{
    k_work_init(&s_rx_work, rx_work_handler);
    esb_radio_set_rx_callback(esb_rx_callback);
}

void mesh_protocol_rx_stop(void)
{
    k_work_cancel(&s_rx_work);
    k_spinlock_key_t key = k_spin_lock(&s_rx_ring_lock);
    s_rx_ring_head = 0;
    s_rx_ring_tail = 0;
    k_spin_unlock(&s_rx_ring_lock, key);
}
