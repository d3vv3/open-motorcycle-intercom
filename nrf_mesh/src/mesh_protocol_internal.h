/**
 * @file mesh_protocol_internal.h
 * @brief Private mesh protocol state shared by future protocol modules
 */

#ifndef OMI_MESH_PROTOCOL_INTERNAL_H
#define OMI_MESH_PROTOCOL_INTERNAL_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "audio_bundle.h"
#include "mesh_core.h"
#include "mesh_adaptive.h"
#include "mesh_protocol.h"

#define SCAN_TIMEOUT_MS           3000
#define SCAN_BACKOFF_MAX_MS       500
#define JOIN_TIMEOUT_MS           5000
#define JOIN_RETRY_MS             500
#define JOIN_RETRY_COUNT          10
#define STATUS_INTERVAL_MS        1000
#define ACTIVE_SPEAKER_TIMEOUT_MS 1500
#define RELAY_RING_SIZE           16
#define CONTROL_RING_SIZE         32
#define TX_AUDIO_RING_SIZE        16
#define RX_RING_SIZE              8
#define NRF_SYNC_RX_LATENCY_US    300
#define MESH_PACKET_PAYLOAD_MAX   MESH_AUDIO_V2_MAX_BUNDLE_SIZE
#define MESH_PACKET_OUTER_MAX     MESH_AUDIO_V2_MAX_PACKET_SIZE
#define ESB_NORMAL_RAMP_US        129U
/* 2 Mbps on-air overhead: 2-byte preamble + 5-byte address + ~2-byte PCF + 2-byte CRC. */
#define ESB_2MBPS_OVERHEAD_BYTES 11U
#define ESB_2MBPS_US_PER_BYTE    4U
#define AUDIO_TX_MARGIN_US       100U
#define SYNC_TIMEOUT_MS          5000 /* 5 seconds timeout to allow for some packet loss */

struct audio_ingress_entry {
    uint8_t data[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    uint8_t len;
    uint8_t audio_flags;
    uint8_t packet_type;
};

struct tx_audio_entry {
    uint8_t data[MESH_AUDIO_V2_MAX_BUNDLE_SIZE];
    uint8_t len;
    uint8_t audio_flags;
    uint8_t packet_type;
};

struct rx_ring_entry {
    uint8_t data[256];
    uint8_t len;
    int8_t rssi;
    int64_t timestamp_us;
};

struct relay_entry {
    uint8_t data[MESH_PACKET_OUTER_MAX];
    uint8_t len;
    uint32_t queued_ms;
};

struct rf_seq16_tracker {
    uint16_t last;
    bool initialized;
};

enum local_tx_outcome {
    LOCAL_TX_SUCCESS,
    LOCAL_TX_FAILED,
    LOCAL_TX_FALLBACK_ORIGINAL,
};

/* Ordinary protocol state. Zephyr objects initialized by declaration macros
 * remain owned by mesh_protocol.c so their initialization and synchronization
 * semantics are unchanged. */
typedef struct {
    /* Lifecycle, state, and identity. */
    mesh_state_t state;
    mesh_role_t role;
    uint8_t node_id;
    int8_t slot_index;
    uint8_t coordinator_id;
    uint8_t talk_channel;
    bool participant_membership_known;
    uint8_t local_addr[5];
    uint8_t tx_seq;
    uint32_t last_sync_time;
    uint32_t term;
    mesh_wire_identity_t leader_identity;
    /* RX, status, audio and TDMA work callbacks share Zephyr's single system
     * workqueue. The radio ISR only publishes its protected RX ring. */
    mesh_adaptive_t adaptive;
    uint32_t direct_seq[MESH_MAX_NODES];
    bool direct_seq_valid[MESH_MAX_NODES];
    uint8_t direct_quality[MESH_MAX_NODES];
    int8_t direct_rssi[MESH_MAX_NODES];
    uint32_t direct_seen[MESH_MAX_NODES];
    uint32_t graph_report_seq;
    uint32_t air_report_seq;
    uint32_t local_request_seq;
    uint32_t last_local_active_ms;
    uint32_t last_request_ms;
    bool local_request_active;
    uint8_t control_turn;
    uint32_t last_prepare_tx;
    uint32_t last_commit_tx;
    uint32_t last_forward_sync;
    uint32_t last_sync_log;
    uint32_t convergence_until_ms;
    uint32_t grant_hold_until_ms;
    uint32_t last_membership_tx;
    uint8_t authoritative_members;
    int join_attempts;
    mesh_core_dedupe_t dedupe;
    mesh_core_dedupe_t control_presence_dedupe;
    uint8_t join_last_seq[MESH_MAX_NODES], ack_last_seq[MESH_MAX_NODES];
    uint8_t join_seq_seen, ack_seq_seen;

    /* Peers and speakers. */
    mesh_peer_info_t peers[MESH_MAX_NODES];
    uint8_t peer_count;
    uint8_t active_speaker_ids[MESH_MAX_ACTIVE_SPEAKERS];
    uint8_t relay_masks[MESH_MAX_ACTIVE_SPEAKERS];
    int64_t active_speaker_deadline_ms[MESH_MAX_NODES + 1];
    int64_t speaker_active_since_ms[MESH_MAX_NODES + 1];
    uint8_t heard_bitmap;
    uint8_t relay_bitmap;

    /* Packet and audio rings. */
    struct tx_audio_entry tx_audio_ring[TX_AUDIO_RING_SIZE];
    uint8_t tx_head;
    uint8_t tx_tail;
    bool relay_contention_turn;
    bool local_deferred_pending;
    uint16_t local_deferred_seq;
    struct rx_ring_entry rx_ring[RX_RING_SIZE];
    uint8_t rx_ring_head;
    uint8_t rx_ring_tail;
    struct relay_entry relay_ring[RELAY_RING_SIZE];
    uint8_t relay_head;
    uint8_t relay_tail;
    struct relay_entry control_ring[CONTROL_RING_SIZE];
    uint8_t control_head;
    uint8_t control_tail;
    struct relay_entry priority_control;
    bool priority_pending;

    /* Counters and telemetry. */
    uint32_t stat_ingress_purge_drop;
    uint32_t stat_tx_purge_drop;
    uint32_t stat_tx_count;
    uint32_t stat_tx_fail;
    uint32_t stat_tx_starvation;
    uint32_t stat_tx_queue_drain;
    uint32_t stat_rx_count;
    uint32_t stat_topology_tx_ok;
    uint32_t stat_topology_rx_direct;
    uint32_t stat_topology_rx_relayed;
    atomic_t stat_rx_drop;
    uint32_t stat_audio_fwd;
    atomic_t stat_tx_overwrite;
    uint32_t stat_spi_audio_in;
    uint32_t stat_ingress_inactive_drop;
    atomic_t stat_ingress_msgq_drop;
    uint32_t stat_tx_ring_drop;
    uint32_t stat_relay_ring_drop;
    uint32_t stat_control_ring_drop;
    uint32_t stat_rf_audio_try;
    uint32_t stat_rf_audio_ok;
    uint32_t stat_rf_audio_fail;
    uint32_t stat_rf_rx_audio_ok;
    uint32_t stat_rf_rx_malformed;
    uint32_t stat_rf_rx_version_drop;
    uint32_t stat_rf_rx_self_drop;
    uint32_t stat_rf_rx_duplicate_drop;
    uint32_t stat_rf_rx_inactive_drop;
    uint32_t stat_spi_out_ok;
    uint32_t stat_spi_out_drop;
    uint32_t stat_bundle_tx;
    uint32_t stat_bundle_rx;
    uint32_t stat_bundle_bad;
    uint32_t stat_prev1_forwarded;
    uint32_t stat_prev2_forwarded;
    uint32_t stat_prev1_stripped;
    uint32_t stat_prev2_stripped;
    uint32_t stat_bundle_late_drop;
    uint32_t stat_bundle_max_bytes;
    uint32_t stat_local_deferred_recovery;
    uint32_t last_audio_in_time;
    /* Ingress arrival phase: histogram of time-to-own-slot at SPI audio
     * arrival, 2 ms buckets over one 20 ms frame. Bucket 0 = arrived just
     * before our slot (sent immediately); bucket 9 = arrived just after
     * (waits a full frame). */
    uint32_t aphase_hist[10];
    uint8_t tx_queue_depth_dbg;
    uint32_t status_log_decim;
    mesh_core_seq16_t e2e_spi_in_src[256];
    struct rf_seq16_tracker e2e_rf_rx_src[256];
    uint32_t e2e_spi_in_frames;
    uint32_t e2e_spi_in_gap_evt;
    uint32_t e2e_spi_in_gap_fr;
    uint32_t e2e_spi_in_reset_evt;
    uint32_t e2e_rf_tx_frames;
    uint32_t e2e_rf_rx_frames;
    uint32_t e2e_rf_rx_gap_evt;
    uint32_t e2e_rf_rx_gap_fr;
    uint32_t e2e_rf_rx_reset_evt;
    uint32_t e2e_spi_out_frames;
    uint32_t skip_count;
    uint32_t auto_ticks;

    /* Ingress and command state. */
    bool audio_ingress_enabled;
    atomic_t local_audio_enabled; /* call-state gate, independent of mesh membership */
    int64_t local_audio_receive_cutoff_us; /* owner-only RF receipt boundary; boot defaults to 0 */
    atomic_t local_audio_epoch;   /* invalidates producers waiting at ingress mutex */
    atomic_t pending_request; /* command | generation << 8 | talk_channel << 16; zero = none */
    atomic_t status_pending;
} mesh_protocol_context_t;

/* Private module boundary; this is not included by the public protocol API. */
mesh_protocol_context_t *mesh_protocol_context_get(void);
void mesh_protocol_update_peer_last_seen(uint8_t node_id, int8_t rssi);
void mesh_protocol_note_peer_presence(uint8_t node_id);
mesh_wire_identity_t mesh_protocol_local_identity(void);
void mesh_protocol_adaptive_members(void);
bool mesh_protocol_adaptive_bind_member(uint8_t id, const mesh_wire_identity_t *identity);
bool mesh_protocol_adaptive_control_sequence_accept(uint8_t id, mesh_pkt_type_t type,
                                                     uint8_t seq);
int mesh_protocol_adaptive_publish_membership(void);
bool mesh_protocol_adaptive_converging(void);
void mesh_protocol_adaptive_observe(const mesh_header_t *hdr,
                                    const mesh_topology_payload_t *report, int8_t rssi);
void mesh_protocol_adaptive_report_tick(void);
void mesh_protocol_adaptive_frame_tick(uint32_t frame);
void mesh_protocol_adaptive_local_voice(bool active);
bool mesh_protocol_adaptive_control_rx(const mesh_header_t *hdr, const uint8_t *payload,
                                       int8_t rssi, int64_t timestamp_us);
void mesh_protocol_adaptive_reset(void);

void mesh_protocol_rx_init(void);
void mesh_protocol_rx_stop(void);

void mesh_protocol_membership_bind_work(struct k_work_delayable *scan_work,
                                        struct k_work_delayable *join_work,
                                        struct k_work_delayable *status_work);
uint32_t mesh_protocol_membership_scan_timeout_ms(void);
uint8_t mesh_protocol_membership_bridge_peer_count(void);
void mesh_protocol_membership_reset_session_data(void);
void mesh_protocol_membership_discover(uint8_t leader_id);
void mesh_protocol_membership_demote(uint8_t leader_id, uint32_t term,
                                      mesh_wire_identity_t identity);
void mesh_protocol_membership_rejoin(uint8_t leader_id, uint32_t term,
                                     mesh_wire_identity_t identity);
bool mesh_protocol_membership_process_rx_packet(const mesh_header_t *hdr, const uint8_t *payload,
                                                int8_t rssi, int64_t timestamp_us);
void mesh_protocol_membership_scan_work_handler(struct k_work *work);
void mesh_protocol_membership_join_work_handler(struct k_work *work);
void mesh_protocol_membership_check_peer_timeouts(void);
bool mesh_protocol_membership_handle_coordinator_timeout(void);

void mesh_protocol_audio_init(void);
void mesh_protocol_audio_cancel_work(void);
void mesh_protocol_audio_set_ingress_enabled(bool enabled, bool purge);
void mesh_protocol_audio_set_local_enabled(bool enabled);
void mesh_protocol_audio_purge_tx_ring(void);
void mesh_protocol_audio_reset_rf_e2e_tracker(uint8_t node_id);
void mesh_protocol_audio_reset_all_rf_e2e_trackers(void);
void mesh_protocol_audio_clear_relay_ring(void);
void mesh_protocol_audio_clear_speaker_activity(void);
void mesh_protocol_audio_clear_speaker_grants(void);
void mesh_protocol_audio_clear_heard_relay_bitmaps(void);
void mesh_protocol_audio_update_speaker_grants(void);
void mesh_protocol_audio_expire_speakers(uint32_t now_ms);
void mesh_protocol_audio_apply_slot_map_speakers(const mesh_slot_map_payload_t *slot_map);
void mesh_protocol_audio_apply_speaker_grant(const mesh_speaker_grant_payload_t *grant);
void mesh_protocol_audio_apply_speaker_release(const mesh_speaker_release_payload_t *release);
bool mesh_protocol_audio_process_rx_packet(const uint8_t *data, uint8_t len, int8_t rssi,
                                           int64_t received_us);

int mesh_protocol_tx_send_packet_ex(mesh_pkt_type_t type, const void *payload, uint16_t len,
                                    uint8_t ttl, uint8_t flags, uint8_t src_id, uint8_t seq);
int mesh_protocol_tx_send_packet(mesh_protocol_context_t *context, mesh_pkt_type_t type,
                                 const void *payload, uint16_t len);
int mesh_protocol_tx_queue_control(mesh_protocol_context_t *context, mesh_pkt_type_t type,
                                   const void *payload, uint16_t len);
int mesh_protocol_tx_send_join_request(mesh_protocol_context_t *context);
int mesh_protocol_tx_send_keepalive(mesh_protocol_context_t *context);
int mesh_protocol_tx_send_slot_map(mesh_protocol_context_t *context);
int mesh_protocol_tx_send_status_packet(mesh_protocol_context_t *context);
int mesh_protocol_tx_send_join_ack(mesh_protocol_context_t *context, uint8_t assigned_id,
                                    uint8_t slot_index, const uint8_t target_addr[5]);
mesh_slot_map_payload_t mesh_protocol_tx_slot_snapshot(mesh_protocol_context_t *context);
int mesh_protocol_tx_queue_priority(mesh_pkt_type_t type, const void *payload, uint16_t len);
void mesh_protocol_tx_control_handler(uint32_t frame_counter);

#endif /* OMI_MESH_PROTOCOL_INTERNAL_H */
