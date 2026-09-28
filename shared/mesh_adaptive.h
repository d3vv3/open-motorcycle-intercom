#ifndef OMI_MESH_ADAPTIVE_H
#define OMI_MESH_ADAPTIVE_H

#include <stdbool.h>
#include <stdint.h>
#include "mesh_protocol_defs.h"

#define MESH_ADAPTIVE_REPORT_MS 1000U
#define MESH_ADAPTIVE_EXPIRE_MS 3000U
#define MESH_ADAPTIVE_HOLD_MS 10000U
#define MESH_ADAPTIVE_BETTER_MS 3000U
#define MESH_ADAPTIVE_SWITCH_LEAD_FRAMES 160U /* 3200 ms */
#define MESH_ADAPTIVE_PREPARE_MIN_FRAMES 20U
#define MESH_ADAPTIVE_ACK_DEADLINE_MS 2400U
#define MESH_ADAPTIVE_RECOVERY_GRACE_MS 2000U
#define MESH_ADAPTIVE_SYNC_MAX_AGE_MS 600U
#define MESH_ADAPTIVE_FORWARD_CACHE 32U
#define MESH_ADAPTIVE_FORWARD_QUEUE 8U
#define MESH_ADAPTIVE_REQUEST_MAX_AGE_FRAMES 50U
#define MESH_ADAPTIVE_REQUEST_ACTIVE_MS 1500U

typedef struct {
    bool present;
    bool report_seen;
    bool seq_seen; /* retained across report expiry, reset only on membership/term change */
    mesh_wire_identity_t identity;
    uint32_t last_seen_ms;
    uint32_t report_ms;
    uint32_t report_seq;
    uint8_t loss_quality; /* diagnostic delivery of this sender's reports, not an RF edge */
    uint8_t direct_mask;
    int8_t rssi[MESH_MAX_NODES];
    uint8_t quality[MESH_MAX_NODES];
    uint8_t leader_id;
    uint32_t term;
} mesh_adaptive_member_t;

typedef struct {
    uint8_t type, src_id, seq;
    uint32_t term, seen_ms;
    mesh_wire_identity_t origin;
    bool used;
} mesh_adaptive_forward_key_t;

typedef struct {
    mesh_header_t header;
    uint8_t payload[MESH_CONTROL_MAX_PACKET_SIZE - sizeof(mesh_header_t)];
    uint16_t payload_len;
    uint32_t enqueued_ms;
} mesh_adaptive_forward_packet_t;

typedef struct {
    uint8_t local_id, leader_id, members;
    uint32_t term, leader_since_ms, better_since_ms;
    uint8_t better_id;
    mesh_adaptive_member_t nodes[MESH_MAX_NODES];
    uint8_t edge_live[MESH_MAX_NODES];
    mesh_handover_payload_t pending;
    uint8_t ack_mask;
    uint32_t deadline_ms;
    uint32_t recovery_deadline_ms;
    uint32_t membership_revision, publish_revision;
    bool membership_revision_seen;
    uint32_t request_seq[MESH_MAX_NODES];
    uint32_t request_deadline_ms[MESH_MAX_NODES];
    bool request_seq_seen[MESH_MAX_NODES], request_active[MESH_MAX_NODES];
    bool prepared, committed, transition_done;
    mesh_adaptive_forward_key_t cache[MESH_ADAPTIVE_FORWARD_CACHE];
    mesh_adaptive_forward_packet_t queue[MESH_ADAPTIVE_FORWARD_QUEUE];
    uint8_t queue_head, queue_count;
} mesh_adaptive_t;

/* now_ms is monotonic uint32 milliseconds; all elapsed comparisons are wrap-safe
 * for intervals <2^31 ms. Adapter binds IDs from JOIN_ACK/known neighbor or an
 * authoritative identity map; self-declared report identities cannot bind IDs.
 * These checks are membership binding, NOT cryptographic authentication. */
void mesh_adaptive_init(mesh_adaptive_t *s, uint8_t local_id, uint8_t leader_id,
                        uint32_t term, uint8_t members, uint32_t now_ms);
bool mesh_adaptive_identity_valid(const mesh_wire_identity_t *identity);
/* For cold term-1 singleton discovery only (caller enforces no pending or
 * previous transition): local established clusters never destructively merge.
 * A singleton joins an established remote cluster regardless of address;
 * between singletons, the lexicographically lower identity stays leader.
 * Both identities must be valid, same address length, distinct; counts 1..8. */
bool mesh_adaptive_cold_join(const mesh_wire_identity_t *local_identity,
                             uint8_t local_count,
                             const mesh_wire_identity_t *remote_identity,
                             uint8_t remote_count);
/* Set/replace an ID binding (and add its bit), or NULL to remove it. Replacement
 * resets its report/sequence, adjacent edges, forwarding cache and pending
 * handover; removal of local ID or current leader is rejected. Mask updates
 * clear removed/newly added IDs; new IDs remain unbound until set_member. */
bool mesh_adaptive_set_member(mesh_adaptive_t *s, uint8_t id,
                              const mesh_wire_identity_t *identity);
bool mesh_adaptive_update_members(mesh_adaptive_t *s, uint8_t members);
/* Full authoritative snapshot. Invalid slot maps, absent/nonzero empty IDs,
 * duplicate identities, wrong term/leader/source and local/leader identity
 * replacement leave state byte-for-byte unchanged. Empty IDs must have seven
 * zero bytes. Within a term, revision must advance by 1..2^31-1 (mod 2^32);
 * duplicates and older maps are rejected even if payload is otherwise valid.
 * No new term is accepted here: handover reconciles that first.
 * Apply before reports from newly joined IDs. This is a logical ownership
 * check, not radio authentication. */
bool mesh_adaptive_membership_valid(const mesh_membership_v3_payload_t *snapshot,
                                    uint8_t *members);
bool mesh_adaptive_apply_membership(mesh_adaptive_t *s, uint8_t source,
                                     const mesh_membership_v3_payload_t *snapshot);
/* Pure: true only when a fully valid, newer snapshot from the current leader
 * omits this joined local ID and carries the leader's already-bound identity.
 * The adapter then leaves data TDMA and starts targeted rejoin; apply_membership
 * deliberately still rejects omission. This function never changes state. */
bool mesh_adaptive_membership_removes_local(const mesh_adaptive_t *s, uint8_t source,
                                            const mesh_membership_v3_payload_t *snapshot);
/* Call exactly once when leader queues each new snapshot; increments per-term
 * publish revision (gaps after queue loss are allowed). Reset on term change. */
uint32_t mesh_adaptive_next_membership_revision(mesh_adaptive_t *s);
/* Validate control ownership before applying any received state mutation.
 * JOIN_V3 is the only permitted unassigned source; topology and ACK belong to
 * joined members, while SYNC, map/grants and handover decisions belong to the
 * current leader. Relayed header src_id remains the original author. */
bool mesh_adaptive_control_authorized(const mesh_adaptive_t *s, uint8_t type,
                                      uint8_t source);
bool mesh_adaptive_report(mesh_adaptive_t *s, uint8_t source, const mesh_topology_payload_t *report,
                          bool relayed, uint32_t now_ms);
/* Pure identity/term/fields check independent of sequence ordering: adapters
 * may observe an original direct RX after its relayed duplicate updated graph
 * sequence. Invalid report data must never become a direct measurement. */
bool mesh_adaptive_report_valid(const mesh_adaptive_t *s, uint8_t source,
                                const mesh_topology_payload_t *report);
/* Accept pre-grant activity from a bound joined source. frame age is 0..50
 * frames, inclusive, with uint32 wrap; activity expires after 1500 ms unless
 * refreshed. Does not grant a speaker or affect audio forwarding. */
bool mesh_adaptive_speaker_request(mesh_adaptive_t *s, uint8_t source,
                                   const mesh_speaker_request_payload_t *request,
                                   uint32_t current_frame, uint32_t now_ms);
bool mesh_adaptive_speaker_request_active(const mesh_adaptive_t *s, uint8_t source,
                                          uint32_t now_ms);
/* Reporter fills its own direct row from ORIGINAL direct RF observations only.
 * Smoothed row RSSI and per-neighbor PDR belong to the adapter's direct-link
 * estimator; report-sequence loss_quality is diagnostic, never used as an edge
 * quality or assigned to remote links. Call at ~1 Hz including VOX silence. */
void mesh_adaptive_expire(mesh_adaptive_t *s, uint32_t now_ms);
uint8_t mesh_adaptive_direct_graph(mesh_adaptive_t *s, uint8_t node_id, uint32_t now_ms);
/* Returns minimum number of forwarding nodes (0..7), or -1 when a member
 * cannot be reached in <=2 RF hops. Ties use ascending relay ID. */
int mesh_adaptive_route(mesh_adaptive_t *s, uint8_t speaker_id, uint32_t now_ms,
                        uint8_t *relay_mask);
/* Only authoritative leader calls this, once per report or timer tick. */
uint8_t mesh_adaptive_candidate(mesh_adaptive_t *s, uint32_t now_ms);

/* PREPARE chooses switch_frame = current + 160 frames; receivers accept >=20
 * frames lead. The proposing leader requires full fresh universal candidate
 * reachability; the candidate verifies its full bidirectional graph. Other
 * followers need only fresh bidirectional direct reports for their own edge
 * to the bound candidate, not reports from non-neighbors. All-member ACKs
 * collectively confirm candidate coverage; a missing reverse edge declines.
 * Snapshot preserves every slot/grant; never renumber.
 * ACKs from every known member (including candidate) required for COMMIT.
 * ACK deadline is 2400 ms; COMMIT can be retried through switch+2000 ms.
 * Candidate alone transitions on tick after COMMIT at switch frame. Old leader
 * and other followers keep their old role until reconcile_sync validates new
 * candidate SYNC. If none arrives by recovery deadline, retain old leader.
 * Adapters must emit candidate SYNC repeatedly in its own normal control
 * windows as well as coordinator-reserved SYNC windows to avoid collision. */
bool mesh_adaptive_prepare(mesh_adaptive_t *s, uint8_t candidate, uint32_t frame,
                           const mesh_slot_map_payload_t *slots, uint32_t now_ms,
                           mesh_handover_payload_t *out);
bool mesh_adaptive_receive_prepare(mesh_adaptive_t *s, uint8_t sender,
                                   const mesh_handover_payload_t *p, uint32_t frame,
                                   uint32_t now_ms);
/* True for a timely matching ACK at any prepared joined node so a relay can
 * forward it. Only the current leader records sender in ack_mask; followers
 * cannot commit from ACKs they have observed. Reject before forwarding on
 * false. No cryptographic authentication is implied. */
bool mesh_adaptive_ack(mesh_adaptive_t *s, uint8_t sender,
                       const mesh_handover_ack_payload_t *ack, uint32_t now_ms);
bool mesh_adaptive_commit(mesh_adaptive_t *s, mesh_handover_payload_t *out, uint32_t now_ms);
bool mesh_adaptive_receive_commit(mesh_adaptive_t *s, uint8_t sender,
                                  const mesh_handover_payload_t *p, uint32_t now_ms);
bool mesh_adaptive_cancel(mesh_adaptive_t *s, uint8_t sender,
                          const mesh_handover_payload_t *p);
/* Call each frame; returns a role action only for the committed candidate.
 * For other nodes reconcile_sync returns the transition action instead. */
bool mesh_adaptive_tick(mesh_adaptive_t *s, uint32_t frame, uint32_t now_ms);
/* Higher-term authoritative SYNC can recover a missed COMMIT only if it
 * matches the pending proposal, including candidate identity and frame. */
bool mesh_adaptive_reconcile_sync(mesh_adaptive_t *s, uint8_t sender,
                                   const mesh_sync_v3_payload_t *sync,
                                   uint32_t now_ms);

/* Sender must be joined and synchronized; pass local clock frame/phase sampled
 * immediately at TX, including relay. No receive timestamp is reusable. */
bool mesh_adaptive_stamp_sync(mesh_sync_v3_payload_t *sync, bool relay,
                              uint32_t frame, uint16_t phase_us,
                              uint32_t upstream_age_ms);
/* Returns false without queue/cache mutation for JOIN, speaker requests,
 * topology reports and handover ACKs terminated at the local leader; reports
 * and requests originating at the leader anywhere; and JOIN_ACK_V3 targeted
 * to the local bound identity. Other whitelisted control retains TTL 2->1. */
bool mesh_adaptive_forward_enqueue(mesh_adaptive_t *s, const mesh_header_t *h,
                                   const uint8_t *payload, uint16_t len,
                                   const mesh_wire_identity_t *origin, uint32_t term,
                                   bool joined_synced, uint32_t now_ms);
/* Dequeue immediately before TX. SYNC is stamped from the caller's current
 * synchronized mesh clock; a stale upstream clock drops that queued packet. */
bool mesh_adaptive_forward_dequeue(mesh_adaptive_t *s, mesh_adaptive_forward_packet_t *out,
                                   uint32_t frame, uint16_t phase_us,
                                   uint32_t upstream_age_ms);

#endif
