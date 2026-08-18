/**
 * @file mesh_protocol_runtime.c
 * @brief Mesh protocol lifecycle, bridge command coalescing, and status work
 */

#include "mesh_protocol.h"
#include "mesh_protocol_internal.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "esb_radio.h"
#include "tdma.h"
#include "uart_bridge.h"
#include "ws_sync.h"

LOG_MODULE_DECLARE(mesh);

#define C mesh_protocol_context_get()
#define s_state C->state
#define s_role C->role
#define s_node_id C->node_id
#define s_slot_index C->slot_index
#define s_coordinator_id C->coordinator_id
#define s_local_addr C->local_addr
#define s_peers C->peers
#define s_peer_count C->peer_count
#define s_dedupe C->dedupe
#define s_relay_ring C->relay_ring
#define s_relay_head C->relay_head
#define s_relay_tail C->relay_tail
#define s_control_ring C->control_ring
#define s_control_head C->control_head
#define s_control_tail C->control_tail
#define s_active_speaker_deadline_ms C->active_speaker_deadline_ms
#define s_speaker_active_since_ms C->speaker_active_since_ms
#define s_heard_bitmap C->heard_bitmap
#define s_relay_bitmap C->relay_bitmap
#define s_skip_count C->skip_count
#define s_auto_ticks C->auto_ticks
#define s_status_log_decim C->status_log_decim
#define s_tx_queue_depth_dbg C->tx_queue_depth_dbg
#define s_requested_enabled C->requested_enabled
#define s_control_pending C->control_pending
#define s_status_pending C->status_pending
#define s_requested_command C->requested_command
#define s_requested_generation C->requested_generation
#define s_stat_tx_count C->stat_tx_count
#define s_stat_tx_fail C->stat_tx_fail
#define s_stat_tx_starvation C->stat_tx_starvation
#define s_stat_tx_queue_drain C->stat_tx_queue_drain
#define s_stat_rx_count C->stat_rx_count
#define s_stat_rx_drop C->stat_rx_drop
#define s_stat_audio_fwd C->stat_audio_fwd
#define s_stat_tx_overwrite C->stat_tx_overwrite
#define s_stat_spi_audio_in C->stat_spi_audio_in
#define s_stat_ingress_inactive_drop C->stat_ingress_inactive_drop
#define s_stat_ingress_msgq_drop C->stat_ingress_msgq_drop
#define s_stat_ingress_purge_drop C->stat_ingress_purge_drop
#define s_stat_tx_ring_drop C->stat_tx_ring_drop
#define s_stat_tx_purge_drop C->stat_tx_purge_drop
#define s_stat_relay_ring_drop C->stat_relay_ring_drop
#define s_stat_control_ring_drop C->stat_control_ring_drop
#define s_stat_rf_audio_try C->stat_rf_audio_try
#define s_stat_rf_audio_ok C->stat_rf_audio_ok
#define s_stat_rf_audio_fail C->stat_rf_audio_fail
#define s_stat_rf_rx_audio_ok C->stat_rf_rx_audio_ok
#define s_stat_rf_rx_malformed C->stat_rf_rx_malformed
#define s_stat_rf_rx_version_drop C->stat_rf_rx_version_drop
#define s_stat_rf_rx_self_drop C->stat_rf_rx_self_drop
#define s_stat_rf_rx_duplicate_drop C->stat_rf_rx_duplicate_drop
#define s_stat_rf_rx_inactive_drop C->stat_rf_rx_inactive_drop
#define s_stat_spi_out_ok C->stat_spi_out_ok
#define s_stat_spi_out_drop C->stat_spi_out_drop
#define s_stat_bundle_tx C->stat_bundle_tx
#define s_stat_bundle_rx C->stat_bundle_rx
#define s_stat_bundle_bad C->stat_bundle_bad
#define s_stat_prev1_forwarded C->stat_prev1_forwarded
#define s_stat_prev2_forwarded C->stat_prev2_forwarded
#define s_stat_prev1_stripped C->stat_prev1_stripped
#define s_stat_prev2_stripped C->stat_prev2_stripped
#define s_stat_bundle_late_drop C->stat_bundle_late_drop
#define s_stat_bundle_max_bytes C->stat_bundle_max_bytes
#define s_stat_local_deferred_recovery C->stat_local_deferred_recovery
#define s_e2e_spi_in_frames C->e2e_spi_in_frames
#define s_e2e_spi_in_gap_evt C->e2e_spi_in_gap_evt
#define s_e2e_spi_in_gap_fr C->e2e_spi_in_gap_fr
#define s_e2e_spi_in_reset_evt C->e2e_spi_in_reset_evt
#define s_e2e_rf_tx_frames C->e2e_rf_tx_frames
#define s_e2e_rf_rx_frames C->e2e_rf_rx_frames
#define s_e2e_rf_rx_gap_evt C->e2e_rf_rx_gap_evt
#define s_e2e_rf_rx_gap_fr C->e2e_rf_rx_gap_fr
#define s_e2e_rf_rx_reset_evt C->e2e_rf_rx_reset_evt
#define s_e2e_spi_out_frames C->e2e_spi_out_frames

/* Work items: the runtime owns scheduling; membership borrows scan/join/status
 * through mesh_protocol_membership_bind_work(). */
static struct k_work_delayable s_scan_work;
static struct k_work_delayable s_join_work;
static struct k_work_delayable s_status_work;
static struct k_work s_command_work;

static void status_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (s_state != MESH_STATE_ACTIVE) {
        return;
    }

    /* Reap silent peers once per status tick (1 Hz). */
    mesh_protocol_membership_check_peer_timeouts();

    /* Send mesh status and keepalive over RF */
    if (s_role == MESH_ROLE_COORDINATOR) {
        mesh_protocol_audio_update_speaker_grants();
    }
    int status_ret = mesh_protocol_tx_send_status_packet(C);
    mesh_protocol_tx_send_keepalive(C);
    if (status_ret == 0) {
        mesh_protocol_audio_clear_heard_relay_bitmaps();
    }

    /* Log packet stats */
    esb_radio_timing_stats_t esb_stats = {0};
    esb_radio_get_timing_stats(&esb_stats);
    tdma_stats_t tdma_stats = {0};
    tdma_get_stats(&tdma_stats);

    bool log_now = ((s_status_log_decim++ % 2) == 0);
    if (log_now) {
        printk("[MESH] r=%u id=%u sl=%d tx=%u(err=%u) rx=%u drop=%u fwd=%u | spi_in=%u overwr=%u starve=%u drain=%u q=%u\n",
               s_role, s_node_id, s_slot_index, s_stat_tx_count, s_stat_tx_fail, s_stat_rx_count,
               (uint32_t)atomic_get(&s_stat_rx_drop), s_stat_audio_fwd, s_stat_spi_audio_in,
               (uint32_t)atomic_get(&s_stat_tx_overwrite),
                s_stat_tx_starvation, s_stat_tx_queue_drain, s_tx_queue_depth_dbg);
        printk("[ESB_TIM] tx=%u to=%u txwait=%u/%u rxpause=%u/%u us\n", esb_stats.tx_count,
               esb_stats.tx_timeout_count, esb_stats.tx_wait_us_avg, esb_stats.tx_wait_us_max,
               esb_stats.rx_pause_us_avg, esb_stats.rx_pause_us_max);
        /* Cumulative 2 ms histogram of time-to-own-slot at audio ingress. */
        printk("[APHASE] id=%u h=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", s_node_id,
               C->aphase_hist[0], C->aphase_hist[1], C->aphase_hist[2], C->aphase_hist[3],
               C->aphase_hist[4], C->aphase_hist[5], C->aphase_hist[6], C->aphase_hist[7],
               C->aphase_hist[8], C->aphase_hist[9]);
    }

    s_auto_ticks++;

    if (log_now) {
        ws_sync_diag_t ws_diag = {0};
        ws_sync_get_diag(&ws_diag);
        printk("[ATUNE] r=%u id=%u q=%u skip=%u/%u ws_e=%u ws_n=%u ws_ok=%u ws_no=%u ws_rej=%u ws_delta=%u ws_c=%d ws_d=%d td_req=%u td_app=%u td_sum=%lld td_pend=%d td_last=%d td_cmd=%u td_meas=%u td_jit=%d td_jit_max=%u\n",
               s_role, s_node_id, s_tx_queue_depth_dbg,
               s_skip_count, s_auto_ticks,
               ws_diag.total_edges, ws_diag.sample_count, ws_diag.valid_count,
               ws_diag.no_signal_count, ws_diag.rejected_count,
               ws_diag.last_delta_edges, ws_diag.last_correction_us,
               ws_diag.cumulative_drift_us, tdma_stats.tune_request_count,
               tdma_stats.correction_apply_count,
               (long long)tdma_stats.correction_applied_us,
               tdma_stats.correction_pending_us, tdma_stats.last_correction_us,
               tdma_stats.commanded_period_us, tdma_stats.measured_interval_us,
               tdma_stats.callback_jitter_us, tdma_stats.callback_jitter_max_us);
        printk("[E2E_NRF] id=%u spi_in=%u spi_gap=%u/%u spi_reset=%u rf_tx=%u rf_rx=%u rf_gap=%u/%u rf_reset=%u spi_out=%u\n",
               s_node_id,
               s_e2e_spi_in_frames, s_e2e_spi_in_gap_evt, s_e2e_spi_in_gap_fr, s_e2e_spi_in_reset_evt,
               s_e2e_rf_tx_frames, s_e2e_rf_rx_frames, s_e2e_rf_rx_gap_evt, s_e2e_rf_rx_gap_fr,
               s_e2e_rf_rx_reset_evt,
                s_e2e_spi_out_frames);
        printk("PIPE v=1 dev=nrf stage=mesh node=%u ingress_ok=%u ingress_inactive_drop=%u ingress_q_drop=%u ingress_purge_drop=%u tx_ring_drop=%u tx_purge_drop=%u prefill_skip=%u rf_tx_try=%u rf_tx_ok=%u rf_tx_fail=%u rf_rx_ok=%u rf_rx_ring_drop=%u rf_rx_malformed=%u rf_rx_version_drop=%u rf_rx_self_drop=%u rf_rx_dup_drop=%u rf_rx_inactive_drop=%u relay_q_drop=%u control_q_drop=%u spi_out_ok=%u spi_out_drop=%u q_depth=%u bundle_tx=%u bundle_rx=%u bundle_bad=%u prev1_forwarded=%u prev2_forwarded=%u prev1_stripped=%u prev2_stripped=%u bundle_late_drop=%u bundle_max_bytes=%u local_deferred_recovery=%u tx_duration_max_us=%u\n",
               s_node_id, s_stat_spi_audio_in, s_stat_ingress_inactive_drop,
               (uint32_t)atomic_get(&s_stat_ingress_msgq_drop), s_stat_ingress_purge_drop,
               s_stat_tx_ring_drop, s_stat_tx_purge_drop, s_skip_count,
               s_stat_rf_audio_try, s_stat_rf_audio_ok,
               s_stat_rf_audio_fail, s_stat_rf_rx_audio_ok,
               (uint32_t)atomic_get(&s_stat_rx_drop), s_stat_rf_rx_malformed,
               s_stat_rf_rx_version_drop, s_stat_rf_rx_self_drop,
               s_stat_rf_rx_duplicate_drop, s_stat_rf_rx_inactive_drop,
               s_stat_relay_ring_drop, s_stat_control_ring_drop, s_stat_spi_out_ok,
                s_stat_spi_out_drop, s_tx_queue_depth_dbg, s_stat_bundle_tx,
                 s_stat_bundle_rx, s_stat_bundle_bad, s_stat_prev1_forwarded,
                 s_stat_prev2_forwarded, s_stat_prev1_stripped,
                 s_stat_prev2_stripped, s_stat_bundle_late_drop,
                 s_stat_bundle_max_bytes, s_stat_local_deferred_recovery,
                 esb_stats.tx_wait_us_max);
        printk("PIPE v=1 dev=nrf stage=tdma node=%u slot_due=%u slot_submit_drop=%u slot_late_drop=%u control_due=%u control_submit_drop=%u control_late_drop=%u discipline_due=%u discipline_submit_drop=%u discipline_capture_drop=%u tune_req=%u tune_clamp=%u correction_apply=%u correction_applied_us=%lld correction_pending_us=%d last_correction_us=%d commanded_period_us=%u measured_interval_us=%u callback_jitter_us=%d callback_jitter_max_us=%u skipped_frames=%u sync_acquire=%u sync_reacquire=%u sync_history_miss=%u sync_frame_diff=%d sync_phase_us=%d\n",
               s_node_id, tdma_stats.slot_due, tdma_stats.slot_submit_drop,
               tdma_stats.slot_late_drop, tdma_stats.control_due,
               tdma_stats.control_submit_drop, tdma_stats.control_late_drop,
               tdma_stats.discipline_due, tdma_stats.discipline_submit_drop,
               tdma_stats.discipline_capture_drop, tdma_stats.tune_request_count,
               tdma_stats.tune_clamp_count, tdma_stats.correction_apply_count,
               (long long)tdma_stats.correction_applied_us,
               tdma_stats.correction_pending_us, tdma_stats.last_correction_us,
               tdma_stats.commanded_period_us, tdma_stats.measured_interval_us,
               tdma_stats.callback_jitter_us, tdma_stats.callback_jitter_max_us,
               tdma_stats.skipped_frame_count, tdma_stats.sync_acquire_count,
               tdma_stats.sync_reacquire_count, tdma_stats.sync_history_miss_count,
               tdma_stats.sync_frame_diff, tdma_stats.sync_phase_correction_us);
        printk("PIPE v=1 dev=nrf stage=rf node=%u tx_ok=%u tx_timeout=%u tx_busy=%u tx_write_drop=%u tx_event_fail=%u rx_no_callback=%u rx_flush_drop=%u rx_restart_drop=%u tx_wait_max_us=%u rx_pause_max_us=%u\n",
               s_node_id, esb_stats.tx_count, esb_stats.tx_timeout_count,
               esb_stats.tx_busy_count, esb_stats.tx_write_fail_count,
               esb_stats.tx_failed_event_count, esb_stats.rx_no_callback_count,
               esb_stats.rx_flush_drop_count, esb_stats.rx_restart_fail_count,
               esb_stats.tx_wait_us_max, esb_stats.rx_pause_us_max);
    }

    if (mesh_protocol_membership_handle_coordinator_timeout()) {
        return; /* Membership rescheduled scanning; don't reschedule status work */
    }

    /* Reschedule */
    k_work_schedule(&s_status_work, K_MSEC(STATUS_INTERVAL_MS));
}

static void command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    do {
        if (atomic_set(&s_control_pending, 0) != 0) {
            bool enable = atomic_get(&s_requested_enabled) != 0;
            uint8_t command = (uint8_t)atomic_get(&s_requested_command);
            uint8_t generation = (uint8_t)atomic_get(&s_requested_generation);
            int result = 0;
            if (enable && s_state == MESH_STATE_IDLE) {
                result = mesh_protocol_start();
            } else if (!enable && s_state != MESH_STATE_IDLE) {
                mesh_protocol_stop();
            }
            uart_bridge_send_command_ack(command, generation, result);
            uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                    s_node_id, s_slot_index, s_coordinator_id);
        }
        if (atomic_set(&s_status_pending, 0) != 0) {
            uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                                    s_node_id, s_slot_index, s_coordinator_id);
        }
    } while (atomic_get(&s_control_pending) != 0 || atomic_get(&s_status_pending) != 0);
}

void mesh_protocol_request_start(uint8_t generation)
{
    atomic_set(&s_requested_enabled, 1);
    atomic_set(&s_requested_command, BRIDGE_COMMAND_MESH_START);
    atomic_set(&s_requested_generation, generation);
    atomic_set(&s_control_pending, 1);
    k_work_submit(&s_command_work);
}

void mesh_protocol_request_stop(uint8_t generation)
{
    atomic_set(&s_requested_enabled, 0);
    atomic_set(&s_requested_command, BRIDGE_COMMAND_MESH_STOP);
    atomic_set(&s_requested_generation, generation);
    atomic_set(&s_control_pending, 1);
    k_work_submit(&s_command_work);
}

void mesh_protocol_request_status(void)
{
    atomic_set(&s_status_pending, 1);
    k_work_submit(&s_command_work);
}

int mesh_protocol_init(void)
{
    LOG_INF("Initializing mesh protocol");

    /* Initialize work items */
    k_work_init_delayable(&s_scan_work, mesh_protocol_membership_scan_work_handler);
    k_work_init_delayable(&s_join_work, mesh_protocol_membership_join_work_handler);
    k_work_init_delayable(&s_status_work, status_work_handler);
    k_work_init(&s_command_work, command_work_handler);
    mesh_protocol_membership_bind_work(&s_scan_work, &s_join_work, &s_status_work);
    mesh_protocol_audio_init();

    /* Set callbacks */
    mesh_protocol_rx_init();
    tdma_set_control_callback(mesh_protocol_tx_control_handler);

    /* Get local address */
    esb_radio_get_address(s_local_addr);

    s_state = MESH_STATE_IDLE;
    mesh_protocol_audio_reset_all_rf_e2e_trackers();
    mesh_protocol_audio_set_ingress_enabled(false, false);
    return 0;
}

int mesh_protocol_start(void)
{
    if (s_state != MESH_STATE_IDLE) {
        return -EALREADY;
    }

    LOG_INF("Starting mesh protocol");
    printk("[MESH] mesh_protocol_start called\n");
    mesh_protocol_audio_reset_all_rf_e2e_trackers();

    /* Start RX so we can hear SYNC broadcasts from an existing coordinator */
    esb_radio_start_rx();

    /* Enter scanning state */
    s_state = MESH_STATE_SCANNING;
    uint32_t delay_ms = mesh_protocol_membership_scan_timeout_ms();
    printk("[MESH] Scanning for existing mesh (%ums timeout)...\n", delay_ms);
    k_work_schedule(&s_scan_work, K_MSEC(delay_ms));

    printk("[MESH] mesh_protocol_start complete\n");
    return 0;
}

void mesh_protocol_stop(void)
{
    LOG_INF("Stopping mesh protocol");

    mesh_protocol_audio_set_ingress_enabled(false, false);
    k_work_cancel_delayable(&s_scan_work);
    k_work_cancel_delayable(&s_join_work);
    k_work_cancel_delayable(&s_status_work);

    if (s_state == MESH_STATE_ACTIVE && s_node_id != 0) {
        mesh_leave_v2_payload_t leave;
        memcpy(leave.sender_addr, s_local_addr, sizeof(leave.sender_addr));
        mesh_protocol_tx_send_packet(C, MESH_PKT_LEAVE, &leave, sizeof(leave));
    }

    s_state = MESH_STATE_IDLE;
    tdma_stop();
    esb_radio_stop_rx();
    mesh_protocol_rx_stop();
    mesh_protocol_audio_cancel_work();

    memset(s_peers, 0, sizeof(C->peers));
    mesh_core_dedupe_reset(&s_dedupe);
    mesh_protocol_audio_reset_all_rf_e2e_trackers();
    memset(s_relay_ring, 0, sizeof(C->relay_ring));
    memset(s_control_ring, 0, sizeof(C->control_ring));
    memset(s_active_speaker_deadline_ms, 0, sizeof(C->active_speaker_deadline_ms));
    memset(s_speaker_active_since_ms, 0, sizeof(C->speaker_active_since_ms));
    mesh_protocol_audio_clear_speaker_grants();
    s_heard_bitmap = 0;
    s_relay_bitmap = 0;
    s_peer_count = 0;
    s_relay_head = 0;
    s_relay_tail = 0;
    s_control_head = 0;
    s_control_tail = 0;
    mesh_protocol_audio_purge_tx_ring();
    mesh_protocol_audio_set_ingress_enabled(false, true);
    s_skip_count = 0;

    s_role = MESH_ROLE_NONE;
    s_node_id = 0;
    s_slot_index = -1;
    s_coordinator_id = 0;
    uart_bridge_send_status(s_state, s_role, mesh_protocol_membership_bridge_peer_count(),
                            s_node_id, s_slot_index, s_coordinator_id);
    uart_bridge_send_event(BRIDGE_EVENT_MESH_STOPPED, NULL, 0);
}
