import unittest
from types import SimpleNamespace
from unittest.mock import patch

from benchmark import (
    PortReader,
    PortStats,
    _build_port_json,
    _health_line,
    _nrf_starvation_delta,
    _reconnect_candidates,
    _report_lines_for_port,
    _u32_cumulative_series_delta,
    compute_correlated_delivery,
    compute_hop_pct,
    parse_pipeline_logfmt,
)


class PipelineLogTest(unittest.TestCase):
    def test_parses_esp_idf_prefixed_record(self):
        record = parse_pipeline_logfmt(
            "I (1234) MAIN: PIPE v=1 dev=esp stage=transport node=2 source=100 gate_drop=3"
        )
        self.assertEqual(
            record,
            {
                "v": 1,
                "dev": "esp",
                "stage": "transport",
                "node": 2,
                "source": 100,
                "gate_drop": 3,
            },
        )

    def test_rejects_unknown_or_incomplete_schema(self):
        self.assertIsNone(parse_pipeline_logfmt("PIPE v=2 dev=nrf stage=rf tx_done=1"))
        self.assertIsNone(parse_pipeline_logfmt("PIPE v=1 stage=rf tx_done=1"))
        self.assertIsNone(
            parse_pipeline_logfmt("PIPE v=1 dev=nrf stage=rf tx_ok=-1")
        )
        self.assertIsNone(
            parse_pipeline_logfmt("PIPE v=1 dev=nrf stage=tdma node=-1 sync_frame_diff=-2")
        )
        self.assertIsNone(parse_pipeline_logfmt("ordinary firmware log"))

    def test_parses_complete_signed_tdma_record_as_snapshot_gauges(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        lines = [
            "PIPE v=1 dev=nrf stage=tdma node=2 slot_due=100 slot_submit_drop=1 "
            "slot_late_drop=2 control_due=40 control_submit_drop=3 control_late_drop=4 "
            "discipline_due=50 discipline_submit_drop=5 discipline_capture_drop=6 tune_req=20 "
            "tune_clamp=7 correction_apply=18 correction_applied_us=-120 "
            "correction_pending_us=-8 last_correction_us=-4 commanded_period_us=19998 "
            "measured_interval_us=20011 callback_jitter_us=-11 callback_jitter_max_us=25 "
            "skipped_frames=8 sync_acquire=9 sync_reacquire=10 sync_history_miss=11 "
            "sync_frame_diff=-2 sync_phase_us=-35",
            "PIPE v=1 dev=nrf stage=tdma node=2 slot_due=110 slot_submit_drop=2 "
            "slot_late_drop=3 control_due=45 control_submit_drop=4 control_late_drop=5 "
            "discipline_due=55 discipline_submit_drop=6 discipline_capture_drop=7 tune_req=25 "
            "tune_clamp=8 correction_apply=23 correction_applied_us=-135 "
            "correction_pending_us=6 last_correction_us=3 commanded_period_us=20001 "
            "measured_interval_us=19991 callback_jitter_us=9 callback_jitter_max_us=28 "
            "skipped_frames=9 sync_acquire=10 sync_reacquire=11 sync_history_miss=12 "
            "sync_frame_diff=1 sync_phase_us=14",
        ]
        for line in lines:
            reader._parse_line(line)

        pipeline = _build_port_json(stats)["pipeline"]["nrf:tdma:2"]
        self.assertEqual(pipeline["first"]["correction_applied_us"], -120)
        self.assertEqual(pipeline["first"]["callback_jitter_us"], -11)
        self.assertEqual(pipeline["first"]["sync_frame_diff"], -2)
        self.assertEqual(pipeline["first"]["sync_phase_us"], -35)
        self.assertEqual(pipeline["delta"]["slot_due"], 10)
        for gauge in (
            "correction_applied_us",
            "correction_pending_us",
            "last_correction_us",
            "callback_jitter_us",
            "sync_frame_diff",
            "sync_phase_us",
        ):
            self.assertNotIn(gauge, pipeline["delta"])
            self.assertNotIn(gauge, pipeline["reset_epochs"])

    def test_parses_complete_current_atune_line(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        lines = [
            "[00:01:23.456,789] <inf> mesh: [ATUNE] r=2 id=7 q=3 under_d=1 "
            "skip=12/400 ws_e=76800 ws_n=399 ws_ok=390 ws_no=4 ws_rej=5 "
            "ws_delta=192 ws_c=-6 ws_d=-143 td_req=390 td_app=388 td_sum=-119 "
            "td_pend=-8 td_last=-4 td_cmd=19998 td_meas=20011 td_jit=-11 td_jit_max=25",
            "[00:01:28.456,789] <inf> mesh: [ATUNE] r=2 id=7 q=4 under_d=0 "
            "skip=14/500 ws_e=96000 ws_n=499 ws_ok=489 ws_no=5 ws_rej=5 "
            "ws_delta=192 ws_c=3 ws_d=-137 td_req=490 td_app=488 td_sum=-113 "
            "td_pend=6 td_last=3 td_cmd=20001 td_meas=19991 td_jit=9 td_jit_max=28",
        ]
        for line in lines:
            reader._parse_line(line)

        summary = _build_port_json(stats)
        self.assertEqual(stats.atune_samples, 2)
        self.assertEqual(stats.first_atune["ws_corr"], -6)
        self.assertEqual(stats.first_atune["ws_drift"], -143)
        self.assertEqual(stats.first_atune["td_sum"], -119)
        self.assertEqual(stats.first_atune["td_jit"], -11)
        self.assertEqual(stats.first_atune["skip_pct"], 3.0)
        self.assertEqual(summary["atune_delta"]["ticks"], 100)
        for gauge in ("ws_corr", "ws_drift", "td_sum", "td_pend", "td_last", "td_jit"):
            self.assertNotIn(gauge, summary["atune_delta"])
            self.assertNotIn(gauge, summary["atune_reset_epochs"])

    def test_parses_cumulative_tx_starvation_and_avoids_duplicate_warning(self):
        stats = PortStats(port="test", open_ok=True, lines=4)
        reader = PortReader("test", 115200, None, "unused", stats)
        for line in (
            "[MESH] r=1 id=2 sl=1 tx=10(err=0) rx=5 drop=0 fwd=2 | "
            "spi_in=10 overwr=0 starve=4 drain=8 q=0",
            "[TXSTARVE] total=4 r=1 id=2 sl=1 q=0 tts=100",
            "[MESH] r=1 id=2 sl=1 tx=12(err=0) rx=7 drop=0 fwd=3 | "
            "spi_in=12 overwr=0 starve=6 drain=11 q=0",
            "[TXSTARVE] total=6 r=1 id=2 sl=1 q=0 tts=100",
        ):
            reader._parse_line(line)

        summary = _build_port_json(stats)
        self.assertEqual(summary["first_mesh"]["starve"], 4)
        self.assertEqual(summary["last_mesh"]["starve"], 6)
        self.assertEqual(summary["mesh_delta"]["starve"], 2)
        self.assertEqual(summary["mesh_delta"]["drain"], 3)
        self.assertNotIn("starve", summary["mesh_reset_epochs"])
        self.assertEqual(summary["txstarve_delta"]["total"], 2)
        health = _health_line(stats)
        self.assertEqual(health, "WARN (nrf_starve+2)")
        self.assertEqual(health.count("nrf_starve+2"), 1)
        report = "\n".join(_report_lines_for_port(stats, 60))
        self.assertIn("nRF TX starvation: total=6 delta=2 (events/min=2.0)", report)
        self.assertIn("Delta MESH:", report)
        self.assertIn("starve=2", report)

    def test_legacy_uflow_is_visible_but_never_gates_starvation_health(self):
        stats = PortStats(port="test", open_ok=True, lines=2)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line("[UFLOW] under=64 reason=drain0")
        reader._parse_line("[UFLOW] under=65 reason=empty")

        summary = _build_port_json(stats)
        self.assertEqual(summary["uflow_under_delta"], 1)
        self.assertIsNone(_nrf_starvation_delta(stats))
        self.assertEqual(_health_line(stats), "OK (no error/drops/CRC growth observed)")
        report = "\n".join(_report_lines_for_port(stats, 60))
        self.assertIn("nRF legacy UFLOW: total=65 delta=1 (not health)", report)
        self.assertNotIn("nRF TX starvation", report)
        self.assertNotIn("reasons", report)

    def test_single_mesh_starve_sample_falls_back_to_txstarve_series(self):
        stats = PortStats(port="test", open_ok=True, lines=3)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "[MESH] r=1 id=2 sl=1 tx=10(err=0) rx=5 drop=0 fwd=2 | "
            "spi_in=10 overwr=0 starve=99 drain=8 q=0"
        )
        reader._parse_line("[TXSTARVE] total=4 r=1 id=2 sl=1 q=0 tts=100")
        reader._parse_line("[TXSTARVE] total=7 r=1 id=2 sl=1 q=0 tts=100")

        self.assertEqual(_health_line(stats), "WARN (nrf_starve+3)")
        report = "\n".join(_report_lines_for_port(stats, 60))
        self.assertIn("nRF TX starvation: total=7 delta=3 (events/min=3.0)", report)

    def test_u32_starvation_delta_recognizes_rollover_but_not_reset(self):
        self.assertEqual(_u32_cumulative_series_delta([0xFFFFFFFD, 3]), (6, 0))
        self.assertEqual(_u32_cumulative_series_delta([100, 4, 9]), (9, 1))

    def test_mesh_starvation_uses_u32_rollover_delta(self):
        stats = PortStats(port="test", open_ok=True, lines=2)
        reader = PortReader("test", 115200, None, "unused", stats)
        for total in (0xFFFFFFFD, 3):
            reader._parse_line(
                "[MESH] r=1 id=2 sl=1 tx=10(err=0) rx=5 drop=0 fwd=2 | "
                f"spi_in=10 overwr=0 starve={total} drain=8 q=0"
            )

        self.assertEqual(_nrf_starvation_delta(stats), 6)
        self.assertEqual(_health_line(stats), "WARN (nrf_starve+6)")
        summary = _build_port_json(stats)
        self.assertEqual(summary["mesh_delta"]["starve"], 6)
        self.assertNotIn("starve", summary["mesh_reset_epochs"])
        self.assertIn("starve=6", "\n".join(_report_lines_for_port(stats, 60)))

    def test_mesh_starvation_reset_matches_json_text_and_health(self):
        stats = PortStats(port="test", open_ok=True, lines=3)
        reader = PortReader("test", 115200, None, "unused", stats)
        for total in (100, 4, 9):
            reader._parse_line(
                "[MESH] r=1 id=2 sl=1 tx=10(err=0) rx=5 drop=0 fwd=2 | "
                f"spi_in=10 overwr=0 starve={total} drain=8 q=0"
            )

        summary = _build_port_json(stats)
        self.assertEqual(summary["mesh_delta"]["starve"], 9)
        self.assertEqual(summary["mesh_reset_epochs"]["starve"], 1)
        self.assertEqual(_nrf_starvation_delta(stats), 9)
        self.assertEqual(_health_line(stats), "WARN (nrf_starve+9)")
        self.assertIn("starve=9", "\n".join(_report_lines_for_port(stats, 60)))

    def test_txstarve_uses_u32_rollover_delta(self):
        stats = PortStats(port="test", open_ok=True, lines=2)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "[TXSTARVE] total=4294967293 r=1 id=2 sl=1 q=0 tts=100"
        )
        reader._parse_line("[TXSTARVE] total=3 r=1 id=2 sl=1 q=0 tts=100")

        self.assertEqual(_nrf_starvation_delta(stats), 6)
        self.assertEqual(_health_line(stats), "WARN (nrf_starve+6)")

    def test_parses_current_adaptive_playout_and_per_source_depth_lines(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "I (123456) audio:   Adaptive playout: hold=27 catchup=9 sources=3"
        )
        reader._parse_line(
            "I (123457) audio:   RX queue depth/source: min=1 avg=4 max=7 (total now=12)"
        )

        self.assertEqual(stats.last_adaptive, {"hold": 27, "catchup": 9, "sources": 3})
        self.assertEqual(
            stats.last_rx_depth,
            {"rx_q_min": 1, "rx_q_avg": 4, "rx_q_max": 7, "rx_q_total": 12},
        )

    def test_parses_current_concealment_line_with_loss_fields(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "I (123456) audio:   Concealment: plc=111 grace_empty=98 conceal=30 "
            "seq_gap=34 seq_reset=2 seq_stale=1"
        )

        self.assertEqual(
            stats.last_conceal,
            {
                "plc": 111,
                "grace_empty": 98,
                "conceal": 30,
                "seq_gap_frames": 34,
                "seq_reset": 2,
                "seq_stale": 1,
            },
        )

    def test_parses_legacy_concealment_line_without_loss_fields(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line("I (123456) audio:   Concealment: plc=5 grace_empty=4")

        self.assertEqual(stats.last_conceal, {"plc": 5, "grace_empty": 4})

    def test_audio_pipe_record_tracks_conceal_and_lock_drop_counters(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        base = (
            "PIPE v=1 dev=esp stage=audio capture_ok={ok} capture_short=0 "
            "capture_timeout=0 capture_err=0 encode_ok={ok} encode_err=0 dtx_drop=0 "
            "rx_q_drop=0 rx_lock_drop={lock} rx_src_drop=0 jitter_drop=0 "
            "decode_ok={ok} decode_err=0 plc=0 hold=0 catchup=0 conceal={conceal} "
            "seq_gap={gap} seq_reset=0 seq_stale=0 glitch=0 play_ok={ok} i2s_err=0 "
            "notify_drop=0 rx_sources=1"
        )
        reader._parse_line(base.format(ok=100, lock=1, conceal=2, gap=2))
        reader._parse_line(base.format(ok=200, lock=3, conceal=7, gap=9))

        pipeline = _build_port_json(stats)["pipeline"]["esp:audio:na"]
        self.assertEqual(pipeline["delta"]["conceal"], 5)
        self.assertEqual(pipeline["delta"]["seq_gap"], 7)
        self.assertEqual(pipeline["delta"]["rx_lock_drop"], 2)
        self.assertEqual(pipeline["delta"]["glitch"], 0)

    def test_audio_pipe_accepts_signed_asrc_gauge(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        base = (
            "PIPE v=1 dev=esp stage=audio capture_ok={ok} conceal={conceal} "
            "rx_sources=1 asrc_ppm={ppm} asrc_abs_max_ppm={maximum} asrc_recovery={recovery}"
        )
        reader._parse_line(base.format(ok=100, conceal=2, ppm=-375, maximum=375, recovery=0))
        reader._parse_line(base.format(ok=200, conceal=7, ppm=250, maximum=500, recovery=1))

        pipeline = _build_port_json(stats)["pipeline"]["esp:audio:na"]
        self.assertEqual(pipeline["delta"]["capture_ok"], 100)
        self.assertEqual(pipeline["delta"]["conceal"], 5)
        self.assertNotIn("asrc_ppm", pipeline["delta"])
        self.assertNotIn("asrc_abs_max_ppm", pipeline["delta"])
        self.assertNotIn("rx_sources", pipeline["delta"])
        self.assertNotIn("asrc_recovery", pipeline["delta"])
        self.assertEqual(pipeline["last"]["asrc_ppm"], 250)
        self.assertEqual(pipeline["last"]["asrc_recovery"], 1)

    def test_records_first_last_and_cumulative_delta(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok=10 rf_tx_ok=8 q_depth=2 tx_wait_avg_us=1200"
        )
        reader._parse_line(
            "PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok=25 rf_tx_ok=20 q_depth=5 tx_wait_avg_us=900"
        )

        pipeline = _build_port_json(stats)["pipeline"]["nrf:mesh:3"]
        self.assertEqual(pipeline["delta"]["ingress_ok"], 15)
        self.assertEqual(pipeline["delta"]["rf_tx_ok"], 12)
        self.assertNotIn("q_depth", pipeline["delta"])
        self.assertNotIn("tx_wait_avg_us", pipeline["delta"])

    def test_bridge_status_counters_exclude_snapshot_fields(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "PIPE v=1 dev=esp stage=bridge_status valid_rx=100 expire=1 age_ms=20 "
            "max_age_ms=3100 gen=100 state=3 exp_gen=90 exp_state=3 gate_stale=4"
        )
        reader._parse_line(
            "PIPE v=1 dev=esp stage=bridge_status valid_rx=110 expire=2 age_ms=40 "
            "max_age_ms=5200 gen=110 state=3 exp_gen=105 exp_state=3 gate_stale=7"
        )

        pipeline = _build_port_json(stats)["pipeline"]["esp:bridge_status:na"]
        self.assertEqual(pipeline["delta"]["valid_rx"], 10)
        self.assertEqual(pipeline["delta"]["expire"], 1)
        self.assertEqual(pipeline["delta"]["gate_stale"], 3)
        for gauge in ("age_ms", "max_age_ms", "gen", "state", "exp_gen", "exp_state"):
            self.assertNotIn(gauge, pipeline["delta"])
            self.assertNotIn(gauge, pipeline["reset_epochs"])

    def test_two_predecessor_redundancy_counters_delta(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "PIPE v=1 dev=esp stage=transport bundle_tx=10 bundle_rx=20 bundle_bad=2 "
            "prev1_attached=7 prev2_attached=4 prev1_offer=12 prev1_accept=9 "
            "prev1_reject=3 prev2_offer=8 prev2_accept=5 prev2_reject=3 recovered=11"
        )
        reader._parse_line(
            "PIPE v=1 dev=esp stage=transport bundle_tx=18 bundle_rx=31 bundle_bad=4 "
            "prev1_attached=13 prev2_attached=9 prev1_offer=19 prev1_accept=14 "
            "prev1_reject=5 prev2_offer=14 prev2_accept=9 prev2_reject=5 recovered=20"
        )

        pipeline = _build_port_json(stats)["pipeline"]["esp:transport:na"]
        self.assertEqual(
            pipeline["delta"],
            {
                "bundle_bad": 2,
                "bundle_rx": 11,
                "bundle_tx": 8,
                "prev1_accept": 5,
                "prev1_attached": 6,
                "prev1_offer": 7,
                "prev1_reject": 2,
                "prev2_accept": 4,
                "prev2_attached": 5,
                "prev2_offer": 6,
                "prev2_reject": 2,
                "recovered": 9,
            },
        )

    def test_e2e_recovery_parses_and_reports_effective_gap(self):
        stats = PortStats(port="test", open_ok=True, lines=2)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "[E2E_ESP] tx=100 rx=80 gap_evt=8 gap_fr=10 reset_evt=0 "
            "recovered=4 effective_gap=6"
        )
        reader._parse_line(
            "[E2E_ESP] tx=120 rx=98 gap_evt=12 gap_fr=15 reset_evt=0 "
            "recovered=7 effective_gap=8"
        )

        summary = _build_port_json(stats)
        self.assertEqual(summary["last_e2e_esp"]["recovered"], 7)
        self.assertEqual(summary["last_e2e_esp"]["effective_gap"], 8)
        self.assertEqual(summary["e2e_esp_delta"]["gap_fr"], 5)
        self.assertEqual(summary["e2e_esp_delta"]["recovered"], 3)
        self.assertEqual(summary["e2e_esp_delta"]["effective_gap"], 2)
        self.assertEqual(summary["hop_pct"]["esp_e2e_raw_gap_pct"], 21.74)
        self.assertEqual(summary["hop_pct"]["esp_e2e_effective_gap_pct"], 10.0)
        report = "\n".join(_report_lines_for_port(stats, 60))
        self.assertIn("raw_gap=5 recovered=3 effective_gap=2", report)
        self.assertIn("e2e_esp_effective_gap+2", _health_line(stats))

    def test_e2e_recovery_keeps_legacy_logs_and_clears_health_when_repaired(self):
        legacy = PortStats(port="legacy")
        legacy_reader = PortReader("legacy", 115200, None, "unused", legacy)
        legacy_reader._parse_line(
            "[E2E_ESP] tx=10 rx=8 gap_evt=1 gap_fr=2 reset_evt=0"
        )
        self.assertNotIn("recovered", legacy.last_e2e_esp)

        repaired = PortStats(port="test", open_ok=True, lines=2)
        repaired_reader = PortReader("test", 115200, None, "unused", repaired)
        repaired_reader._parse_line(
            "[E2E_ESP] tx=100 rx=80 gap_evt=8 gap_fr=10 reset_evt=0 "
            "recovered=4 effective_gap=6"
        )
        repaired_reader._parse_line(
            "[E2E_ESP] tx=120 rx=98 gap_evt=12 gap_fr=15 reset_evt=0 "
            "recovered=9 effective_gap=6"
        )
        self.assertEqual(_health_line(repaired), "OK (no error/drops/CRC growth observed)")

    def test_e2e_recovery_credits_only_outstanding_raw_gaps(self):
        scenarios = (
            # Join-midstream predecessor accepted as prefill utility, not recovery.
            ((0, 0), (0, 0), 0),
            # One newly observed gap is recovered by its predecessor.
            ((0, 0), (1, 1), 0),
            # Earlier recovery cannot hide a later independent gap.
            ((1, 1), (2, 1), 1),
            # One redundant predecessor repairs only one frame of a larger gap.
            ((2, 1), (5, 2), 2),
            # Recovery accumulated before the window cannot offset a new gap.
            ((10, 10), (11, 10), 1),
        )

        for index, (first, last, expected_effective) in enumerate(scenarios):
            with self.subTest(index=index):
                stats = PortStats(port="test", open_ok=True, lines=2)
                reader = PortReader("test", 115200, None, "unused", stats)
                reader._parse_line(
                    f"[E2E_ESP] tx=100 rx=80 gap_evt=0 gap_fr={first[0]} "
                    f"reset_evt=0 recovered={first[1]} "
                    f"effective_gap={max(first[0] - first[1], 0)}"
                )
                reader._parse_line(
                    f"[E2E_ESP] tx=120 rx=98 gap_evt=1 gap_fr={last[0]} "
                    f"reset_evt=0 recovered={last[1]} "
                    f"effective_gap={max(last[0] - last[1], 0)}"
                )

                summary = _build_port_json(stats)
                self.assertEqual(
                    summary["e2e_esp_delta"]["effective_gap"], expected_effective
                )
                health = _health_line(stats)
                if expected_effective == 0:
                    self.assertEqual(health, "OK (no error/drops/CRC growth observed)")
                else:
                    self.assertIn(
                        f"e2e_esp_effective_gap+{expected_effective}", health
                    )

    def test_late_current_does_not_create_a_later_effective_gap(self):
        stats = PortStats(port="test", open_ok=True, lines=3)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "[E2E_ESP] tx=100 rx=80 gap_evt=0 gap_fr=0 reset_evt=0 "
            "recovered=0 effective_gap=0"
        )
        # A late standalone current increments only the reorder/reset diagnostic.
        reader._parse_line(
            "[E2E_ESP] tx=101 rx=81 gap_evt=0 gap_fr=0 reset_evt=1 "
            "recovered=0 effective_gap=0"
        )
        # The next expected current remains in order because last_seq did not regress.
        reader._parse_line(
            "[E2E_ESP] tx=102 rx=82 gap_evt=0 gap_fr=0 reset_evt=1 "
            "recovered=0 effective_gap=0"
        )

        summary = _build_port_json(stats)
        self.assertEqual(summary["e2e_esp_delta"]["gap_fr"], 0)
        self.assertEqual(summary["e2e_esp_delta"]["effective_gap"], 0)
        self.assertEqual(summary["e2e_esp_delta"]["reset_evt"], 1)
        self.assertEqual(_health_line(stats), "OK (no error/drops/CRC growth observed)")

    def test_true_forward_gap_can_still_be_recovered(self):
        stats = PortStats(port="test", open_ok=True, lines=2)
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line(
            "[E2E_ESP] tx=100 rx=80 gap_evt=0 gap_fr=0 reset_evt=0 "
            "recovered=0 effective_gap=0"
        )
        reader._parse_line(
            "[E2E_ESP] tx=102 rx=81 gap_evt=1 gap_fr=1 reset_evt=0 "
            "recovered=1 effective_gap=0"
        )

        summary = _build_port_json(stats)
        self.assertEqual(summary["e2e_esp_delta"]["gap_fr"], 1)
        self.assertEqual(summary["e2e_esp_delta"]["recovered"], 1)
        self.assertEqual(summary["e2e_esp_delta"]["effective_gap"], 0)
        self.assertEqual(_health_line(stats), "OK (no error/drops/CRC growth observed)")

    def test_reset_epochs_are_accumulated_without_negative_delta(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        for value in (100, 120, 3, 8):
            reader._parse_line(
                f"PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok={value}"
            )

        pipeline = _build_port_json(stats)["pipeline"]["nrf:mesh:3"]
        self.assertEqual(pipeline["delta"]["ingress_ok"], 28)
        self.assertEqual(pipeline["reset_epochs"]["ingress_ok"], 1)

    def test_missing_metric_does_not_create_negative_or_crash(self):
        stats = PortStats(port="test")
        reader = PortReader("test", 115200, None, "unused", stats)
        reader._parse_line("PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok=10")
        reader._parse_line("PIPE v=1 dev=nrf stage=mesh node=3 q_depth=2")
        reader._parse_line("PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok=15")
        reader._parse_line('PIPE v=1 dev=nrf stage="unterminated')

        pipeline = _build_port_json(stats)["pipeline"]["nrf:mesh:3"]
        self.assertEqual(pipeline["delta"]["ingress_ok"], 5)

    def test_two_ports_correlate_only_with_session_endpoints_and_stage(self):
        sender = PortStats(port="sender")
        receiver = PortStats(port="receiver")
        sender_reader = PortReader("sender", 115200, None, "unused", sender)
        receiver_reader = PortReader("receiver", 115200, None, "unused", receiver)
        for value in (10, 110):
            sender_reader._parse_line(
                f"PIPE v=1 dev=nrf stage=rf_tx session=run7 node=1 peer=2 tx_ok={value}"
            )
        for value in (20, 115):
            receiver_reader._parse_line(
                f"PIPE v=1 dev=nrf stage=rf_rx session=run7 node=2 peer=1 rx_ok={value}"
            )

        result = compute_correlated_delivery([sender, receiver])
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["links"][0]["delivery_pct"], 95.0)

    def test_unmatched_local_tx_rx_never_produces_delivery_percentage(self):
        stats = PortStats(port="single")
        stats.first_e2e_esp = {"tx": 100, "rx": 100, "gap_fr": 0}
        stats.last_e2e_esp = {"tx": 110, "rx": 125, "gap_fr": 0}

        self.assertNotIn("esp_e2e_delivery_pct", compute_hop_pct(stats))
        result = compute_correlated_delivery([stats])
        self.assertEqual(result["status"], "insufficient correlated data")

    def test_correlated_rx_over_tx_is_flagged_not_reported_as_over_100_pct(self):
        sender = PortStats(port="sender")
        receiver = PortStats(port="receiver")
        sender_reader = PortReader("sender", 115200, None, "unused", sender)
        receiver_reader = PortReader("receiver", 115200, None, "unused", receiver)
        for value in (0, 10):
            sender_reader._parse_line(
                f"PIPE v=1 dev=nrf stage=rf_tx session=run8 node=1 peer=2 tx_ok={value}"
            )
        for value in (0, 12):
            receiver_reader._parse_line(
                f"PIPE v=1 dev=nrf stage=rf_rx session=run8 node=2 peer=1 rx_ok={value}"
            )

        link = compute_correlated_delivery([sender, receiver])["links"][0]
        self.assertEqual(link["status"], "inconsistent correlated data")
        self.assertIsNone(link["delivery_pct"])

    @patch("benchtool.capture.list_ports.comports")
    def test_reconnect_candidates_follow_stable_usb_identity(self, comports):
        comports.return_value = [
            SimpleNamespace(device="/dev/ttyACM1", serial_number="abc", vid=1, pid=2),
            SimpleNamespace(device="/dev/ttyACM2", serial_number="other", vid=1, pid=2),
        ]
        self.assertEqual(
            _reconnect_candidates("/dev/ttyACM0", ("abc", 1, 2)),
            ["/dev/ttyACM0", "/dev/ttyACM1"],
        )
        self.assertEqual(
            _reconnect_candidates("/dev/ttyACM0", (None, 1, 2)),
            ["/dev/ttyACM0"],
        )
        comports.return_value.append(
            SimpleNamespace(
                device="/dev/ttyACM0", serial_number="replacement", vid=1, pid=2
            )
        )
        self.assertEqual(
            _reconnect_candidates("/dev/ttyACM0", ("abc", 1, 2)),
            ["/dev/ttyACM1"],
        )


if __name__ == "__main__":
    unittest.main()
