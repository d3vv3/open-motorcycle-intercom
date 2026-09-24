import unittest
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

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
from benchtool.report import _overall_health


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
        self.assertIsNone(parse_pipeline_logfmt("PIPE v=1 dev=nrf stage=rf tx_ok=-1"))
        self.assertIsNone(
            parse_pipeline_logfmt(
                "PIPE v=1 dev=nrf stage=tdma node=-1 sync_frame_diff=-2"
            )
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
        reader._parse_line("[TXSTARVE] total=4294967293 r=1 id=2 sl=1 q=0 tts=100")
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
        reader._parse_line(
            base.format(ok=100, conceal=2, ppm=-375, maximum=375, recovery=0)
        )
        reader._parse_line(
            base.format(ok=200, conceal=7, ppm=250, maximum=500, recovery=1)
        )

        pipeline = _build_port_json(stats)["pipeline"]["esp:audio:na"]
        self.assertEqual(pipeline["delta"]["capture_ok"], 100)
        self.assertEqual(pipeline["delta"]["conceal"], 5)
        self.assertNotIn("asrc_ppm", pipeline["delta"])
        self.assertNotIn("asrc_abs_max_ppm", pipeline["delta"])
        self.assertNotIn("rx_sources", pipeline["delta"])
        self.assertNotIn("asrc_recovery", pipeline["delta"])
        self.assertEqual(pipeline["last"]["asrc_ppm"], 250)
        self.assertEqual(pipeline["last"]["asrc_recovery"], 1)

    def test_cpu_summary_snapshots_are_gauges_and_do_not_change_other_stages(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=cpu part=summary epoch_id=0x01 node_mac=aa:bb"
        for uptime, valid, total, interval, new, reset, failed, idle0, idle1 in (
            (1000, 1, 5000000000, 1000000, 2, 1, 0, 1, 1),
            (2000, 0, 6000000000, 0, 0, 0, 1, 0, 0),
        ):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} valid={valid} total_us={total} "
                f"interval_us={interval} task_count=20 "
                "task_count_hint=24 capacity=32 "
                f"matched=18 new={new} gone=1 reset={reset} coverage_permille=900 "
                f"snapshot_fail={failed}"
            )
            reader._parse_line(
                f"{meta.replace('part=summary', 'part=idle')} uptime_ms={uptime} "
                f"valid={valid} interval_us={interval} accounted_us=800000 "
                f"accounted_permille=800 chip_permille=400 idle0_valid={idle0} "
                "idle0_us=100000 idle0_permille=100 "
                f"idle1_valid={idle1} idle1_us=200000 idle1_permille=200"
            )
            reader._parse_line(
                f"{meta.replace('part=summary', 'part=overhead')} uptime_ms={uptime} "
                f"snapshot_us={70 + valid} collect_us={90 + valid}"
            )
        reader._parse_line("PIPE v=1 dev=esp stage=other new=3 reset=4")
        reader._parse_line("PIPE v=1 dev=esp stage=other new=5 reset=6")
        pipeline = _build_port_json(stats)["pipeline"]
        cpu = pipeline["esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=summary"]
        idle = pipeline["esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=idle"]
        overhead = pipeline["esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=overhead"]
        self.assertEqual(cpu["first"]["total_us"], 5000000000)
        self.assertEqual(cpu["last"]["snapshot_fail"], 1)
        self.assertEqual(cpu["last"]["capacity"], 32)
        self.assertEqual(cpu["last"]["task_count_hint"], 24)
        self.assertNotIn("accounted_us", cpu["last"])
        self.assertEqual(idle["first"]["idle0_valid"], 1)
        self.assertEqual(idle["first"]["idle1_valid"], 1)
        self.assertEqual(idle["last"]["idle0_valid"], 0)
        self.assertEqual(idle["last"]["idle1_valid"], 0)
        self.assertEqual(idle["last"]["idle1_permille"], 200)
        self.assertEqual(idle["last"]["accounted_us"], 800000)
        self.assertEqual(overhead["first"]["snapshot_us"], 71)
        self.assertEqual(overhead["last"]["snapshot_us"], 70)
        for row in (cpu, idle, overhead):
            self.assertEqual(row["delta"], {})
            self.assertEqual(row["reset_epochs"], {})
        reader._parse_line(
            "PIPE v=1 dev=esp stage=cpu part=idle epoch_id=0x02 node_mac=aa:bb "
            "uptime_ms=100 valid=0 interval_us=0 idle0_valid=0 idle1_valid=0"
        )
        reader._parse_line(
            "PIPE v=1 dev=esp stage=cpu part=task epoch_id=0x01 node_mac=aa:bb "
            "task_id=0x01 task_handle=0x1234 runtime_us=5000000000"
        )
        rows = _build_port_json(stats)["pipeline"]
        self.assertEqual(rows["esp:cpu:na:node_mac=aa:bb:epoch_id=0x02:part=idle"]["delta"], {})
        task_key = ("esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=task:"
                    "task_id=0x01:task_handle=0x1234")
        self.assertEqual(rows[task_key]["last"]["runtime_us"], 5000000000)
        idle_key = "esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=idle"
        self.assertEqual(stats.pipe_samples[idle_key], 2)
        self.assertEqual(pipeline["esp:other:na"]["delta"], {"new": 2, "reset": 2})

    def test_cpu_span_parts_have_cumulative_wall_sums_and_snapshot_maxima(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        base = "PIPE v=1 dev=esp stage=cpu_span node_mac=aa:bb"
        parts = ("cap_convert", "cap_hpf", "cap_cleanup", "play_remote",
                 "play_voice_convert", "play_far_reference")
        for part in parts:
            for epoch, count, total, maximum in (
                ("0x01", 10, 5000000000, 900),
                ("0x01", 14, 5000000500, 700),
                ("0x02", 1, 6000000000, 100),
                ("0x02", 3, 6000000200, 80),
            ):
                reader._parse_line(
                    f"{base} part={part} epoch_id={epoch} uptime_ms={count * 1000} "
                    f"count={count} wall_us_sum={total} wall_us_max={maximum}"
                )
        pipeline = _build_port_json(stats)["pipeline"]
        self.assertEqual(len(pipeline), len(parts) * 2)
        for part in parts:
            for epoch, delta_count, delta_sum, last_max in (
                ("0x01", 4, 500, 700), ("0x02", 2, 200, 80)
            ):
                row = pipeline[f"esp:cpu_span:na:node_mac=aa:bb:epoch_id={epoch}:part={part}"]
                self.assertEqual(row["delta"], {"count": delta_count, "wall_us_sum": delta_sum})
                self.assertEqual(row["last"]["wall_us_max"], last_max)
                self.assertNotIn("wall_us_max", row["reset_epochs"])

    def test_cpu_tasks_use_epoch_mac_id_and_handle_not_name_hint(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        def task(epoch, mac, task_id, handle, hint, runtime, delta, affinity=-1):
            return (
                f"PIPE v=1 dev=esp stage=cpu part=task epoch_id={epoch} node_mac={mac} "
                f"task_id={task_id} task_handle={handle} name_hint={hint} "
                f"priority=7 affinity={affinity} runtime_us={runtime} "
                f"delta_us={delta} interval_us=1000000 cpu_permille={delta // 1000}"
            )

        cases = (
            ("0x01", "aa:bb", "0x00000001", "0x1234", "audio", 5000000000, 200000),
            ("0x01", "aa:bb", "0x00000001", "0x1234", "renamed", 5000300000, 300000),
            ("0x01", "aa:bb", "0x00000002", "0x1234", "audio", 100000, 100000),
            ("0x01", "aa:bb", "0x00000001", "0x5678", "audio", 200000, 200000),
            ("0x02", "aa:bb", "0x00000001", "0x1234", "audio", 300000, 300000),
            ("0x01", "cc:dd", "0x00000001", "0x1234", "audio", 400000, 400000),
        )
        for args in cases:
            reader._parse_line(task(*args))
        pipeline = _build_port_json(stats)["pipeline"]
        self.assertEqual(len(pipeline), 5)
        key = "esp:cpu:na:node_mac=aa:bb:epoch_id=0x01:part=task:task_id=0x00000001:task_handle=0x1234"
        self.assertEqual(stats.pipe_samples[key], 2)
        self.assertEqual(stats.pipe_identity[key]["name_hint"], "renamed")
        self.assertEqual(stats.pipe_identity[key]["task_id"], "0x00000001")
        self.assertEqual(stats.pipe_identity[key]["task_handle"], "0x1234")
        self.assertEqual(pipeline[key]["delta"], {"runtime_us": 300000})
        self.assertEqual(pipeline[key]["last"]["affinity"], -1)
        self.assertEqual(pipeline[key]["last"]["cpu_permille"], 300)
        self.assertNotIn("cumulative", pipeline[key]["last"])
        for row in pipeline.values():
            self.assertEqual(row["reset_epochs"], {})

    def test_cpu_sparse_top_tasks_do_not_imply_zero_utilization(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=cpu part=task epoch_id=0x1 node_mac=aa:bb"
        for task_id, runtime, affinity in (("0x01", 5000000000, 2147483647),
                                           ("0x02", 9000000000, -1),
                                           ("0x01", 5000000050, 2147483647)):
            reader._parse_line(
                f"{meta} task_id={task_id} task_handle=0x1234 name_hint=unknown "
                f"runtime_us={runtime} affinity={affinity} delta_us=50 cpu_permille=5"
            )
        pipeline = _build_port_json(stats)["pipeline"]
        missing = next(row for key, row in pipeline.items() if "task_id=0x02" in key)
        self.assertEqual(missing["delta"], {})
        self.assertEqual(missing["last"]["affinity"], -1)
        self.assertEqual(next(row for key, row in pipeline.items() if "task_id=0x01" in key)["delta"],
                         {"runtime_us": 50})

    def test_cpu_task_requires_both_hex_identity_fields(self):
        for fields in ("task_id=0x1", "task_handle=0x1", "task_id=1 task_handle=0x2"):
            self.assertIsNone(parse_pipeline_logfmt(
                f"PIPE v=1 dev=esp stage=cpu part=task {fields} runtime_us=10"
            ))

    def test_espnow_split_parts_are_independent_measured_intervals(self):
        stats = PortStats(port="sender")
        reader = PortReader("sender", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=espnow epoch_id=0x1234 node_mac=aa:bb:cc:dd:ee:ff node_id=1 role=coordinator"
        for uptime, sent, received in ((1000, 10, 3), (3000, 30, 8)):
            reader._parse_line(f"{meta} uptime_ms={uptime} part=tx tx_offer={sent} tx_radio_ok={sent} tx_submit_err={sent // 10} tx_queue_full=1 tx_depth={sent // 10} tx_inflight=1")
            reader._parse_line(f"{meta} uptime_ms={uptime} part=rx rx_audio_accept={received} rx_deliver={received} seq_gap={received} jitter_depth=2")
        rows = _build_port_json(stats)["esp_stage_rows"]
        self.assertEqual(len(rows), 2)
        tx = next(row for row in rows if row["identity"]["part"] == "tx")
        rx = next(row for row in rows if row["identity"]["part"] == "rx")
        self.assertEqual(tx["samples"], 2)
        self.assertEqual(tx["elapsed_ms"], 2000)
        self.assertEqual(tx["rates_per_s"]["tx_radio_ok"], 10.0)
        self.assertEqual(tx["counters"]["attempts_filters"]["tx_submit_err"], 2)
        self.assertNotIn("tx_submit_err", tx["counters"]["discards"])
        self.assertEqual(rx["counters"]["frames"]["rx_audio_accept"], 5)
        self.assertEqual(rx["counters"]["diagnostics"]["seq_gap"], 5)
        for row in rows:
            self.assertNotIn("uptime_ms", _build_port_json(stats)["pipeline"][row["series"]]["delta"])
            self.assertNotIn("node_id", _build_port_json(stats)["pipeline"][row["series"]]["delta"])
            self.assertNotIn("tx_depth", _build_port_json(stats)["pipeline"][row["series"]]["delta"])
        self.assertIn("Air loss unknown", "\n".join(_report_lines_for_port(stats, 60)))

    def test_espnow_timing_counters_delta_but_maxima_remain_gauges(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = ("PIPE v=1 dev=esp stage=espnow_timing epoch_id=0x1234 "
                "node_mac=aa:bb:cc:dd:ee:ff node_id=1 role=coordinator")
        for uptime, count, total, maximum, depth in (
            (1000, 10, 500, 80, 3), (2000, 20, 1400, 110, 4)
        ):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} part=rx rx_dequeue_age_count={count} "
                f"rx_dequeue_age_us_sum={total} rx_dequeue_age_us_max={maximum} "
                f"jitter_expired_age_us_max={maximum} "
                f"jitter_expired_with_pending_count={count}"
            )
            reader._parse_line(
                f"{meta} uptime_ms={uptime} part=tx tx_esp_now_send_count={count} "
                f"tx_esp_now_send_us_sum={total} tx_esp_now_send_us_max={maximum} "
                f"tx_slot_late_count={count} tx_depth_high_water={depth}"
            )
        pipeline = _build_port_json(stats)["pipeline"]
        rx = next(row for key, row in pipeline.items() if key.endswith(":part=rx"))
        tx = next(row for key, row in pipeline.items() if key.endswith(":part=tx"))
        self.assertEqual(rx["delta"]["rx_dequeue_age_count"], 10)
        self.assertEqual(rx["delta"]["rx_dequeue_age_us_sum"], 900)
        self.assertEqual(rx["delta"]["jitter_expired_with_pending_count"], 10)
        self.assertEqual(tx["delta"]["tx_esp_now_send_count"], 10)
        self.assertEqual(tx["delta"]["tx_slot_late_count"], 10)
        for row, maxima in ((rx, ("rx_dequeue_age_us_max", "jitter_expired_age_us_max")),
                            (tx, ("tx_esp_now_send_us_max", "tx_depth_high_water"))):
            for key in maxima:
                self.assertNotIn(key, row["delta"])
                self.assertNotIn(key, row["reset_epochs"])
        self.assertEqual(tx["last"]["tx_depth_high_water"], 4)

    def test_espnow_radio_timing_maxima_are_gauges(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=espnow_timing epoch_id=0x1234 part=radio"
        for uptime, count, total, maximum in ((1000, 2, 500, 300),
                                               (2000, 5, 1800, 700)):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} origin_complete_count={count} "
                f"origin_complete_us_sum={total} origin_complete_us_max={maximum} "
                f"control_complete_count={count} control_complete_us_sum={total} "
                f"control_complete_us_max={maximum} tx_busy_audio_count={count} "
                f"tx_busy_control_count={count} tx_busy_age_us_max={maximum}"
            )
        row = next(value for key, value in _build_port_json(stats)["pipeline"].items()
                   if key.endswith(":part=radio"))
        for key in ("origin_complete_us_max", "control_complete_us_max",
                    "tx_busy_age_us_max"):
            self.assertEqual(row["last"][key], 700)
            self.assertNotIn(key, row["delta"])
            self.assertNotIn(key, row["reset_epochs"])
        for key in ("origin_complete_count", "control_complete_count",
                    "tx_busy_audio_count", "tx_busy_control_count"):
            self.assertEqual(row["delta"][key], 3)
        self.assertEqual(row["delta"]["origin_complete_us_sum"], 1300)

    def test_notification_timing_counts_and_sums_delta_but_maxima_are_gauges(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=audio_timing part=notify epoch_id=0x1234"
        for uptime, count, total, maximum in (
            (1000, 1, 100, 25), (2000, 3, 400, 80)
        ):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} notify_started_count={count} "
                f"notify_completed_count={count} notify_mix_count={count} "
                f"notify_mix_us_sum={total} notify_mix_us_max={maximum} "
                f"notify_frame_gap_count={count} notify_frame_gap_us_sum={total} "
                f"notify_frame_gap_us_max={maximum} notify_frame_gap_over25ms_count={count} "
                f"notify_work_count={count} notify_work_us_sum={total} "
                f"notify_work_us_max={maximum} notify_write_count={count} "
                f"notify_write_us_sum={total} notify_write_us_max={maximum} "
                f"notify_write_gap_count={count} notify_write_gap_us_sum={total} "
                f"notify_write_gap_us_max={maximum}"
            )
        pipeline = next(
            value for key, value in _build_port_json(stats)["pipeline"].items()
            if key.startswith("esp:audio_timing:") and key.endswith(":part=notify")
        )
        for key in (
            "notify_started_count", "notify_completed_count", "notify_mix_count",
            "notify_frame_gap_count", "notify_frame_gap_over25ms_count",
            "notify_work_count", "notify_write_count", "notify_write_gap_count",
        ):
            self.assertEqual(pipeline["delta"][key], 2)
        for key in (
            "notify_mix_us_sum", "notify_frame_gap_us_sum", "notify_work_us_sum",
            "notify_write_us_sum", "notify_write_gap_us_sum",
        ):
            self.assertEqual(pipeline["delta"][key], 300)
        for key in (
            "notify_mix_us_max", "notify_frame_gap_us_max", "notify_work_us_max",
            "notify_write_us_max", "notify_write_gap_us_max",
        ):
            self.assertEqual(pipeline["last"][key], 80)
            self.assertNotIn(key, pipeline["delta"])
            self.assertNotIn(key, pipeline["reset_epochs"])

    def test_send_errors_distinguish_counters_from_heap_and_last_error_gauges(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=espnow_timing part=errors epoch_id=0x1234"
        for uptime, no_mem, other, last, free, snapshot_time, snapshot_error in (
            (1000, 1, 0, 257, 30000, 990, 257),
            (2000, 2, 0, 257, 28000, 990, 257),
            (3000, 3, 1, 12345, 25000, 2990, 12345),
        ):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} send_err_nomem={no_mem} "
                f"send_err_other={other} send_last_error={last} "
                f"send_last_error_uptime_ms={uptime - 10} "
                f"send_heap_snapshot_uptime_ms={snapshot_time} "
                f"send_heap_snapshot_error={snapshot_error} send_heap_snapshot_valid=1 "
                f"internal_8bit_free={free} "
                f"internal_8bit_largest={free // 2} internal_8bit_min={free - 1000} "
                f"send_error_internal_free={30000 if uptime == 2000 else free} "
                f"send_error_internal_largest={(30000 if uptime == 2000 else free) // 2} "
                f"send_error_internal_min={(30000 if uptime == 2000 else free) - 1000}"
            )
            if uptime == 2000:
                same_code = next(
                    value for key, value in _build_port_json(stats)["pipeline"].items()
                    if key.endswith(":part=errors")
                )["last"]
                self.assertEqual(same_code["send_last_error_uptime_ms"], 1990)
                self.assertEqual(same_code["send_heap_snapshot_uptime_ms"], 990)
                self.assertEqual(same_code["send_error_internal_free"], 30000)
        row = next(value for key, value in _build_port_json(stats)["pipeline"].items()
                   if key.endswith(":part=errors"))
        self.assertEqual(row["delta"]["send_err_nomem"], 2)
        self.assertEqual(row["delta"]["send_err_other"], 1)
        self.assertEqual(row["last"]["send_heap_snapshot_uptime_ms"], 2990)
        for key in ("send_last_error", "send_last_error_uptime_ms",
                    "send_heap_snapshot_uptime_ms", "send_heap_snapshot_error",
                    "send_heap_snapshot_valid", "internal_8bit_free",
                    "internal_8bit_largest", "internal_8bit_min", "send_error_internal_free",
                    "send_error_internal_largest", "send_error_internal_min"):
            self.assertNotIn(key, row["delta"])
            self.assertNotIn(key, row["reset_epochs"])

    def test_music_timing_maxima_and_format_are_gauges(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=audio_timing part=music epoch_id=0x1234"
        for uptime, count, total, maximum, rate in (
            (1000, 2, 100, 80, 44100), (2000, 5, 450, 150, 48000)
        ):
            reader._parse_line(
                f"{meta} uptime_ms={uptime} " + " ".join(
                    f"music_{part}_count={count} music_{part}_us_sum={total} "
                    f"music_{part}_us_max={maximum}"
                    for part in ("mutex_wait", "route_read", "convert", "mix")
                ) + f" music_render_over20ms_count={count} music_chunks_count={count} "
                f"music_iterations_count={count} music_input_frames={total} "
                f"music_output_frames={total} music_rate_hz={rate} music_channels=2 "
                "music_format_valid=1"
            )
        row = next(value for key, value in _build_port_json(stats)["pipeline"].items()
                   if key.endswith(":part=music"))
        self.assertEqual(row["delta"]["music_convert_us_sum"], 350)
        self.assertEqual(row["delta"]["music_chunks_count"], 3)
        for key in ("music_mutex_wait_us_max", "music_route_read_us_max",
                    "music_convert_us_max", "music_mix_us_max", "music_rate_hz",
                    "music_channels", "music_format_valid"):
            self.assertNotIn(key, row["delta"])
            self.assertNotIn(key, row["reset_epochs"])

    def test_esp_epochs_are_separate_even_if_new_boot_counters_increase(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        for epoch, uptime, count in (("0x00000001", 100, 10), ("0x00000001", 1100, 20),
                                      ("0x00000002", 50, 100), ("0x00000002", 1050, 104)):
            reader._parse_line(f"PIPE v=1 dev=esp stage=audio epoch_id={epoch} uptime_ms={uptime} encode_ok={count} rx_store_depth=3 rx_store_depth_valid=1")
        rows = _build_port_json(stats)["esp_stage_rows"]
        self.assertEqual([r["counters"]["frames"]["encode_ok"] for r in rows], [10, 4])
        self.assertEqual([r["elapsed_ms"] for r in rows], [1000, 1000])
        for row in rows:
            self.assertEqual(row["gauges"]["rx_store_depth_valid"], 1)
            self.assertNotIn("rx_store_depth_valid", _build_port_json(stats)["pipeline"][row["series"]]["delta"])

    def test_esp_single_sample_missing_stage_and_reset_are_unavailable(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        reader._parse_line("PIPE v=1 dev=esp stage=audio epoch_id=0x42 uptime_ms=100 encode_ok=5")
        meta = "PIPE v=1 dev=esp stage=espnow epoch_id=0x43 part=tx"
        reader._parse_line(f"{meta} uptime_ms=100 tx_radio_ok=10")
        reader._parse_line(f"{meta} uptime_ms=200 tx_radio_ok=2")
        rows = _build_port_json(stats)["esp_stage_rows"]
        self.assertEqual([row["status"] for row in rows], ["unavailable", "unavailable"])
        self.assertTrue(all(all(rate is None for rate in row["rates_per_s"].values()) for row in rows))

    def test_gauge_only_stage_window_is_unavailable(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=espnow epoch_id=0x43 part=rx"
        reader._parse_line(f"{meta} uptime_ms=100 jitter_depth=2")
        reader._parse_line(f"{meta} uptime_ms=200 jitter_depth=3")
        row = _build_port_json(stats)["esp_stage_rows"][0]
        self.assertEqual(row["status"], "unavailable")
        self.assertEqual(row["gauges"]["jitter_depth"], 3)
        self.assertFalse(any(row["counters"].values()))

    def test_sparse_frame_counters_never_use_wider_stage_uptime(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=audio part=tx epoch_id=0x42"
        reader._parse_line(f"{meta} uptime_ms=1000 capture_ok=10 encode_ok=10")
        reader._parse_line(f"{meta} uptime_ms=2000 capture_ok=20 encode_ok=20 tx_handoff=20")
        reader._parse_line(f"{meta} uptime_ms=3000 capture_ok=30 tx_handoff=25")
        row = _build_port_json(stats)["esp_stage_rows"][0]
        self.assertEqual(row["status"], "measured")
        self.assertEqual(row["elapsed_ms"], 2000)
        self.assertEqual(row["rates_per_s"]["capture_ok"], 10.0)
        self.assertIsNone(row["rates_per_s"]["encode_ok"])
        self.assertIsNone(row["rates_per_s"]["tx_handoff"])
        self.assertNotIn("encode_ok", row["counters"]["frames"])
        self.assertNotIn("tx_handoff", row["counters"]["frames"])
        self.assertIn("encode_ok=n/a", "\n".join(_report_lines_for_port(stats, 60)))

    def test_missing_middle_and_reset_counters_do_not_hide_valid_rates(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=audio part=playout epoch_id=0x42"
        reader._parse_line(f"{meta} uptime_ms=1000 decode_ok=10 play_ok=100 plc=4")
        reader._parse_line(f"{meta} uptime_ms=2000 decode_ok=20 play_ok=110")
        reader._parse_line(f"{meta} uptime_ms=3000 decode_ok=30 play_ok=2 plc=7")
        row = _build_port_json(stats)["esp_stage_rows"][0]
        self.assertEqual(row["rates_per_s"]["decode_ok"], 10.0)
        self.assertIsNone(row["rates_per_s"]["play_ok"])
        self.assertNotIn("plc", row["counters"]["diagnostics"])

    def test_audio_parts_and_espnow_rx_drop_classification(self):
        stats = PortStats(port="one")
        reader = PortReader("one", 115200, None, "unused", stats)
        audio = "PIPE v=1 dev=esp stage=audio epoch_id=0x42"
        mesh = "PIPE v=1 dev=esp stage=espnow epoch_id=0x99 part=rx"
        for uptime, value in ((1000, 2), (2000, 5)):
            reader._parse_line(f"{audio} part=tx uptime_ms={uptime} capture_fifo_discard_samples={value} encode_ok={value}")
            reader._parse_line(f"{audio} part=rx uptime_ms={uptime} rx_store_reject={value} rx_src_evict={value} rx_store_ok={value}")
            reader._parse_line(f"{audio} part=playout uptime_ms={uptime} decode_ok={value}")
            reader._parse_line(f"{audio} part=bt uptime_ms={uptime} bt_music_overflow={value}")
            reader._parse_line(f"{mesh} uptime_ms={uptime} rx_audio_purge={value} jitter_late={value} control_queue_drops={value} rx_queue_overflows={value} rx_audio_queue_full={value}")
        rows = _build_port_json(stats)["esp_stage_rows"]
        self.assertEqual(len(rows), 5)
        audio_rx = next(r for r in rows if r["identity"].get("part") == "rx" and r["identity"]["stage"] == "audio")
        mesh_rx = next(r for r in rows if r["identity"]["stage"] == "espnow")
        self.assertEqual(audio_rx["counters"]["discards"]["rx_store_reject"], 3)
        self.assertEqual(audio_rx["counters"]["diagnostics"]["rx_src_evict"], 3)
        self.assertNotIn("rx_src_evict", audio_rx["counters"]["discards"])
        self.assertEqual(mesh_rx["counters"]["discards"]["rx_audio_purge"], 3)
        self.assertEqual(mesh_rx["counters"]["discards"]["jitter_late"], 3)
        for key in ("control_queue_drops", "rx_queue_overflows"):
            self.assertEqual(mesh_rx["counters"]["diagnostics"][key], 3)
            self.assertNotIn(key, mesh_rx["counters"]["discards"])
        tx = next(r for r in rows if r["identity"].get("part") == "tx")
        self.assertEqual(tx["counters"]["diagnostics"]["capture_fifo_discard_samples"], 3)
        self.assertNotIn("capture_fifo_discard_samples", tx["counters"]["discards"])
        report = "\n".join(_report_lines_for_port(stats, 60))
        self.assertIn("counts samples, not frames", report)
        self.assertIn("queue diagnostics can overlap", report)

    def test_measured_esp_discards_warn_in_overall_health_without_adding_overlap(self):
        stats = PortStats(port="one", open_ok=True, lines=4)
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=espnow epoch_id=0x23"
        for uptime, queue, late in ((1000, 5, 10), (2000, 12, 29)):
            reader._parse_line(f"{meta} part=tx uptime_ms={uptime} tx_queue_full={queue} tx_submit_err={queue}")
            reader._parse_line(f"{meta} part=rx uptime_ms={uptime} jitter_late={late} rx_audio_accept={late}")
        health = _overall_health(stats)
        self.assertIn("WARN", health)
        self.assertIn("tx_queue_full+7", health)
        self.assertIn("jitter_late+19", health)
        self.assertNotIn("tx_submit_err", health)
        self.assertIn(f"Health: {health}", "\n".join(_report_lines_for_port(stats, 10)))
        self.assertEqual(_build_port_json(stats)["health"], health)
        self.assertEqual(len(_build_port_json(stats)["esp_discard_health"]["warnings"]), 2)

    def test_retry_only_and_unavailable_esp_discards_do_not_claim_health(self):
        retry = PortStats(port="retry", open_ok=True, lines=2)
        reader = PortReader("retry", 115200, None, "unused", retry)
        for uptime, attempts in ((1000, 2), (2000, 9)):
            reader._parse_line(f"PIPE v=1 dev=esp stage=espnow epoch_id=0x24 part=tx uptime_ms={uptime} tx_submit_err={attempts} tx_queue_full=4")
        self.assertEqual(_build_port_json(retry)["esp_discard_health"]["warnings"], [])
        self.assertNotIn("WARN", _overall_health(retry))
        self.assertNotIn("tx_submit_err", _overall_health(retry))

        single = PortStats(port="single", open_ok=True, lines=1)
        PortReader("single", 115200, None, "unused", single)._parse_line(
            "PIPE v=1 dev=esp stage=espnow epoch_id=0x25 part=rx uptime_ms=100 jitter_late=999"
        )
        self.assertEqual(_build_port_json(single)["esp_discard_health"]["warnings"], [])
        self.assertIn("UNKNOWN", _overall_health(single))
        self.assertIn("telemetry unavailable", _overall_health(single))

    def test_missing_split_part_is_unknown_with_zero_measured_discards(self):
        stats = PortStats(port="one", open_ok=True, lines=4)
        reader = PortReader("one", 115200, None, "unused", stats)
        for uptime in (1000, 2000):
            reader._parse_line(f"PIPE v=1 dev=esp stage=audio part=tx epoch_id=0x12 uptime_ms={uptime} tx_no_cb=0")
            reader._parse_line(f"PIPE v=1 dev=esp stage=espnow part=tx epoch_id=0x34 node_mac=aa:bb uptime_ms={uptime} tx_queue_full=0")
        health = _overall_health(stats)
        self.assertTrue(health.startswith("UNKNOWN ("))
        unavailable = _build_port_json(stats)["esp_discard_health"]["unavailable"]
        self.assertIn("audio/rx epoch=0x12 (missing part)", unavailable)
        self.assertIn("audio/bt epoch=0x12 (missing part)", unavailable)
        self.assertIn("espnow/rx epoch=0x34 mac=aa:bb (missing part)", unavailable)
        self.assertNotIn("espnow/rx epoch=0x12", str(unavailable))

    def test_bt_overflow_warns_in_pcm_units_but_underrun_does_not(self):
        stats = PortStats(port="one", open_ok=True, lines=2)
        reader = PortReader("one", 115200, None, "unused", stats)
        meta = "PIPE v=1 dev=esp stage=audio part=bt epoch_id=0x55"
        reader._parse_line(f"{meta} uptime_ms=1000 bt_music_overflow=10 bt_call_overflow=1 bt_mic_overflow=20 bt_music_underrun=1")
        reader._parse_line(f"{meta} uptime_ms=2000 bt_music_overflow=14 bt_call_overflow=1 bt_mic_overflow=23 bt_music_underrun=9")
        health = _overall_health(stats)
        self.assertIn("WARN (", health)
        self.assertIn("bt_music_overflow+4", health)
        self.assertIn("bt_mic_overflow+3", health)
        self.assertNotIn("bt_music_underrun", health)
        self.assertIn("rejected PCM frames", "\n".join(_report_lines_for_port(stats, 1)))

    def test_incomplete_esp_telemetry_preserves_legacy_warn_and_fail(self):
        stats = PortStats(port="one", open_ok=True, lines=2)
        reader = PortReader("one", 115200, None, "unused", stats)
        reader._parse_line("[MESH] r=1 id=2 sl=1 tx=10(err=0) rx=5 drop=1 fwd=2 | spi_in=10 overwr=0 starve=0 drain=8 q=0")
        reader._parse_line("[MESH] r=1 id=2 sl=1 tx=12(err=0) rx=7 drop=2 fwd=3 | spi_in=12 overwr=0 starve=0 drain=11 q=0")
        reader._parse_line("PIPE v=1 dev=esp stage=espnow part=tx epoch_id=0x01 uptime_ms=100 tx_queue_full=0")
        self.assertIn("WARN (mesh_drop+1)", _overall_health(stats))
        self.assertIn("telemetry unavailable", _overall_health(stats))
        stats.open_ok = False
        self.assertTrue(_overall_health(stats).startswith("FAIL (port could not be opened)"))

    def test_legacy_unsplit_audio_does_not_require_split_parts(self):
        stats = PortStats(port="one", open_ok=True, lines=2)
        reader = PortReader("one", 115200, None, "unused", stats)
        for uptime in (1000, 2000):
            reader._parse_line(f"PIPE v=1 dev=esp stage=audio epoch_id=0x11 uptime_ms={uptime} rx_store_reject=0")
        self.assertEqual(_build_port_json(stats)["esp_discard_health"]["unavailable"], [])
        self.assertTrue(_overall_health(stats).startswith("OK ("))

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
        legacy_reader._parse_line("[E2E_ESP] tx=10 rx=8 gap_evt=1 gap_fr=2 reset_evt=0")
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
        self.assertEqual(
            _health_line(repaired), "OK (no error/drops/CRC growth observed)"
        )

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
                    self.assertIn(f"e2e_esp_effective_gap+{expected_effective}", health)

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
            reader._parse_line(f"PIPE v=1 dev=nrf stage=mesh node=3 ingress_ok={value}")

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


class PortReaderLifecycleTest(unittest.TestCase):
    @patch("benchtool.capture.serial.Serial")
    @patch("benchtool.capture._reconnect_candidates")
    @patch("benchtool.capture._serial_identity", return_value=("id", 1, 2))
    def test_reconnects_using_candidates_in_order(self, _, candidates, serial):
        stop_event = Mock()
        stop_event.is_set.side_effect = [False, False, False, True]
        first_connection = Mock()
        first_connection.readline.side_effect = OSError("disconnected")
        second_connection = Mock()
        second_connection.readline.return_value = b""
        candidates.return_value = ["/dev/ttyACM0", "/dev/ttyACM1"]
        serial.side_effect = [
            OSError("missing"),
            OSError("unavailable"),
            first_connection,
            second_connection,
        ]
        stats = PortStats(port="/dev/ttyACM0")

        def record_wait(timeout):
            if timeout == 0.25:
                self.assertEqual(stats.open_error, "Open error: unavailable")

        stop_event.wait.side_effect = record_wait
        with TemporaryDirectory() as directory:
            PortReader(
                "/dev/ttyACM0", 115200, stop_event, f"{directory}/port.log", stats
            ).run()
        self.assertEqual(
            [serial_call.args[0] for serial_call in serial.call_args_list],
            [
                "/dev/ttyACM0",
                "/dev/ttyACM1",
                "/dev/ttyACM0",
                "/dev/ttyACM0",
            ],
        )
        self.assertTrue(stats.open_ok)
        self.assertIsNone(stats.open_error)
        self.assertEqual(stats.reconnects, 1)
        self.assertEqual(stop_event.wait.call_args_list, [call(0.25), call(0.1)])
        first_connection.close.assert_called_once_with()
        second_connection.close.assert_called_once_with()

    @patch("benchtool.capture.serial.Serial")
    @patch("benchtool.capture._serial_identity", return_value=(None, None, None))
    def test_ignores_os_error_when_closing_after_read_failure(self, _, serial):
        stop_event = Mock()
        stop_event.is_set.side_effect = [False, True]
        connection = Mock()
        connection.readline.side_effect = OSError("disconnected")
        connection.close.side_effect = OSError("close failed")
        serial.return_value = connection
        stats = PortStats(port="/dev/ttyACM0")

        with TemporaryDirectory() as directory:
            PortReader(
                "/dev/ttyACM0", 115200, stop_event, f"{directory}/port.log", stats
            ).run()

        self.assertEqual(stats.open_error, "Read error: disconnected")
        connection.close.assert_called_once_with()
        stop_event.wait.assert_called_once_with(0.1)


if __name__ == "__main__":
    unittest.main()
