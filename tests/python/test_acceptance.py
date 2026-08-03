import argparse
import copy
import io
import json
import os
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import ANY, call, patch

from scripts import acceptance


class AcceptanceAssertionTest(unittest.TestCase):
    def setUp(self):
        self.args = argparse.Namespace(max_glitches=2, max_effective_gap_pct=0.75)
        self.port = "/dev/esp0"

    def passing_entry(self, port=None):
        return {
            "port": port or self.port,
            "open_ok": True,
            "glitch_delta": {"glitches": 2},
            "hop_pct": {"esp_e2e_effective_gap_pct": 0.75},
            "pipeline": {
                "esp:audio:na": {"delta": {"pcm_underrun": 0}},
            },
        }

    def check(self, summary, ports=None):
        with tempfile.TemporaryDirectory() as run_dir:
            with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as fh:
                json.dump(summary, fh)
            output = io.StringIO()
            with redirect_stdout(output):
                result = acceptance.assert_run(
                    run_dir, ports if ports is not None else [self.port], self.args
                )
        return result, output.getvalue()

    def assert_fails_closed(self, summary, ports=None):
        result, output = self.check(summary, ports)
        self.assertFalse(result)
        self.assertIn("FAIL", output)
        self.assertNotIn("SKIP", output)

    def test_full_passing_fixture(self):
        second = self.passing_entry("/dev/esp1")
        result, output = self.check(
            {"ports": [self.passing_entry(), second]},
            [self.port, "/dev/esp1"],
        )
        self.assertTrue(result)
        self.assertIn("RESULT: PASS", output)
        self.assertNotIn("FAIL", output)

    def test_missing_required_port_fails(self):
        self.assert_fails_closed({"ports": []})

    def test_open_false_fails(self):
        entry = self.passing_entry()
        entry["open_ok"] = False
        self.assert_fails_closed({"ports": [entry]})

    def test_each_missing_metric_fails(self):
        paths = (
            ("glitch_delta", "glitches"),
            ("hop_pct", "esp_e2e_effective_gap_pct"),
            ("pipeline", "esp:audio:na", "delta", "pcm_underrun"),
        )
        for path in paths:
            with self.subTest(path=path):
                entry = self.passing_entry()
                target = entry
                for key in path[:-1]:
                    target = target[key]
                del target[path[-1]]
                self.assert_fails_closed({"ports": [entry]})

    def test_invalid_metric_values_fail(self):
        invalid_values = ("0", None, True, float("nan"), float("inf"), float("-inf"))
        metric_paths = (
            ("glitch_delta", "glitches"),
            ("hop_pct", "esp_e2e_effective_gap_pct"),
            ("pipeline", "esp:audio:na", "delta", "pcm_underrun"),
        )
        for path in metric_paths:
            for value in invalid_values:
                with self.subTest(path=path, value=value):
                    entry = self.passing_entry()
                    target = entry
                    for key in path[:-1]:
                        target = target[key]
                    target[path[-1]] = value
                    self.assert_fails_closed({"ports": [entry]})

    def test_negative_metrics_fail(self):
        for path, value in (
            (("glitch_delta", "glitches"), -1),
            (("hop_pct", "esp_e2e_effective_gap_pct"), -0.01),
        ):
            with self.subTest(path=path):
                entry = self.passing_entry()
                target = entry
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
                self.assert_fails_closed({"ports": [entry]})

    def test_malformed_metric_containers_fail(self):
        for key, value in (("glitch_delta", []), ("hop_pct", "bad")):
            with self.subTest(key=key):
                entry = self.passing_entry()
                entry[key] = value
                self.assert_fails_closed({"ports": [entry]})

    def test_fractional_integer_metrics_fail(self):
        for path in (
            ("glitch_delta", "glitches"),
            ("pipeline", "esp:audio:na", "delta", "pcm_underrun"),
        ):
            with self.subTest(path=path):
                entry = self.passing_entry()
                target = entry
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = 0.5
                self.assert_fails_closed({"ports": [entry]})

    def test_threshold_equality_and_below_pass_but_exceed_fails(self):
        passing_values = ((2, 0.75), (1, 0.5), (2.0, 0.75))
        for glitches, gap in passing_values:
            with self.subTest(glitches=glitches, gap=gap):
                entry = self.passing_entry()
                entry["glitch_delta"]["glitches"] = glitches
                entry["hop_pct"]["esp_e2e_effective_gap_pct"] = gap
                self.assertTrue(self.check({"ports": [entry]})[0])

        for glitches, gap in ((3, 0.75), (2, 0.751)):
            with self.subTest(glitches=glitches, gap=gap):
                entry = self.passing_entry()
                entry["glitch_delta"]["glitches"] = glitches
                entry["hop_pct"]["esp_e2e_effective_gap_pct"] = gap
                self.assert_fails_closed({"ports": [entry]})

    def test_multiple_pcm_pipelines_are_summed(self):
        entry = self.passing_entry()
        entry["pipeline"] = {
            "first": {"delta": {"pcm_underrun": 0}},
            "second": {"delta": {"pcm_underrun": 0}},
            "other": {"delta": {"another_metric": 4}},
        }
        self.assertTrue(self.check({"ports": [entry]})[0])

        failing = copy.deepcopy(entry)
        failing["pipeline"]["second"]["delta"]["pcm_underrun"] = 1
        self.assert_fails_closed({"ports": [failing]})

        negative = copy.deepcopy(entry)
        negative["pipeline"]["second"]["delta"]["pcm_underrun"] = -1
        self.assert_fails_closed({"ports": [negative]})

    def test_malformed_pipeline_shapes_fail_without_crashing(self):
        malformed = (
            [],
            {"audio": []},
            {"audio": {"delta": []}},
            {"audio": {"delta": {"pcm_underrun": 0}}, "bad": None},
        )
        for pipeline in malformed:
            with self.subTest(pipeline=pipeline):
                entry = self.passing_entry()
                entry["pipeline"] = pipeline
                self.assert_fails_closed({"ports": [entry]})

    def test_malformed_summary_shapes_fail_without_crashing(self):
        malformed = (
            None,
            [],
            {},
            {"ports": {}},
            {"ports": ["bad"]},
            {"ports": [{"port": []}]},
        )
        for summary in malformed:
            with self.subTest(summary=summary):
                self.assert_fails_closed(summary)

    def test_missing_or_invalid_summary_json_fails_without_crashing(self):
        for contents in (None, "{not json"):
            with self.subTest(contents=contents):
                with tempfile.TemporaryDirectory() as run_dir:
                    if contents is not None:
                        with open(
                            os.path.join(run_dir, "summary.json"),
                            "w",
                            encoding="utf-8",
                        ) as fh:
                            fh.write(contents)
                    output = io.StringIO()
                    with redirect_stdout(output):
                        result = acceptance.assert_run(
                            run_dir, [self.port], self.args
                        )
                self.assertFalse(result)
                self.assertIn("FAIL", output.getvalue())
                self.assertNotIn("SKIP", output.getvalue())


class ForceTxRestorationTest(unittest.TestCase):
    def run_main_with_failure(self, flash_side_effect, benchmark_side_effect=None):
        argv = [
            "acceptance.py",
            "--force-tx",
            "on",
            "--esp-ports",
            "esp0",
            "esp1",
        ]
        with (
            patch.object(acceptance.sys, "argv", argv),
            patch.object(acceptance, "set_force_tx_define") as set_define,
            patch.object(acceptance, "flash_esp", side_effect=flash_side_effect) as flash,
            patch.object(
                acceptance, "run_benchmark", side_effect=benchmark_side_effect
            ) as benchmark,
            patch.object(acceptance.time, "sleep"),
            redirect_stdout(io.StringIO()),
        ):
            with self.assertRaises(RuntimeError):
                acceptance.main()
        self.assertEqual(set_define.call_args_list, [call(1), call(0)])
        return flash.call_args_list, benchmark

    def test_restores_and_reflashes_after_benchmark_failure(self):
        flash_calls, benchmark = self.run_main_with_failure(
            [None, None, None, None], RuntimeError("benchmark failed")
        )
        self.assertEqual(
            flash_calls,
            [call("esp0", ANY), call("esp1", ANY),
             call("esp0", ANY), call("esp1", ANY)],
        )
        benchmark.assert_called_once()

    def test_restores_and_reflashes_after_first_initial_flash_failure(self):
        flash_calls, benchmark = self.run_main_with_failure(
            [RuntimeError("first flash failed"), None, None]
        )
        self.assertEqual(
            flash_calls,
            [call("esp0", ANY), call("esp0", ANY), call("esp1", ANY)],
        )
        benchmark.assert_not_called()

    def test_restores_and_reflashes_after_second_initial_flash_failure(self):
        flash_calls, benchmark = self.run_main_with_failure(
            [None, RuntimeError("second flash failed"), None, None]
        )
        self.assertEqual(
            flash_calls,
            [call("esp0", ANY), call("esp1", ANY),
             call("esp0", ANY), call("esp1", ANY)],
        )
        benchmark.assert_not_called()

    def test_restore_flash_failure_cannot_return_success(self):
        argv = [
            "acceptance.py",
            "--force-tx",
            "on",
            "--esp-ports",
            "esp0",
            "esp1",
        ]
        with (
            patch.object(acceptance.sys, "argv", argv),
            patch.object(acceptance, "set_force_tx_define"),
            patch.object(
                acceptance,
                "flash_esp",
                side_effect=[None, None, RuntimeError("restore flash failed"), None],
            ) as flash,
            patch.object(acceptance, "run_benchmark", return_value="run"),
            patch.object(acceptance, "assert_run", return_value=True),
            patch.object(acceptance.time, "sleep"),
            redirect_stdout(io.StringIO()),
        ):
            with self.assertRaisesRegex(RuntimeError, "failed to restore"):
                acceptance.main()
        self.assertEqual(
            flash.call_args_list,
            [call("esp0", ANY), call("esp1", ANY),
             call("esp0", ANY), call("esp1", ANY)],
        )


if __name__ == "__main__":
    unittest.main()
