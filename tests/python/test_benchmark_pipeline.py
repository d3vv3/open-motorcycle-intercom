import unittest
from types import SimpleNamespace
from unittest.mock import patch

from benchmark import (
    PortReader,
    PortStats,
    _build_port_json,
    _reconnect_candidates,
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
        self.assertIsNone(parse_pipeline_logfmt("ordinary firmware log"))
        self.assertIsNone(parse_pipeline_logfmt("PIPE v=1 dev=nrf stage=rf tx_ok=-1"))

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

    @patch("benchmark.list_ports.comports")
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
