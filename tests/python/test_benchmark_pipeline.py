import unittest

from benchmark import PortReader, PortStats, _build_port_json, parse_pipeline_logfmt


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


if __name__ == "__main__":
    unittest.main()
