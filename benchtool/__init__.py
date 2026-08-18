"""Serial benchmark tooling for the OMI intercom rig.

Package layout:
- parsers: log-line regexes, key tables, and the PIPE logfmt parser
- series: pure delta/rollover math for cumulative counter series
- stats: per-port accumulator and derived metrics
- capture: serial port discovery, identity, and reader thread
- health: per-port health assessment rules
- report: report.txt, summary.json, and stdout rendering
"""
