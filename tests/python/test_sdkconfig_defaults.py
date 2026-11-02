import re
from pathlib import Path

DEFAULTS = Path(__file__).resolve().parents[2] / "sdkconfig.defaults"


def test_rtc_fast_mem_is_not_used_as_heap():
    # RTC fast RAM must stay out of the heap (bench 2026-11-03). Removing this
    # setting lets the SPI/audio task stacks land in slow RTC fast RAM, which
    # caused ~40% S31 bridge RX loss and badly stuttering mesh audio while BT
    # A2DP music played. Conversely, the setting only fits because Opus is gone:
    # re-adding Opus or other large internal-RAM users can push ESP-NOW
    # esp_wifi_init back into ESP_ERR_NO_MEM (mesh.c, seen on phase 1/2).
    # Watch the boot "Internal heap after mesh/audio init" log line.
    text = DEFAULTS.read_text()
    active = [line.strip() for line in text.splitlines()
              if re.match(r"\s*CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP\s*=", line)]
    assert active == ["CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP=n"], active
