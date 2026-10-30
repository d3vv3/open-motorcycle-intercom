import re
from pathlib import Path

DEFAULTS = Path(__file__).resolve().parents[2] / "sdkconfig.defaults"


def test_rtc_fast_mem_stays_available_as_heap():
    # RTC fast RAM is needed as heap for WiFi+BT coexistence on this C build;
    # see bench 2026-10-30. Disabling it made esp_wifi_init fail with
    # ESP_ERR_NO_MEM (mesh.c:128) and left ESP-NOW silent.
    text = DEFAULTS.read_text()
    active = [line.strip() for line in text.splitlines()
              if re.match(r"\s*CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP\s*=", line)]
    assert all(line.split("=", 1)[1].strip() == "y" for line in active), active
    assert not re.search(r"^\s*#\s*CONFIG_ESP_SYSTEM_ALLOW_RTC_FAST_MEM_AS_HEAP is not set",
                         text, re.MULTILINE)
