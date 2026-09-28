#include "esb_rssi.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(esb_rssi_to_dbm(1) == -1);
    assert(esb_rssi_to_dbm(60) == -60);
    assert(esb_rssi_to_dbm(127) == -127);
    assert(esb_rssi_to_dbm(0) == INT8_MAX);
    assert(esb_rssi_to_dbm(-1) == INT8_MAX);
    assert(esb_rssi_to_dbm(-128) == INT8_MAX);
    assert(esb_rssi_to_dbm(128) == INT8_MAX);
    puts("ESB RSSI conversion tests passed");
    return 0;
}
