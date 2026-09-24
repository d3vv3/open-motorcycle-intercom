#include "shared/cpu_profile_metrics.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    cpu_profile_sample_t old = {0x1234, 10, UINT64_C(5000000000)};
    cpu_profile_sample_t now = {0x1234, 10, UINT64_C(5007500000)};
    uint64_t delta = 0;
    assert(cpu_profile_delta(&old, &now, 10000000, &delta));
    assert(delta == 7500000);
    assert(cpu_profile_permille(delta, 10000000, 1) == 750);
    assert(cpu_profile_permille(delta, 10000000, 2) == 375);
    assert(cpu_profile_permille(20000000, 10000000, 1) == 2000);
    assert(cpu_profile_permille(20000000, 10000000, 2) == 1000);
    assert(cpu_profile_permille(UINT64_C(2500000000), UINT64_C(5000000000), 1) == 500);
    assert(cpu_profile_permille(UINT64_MAX - 1, UINT64_MAX, 1) <= 1000);
    assert(cpu_profile_permille(delta, 0, 1) == 0);
    assert(cpu_profile_permille(delta, 10000000, 0) == 0);

    now.creation_id = 11; /* Handle reused for a different task. */
    assert(!cpu_profile_delta(&old, &now, 10000000, &delta));
    now.handle = 0x5678;
    now.creation_id = 10;
    assert(!cpu_profile_delta(&old, &now, 10000000, &delta));
    now.handle = old.handle;
    assert(!cpu_profile_delta(&old, &now, 0, &delta));
    now.runtime_us = 1; /* Counter reset. */
    assert(!cpu_profile_delta(&old, &now, 10000000, &delta));
    puts("cpu_profile_metrics tests passed");
    return 0;
}
