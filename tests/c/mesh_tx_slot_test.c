#include "shared/mesh_tx_slot.h"

#include <assert.h>
#include <stdio.h>

static mesh_tx_slot_check_t check(mesh_tx_slot_state_t *slot, int64_t now)
{
    return mesh_tx_slot_check(slot, true, false, 2, 7, 7, 1000, 2500, now);
}

static mesh_tx_slot_state_t installed(void)
{
    return (mesh_tx_slot_state_t){.assigned_index = 2};
}

static void test_busy_recovered_once(void)
{
    mesh_tx_slot_state_t slot = installed();
    unsigned submissions = 0;
    assert(check(&slot, 1000) == MESH_TX_SLOT_ATTEMPT);
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1050, 2500, 500) ==
           MESH_TX_SLOT_RETRY);
    assert(slot.retried && slot.retry_due_us == 1250 && !slot.completed);
    assert(check(&slot, 1200) == MESH_TX_SLOT_EARLY);
    assert(check(&slot, 1250) == MESH_TX_SLOT_ATTEMPT);
    submissions++;
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_SUBMITTED, 1300, 2500, 500) ==
           MESH_TX_SLOT_RECOVERED);
    assert(check(&slot, 1400) == MESH_TX_SLOT_IGNORED);
    assert(submissions == 1);
}

static void test_expiry_and_insufficient_time(void)
{
    mesh_tx_slot_state_t slot = installed();
    assert(check(&slot, 2501) == MESH_TX_SLOT_EXPIRED);
    assert(check(&slot, 2600) == MESH_TX_SLOT_IGNORED);
    slot = installed();
    assert(check(&slot, 1801) == MESH_TX_SLOT_ATTEMPT);
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1801, 2500, 500) ==
           MESH_TX_SLOT_MISSED);
    assert(slot.completed && !slot.retried);
    assert(check(&slot, 2000) == MESH_TX_SLOT_IGNORED);
}

static void test_cancellation(void)
{
    mesh_tx_slot_state_t slot = installed();
    assert(mesh_tx_slot_check(&slot, true, false, 2, 7, 8, 1000, 2500, 1200) ==
           MESH_TX_SLOT_INVALID);
    assert(mesh_tx_slot_check(&slot, true, true, 2, 7, 7, 1000, 2500, 1200) ==
           MESH_TX_SLOT_INVALID);
    assert(mesh_tx_slot_check(&slot, false, false, 2, 7, 7, 1000, 2500, 1200) ==
           MESH_TX_SLOT_INVALID);
    assert(mesh_tx_slot_check(&slot, true, false, 3, 7, 7, 1000, 2500, 1200) ==
           MESH_TX_SLOT_INVALID);
    assert(!slot.completed && !slot.retried);
}

static void test_exhaustion_and_empty(void)
{
    mesh_tx_slot_state_t slot = installed();
    unsigned misses = 0;
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1100, 2500, 500) ==
           MESH_TX_SLOT_RETRY);
    assert(check(&slot, 1300) == MESH_TX_SLOT_ATTEMPT);
    if (mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1900, 2500, 500) ==
        MESH_TX_SLOT_EXHAUSTED) misses++;
    assert(misses == 1 && slot.completed);
    assert(check(&slot, 2100) == MESH_TX_SLOT_IGNORED);
    assert(misses == 1);

    slot = installed();
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1100, 2500, 500) ==
           MESH_TX_SLOT_RETRY);
    assert(check(&slot, 1300) == MESH_TX_SLOT_ATTEMPT);
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_EMPTY, 1400, 2500, 500) ==
           MESH_TX_SLOT_FINISHED);
    assert(check(&slot, 1500) == MESH_TX_SLOT_IGNORED);
}

static void test_rollover_before_old_wake(void)
{
    mesh_tx_slot_state_t slot = installed();
    unsigned misses = 0;
    assert(!mesh_tx_slot_rollover(&slot, 7, 8, 2500, 3000)); /* Resync canceled it. */
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 2500, 2500));
    if (mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3000)) misses++;
    assert(misses == 1 && slot.completed);
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3200));
    assert(check(&slot, 3200) == MESH_TX_SLOT_IGNORED);
    assert(misses == 1);

    slot = installed(); /* Next frame installed before the stale wake arrives. */
    assert(mesh_tx_slot_check(&slot, true, false, 2, 7, 7, 21000, 22500, 3200) ==
           MESH_TX_SLOT_EARLY);
    assert(misses == 1 && !slot.completed);

    slot = installed();
    slot.completed = true; /* Already submitted or empty. */
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3000));
    slot = (mesh_tx_slot_state_t){.completed = true, .assigned_index = -1};
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 0, 3000));

    slot = installed();
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1100, 2500, 500) ==
           MESH_TX_SLOT_RETRY);
    assert(mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3000));
    assert(slot.retried && slot.completed);
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3100));
}

static void test_deadline_after_preparation(void)
{
    mesh_tx_slot_state_t slot = installed();
    unsigned submissions = 0;
    unsigned misses = 0;
    assert(check(&slot, 2400) == MESH_TX_SLOT_ATTEMPT);
    /* Simulated reservation/preparation completes after the slot deadline. */
    if (mesh_tx_slot_deadline_passed(2500, 2501)) {
        if (mesh_tx_slot_finish(&slot, MESH_TX_SLOT_ERROR, 2501, 2500, 500) ==
            MESH_TX_SLOT_MISSED) misses++;
    } else {
        submissions++;
    }
    assert(submissions == 0 && misses == 1);
    assert(check(&slot, 2600) == MESH_TX_SLOT_IGNORED);
    assert(!mesh_tx_slot_rollover(&slot, 7, 7, 2500, 3000));
    assert(misses == 1);

    assert(!mesh_tx_slot_deadline_passed(0, 999999)); /* Control has no deadline. */
    assert(!mesh_tx_slot_deadline_passed(2500, 2500));
    assert(mesh_tx_slot_deadline_passed(2500, 2501));

    slot = installed();
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_BUSY, 1100, 2500, 500) ==
           MESH_TX_SLOT_RETRY);
    assert(check(&slot, 1300) == MESH_TX_SLOT_ATTEMPT);
    assert(mesh_tx_slot_finish(&slot, MESH_TX_SLOT_ERROR, 2501, 2500, 500) ==
           MESH_TX_SLOT_EXHAUSTED);
}

int main(void)
{
    test_busy_recovered_once();
    test_expiry_and_insufficient_time();
    test_cancellation();
    test_exhaustion_and_empty();
    test_rollover_before_old_wake();
    test_deadline_after_preparation();
    puts("mesh_tx_slot tests passed");
    return 0;
}
