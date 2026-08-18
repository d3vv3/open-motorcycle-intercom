#include "shared/mesh_tx_metrics.h"

#include <assert.h>
#include <stdio.h>

static mesh_tx_metrics_result_t classify(bool local_pending, bool relay_pending,
                                         bool ingress_recent, bool local_consumed,
                                         bool local_consumed_successfully,
                                         bool empty_after_consume)
{
    const mesh_tx_metrics_input_t input = {
        .scheduled_local_slot = true,
        .local_pending = local_pending,
        .relay_pending = relay_pending,
        .ingress_recent = ingress_recent,
        .local_consumed = local_consumed,
        .local_consumed_successfully = local_consumed_successfully,
        .local_queue_empty_after_consume = empty_after_consume,
    };
    return mesh_tx_metrics_classify(&input);
}

static void test_successful_one_frame_drain(void)
{
    mesh_tx_metrics_result_t result = classify(false, false, true, true, true, true);
    assert(result.queue_drained);
    assert(!result.slot_starved);
}

static void test_recovery_drain(void)
{
    mesh_tx_metrics_result_t result = classify(false, true, true, true, true, true);
    assert(result.queue_drained);
    assert(!result.slot_starved);
}

static void test_fresh_source_empty_slot_starves(void)
{
    mesh_tx_metrics_result_t result = classify(false, false, true, false, false, false);
    assert(!result.queue_drained);
    assert(result.slot_starved);
}

static void test_stale_source_and_relay_pending_are_not_starvation(void)
{
    mesh_tx_metrics_result_t stale = classify(false, false, false, false, false, false);
    mesh_tx_metrics_result_t relay = classify(false, true, true, false, false, false);
    assert(!stale.slot_starved);
    assert(!relay.slot_starved);
}

static void test_tx_failure_and_late_drop_do_not_classify_as_starvation(void)
{
    /* TX result and deadline accounting remain independent of queue-state metrics. */
    mesh_tx_metrics_result_t tx_failure = classify(false, false, true, true, false, true);
    mesh_tx_metrics_result_t late_drop = classify(false, false, true, true, false, true);
    assert(!tx_failure.queue_drained && !tx_failure.slot_starved);
    assert(!late_drop.queue_drained && !late_drop.slot_starved);
}

static void test_starvation_log_cadence_wraps_without_state(void)
{
    assert(mesh_tx_metrics_should_log_starvation(1u));
    assert(!mesh_tx_metrics_should_log_starvation(64u));
    assert(mesh_tx_metrics_should_log_starvation(65u));
    assert(!mesh_tx_metrics_should_log_starvation(UINT32_MAX));
    assert(!mesh_tx_metrics_should_log_starvation(0u));
    assert(mesh_tx_metrics_should_log_starvation(1u));
}

int main(void)
{
    test_successful_one_frame_drain();
    test_recovery_drain();
    test_fresh_source_empty_slot_starves();
    test_stale_source_and_relay_pending_are_not_starvation();
    test_tx_failure_and_late_drop_do_not_classify_as_starvation();
    test_starvation_log_cadence_wraps_without_state();
    puts("mesh_tx_metrics tests passed");
    return 0;
}
