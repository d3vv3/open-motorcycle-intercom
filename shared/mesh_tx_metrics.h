#ifndef OMI_MESH_TX_METRICS_H
#define OMI_MESH_TX_METRICS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Slot metrics describe local queue state only. A drain records normal
 * consumption that leaves the queue empty; starvation requires recent local
 * ingress and no packet of either kind to service in the scheduled local slot.
 * A drain requires successful local transmission; TX failures and late drops
 * remain outside these metrics.
 */
typedef struct {
    bool scheduled_local_slot;
    bool local_pending;
    bool relay_pending;
    bool ingress_recent;
    bool local_consumed;
    bool local_consumed_successfully;
    bool local_queue_empty_after_consume;
} mesh_tx_metrics_input_t;

typedef struct {
    bool queue_drained;
    bool slot_starved;
} mesh_tx_metrics_result_t;

static inline mesh_tx_metrics_result_t
mesh_tx_metrics_classify(const mesh_tx_metrics_input_t *input)
{
    mesh_tx_metrics_result_t result = {
        .queue_drained = input->local_consumed && input->local_consumed_successfully &&
                         input->local_queue_empty_after_consume,
        .slot_starved = input->scheduled_local_slot && !input->local_consumed &&
                         !input->local_pending && !input->relay_pending && input->ingress_recent,
    };

    return result;
}

static inline bool mesh_tx_metrics_should_log_starvation(uint32_t total)
{
    return total != 0u && (total % 64u) == 1u;
}

#endif /* OMI_MESH_TX_METRICS_H */
