#ifndef OMI_MESH_TX_SLOT_H
#define OMI_MESH_TX_SLOT_H

#include <stdbool.h>
#include <stdint.h>

#define MESH_TX_SLOT_RETRY_US 200

/* Zero disables admission gating for non-slot traffic. */
static inline bool mesh_tx_slot_deadline_passed(int64_t deadline_us, int64_t now_us)
{
    return deadline_us != 0 && now_us > deadline_us;
}

typedef struct {
    bool completed;
    bool retried;
    int8_t assigned_index;
    int64_t retry_due_us;
} mesh_tx_slot_state_t;

typedef enum {
    MESH_TX_SLOT_IGNORED,
    MESH_TX_SLOT_INVALID,
    MESH_TX_SLOT_EARLY,
    MESH_TX_SLOT_EXPIRED,
    MESH_TX_SLOT_ATTEMPT,
} mesh_tx_slot_check_t;

typedef enum {
    MESH_TX_SLOT_SUBMITTED,
    MESH_TX_SLOT_EMPTY,
    MESH_TX_SLOT_BUSY,
    MESH_TX_SLOT_ERROR,
} mesh_tx_slot_send_result_t;

typedef enum {
    MESH_TX_SLOT_FINISHED,
    MESH_TX_SLOT_RECOVERED,
    MESH_TX_SLOT_RETRY,
    MESH_TX_SLOT_MISSED,
    MESH_TX_SLOT_EXHAUSTED,
} mesh_tx_slot_outcome_t;

static inline mesh_tx_slot_check_t mesh_tx_slot_check(mesh_tx_slot_state_t *slot,
                                                      bool active, bool stopping,
                                                      int8_t index, uint32_t generation,
                                                      uint32_t current_generation,
                                                      int64_t start_us, int64_t deadline_us,
                                                      int64_t now_us)
{
    if (slot->completed) return MESH_TX_SLOT_IGNORED;
    if (!active || stopping || index < 0 || index != slot->assigned_index ||
        generation != current_generation) return MESH_TX_SLOT_INVALID;
    if (now_us < start_us || now_us < slot->retry_due_us) return MESH_TX_SLOT_EARLY;
    if (mesh_tx_slot_deadline_passed(deadline_us, now_us)) {
        slot->completed = true;
        return MESH_TX_SLOT_EXPIRED;
    }
    return MESH_TX_SLOT_ATTEMPT;
}

/* Called before installing the next frame's slot, while the slot mutex is held. */
static inline bool mesh_tx_slot_rollover(mesh_tx_slot_state_t *slot,
                                         uint32_t slot_generation, uint32_t current_generation,
                                         int64_t deadline_us, int64_t now_us)
{
    if (slot->completed || slot_generation != current_generation ||
        !mesh_tx_slot_deadline_passed(deadline_us, now_us)) return false;
    slot->completed = true;
    return true;
}

/* deadline_us already excludes the TDMA guard; reserve another guard for a retry. */
static inline mesh_tx_slot_outcome_t mesh_tx_slot_finish(mesh_tx_slot_state_t *slot,
                                                          mesh_tx_slot_send_result_t result,
                                                          int64_t now_us, int64_t deadline_us,
                                                          int64_t guard_us)
{
    if (result == MESH_TX_SLOT_BUSY && now_us <= deadline_us &&
        deadline_us - now_us >= MESH_TX_SLOT_RETRY_US + guard_us) {
        slot->retried = true;
        slot->retry_due_us = now_us + MESH_TX_SLOT_RETRY_US;
        return MESH_TX_SLOT_RETRY;
    }
    slot->completed = true;
    if (result == MESH_TX_SLOT_SUBMITTED) {
        return slot->retried ? MESH_TX_SLOT_RECOVERED : MESH_TX_SLOT_FINISHED;
    }
    if (result == MESH_TX_SLOT_EMPTY) return MESH_TX_SLOT_FINISHED;
    return slot->retried && (result == MESH_TX_SLOT_BUSY ||
                            mesh_tx_slot_deadline_passed(deadline_us, now_us)) ?
               MESH_TX_SLOT_EXHAUSTED : MESH_TX_SLOT_MISSED;
}

#endif /* OMI_MESH_TX_SLOT_H */
