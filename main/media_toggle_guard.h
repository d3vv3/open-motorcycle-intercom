#ifndef MEDIA_TOGGLE_GUARD_H
#define MEDIA_TOGGLE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define MEDIA_TOGGLE_GUARD_SETTLE_MS 5000
#define MEDIA_TOGGLE_GUARD_TIMEOUT_MS 10000

typedef struct {
    bool waiting;
    bool queued;
    bool sent_streaming;
    bool transition_observed;
    bool armed;
    int64_t sent_ms;
    int64_t transition_ms;
} media_toggle_guard_t;

typedef enum {
    MEDIA_TOGGLE_NONE,
    MEDIA_TOGGLE_DISPATCH,
    MEDIA_TOGGLE_DROPPED,
} media_toggle_decision_t;

void media_toggle_guard_reset(media_toggle_guard_t *guard);
media_toggle_decision_t media_toggle_guard_step(media_toggle_guard_t *guard,
                                                 bool request_pending,
                                                 bool context_valid,
                                                 bool streaming,
                                                 int64_t transition_ms,
                                                 int64_t now_ms);
void media_toggle_guard_command_result(media_toggle_guard_t *guard,
                                       bool arm_guard,
                                       bool sent_streaming,
                                       int64_t now_ms);

#endif
