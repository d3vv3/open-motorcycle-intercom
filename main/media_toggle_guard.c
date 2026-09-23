#include "media_toggle_guard.h"

void media_toggle_guard_reset(media_toggle_guard_t *guard)
{
    *guard = (media_toggle_guard_t){0};
}

media_toggle_decision_t media_toggle_guard_step(media_toggle_guard_t *guard,
                                                 bool request_pending,
                                                 bool context_valid,
                                                 bool streaming,
                                                 int64_t transition_ms,
                                                 int64_t now_ms)
{
    if (!context_valid) {
        media_toggle_guard_reset(guard);
        return MEDIA_TOGGLE_NONE;
    }

    if (!guard->waiting) {
        return request_pending ? MEDIA_TOGGLE_DISPATCH : MEDIA_TOGGLE_NONE;
    }
    if (!guard->armed) return MEDIA_TOGGLE_NONE;
    if (request_pending) guard->queued = true;

    if (streaming == guard->sent_streaming) {
        guard->transition_observed = false;
        guard->transition_ms = 0;
    } else if (transition_ms > 0 &&
               (!guard->transition_observed || guard->transition_ms != transition_ms)) {
        guard->transition_observed = true;
        guard->transition_ms = transition_ms;
    }

    if (guard->queued && guard->transition_observed &&
        streaming != guard->sent_streaming &&
        guard->transition_ms - guard->sent_ms > MEDIA_TOGGLE_GUARD_TIMEOUT_MS) {
        media_toggle_guard_reset(guard);
        return MEDIA_TOGGLE_DROPPED;
    }

    if (guard->queued && guard->transition_observed &&
        streaming != guard->sent_streaming &&
        now_ms - guard->sent_ms >= MEDIA_TOGGLE_GUARD_SETTLE_MS &&
        now_ms - guard->transition_ms >= MEDIA_TOGGLE_GUARD_SETTLE_MS) {
        guard->queued = false;
        guard->waiting = false;
        guard->transition_observed = false;
        return MEDIA_TOGGLE_DISPATCH;
    }

    if (streaming == guard->sent_streaming &&
        now_ms - guard->sent_ms >= MEDIA_TOGGLE_GUARD_TIMEOUT_MS) {
        bool dropped = guard->queued;
        media_toggle_guard_reset(guard);
        return dropped ? MEDIA_TOGGLE_DROPPED : MEDIA_TOGGLE_NONE;
    }
    return MEDIA_TOGGLE_NONE;
}

void media_toggle_guard_command_result(media_toggle_guard_t *guard,
                                       bool arm_guard,
                                       bool sent_streaming,
                                       int64_t now_ms)
{
    if (!arm_guard) {
        media_toggle_guard_reset(guard);
        return;
    }
    guard->waiting = true;
    guard->armed = true;
    guard->sent_streaming = sent_streaming;
    guard->transition_observed = false;
    guard->sent_ms = now_ms;
}
