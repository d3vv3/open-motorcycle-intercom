#ifndef CPU_PROFILE_H
#define CPU_PROFILE_H

#include <stdint.h>

typedef enum {
    CPU_PROFILE_SPAN_CAP_CONVERT,
    CPU_PROFILE_SPAN_CAP_HPF,
    CPU_PROFILE_SPAN_CAP_CLEANUP,
    CPU_PROFILE_SPAN_PLAY_REMOTE,
    CPU_PROFILE_SPAN_PLAY_VOICE_CONVERT,
    CPU_PROFILE_SPAN_PLAY_FAR_REFERENCE,
    CPU_PROFILE_SPAN_COUNT,
} cpu_profile_span_t;

/* Wall-time spans include preemption and waits (including codec owner waits in
 * play_remote). They are not CPU cycles or per-core CPU utilization.
 * uxTaskGetSystemState scans stacks; measured overhead is not bounded.
 * Runtime percentages include ISR time and may lag until context switches;
 * affinity is not measured core use, and idle is accounted task time only.
 */
#ifdef CPU_PROFILE_ENABLED
void cpu_profile_init(void);
void cpu_profile_log(void);
int64_t cpu_profile_span_begin(void);
void cpu_profile_span_end(cpu_profile_span_t span, int64_t start_us);
#else
static inline void cpu_profile_init(void) {}
static inline void cpu_profile_log(void) {}
static inline int64_t cpu_profile_span_begin(void) { return 0; }
static inline void cpu_profile_span_end(cpu_profile_span_t span, int64_t start_us)
{
    (void)span;
    (void)start_us;
}
#endif

#endif
