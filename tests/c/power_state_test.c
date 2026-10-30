/* Real components/power/power.c on a host SDK shim. Build from the repo root:
 * cc -std=c11 -Wall -Wextra -Werror -pedantic -pthread -D_GNU_SOURCE \
 *    -Itests/c/power_sdk -Itests/c/host_sdk -Icomponents/power/include \
 *    components/power/power.c tests/c/power_state_test.c */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "power.h"

struct test_semaphore { int held; };
static struct test_semaphore s_mutex;
static unsigned s_takes;
static void (*s_on_take)(void);

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &s_mutex; }
void vSemaphoreDelete(SemaphoreHandle_t sem) { assert(sem == &s_mutex && !sem->held); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait)
{
    (void)wait;
    assert(sem == &s_mutex);
    s_takes++;
    /* Another task's transition completes just before this one gets the lock. */
    void (*hook)(void) = s_on_take;
    s_on_take = NULL;
    if (hook) hook();
    assert(!sem->held);
    sem->held = 1;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    assert(sem == &s_mutex && sem->held);
    sem->held = 0;
    return pdTRUE;
}
int64_t esp_timer_get_time(void) { return 1000000; }
void test_sdk_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
const char *esp_err_to_name(esp_err_t result) { (void)result; return "err"; }
esp_err_t esp_pm_lock_create(esp_pm_lock_type_t type, int arg, const char *name,
                             esp_pm_lock_handle_t *handle)
{
    (void)type; (void)arg; (void)name; *handle = NULL; return ESP_OK;
}
esp_err_t esp_pm_lock_acquire(esp_pm_lock_handle_t handle) { (void)handle; return ESP_OK; }
esp_err_t esp_pm_lock_release(esp_pm_lock_handle_t handle) { (void)handle; return ESP_OK; }
esp_err_t esp_pm_lock_delete(esp_pm_lock_handle_t handle) { (void)handle; return ESP_OK; }
uint64_t esp_sleep_get_wakeup_causes(void) { return 0; }
esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us) { (void)time_in_us; return ESP_OK; }
void esp_deep_sleep_start(void) { abort(); }

static void concurrent_standby(void) { assert(power_set_state(POWER_STATE_STANDBY) == ESP_OK); }

int main(void)
{
    assert(power_init() == ESP_OK);
    assert(power_get_state() == POWER_STATE_STANDBY);

    /* Out-of-range state: rejected before any name lookup or transition. */
    assert(power_set_state((power_state_t)4) == ESP_ERR_INVALID_ARG);
    assert(power_set_state((power_state_t)-1) == ESP_ERR_INVALID_ARG);
    assert(power_get_state() == POWER_STATE_STANDBY);
    assert(strcmp(power_state_str((power_state_t)99), "UNKNOWN") == 0);

    assert(power_set_state(POWER_STATE_MESH_IDLE) == ESP_OK);
    power_notify_voice_start();
    assert(power_get_state() == POWER_STATE_ACTIVE_VOICE);
    power_notify_voice_end();
    assert(power_get_state() == POWER_STATE_MESH_IDLE);

    /* Check-then-act: voice start saw MESH_IDLE, but the state moved to
     * STANDBY before it held the mutex. It must not jump to ACTIVE_VOICE. */
    s_on_take = concurrent_standby;
    power_notify_voice_start();
    assert(power_get_state() == POWER_STATE_STANDBY);
    /* Voice end from a non-voice state is a no-op under the lock. */
    power_notify_voice_end();
    assert(power_get_state() == POWER_STATE_STANDBY);
    assert(!s_mutex.held && s_takes > 0);

    assert(power_deinit() == ESP_OK);
    puts("power state tests passed");
    return 0;
}
