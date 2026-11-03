#include <assert.h>
#include <stdio.h>
#include "radio_policy.h"

#define ESP_NOW true
#define NRF false

static void expect_prompts(const omi_radio_plan_t *plan, uint8_t count,
                           omi_radio_prompt_t first, omi_radio_prompt_t second)
{
    assert(plan->prompt_count == count);
    if (count > 0u) assert(plan->prompts[0] == first);
    if (count > 1u) assert(plan->prompts[1] == second);
}

static bool no_bluetooth_change(const omi_radio_plan_t *plan)
{
    return !plan->stop_mesh && !plan->bluetooth_off && !plan->bluetooth_on && !plan->reconnect;
}

static void test_nrf_has_no_exclusivity(void)
{
    for (int mesh = 0; mesh < 2; ++mesh) {
        assert(omi_radio_bluetooth_allowed(NRF, mesh != 0));
        for (int peer = 0; peer < 2; ++peer) {
            omi_radio_plan_t on = omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, NRF, mesh != 0, peer != 0);
            assert(no_bluetooth_change(&on) && !on.open_pairing);
            expect_prompts(&on, 1u, OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_MESH_ON);

            omi_radio_plan_t off = omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, NRF, mesh != 0, peer != 0);
            assert(no_bluetooth_change(&off) && !off.open_pairing);
            expect_prompts(&off, 1u, OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_MESH_OFF);

            /* Pairing never stops the ESB mesh: music and mesh coexist. */
            omi_radio_plan_t pair = omi_radio_plan(OMI_RADIO_EVENT_PAIRING, NRF, mesh != 0, peer != 0);
            assert(no_bluetooth_change(&pair) && pair.open_pairing);
            expect_prompts(&pair, 1u, OMI_RADIO_PROMPT_PHONE_PAIRING, OMI_RADIO_PROMPT_PHONE_PAIRING);
        }
    }
}

/* Transition 1: mesh on turns Bluetooth off; Mesh On, then Bluetooth off. */
static void test_mesh_on_turns_bluetooth_off(void)
{
    omi_radio_plan_t plan = omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, ESP_NOW, false, false);
    assert(plan.bluetooth_off && !plan.bluetooth_on && !plan.stop_mesh);
    assert(!plan.reconnect && !plan.open_pairing);
    expect_prompts(&plan, 2u, OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_BLUETOOTH_OFF);
}

/* Transition 2: pairing gesture with mesh on stops the mesh, Bluetooth on, window open. */
static void test_pairing_turns_mesh_off(void)
{
    for (int peer = 0; peer < 2; ++peer) {
        omi_radio_plan_t plan = omi_radio_plan(OMI_RADIO_EVENT_PAIRING, ESP_NOW, true, peer != 0);
        assert(plan.stop_mesh && plan.bluetooth_on && !plan.bluetooth_off);
        assert(plan.open_pairing);
        assert(!plan.reconnect); /* No outbound connect races the pairing window. */
        expect_prompts(&plan, 2u, OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_PHONE_PAIRING);
    }
    /* With mesh already off, pairing is unchanged from before. */
    omi_radio_plan_t plain = omi_radio_plan(OMI_RADIO_EVENT_PAIRING, ESP_NOW, false, false);
    assert(no_bluetooth_change(&plain) && plain.open_pairing);
    expect_prompts(&plain, 1u, OMI_RADIO_PROMPT_PHONE_PAIRING, OMI_RADIO_PROMPT_PHONE_PAIRING);
}

/* Transition 3: mesh off brings Bluetooth back; Mesh Off, then Bluetooth on. */
static void test_mesh_off_turns_bluetooth_on(void)
{
    omi_radio_plan_t remembered = omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, ESP_NOW, true, true);
    assert(remembered.bluetooth_on && !remembered.bluetooth_off && !remembered.stop_mesh);
    assert(!remembered.open_pairing);
    assert(remembered.reconnect); /* Remembered peer: exactly one connect attempt. */
    expect_prompts(&remembered, 2u, OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_BLUETOOTH_ON);

    omi_radio_plan_t none = omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, ESP_NOW, true, false);
    assert(none.bluetooth_on && !none.reconnect); /* No peer: connectable, no attempt. */
    expect_prompts(&none, 2u, OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_BLUETOOTH_ON);
}

/* Transition 4: boot follows the persisted mesh intent. */
static void test_boot_follows_mesh_intent(void)
{
    assert(!omi_radio_bluetooth_allowed(ESP_NOW, true));
    assert(omi_radio_bluetooth_allowed(ESP_NOW, false));
}

/* Transition 5: with mesh on, a bonded phone's reconnect is refused until
 * the pairing gesture or mesh off re-enables Bluetooth. */
static void test_reconnect_refused_while_mesh_on(void)
{
    bool mesh_on = false;
    bool bluetooth = omi_radio_bluetooth_allowed(ESP_NOW, mesh_on);
    assert(bluetooth);

    omi_radio_plan_t plan = omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, ESP_NOW, mesh_on, false);
    mesh_on = true;
    if (plan.bluetooth_off) bluetooth = false;
    assert(!bluetooth && !omi_radio_bluetooth_allowed(ESP_NOW, mesh_on));

    /* Repeated phone attempts stay refused while the mesh owns the radio. */
    for (int attempt = 0; attempt < 3; ++attempt)
        assert(!omi_radio_bluetooth_allowed(ESP_NOW, mesh_on));

    plan = omi_radio_plan(OMI_RADIO_EVENT_MESH_OFF, ESP_NOW, mesh_on, true);
    mesh_on = false;
    if (plan.bluetooth_on) bluetooth = true;
    assert(bluetooth && omi_radio_bluetooth_allowed(ESP_NOW, mesh_on));
}

/* A full ESP-NOW session queues prompts in the documented order. */
static void test_prompt_sequence(void)
{
    static const omi_radio_prompt_t expected[] = {
        OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_BLUETOOTH_OFF,
        OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_PHONE_PAIRING,
        OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_BLUETOOTH_OFF,
        OMI_RADIO_PROMPT_MESH_OFF, OMI_RADIO_PROMPT_BLUETOOTH_ON,
    };
    const omi_radio_event_t events[] = {
        OMI_RADIO_EVENT_MESH_ON, OMI_RADIO_EVENT_PAIRING,
        OMI_RADIO_EVENT_MESH_ON, OMI_RADIO_EVENT_MESH_OFF,
    };
    omi_radio_prompt_t played[8];
    unsigned count = 0u;
    bool mesh_on = false;
    for (unsigned i = 0; i < sizeof(events) / sizeof(events[0]); ++i) {
        omi_radio_plan_t plan = omi_radio_plan(events[i], ESP_NOW, mesh_on, false);
        for (uint8_t p = 0; p < plan.prompt_count; ++p) {
            assert(count < 8u);
            played[count++] = plan.prompts[p];
        }
        if (events[i] == OMI_RADIO_EVENT_MESH_ON) mesh_on = true;
        else if (events[i] == OMI_RADIO_EVENT_MESH_OFF || plan.stop_mesh) mesh_on = false;
    }
    assert(count == sizeof(expected) / sizeof(expected[0]));
    for (unsigned i = 0; i < count; ++i) assert(played[i] == expected[i]);
}

typedef enum { EV_BLUETOOTH_OFF, EV_MESH_START, EV_PROMPT, EV_BLUETOOTH_ON, EV_PERSIST_OFF } event_t;

typedef struct {
    event_t events[8];
    omi_radio_prompt_t prompts[8];
    unsigned count;
} trace_t;

static void record(trace_t *trace, event_t event, omi_radio_prompt_t prompt)
{
    assert(trace->count < 8u);
    trace->prompts[trace->count] = prompt;
    trace->events[trace->count++] = event;
}

/* Mirrors main.c: the MESH_ON plan's Bluetooth part runs at the request; the
 * start result decides the prompts or the rollback. */
static void run_step_with(trace_t *trace, omi_mesh_on_step_t step, bool esp_now, bool start_ok,
                          bool remembered_peer)
{
    omi_radio_plan_t plan = omi_radio_plan(OMI_RADIO_EVENT_MESH_ON, esp_now, false, false);
    if (step == OMI_MESH_ON_STEP_BLUETOOTH_OFF) {
        assert(plan.bluetooth_off);
        record(trace, EV_BLUETOOTH_OFF, OMI_RADIO_PROMPT_MESH_ON);
    } else if (step == OMI_MESH_ON_STEP_START_MESH) {
        record(trace, EV_MESH_START, OMI_RADIO_PROMPT_MESH_ON);
        omi_radio_plan_t result = omi_radio_mesh_on_result(start_ok, esp_now,
                                                           !start_ok && remembered_peer);
        assert(!result.bluetooth_off && !result.stop_mesh && !result.open_pairing);
        if (result.persist_off) record(trace, EV_PERSIST_OFF, OMI_RADIO_PROMPT_MESH_OFF);
        if (result.bluetooth_on) record(trace, EV_BLUETOOTH_ON, OMI_RADIO_PROMPT_BLUETOOTH_ON);
        for (uint8_t i = 0; i < result.prompt_count; ++i)
            record(trace, EV_PROMPT, result.prompts[i]);
    }
}

static void run_step(trace_t *trace, omi_mesh_on_step_t step, bool esp_now)
{
    run_step_with(trace, step, esp_now, true, false);
}

/* ESP-NOW: Bluetooth off, then mesh_start once the phone link drops, then
 * "Mesh On", "Bluetooth off". */
static void test_mesh_on_bluetooth_off_before_start(void)
{
    omi_mesh_on_seq_t seq = {0};
    trace_t trace = {0};
    run_step(&trace, omi_mesh_on_request(&seq, ESP_NOW, 1000), ESP_NOW);
    assert(seq.waiting);
    for (int64_t now = 1000; now < 1400; now += 20)
        run_step(&trace, omi_mesh_on_poll(&seq, true, now), ESP_NOW); /* Link still up. */
    assert(trace.count == 1u && trace.events[0] == EV_BLUETOOTH_OFF);
    run_step(&trace, omi_mesh_on_poll(&seq, false, 1400), ESP_NOW); /* Link down. */
    assert(!seq.waiting);
    assert(trace.count == 4u);
    assert(trace.events[0] == EV_BLUETOOTH_OFF && trace.events[1] == EV_MESH_START);
    assert(trace.events[2] == EV_PROMPT && trace.prompts[2] == OMI_RADIO_PROMPT_MESH_ON);
    assert(trace.events[3] == EV_PROMPT && trace.prompts[3] == OMI_RADIO_PROMPT_BLUETOOTH_OFF);
    /* Exactly one start per request. */
    assert(omi_mesh_on_poll(&seq, false, 1420) == OMI_MESH_ON_STEP_NONE);
}

/* No phone: the start follows on the same pass as the Bluetooth switch. */
static void test_mesh_on_without_phone_starts_at_once(void)
{
    omi_mesh_on_seq_t seq = {0};
    assert(omi_mesh_on_request(&seq, ESP_NOW, 0) == OMI_MESH_ON_STEP_BLUETOOTH_OFF);
    assert(omi_mesh_on_poll(&seq, false, 0) == OMI_MESH_ON_STEP_START_MESH);
}

/* A link that never drops delays the start by the bounded wait only. */
static void test_mesh_on_link_down_timeout(void)
{
    omi_mesh_on_seq_t seq = {0};
    const int64_t start = 5000;
    assert(omi_mesh_on_request(&seq, ESP_NOW, start) == OMI_MESH_ON_STEP_BLUETOOTH_OFF);
    assert(omi_mesh_on_poll(&seq, true, start + OMI_MESH_ON_LINK_DOWN_TIMEOUT_MS - 1) ==
           OMI_MESH_ON_STEP_NONE);
    assert(omi_mesh_on_poll(&seq, true, start + OMI_MESH_ON_LINK_DOWN_TIMEOUT_MS) ==
           OMI_MESH_ON_STEP_START_MESH);
    assert(!seq.waiting);
}

/* nRF: no Bluetooth step, the mesh starts at the request. */
static void test_mesh_on_nrf_starts_at_once(void)
{
    omi_mesh_on_seq_t seq = {0};
    trace_t trace = {0};
    run_step(&trace, omi_mesh_on_request(&seq, NRF, 0), NRF);
    assert(!seq.waiting);
    run_step(&trace, omi_mesh_on_poll(&seq, true, 0), NRF);
    assert(trace.count == 2u && trace.events[0] == EV_MESH_START);
    assert(trace.events[1] == EV_PROMPT && trace.prompts[1] == OMI_RADIO_PROMPT_MESH_ON);
}

/* A failed start rolls back: intent off, Bluetooth back, nothing announced. */
static void test_mesh_on_start_failure_rolls_back(void)
{
    for (int peer = 0; peer < 2; ++peer) {
        omi_mesh_on_seq_t seq = {0};
        trace_t trace = {0};
        run_step_with(&trace, omi_mesh_on_request(&seq, ESP_NOW, 0), ESP_NOW, false, peer != 0);
        run_step_with(&trace, omi_mesh_on_poll(&seq, false, 20), ESP_NOW, false, peer != 0);
        assert(trace.count == 4u);
        assert(trace.events[0] == EV_BLUETOOTH_OFF && trace.events[1] == EV_MESH_START);
        assert(trace.events[2] == EV_PERSIST_OFF && trace.events[3] == EV_BLUETOOTH_ON);
        for (unsigned i = 0; i < trace.count; ++i) assert(trace.events[i] != EV_PROMPT);

        omi_radio_plan_t result = omi_radio_mesh_on_result(false, ESP_NOW, peer != 0);
        assert(result.persist_off && result.bluetooth_on && result.prompt_count == 0u);
        assert(result.reconnect == (peer != 0)); /* At most one attempt, remembered phone only. */
        assert(omi_mesh_on_poll(&seq, false, 40) == OMI_MESH_ON_STEP_NONE); /* No retry loop. */
    }
    /* nRF never touches Bluetooth; a failure only rolls the intent back. */
    omi_radio_plan_t nrf = omi_radio_mesh_on_result(false, NRF, true);
    assert(nrf.persist_off && no_bluetooth_change(&nrf) && nrf.prompt_count == 0u);
    /* Success never rolls back and never switches Bluetooth again. */
    omi_radio_plan_t ok = omi_radio_mesh_on_result(true, ESP_NOW, false);
    assert(!ok.persist_off && no_bluetooth_change(&ok));
    expect_prompts(&ok, 2u, OMI_RADIO_PROMPT_MESH_ON, OMI_RADIO_PROMPT_BLUETOOTH_OFF);
}

/* Mesh off, pairing or a channel change while waiting: no late start. */
static void test_mesh_on_cancel(void)
{
    omi_mesh_on_seq_t seq = {0};
    assert(!omi_mesh_on_cancel(&seq));
    assert(omi_mesh_on_request(&seq, ESP_NOW, 0) == OMI_MESH_ON_STEP_BLUETOOTH_OFF);
    assert(omi_mesh_on_cancel(&seq));
    assert(omi_mesh_on_poll(&seq, false, OMI_MESH_ON_LINK_DOWN_TIMEOUT_MS) == OMI_MESH_ON_STEP_NONE);
    assert(!omi_mesh_on_cancel(&seq));
}

int main(void)
{
    test_nrf_has_no_exclusivity();
    test_mesh_on_turns_bluetooth_off();
    test_pairing_turns_mesh_off();
    test_mesh_off_turns_bluetooth_on();
    test_boot_follows_mesh_intent();
    test_reconnect_refused_while_mesh_on();
    test_prompt_sequence();
    test_mesh_on_bluetooth_off_before_start();
    test_mesh_on_without_phone_starts_at_once();
    test_mesh_on_link_down_timeout();
    test_mesh_on_nrf_starts_at_once();
    test_mesh_on_cancel();
    test_mesh_on_start_failure_rolls_back();
    puts("radio policy tests passed");
    return 0;
}
