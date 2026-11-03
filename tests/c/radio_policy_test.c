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

int main(void)
{
    test_nrf_has_no_exclusivity();
    test_mesh_on_turns_bluetooth_off();
    test_pairing_turns_mesh_off();
    test_mesh_off_turns_bluetooth_on();
    test_boot_follows_mesh_intent();
    test_reconnect_refused_while_mesh_on();
    test_prompt_sequence();
    puts("radio policy tests passed");
    return 0;
}
