#ifndef OMI_RADIO_POLICY_H
#define OMI_RADIO_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* Mesh/Bluetooth exclusivity. With the ESP-NOW transport the S31 shares one
 * 2.4 GHz radio between the mesh and Classic Bluetooth, so they are mutually
 * exclusive: mesh on means Bluetooth off (not connectable, links dropped),
 * mesh off means Bluetooth on. With the nRF52840/ESB transport the mesh has
 * its own radio and both coexist exactly as before. */

typedef enum {
    OMI_RADIO_PROMPT_MESH_ON,
    OMI_RADIO_PROMPT_MESH_OFF,
    OMI_RADIO_PROMPT_BLUETOOTH_OFF,
    OMI_RADIO_PROMPT_BLUETOOTH_ON,
    OMI_RADIO_PROMPT_PHONE_PAIRING,
} omi_radio_prompt_t;

typedef enum {
    OMI_RADIO_EVENT_MESH_ON,  /* Mesh intent turned on and accepted. */
    OMI_RADIO_EVENT_MESH_OFF, /* Mesh intent turned off (toggle). */
    OMI_RADIO_EVENT_PAIRING,  /* Center-hold pairing gesture. */
} omi_radio_event_t;

#define OMI_RADIO_MAX_PROMPTS 2u

typedef struct {
    bool stop_mesh;     /* Stop the mesh and persist the intent off first. */
    bool bluetooth_off; /* Drop phone links; non-connectable, non-discoverable. */
    bool bluetooth_on;  /* Connectable again. */
    bool reconnect;     /* One connect attempt to the phone mesh-on dropped. */
    bool open_pairing;  /* Open the discoverable pairing window. */
    uint8_t prompt_count;
    omi_radio_prompt_t prompts[OMI_RADIO_MAX_PROMPTS];
} omi_radio_plan_t;

/* Bluetooth may be on (connectable, links accepted) unless the ESP-NOW mesh
 * owns the radio. Used at boot and to refuse inbound links while mesh is on. */
static inline bool omi_radio_bluetooth_allowed(bool esp_now, bool mesh_on)
{
    return !esp_now || !mesh_on;
}

static inline void omi_radio_prompt_add(omi_radio_plan_t *plan, omi_radio_prompt_t prompt)
{
    if (plan->prompt_count < OMI_RADIO_MAX_PROMPTS) plan->prompts[plan->prompt_count++] = prompt;
}

/* mesh_on is the intent before the event; remembered_peer says a phone link
 * was dropped by mesh-on and can be retried once. */
static inline omi_radio_plan_t omi_radio_plan(omi_radio_event_t event, bool esp_now,
                                              bool mesh_on, bool remembered_peer)
{
    omi_radio_plan_t plan = {0};
    switch (event) {
    case OMI_RADIO_EVENT_MESH_ON:
        omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_MESH_ON);
        if (esp_now) {
            plan.bluetooth_off = true;
            omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_BLUETOOTH_OFF);
        }
        break;
    case OMI_RADIO_EVENT_MESH_OFF:
        omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_MESH_OFF);
        if (esp_now) {
            plan.bluetooth_on = true;
            plan.reconnect = remembered_peer;
            omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_BLUETOOTH_ON);
        }
        break;
    case OMI_RADIO_EVENT_PAIRING:
        if (esp_now && mesh_on) {
            plan.stop_mesh = true;
            plan.bluetooth_on = true;
            omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_MESH_OFF);
        }
        /* Pairing is for a phone to find us; no outbound reconnect races it. */
        plan.open_pairing = true;
        omi_radio_prompt_add(&plan, OMI_RADIO_PROMPT_PHONE_PAIRING);
        break;
    }
    return plan;
}

#endif
