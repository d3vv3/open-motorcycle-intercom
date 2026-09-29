/** @file button.h External three-button input driver. */
#ifndef OMI_BUTTON_H
#define OMI_BUTTON_H

#include "esp_err.h"
#include "button_gesture.h"
#include "omi_board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BOOT remains defined for hardware download; the driver never touches it. */
#define BUTTON_BOOT_GPIO OMI_BOARD_GPIO_BOOT_BUTTON
#define BUTTON_MINUS_GPIO OMI_BOARD_GPIO_BUTTON_MINUS
#define BUTTON_CENTER_GPIO OMI_BOARD_GPIO_BUTTON_CENTER
#define BUTTON_PLUS_GPIO OMI_BOARD_GPIO_BUTTON_PLUS

typedef void (*button_callback_t)(button_id_t id, button_event_t event, void *context);

/** Configure the external active-low buttons and start the polling task. */
esp_err_t button_init(void);
/** Stop polling, wait for callbacks to finish, and release driver resources. */
void button_deinit(void);
/** Register a callback invoked from the low-priority button task on release. */
void button_register_callback(button_callback_t callback, void *context);

#ifdef __cplusplus
}
#endif

#endif /* OMI_BUTTON_H */
