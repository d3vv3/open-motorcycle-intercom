/**
 * @file main.c
 * @brief OMI Mesh Firmware Entry Point
 *
 * Initializes ESB radio, UART bridge, TDMA timing, and starts mesh protocol
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include "esb_radio.h"
#include "mesh_protocol.h"
#include "tdma.h"
#include "uart_bridge.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* RF channel - must match other nodes */
#define RF_CHANNEL 40

/* SPI poll cadence for ESP32 bridge */
#define SPI_POLL_MS 2

/* Dedicated SPI bridge thread to minimize main-loop jitter */
#define SPI_THREAD_STACK_SIZE 2048
#define SPI_THREAD_PRIORITY   0

K_THREAD_STACK_DEFINE(s_spi_thread_stack, SPI_THREAD_STACK_SIZE);
static struct k_thread s_spi_thread;
static volatile uint32_t s_spi_loop_count = 0;

static void spi_bridge_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        uart_bridge_process();
        s_spi_loop_count++;
        k_sleep(K_MSEC(SPI_POLL_MS));
    }
}

/* External functions from mesh_protocol.c */
extern int mesh_protocol_init(void);
extern int mesh_protocol_start(void);

int main(void)
{
    int ret;

    /* Initialize UART bridge FIRST - must be ready before ESP32 sends probe.
     * ESP32 sends PING within ~200ms of boot, so this must happen immediately. */
    ret = uart_bridge_init();

    /* Enable USB subsystem for logging (works even without a USB host) */
    ret = usb_enable(NULL);
    if (ret != 0 && ret != -EALREADY) {
        return ret;
    }

    /* Short delay for USB CDC enumeration before first output */
    k_sleep(K_MSEC(1000));

    printk("\n\n*** OMI Mesh Firmware booted ***\n\n");

    if (!uart_bridge_is_initialized()) {
        printk("[DIAG] UART bridge init FAILED\n");
    } else {
        printk("[DIAG] UART bridge init OK\n");
        /* Clock out the queued STATUS packet now that USB is ready for logging */
        uart_bridge_process();
        printk("[DIAG] First SPI transaction done\n");
    }

    /* Initialize ESB radio */
    printk("[DIAG] Initializing ESB radio...\n");
    ret = esb_radio_init(RF_CHANNEL);
    if (ret) {
        printk("[DIAG] ESB radio init FAILED: %d\n", ret);
        goto run_spi_only;
    }
    printk("[DIAG] ESB radio OK\n");

    /* Initialize TDMA timing */
    printk("[DIAG] Initializing TDMA...\n");
    ret = tdma_init();
    if (ret) {
        printk("[DIAG] TDMA init FAILED: %d\n", ret);
        goto run_spi_only;
    }
    printk("[DIAG] TDMA OK\n");

    /* Initialize mesh protocol */
    printk("[DIAG] Initializing mesh protocol...\n");
    ret = mesh_protocol_init();
    if (ret) {
        printk("[DIAG] Mesh protocol init FAILED: %d\n", ret);
        goto run_spi_only;
    }
    printk("[DIAG] Mesh protocol OK\n");

    /* NOTE: Don't start mesh automatically - wait for ESP32 command.
     * This allows both boards to be ready before mesh discovery starts,
     * ensuring proper coordinator election. */
    printk("[DIAG] Mesh initialized, waiting for ESP32 enable command\n");

run_spi_only:
    /* Start dedicated SPI polling thread and let main loop stay lightweight */
    k_thread_create(&s_spi_thread, s_spi_thread_stack, K_THREAD_STACK_SIZEOF(s_spi_thread_stack),
                    spi_bridge_thread, NULL, NULL, NULL, SPI_THREAD_PRIORITY, 0, K_NO_WAIT);
    printk("[DIAG] Entering main loop (SPI thread active)\n");

    uint32_t loop_count = 0;
    while (1) {
        loop_count++;
        if (loop_count % 100 == 0) {
            printk("."); /* Quick heartbeat every 10 seconds */
        }
        if (loop_count % 300 == 0) {
            printk("\n[DIAG] heartbeat: main=%u spi=%u\n", loop_count, s_spi_loop_count);
        }

        /* Sleep to let other tasks run */
        k_sleep(K_MSEC(100));
    }

    return 0;
}
