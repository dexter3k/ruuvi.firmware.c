/**
 * @defgroup main Program main
 *
 */
/** @{ */
/**
 * @file main.c
 * @author Otso Jousimaa <otso@ojousima.net>
 * @date 2020-07-13
 * @copyright Ruuvi Innovations Ltd, license BSD-3-Clause.
 */
#include "app_config.h"
#include "app_button.h"
#include "app_comms.h"
#include "app_heartbeat.h"
#include "app_led.h"
#include "app_log.h"
#include "app_power.h"
#include "app_sensor.h"
#include "main.h"
#include "run_integration_tests.h"
#include "ruuvi_interface_log.h"
#include "ruuvi_interface_power.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_watchdog.h"
#include "ruuvi_interface_yield.h"
#include "ruuvi_task_button.h"
#include "ruuvi_task_flash.h"
#include "ruuvi_task_gpio.h"
#include "ruuvi_task_led.h"
#include "ruuvi_task_adc.h"

#include "ruuvi_interface_communication_ble_advertising.h"
#include "ruuvi_task_advertisement.h"

#include "app_ensto.h"

#include <string.h>

#if (!RUUVI_RUN_TESTS)
#ifndef CEEDLING
static
#endif
void on_wdt (void)
{
    // Store cause of reset to flash - TODO
}
#endif

#ifndef CEEDLING
static
#endif
void app_on_error (const rd_status_t error,
                   const bool fatal,
                   const char * file,
                   const int line)
{
    // TODO: store error source to flash.
    if (fatal)
    {
        ri_power_reset();
    }
}

my_data_entry tags_around[MAX_ENTRIES] = {};
int tags_used_entries = 0;

// if found, return 0, idx_out = index of the entry
// not found, return -1, idx_out = potential index of the entry
int find_tag_idx(uint8_t const * addr, int * idx_out) {
    if (tags_used_entries == 0) {
        *idx_out = 0;
        return -1;
    }

    int c = 0;
    int r = 0;
    int first = 0, last = tags_used_entries - 1;
    do {
        c = (last + first) / 2;
        r = memcmp(addr, tags_around[c].addr, BLE_MAC_ADDRESS_LENGTH);
        if (r == 0) {
            *idx_out = c;
            return 0;
        }

        if (r < 0) {
            last = c - 1;
        } else {
            first = c + 1;
        }
    } while (first <= last);

    if (r < 0) {
        *idx_out = c;
    } else {
        *idx_out = c + 1;
    }
    return -1;
}

// -1 error
// 0 updated
// 1 new added
int update_tag_info(uint8_t const * addr, int8_t rssi) {
    rssi = -rssi;
    if (rssi < 0) {
        rssi = 0;
    }

    int idx = 0;
    if (find_tag_idx(addr, &idx) == -1) {
        // not found, add it
        if (tags_used_entries == MAX_ENTRIES || idx == MAX_ENTRIES) {
            // find entry with worst signal and replace it with current
            ri_log(RI_LOG_LEVEL_INFO, "no space for new entry, eviction not implemented\n");
            return -1;
        }

        memmove(tags_around + idx + 1, tags_around + idx, sizeof(my_data_entry) * (tags_used_entries - idx));
        tags_used_entries++;

        memcpy(tags_around[idx].addr, addr, BLE_MAC_ADDRESS_LENGTH);

        tags_around[idx].rssi = (int)rssi;
        tags_around[idx].reps = 1;
        return 1;
    }

    tags_around[idx].rssi = (tags_around[idx].rssi * HISTORY_FACTOR + (int)rssi * (100 - HISTORY_FACTOR)) / 100;
    tags_around[idx].reps++;
    return 0;
}

rd_status_t my_on_scan_isr (const ri_comm_evt_t evt, void * p_data, size_t data_len)
{
    ri_adv_scan_t * d = p_data;
    if (evt == RI_COMM_TIMEOUT) {
        // char buf[64];
        // sprintf(buf, "scan timeout %d\n", (int)evt);
        // ri_log(RI_LOG_LEVEL_INFO, buf);
        // for (int i = 0; i < tags_used_entries; i++) {
        //     my_data_entry * e = tags_around + i;
        //     sprintf(buf, "%d: %02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX %d %d reps\n",
        //         i,
        //         e->addr[0], e->addr[1], e->addr[2],
        //         e->addr[3], e->addr[4], e->addr[5],
        //         e->rssi, e->reps);
        //     ri_log(RI_LOG_LEVEL_INFO, buf);
        // }

        rt_adv_scan_start (my_on_scan_isr);

        return RD_SUCCESS;
    }
    if (!d) {
        return RD_SUCCESS;
    }
    if (d->data_len != 31 || d->data[6] != 0x04 || d->data[5] != 0x99) {
        // skipping non-ruuvitags
        return RD_SUCCESS;
    }

    char buf[32];
    int ret = update_tag_info(d->addr, d->rssi);
    if (ret != 0 && ret != 1) {
        sprintf(buf, "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX %d\n",
            d->addr[0], d->addr[1], d->addr[2],
            d->addr[3], d->addr[4], d->addr[5],
            (int)d->rssi);
        ri_log (RI_LOG_LEVEL_INFO, buf);

        if (ret == -1) {
            ri_log(RI_LOG_LEVEL_INFO, "failed to update tag info\n");
        } else {
            sprintf(buf, "unknown ret %d\n", ret);
            ri_log(RI_LOG_LEVEL_INFO, buf);
        }
    }

    // // ri_delay_ms (25);

    return RD_SUCCESS;
}

/**
 * @brief setup MCU peripherals and board peripherals.
 *
 */
void setup (void)
{
    rd_status_t err_code = RD_SUCCESS;
    float motion_threshold = APP_MOTION_THRESHOLD;
#   if (!RUUVI_RUN_TESTS)
    err_code |= ri_watchdog_init (APP_WDT_INTERVAL_MS, &on_wdt);
    err_code |= ri_log_init (APP_LOG_LEVEL); // Logging to terminal.
#   endif
    err_code |= ri_yield_init();
    err_code |= ri_timer_init();
    err_code |= ri_scheduler_init();
    err_code |= rt_gpio_init();
    err_code |= ri_yield_low_power_enable (true);
    err_code |= rt_flash_init();
    err_code |= app_led_init();
    err_code |= app_led_activate (RB_LED_STATUS_ERROR);
    err_code |= app_button_init();
    err_code |= app_dc_dc_init();
    err_code |= app_sensor_init();
    err_code |= app_log_init();
    // Allow fail on boards which do not have accelerometer.
    (void) app_sensor_acc_thr_set (&motion_threshold);
    err_code |= app_comms_init (APP_LOCKED_AT_BOOT);
    err_code |= app_sensor_vdd_sample();
    err_code |= app_heartbeat_init();
    err_code |= app_heartbeat_start();
    err_code |= app_led_deactivate (RB_LED_STATUS_ERROR);

    // add adv scanning
    rt_adv_scan_start (my_on_scan_isr);

    if (RD_SUCCESS == err_code)
    {
        err_code |= app_led_activate (RB_LED_STATUS_OK);
        err_code |= ri_delay_ms (APP_SELFTEST_OK_DELAY_MS);
        err_code |= app_led_deactivate (RB_LED_STATUS_OK);
    }

    err_code |= app_led_activity_set (RB_LED_ACTIVITY);
    rd_error_cb_set (&app_on_error);
    RD_ERROR_CHECK (err_code, RD_SUCCESS);
}

#ifdef  CEEDLING
int app_main (void)
#else
int main (void)
#endif
{
#   if RUUVI_RUN_TESTS
    integration_tests_run();
#   endif
    setup();

    do
    {
        ri_scheduler_execute();
        // Sleep - woken up on event
        // Do not indicate activity here to conserve power.
        // Sensor reads take ~20ms, having led on for that time is expensive.
        ri_yield();
        // Prevent loop being optimized away
        __asm__ ("");
    } while (LOOP_FOREVER);
  
    // Intentionally non-reachable code unless unit testing.
    return -1;
}

/** @} */
