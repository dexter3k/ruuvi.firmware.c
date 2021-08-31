#ifndef APP_ENSTO_H
#define APP_ENSTO_H

#include "ruuvi_interface_communication_ble_advertising.h"

#define HISTORY_FACTOR 90
#define MAX_ENTRIES 20

typedef struct {
    uint8_t addr[BLE_MAC_ADDRESS_LENGTH];
    int rssi;
    int reps;
} my_data_entry;

extern my_data_entry tags_around[MAX_ENTRIES];

extern int tags_used_entries;

rd_status_t my_on_scan_isr (const ri_comm_evt_t evt, void * p_data, size_t data_len);

#endif // APP_ENSTO_H
