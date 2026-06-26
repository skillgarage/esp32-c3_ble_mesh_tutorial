#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BLE_MESH"

static void start_scanning();

static void ble_addr_to_str(const ble_addr_t *addr, char *str){
    sprintf(str,
    "%02X:%02X:%02X:%02X:%02X:%02X",
            addr->val[5],
            addr->val[4],
            addr->val[3],
            addr->val[2],
            addr->val[1],
            addr->val[0]); // AA:BB:CC:DD:EE:FF
}

    // ------- Step A: Prepare advertising data ---------------------------
    // const char *name = ble_svc_gap_device_name();

    // fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    // fields.name = (uint8_t *) name;
    // fields.name_len = (uint8_t) strlen(name);
    // fields.name_is_complete = 1;

    // rc = ble_gap_adv_set_fields(&fields);
    // if(rc != 0){
    //     ESP_LOGE(TAG, "Failed to set advertising data %d", rc);
    //     return;
    // }
    // --------------------------------------------------------------------

static int ble_gap_event(struct ble_gap_event *event, void *arg){
    switch(event->type){
        case BLE_GAP_EVENT_DISC:{
            char addr_str[18];
            ble_addr_to_str(&event->disc.addr, addr_str);

            ESP_LOGI(TAG, "Device found addr=%s, rssi=%d dBm, data_len=%d, Advertising PDU type = %d", addr_str, 
                event->disc.rssi, event->disc.length_data, event->disc.event_type);

            ESP_LOG_BUFFER_HEX(TAG, event->disc.data, event->disc.length_data);

            struct ble_hs_adv_fields fields;
            int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if(rc == 0 && fields.name != NULL){
                char name[32];
                int name_len = fields.name_len;
                if(name_len >= sizeof(name)){
                    name_len = sizeof(name) - 1; // \0
                }
                memcpy(name, fields.name, name_len);

                name[name_len] = '\0';
                ESP_LOGI(TAG, "Device name %s", name);
                ESP_LOGI(TAG, " ");
            } else {
                ESP_LOGI(TAG, "Device name <unknown>");
                ESP_LOGI(TAG, " ");
            } 
            
        }
        break;

        default: break;
    }
    return 0;
}

static void start_scanning(){
    int rc;
    uint8_t own_addr_type;

    struct ble_gap_disc_params scan_params = {0};
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to infer address type %d", rc);
        return;
    }

    scan_params.filter_duplicates = 1;
    scan_params.passive = 0;
    scan_params.itvl = 0x0010; // 16 * 0.625 = 10
    scan_params.window = 0x0010;
    scan_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    scan_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, 0, &scan_params, ble_gap_event, NULL);
    if(rc != 0) {
        ESP_LOGE(TAG, "Failed to start scanning %d", rc);
        return;
    } else ESP_LOGI(TAG, "Start scanning.");
}

static void on_stack_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE reset, reason=%d", reason);
}

static void on_stack_sync(void)
{
    ESP_LOGI(TAG, "NimBLE stack synchronized");
    start_scanning();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");

    nimble_port_run();

    nimble_port_freertos_deinit();
}


void app_main(){
    vTaskDelay(pdMS_TO_TICKS(7000));
    // -------------  STEP 1 ---------------------------------------
    esp_err_t res = nvs_flash_init();
    if(res == ESP_ERR_NVS_NO_FREE_PAGES){
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK(res);
// -------------------------------------------------------------

// ------------ STEP 2 ----------------------------------------
    ESP_ERROR_CHECK(nimble_port_init());
// ----------------------------------------------------------

// -------------  STEP 3 ------------------------------------------
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
// ----------------------------------------------------------------

// ------------ STEP 4 ----------------------------------------
    nimble_port_freertos_init(ble_host_task);
// ----------------------------------------------------------
}

// #include <stdio.h>
// #include <string.h>

// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_err.h"

// #include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"

// #include "host/ble_hs.h"
// #include "host/ble_gap.h"
// #include "host/util/util.h"

// #include "services/gap/ble_svc_gap.h"
// #include "services/gatt/ble_svc_gatt.h"

// #define TAG "BLE_MESH"

// static void start_advertising(){
//     struct ble_hs_adv_fields fields = {0};
//     struct ble_hs_adv_fields rsp_fields = {0};
//     int rc;
//     uint8_t own_addr_type;
//     struct ble_gap_adv_params adv_params = {0};

//     // ------- Step A: Prepare advertising data ---------------------------
//     const char *name = ble_svc_gap_device_name();

//     fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

//     rc = ble_gap_adv_set_fields(&fields);
//     if(rc != 0){
//         ESP_LOGE(TAG, "Failed to set advertising data %d", rc);
//         return;
//     }

//     rsp_fields.name = (uint8_t *) name;
//     rsp_fields.name_len = (uint8_t) strlen(name);
//     rsp_fields.name_is_complete = 1;
    // rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
//     if(rc != 0){
//         ESP_LOGE(TAG, "Failed to set scan response data %d", rc);
//         return;
//     }
//     // --------------------------------------------------------------------

//     // ------- Step B: Get BLE address type ---------------------------
//     rc = ble_hs_id_infer_auto(0, &own_addr_type);
//     if (rc != 0){
//         ESP_LOGE(TAG, "Failed to infer address type %d", rc);
//         return;
//     }
//     // --------------------------------------------------------------------

//     // ------- Step C: Configure advertising mode ---------------------------
//     adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
//     adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
//     // --------------------------------------------------------------------

//     // ------- Step D: Start advertising ---------------------------
//     rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
//     if(rc != 0){
//         ESP_LOGE(TAG, "Failed to start advertising, %d", rc);
//         return;
//     }

//     ESP_LOGI(TAG, "Advertising started!"); // nRF Connect
//     // --------------------------------------------------------------------
// }

// static void on_stack_reset(int reason){
//     ESP_LOGI(TAG, "NIMBLE reset, reason %d", reason);
// }

// static void on_stack_sync(){
//     ESP_LOGI(TAG, "NIMBLE stack synchronized");
//     start_advertising();
// }

// static void ble_host_task(void *param){
//     ESP_LOGI(TAG, "BLE host task started.");
//     nimble_port_run();
//     nimble_port_freertos_deinit();
// }

// void app_main(){
// // -------------  STEP 1 ---------------------------------------
//     esp_err_t res = nvs_flash_init();
//     if(res == ESP_ERR_NVS_NO_FREE_PAGES){
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         res = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(res);
// // -------------------------------------------------------------

// // ------------ STEP 2 ----------------------------------------
//     ESP_ERROR_CHECK(nimble_port_init());
// // ----------------------------------------------------------

// // -------------  STEP 3 ------------------------------------------
//     ble_hs_cfg.reset_cb = on_stack_reset;
//     ble_hs_cfg.sync_cb = on_stack_sync;
// // ----------------------------------------------------------------

// // ------------ STEP 4 ----------------------------------------
//     ble_svc_gap_device_name_set("ESP32-C3_BLE");
// // ----------------------------------------------------------

// // ------------ STEP 5 ----------------------------------------
//     nimble_port_freertos_init(ble_host_task);
// // ----------------------------------------------------------
// }
