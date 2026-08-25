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

static void start_advertising(){
    struct ble_hs_adv_fields fields = {0};
    int rc;
    uint8_t own_addr_type;
    struct ble_gap_adv_params adv_params = {0};

    // ------- Step A: Prepare advertising data ---------------------------
    const char *name = ble_svc_gap_device_name();

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *) name;
    fields.name_len = (uint8_t) strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to set advertising data %d", rc);
        return;
    }
    // --------------------------------------------------------------------

    // ------- Step B: Get BLE address type ---------------------------
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0){
        ESP_LOGE(TAG, "Failed to infer address type %d", rc);
        return;
    }
    // --------------------------------------------------------------------

    // ------- Step C: Configure advertising mode ---------------------------
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // --------------------------------------------------------------------

    // ------- Step D: Start advertising ---------------------------
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to start advertising, %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started!"); // nRF Connect
    // --------------------------------------------------------------------
}

static void on_stack_reset(int reason){
    ESP_LOGI(TAG, "NIMBLE reset, reason %d", reason);
}

static void on_stack_sync(){
    ESP_LOGI(TAG, "NIMBLE stack synchronized");
    start_advertising();
}

static void ble_host_task(void *param){
    ESP_LOGI(TAG, "BLE host task started.");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(){
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
    ble_svc_gap_device_name_set("ESP32-C3_BLE");
// ----------------------------------------------------------

// ------------ STEP 5 ----------------------------------------
    nimble_port_freertos_init(ble_host_task);
// ----------------------------------------------------------
}
