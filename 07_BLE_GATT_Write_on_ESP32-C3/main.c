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

// GATT = Generic Attribute Service
// Led Service
// UUID = Universal unique identifier

static const ble_uuid128_t led_service_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a,
    0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12,
    0x00, 0x00, 0x00, 0x01
);

static const ble_uuid128_t led_state_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a,
    0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12,
    0x00, 0x00, 0x00, 0x02
);

static uint8_t led_state = 0;

static int led_state_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    if(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR){
        int rc = os_mbuf_append(ctxt->om, &led_state, sizeof(led_state));
        if(rc != 0) return BLE_ATT_ERR_INSUFFICIENT_RES;
        ESP_LOGI(TAG, "Led state read = %d", led_state);
        return 0;
    }

    if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR){
        uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
        if(data_len != sizeof(led_state)){
            ESP_LOGW(TAG, "Invalid data length: %u, expected: %u", data_len, (unsigned int)sizeof(led_state));
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint8_t new_state;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, &new_state, sizeof(new_state), NULL);
        if(rc != 0){
            ESP_LOGE(TAG, "Failed to read received data.");
            return BLE_ATT_ERR_UNLIKELY;
        }
        if(new_state != 0 && new_state != 1){
            ESP_LOGW(TAG, "Invalid LED state: %u", new_state);
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        led_state = new_state;
        if(led_state == 1) ESP_LOGI(TAG, "LED ON");
        else ESP_LOGI(TAG, "LED OFF");
        ESP_LOGI(TAG, "Write request received, data length = %u", data_len);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_chr_def led_characteristics[] = {
    {
        .uuid = &led_state_uuid.u,
        .access_cb = led_state_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    { 0 }
};

static const struct ble_gatt_svc_def gatt_services[] = {
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &led_service_uuid.u,
            .characteristics = led_characteristics,
        },
        { 0 }
}; // { 0 }

static int gatt_server_init(void){
    int rc;
    rc = ble_gatts_count_cfg(gatt_services);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to count GATT configuration, rc=%d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0){
        ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "LED GATT service registered.");
    return 0;
}

static void start_advertising();

static int ble_gap_event(struct ble_gap_event *event, void *arg){
    switch(event->type){
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete, reason %d", event->adv_complete.reason);
            break;

        case BLE_GAP_EVENT_CONNECT:
            if(event->connect.status == 0){
                ESP_LOGI(TAG, "Connected...");
            } else{
                ESP_LOGI(TAG, "Connection failed, status %d", event->connect.status);
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason %d", event->disconnect.reason);
            start_advertising();
            break;

        default:
            break;
    }
    return 0;
}

static void start_advertising(){
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    int rc;
    uint8_t own_addr_type;
    struct ble_gap_adv_params adv_params = {0};

    // ------- Step A: Prepare advertising data ---------------------------
    const char *name = ble_svc_gap_device_name();

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    rc = ble_gap_adv_set_fields(&fields);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to set advertising data %d", rc);
        return;
    }

    rsp_fields.name = (uint8_t *) name;
    rsp_fields.name_len = (uint8_t) strlen(name);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to set scan response data %d", rc);
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
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
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
vTaskDelay(pdMS_TO_TICKS(5000));
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
    int rc = gatt_server_init();
    if (rc != 0){
        ESP_LOGE(TAG, "GATT server init failed, rc=%d", rc);
        return;
    }
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
