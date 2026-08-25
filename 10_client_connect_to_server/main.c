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
#define TARGET_DEVICE_NAME "ESP32-C3_BLE"
static uint16_t client_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static const ble_uuid128_t led_service_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a,
    0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12,
    0x00, 0x00, 0x00, 0x01
);

static const ble_uuid128_t led_chr_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a,
    0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12,
    0x00, 0x00, 0x00, 0x02
);

static uint16_t led_chr_value_handle;

static uint16_t led_service_start_handle;
static uint16_t led_service_end_handle;


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

static int led_chr_disc_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr,
    void *arg
){
    if(error->status == 0){
        led_chr_value_handle = chr->val_handle;

        ESP_LOGI(TAG,
            "LED characteristic found: "
            "definition_handle=%u, value_handle=%u, properties=0x%02X",
            chr->def_handle,
            chr->val_handle,
            chr->properties
        );

        return 0;
    }

    if(error->status == BLE_HS_EDONE){
        ESP_LOGI(TAG,
            "Characteristic discovery completed. "
            "LED value handle=%u",
            led_chr_value_handle
        );

        return 0;
    }

    ESP_LOGE(TAG,
        "Characteristic discovery failed, status=%d",
        error->status
    );

    return error->status;
}

static int led_service_disc_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg
){
    if(error->status == 0){
        led_service_start_handle = service->start_handle;
        led_service_end_handle = service->end_handle;

        ESP_LOGI(TAG,
                 "LED Service found, start_handle=%u, end_handle=%u",
                 led_service_start_handle,
                 led_service_end_handle);

        return 0;
    }

    if(error->status == BLE_HS_EDONE){
        ESP_LOGI(TAG, "LED Service discovery complete.");
        int rc = ble_gattc_disc_chrs_by_uuid(
            conn_handle,
            led_service_start_handle,
            led_service_end_handle,
            &led_chr_uuid.u,
            led_chr_disc_cb,
            NULL
        );

        if(rc != 0){
            ESP_LOGE(TAG,
                "Failed to start characteristic discovery, rc=%d",
                rc
            );
        }
        return 0;
    }

    ESP_LOGE(TAG,
             "LED Service discovery failed, status=%d",
             error->status);

    return 0;
}

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
                if(strcmp(name, TARGET_DEVICE_NAME) == 0){
                    ESP_LOGI(TAG, "Target server found.");
                    int cancel_rc = ble_gap_disc_cancel();
                    if(cancel_rc == 0){
                        ESP_LOGI(TAG, "Scanning stopped.");
                        uint8_t out_addr_type;
                        int addr_rc = ble_hs_id_infer_auto(0, &out_addr_type);
                        if(addr_rc != 0){
                            ESP_LOGE(TAG, "Failed to infer address type, rc = %d", addr_rc);
                            return 0;
                        }
                        int connect_rc = ble_gap_connect(out_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
                        if(connect_rc != 0){
                            ESP_LOGE(TAG, "Failed to start connection, rc = %d", connect_rc);
                        }else{
                            ESP_LOGI(TAG, "Connection started.");
                        }
                    }else{
                        ESP_LOGE(TAG, "Failed to stop scanning, rc = %d", cancel_rc);
                        return 0;
                    }
                }
                ESP_LOGI(TAG, " ");
            } else {
                ESP_LOGI(TAG, "Device name <unknown>");
                ESP_LOGI(TAG, " ");
            } 
            
        }
        break;

        case BLE_GAP_EVENT_CONNECT:
            if(event->connect.status == 0){
                client_conn_handle = event->connect.conn_handle;
                int rc = ble_gattc_disc_svc_by_uuid(
                    client_conn_handle,
                    &led_service_uuid.u,
                    led_service_disc_cb,
                    NULL
                );

                if(rc != 0){
                    ESP_LOGE(TAG,
                            "Failed to start LED Service discovery, rc=%d",
                            rc);
                } else {
                    ESP_LOGI(TAG, "LED Service discovery started.");
                }
                ESP_LOGI(TAG, "Connected to BLE server, conn_handle=%u", client_conn_handle);
            }else{
                ESP_LOGE(TAG, "Connection failed, status =%d", event->connect.status);
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

// // GATT = Generic Attribute Service
// // Led Service
// // UUID = Universal unique identifier
// // Notifications
// // CCCD - Client Characteristic Configuration Descriptor, start cond. - 0x0000, Notification - 0x0001, Indication - 0x0002
// // Indications

// static const ble_uuid128_t led_service_uuid = BLE_UUID128_INIT(
//     0xf0, 0xde, 0xbc, 0x9a,
//     0x78, 0x56, 0x34, 0x12,
//     0x78, 0x56, 0x34, 0x12,
//     0x00, 0x00, 0x00, 0x01
// );

// static const ble_uuid128_t led_state_uuid = BLE_UUID128_INIT(
//     0xf0, 0xde, 0xbc, 0x9a,
//     0x78, 0x56, 0x34, 0x12,
//     0x78, 0x56, 0x34, 0x12,
//     0x00, 0x00, 0x00, 0x02
// );

// static uint8_t led_state = 0;
// static uint16_t led_state_val_handle;
// static bool led_indicate_enabled = false;
// static bool led_indication_pending = false;

// static int led_state_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
//     if(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR){
//         int rc = os_mbuf_append(ctxt->om, &led_state, sizeof(led_state));
//         if(rc != 0) return BLE_ATT_ERR_INSUFFICIENT_RES;
//         ESP_LOGI(TAG, "Led state read = %d", led_state);
//         return 0;
//     }

//     if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR){
//         uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
//         if(data_len != sizeof(led_state)){
//             ESP_LOGW(TAG, "Invalid data length: %u, expected: %u", data_len, (unsigned int)sizeof(led_state));
//             return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
//         }
//         uint8_t new_state;
//         int rc = ble_hs_mbuf_to_flat(ctxt->om, &new_state, sizeof(new_state), NULL);
//         if(rc != 0){
//             ESP_LOGE(TAG, "Failed to read received data.");
//             return BLE_ATT_ERR_UNLIKELY;
//         }
//         if(new_state != 0 && new_state != 1){
//             ESP_LOGW(TAG, "Invalid LED state: %u", new_state);
//             return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
//         }
//         led_state = new_state;
//         if(led_state == 1) ESP_LOGI(TAG, "LED ON");
//         else ESP_LOGI(TAG, "LED OFF");

//         if(led_indicate_enabled && !led_indication_pending){
//             struct os_mbuf *indicate_data = ble_hs_mbuf_from_flat(&led_state, sizeof(led_state));
//             if(indicate_data == NULL) ESP_LOGE(TAG, "Failed to allocate indication buffer.");
//             else {
//                int indicate_rc = ble_gatts_indicate_custom(conn_handle, led_state_val_handle, indicate_data);
//                if(indicate_rc == 0){
//                 led_indication_pending = true;
//                 ESP_LOGI(TAG, "LED state indication queued: %u", led_state);
//                }else ESP_LOGE(TAG, "Failed to send LED state indication rc=%d", indicate_rc);
//             }
//         }

//         ESP_LOGI(TAG, "Write request received, data length = %u", data_len);
//         return 0;
//     }
//     return BLE_ATT_ERR_UNLIKELY;
// }

// static const struct ble_gatt_chr_def led_characteristics[] = {
//     {
//         .uuid = &led_state_uuid.u,
//         .access_cb = led_state_access,
//         .val_handle = &led_state_val_handle,
//         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
//     },
//     { 0 }
// };

// static const struct ble_gatt_svc_def gatt_services[] = {
//         {
//             .type = BLE_GATT_SVC_TYPE_PRIMARY,
//             .uuid = &led_service_uuid.u,
//             .characteristics = led_characteristics,
//         },
//         { 0 }
// }; // { 0 }

// static int gatt_server_init(void){
//     int rc;
//     rc = ble_gatts_count_cfg(gatt_services);
//     if(rc != 0){
//         ESP_LOGE(TAG, "Failed to count GATT configuration, rc=%d", rc);
//         return rc;
//     }

//     rc = ble_gatts_add_svcs(gatt_services);
//     if (rc != 0){
//         ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
//         return rc;
//     }

//     ESP_LOGI(TAG, "LED GATT service registered.");
//     return 0;
// }

// static void start_advertising();

// static int ble_gap_event(struct ble_gap_event *event, void *arg){
//     switch(event->type){
//         case BLE_GAP_EVENT_ADV_COMPLETE:
//             ESP_LOGI(TAG, "Advertising complete, reason %d", event->adv_complete.reason);
//             break;

//         case BLE_GAP_EVENT_CONNECT:
//             if(event->connect.status == 0){
//                 ESP_LOGI(TAG, "Connected...");
//             } else{
//                 ESP_LOGI(TAG, "Connection failed, status %d", event->connect.status);
//                 start_advertising();
//             }
//             break;

//         case BLE_GAP_EVENT_SUBSCRIBE:
//             if(event->subscribe.attr_handle == led_state_val_handle){
//                 led_indicate_enabled = event->subscribe.cur_indicate;
//                 if(led_indicate_enabled){
//                     ESP_LOGI(TAG, "Client subscribed to LED state indications.");
//                 } else{
//                     ESP_LOGI(TAG, "Client unsubscribed from LED state indications.");
//                 }
//             }
//             break;

//         case BLE_GAP_EVENT_NOTIFY_TX:
//             if(event->notify_tx.indication && event->notify_tx.attr_handle == led_state_val_handle){
//                 if(event->notify_tx.status == 0){
//                     ESP_LOGI(TAG, "Led state indication transmitted, waiting for confirmation.");
//                 } else if(event->notify_tx.status == BLE_HS_EDONE){
//                     led_indication_pending = false;
//                     ESP_LOGI(TAG, "Led state indication confirmed by client.");
//                 }  else if(event->notify_tx.status == BLE_HS_ETIMEOUT){
//                     led_indication_pending = false;
//                     ESP_LOGI(TAG, "Led indication confirmation timeout.");
//                 }  else{
//                     led_indication_pending = false;
//                     ESP_LOGE(TAG, "Led state indication failed, status %d", event->notify_tx.status);
//                 }         
//             }
//             break;

//         case BLE_GAP_EVENT_DISCONNECT:
//             ESP_LOGI(TAG, "Disconnected, reason %d", event->disconnect.reason);
//             led_indicate_enabled = false;
//             led_indication_pending = false;
//             start_advertising();
//             break;

//         default:
//             break;
//     }
//     return 0;
// }

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
//     rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
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
//     rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
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
// vTaskDelay(pdMS_TO_TICKS(5000));
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
//     int rc = gatt_server_init();
//     if (rc != 0){
//         ESP_LOGE(TAG, "GATT server init failed, rc=%d", rc);
//         return;
//     }
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
