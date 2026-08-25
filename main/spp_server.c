/*
    BLE SPP Server Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
*/
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "cmd.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern SemaphoreHandle_t led_sync;
extern int led_ble;

static const char *TAG = "SPP";

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56

#define DEVICE_NAME "ESP32_BRIDGE"  // The Device Name Characteristics in GAP

#define SPP_SVC_INST_ID 0

#define SPP_QUEUE_LEN 256
#define UART_QUEUE_LEN 256
#define QUEUE_SEND_TIMEOUT_MS 10

/// Attributes State Machine
enum {
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NOTIFY_VAL,
    SPP_IDX_SPP_DATA_NOTIFY_CFG,

    SPP_IDX_CTRL_CHAR,
    SPP_IDX_CTRL_VAL,

    SPP_IDX_LINE_CHAR,
    SPP_IDX_LINE_VAL,

    SPP_IDX_STATE_CHAR,
    SPP_IDX_STATE_VAL,
    SPP_IDX_STATE_CFG,

    SPP_IDX_NB,
};

/* Nordic UART Service (NUS), little-endian byte order
 * Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX (client writes):  6E400002-...
 * TX (server notifies): 6E400003-... */
static const uint8_t nus_service_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
static const uint8_t nus_rx_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
static const uint8_t nus_tx_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
/* Vendor extensions beyond standard NUS:
 * 6E400004: control lines, 1 byte, bit0 = DTR, bit1 = RTS (write/read)
 * 6E400005: line coding, 7 bytes, USB CDC layout:
 *           baud LE32 + stop bits + parity + data bits (write/read)
 * 6E400006: serial state, 2 bytes LE, USB CDC SerialState bitmap
 *           (read/notify) */
static const uint8_t nus_ctrl_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x04, 0x00, 0x40, 0x6E};
static const uint8_t nus_line_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x05, 0x00, 0x40, 0x6E};
static const uint8_t nus_state_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x06, 0x00, 0x40, 0x6E};

extern QueueHandle_t xQueueSpp;
extern QueueHandle_t xQueueUartTX;

static uint16_t spp_handle_table[SPP_IDX_NB];

/* Negotiated ATT MTU; notifications carry at most MTU - 3 bytes */
static volatile uint16_t spp_mtu = 23;

/* esptool sync assist: the ESP ROM answers one SYNC command with 8
 * identical back-to-back replies, but this bridge's USB host (single
 * in-flight transfer, software resubmit) loses the tail of that burst to
 * the ROM's USB TX timeout. esptool insists on all 8, so after seeing a
 * SYNC command pass through we top the replies back up to 8 — they are
 * identical, clients cannot tell. Everything else esptool does is plain
 * request/response and needs no help. */
static bool sync_pending = false;

static const uint8_t sync_cmd_magic[4] = {0xC0, 0x00, 0x08, 0x24};
static const uint8_t sync_rsp_magic[4] = {0xC0, 0x01, 0x08, 0x04};

static bool contains_seq(const uint8_t *buf, uint16_t len, const uint8_t *pat,
                         uint16_t pat_len) {
    for (uint16_t i = 0; i + pat_len <= len; i++) {
        if (memcmp(buf + i, pat, pat_len) == 0) return true;
    }
    return false;
}

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

static uint8_t test_manufacturer[3] = {'E', 'S', 'P'};

// config adv data
static esp_ble_adv_data_t spp_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006,  // slave connection min interval, Time =
                             // min_interval * 1.25 msec
    .max_interval = 0x0006,  // slave connection max interval, Time =
                             // max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,  //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(nus_service_uuid),
    .p_service_uuid = (uint8_t *)nus_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
static esp_ble_adv_data_t spp_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the
 * gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is
                                             ESP_GATT_IF_NONE */
        },
};

/*
 *	SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid =
    ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t char_prop_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE |  // Write with response
    ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static const uint8_t char_prop_read_write =
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

/// NUS RX characteristic value (client writes serial data here)
static const uint8_t spp_data_receive_val[SPP_DATA_MAX_LEN] = {0x00};

/// NUS TX characteristic value (serial data notified to client)
static const uint8_t spp_data_notify_val[SPP_DATA_MAX_LEN] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// Control lines: bit0 = DTR, bit1 = RTS, both deasserted by default
static const uint8_t ctrl_lines_val[1] = {0x00};
/// Line coding, USB CDC layout: 115200 baud, 1 stop bit, no parity, 8 data
static const uint8_t line_coding_val[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};
/// Serial state bitmap (USB CDC SerialState), updated from the USB side
static const uint8_t serial_state_val[2] = {0x00, 0x00};
static const uint8_t serial_state_ccc[2] = {0x00, 0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] = {
    // Service Declaration
    [SPP_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                      ESP_GATT_PERM_READ, sizeof(nus_service_uuid),
                      sizeof(nus_service_uuid), (uint8_t *)nus_service_uuid}},

    // Data Receive Characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16,
                                     (uint8_t *)&character_declaration_uuid,
                                     ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                                     CHAR_DECLARATION_SIZE,
                                     (uint8_t *)&char_prop_write}},

    // Data Receive Characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_128, (uint8_t *)nus_rx_uuid, ESP_GATT_PERM_WRITE,
          SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val),
          (uint8_t *)spp_data_receive_val}},

    // Data Notify Characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
          (uint8_t *)&char_prop_read_notify}},

    // Data Notify Characteristic Value
    [SPP_IDX_SPP_DATA_NOTIFY_VAL] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_128, (uint8_t *)nus_tx_uuid,
                                      ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN,
                                      sizeof(spp_data_notify_val),
                                      (uint8_t *)spp_data_notify_val}},

    // Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NOTIFY_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
          sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    // Control Lines Characteristic Declaration
    [SPP_IDX_CTRL_CHAR] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16,
                            (uint8_t *)&character_declaration_uuid,
                            ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                            CHAR_DECLARATION_SIZE,
                            (uint8_t *)&char_prop_read_write}},

    // Control Lines Characteristic Value
    [SPP_IDX_CTRL_VAL] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_128, (uint8_t *)nus_ctrl_uuid,
                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                           sizeof(ctrl_lines_val), sizeof(ctrl_lines_val),
                           (uint8_t *)ctrl_lines_val}},

    // Line Coding Characteristic Declaration
    [SPP_IDX_LINE_CHAR] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16,
                            (uint8_t *)&character_declaration_uuid,
                            ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                            CHAR_DECLARATION_SIZE,
                            (uint8_t *)&char_prop_read_write}},

    // Line Coding Characteristic Value
    [SPP_IDX_LINE_VAL] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_128, (uint8_t *)nus_line_uuid,
                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                           sizeof(line_coding_val), sizeof(line_coding_val),
                           (uint8_t *)line_coding_val}},

    // Serial State Characteristic Declaration
    [SPP_IDX_STATE_CHAR] = {{ESP_GATT_AUTO_RSP},
                            {ESP_UUID_LEN_16,
                             (uint8_t *)&character_declaration_uuid,
                             ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                             CHAR_DECLARATION_SIZE,
                             (uint8_t *)&char_prop_read_notify}},

    // Serial State Characteristic Value
    [SPP_IDX_STATE_VAL] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_128, (uint8_t *)nus_state_uuid,
                            ESP_GATT_PERM_READ, sizeof(serial_state_val),
                            sizeof(serial_state_val),
                            (uint8_t *)serial_state_val}},

    // Serial State Client Characteristic Configuration Descriptor
    [SPP_IDX_STATE_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
         sizeof(serial_state_ccc), (uint8_t *)serial_state_ccc}}};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type) {
    char *key_str = NULL;
    switch (key_type) {
        case ESP_LE_KEY_NONE:
            key_str = "ESP_LE_KEY_NONE";
            break;
        case ESP_LE_KEY_PENC:
            key_str = "ESP_LE_KEY_PENC";
            break;
        case ESP_LE_KEY_PID:
            key_str = "ESP_LE_KEY_PID";
            break;
        case ESP_LE_KEY_PCSRK:
            key_str = "ESP_LE_KEY_PCSRK";
            break;
        case ESP_LE_KEY_PLK:
            key_str = "ESP_LE_KEY_PLK";
            break;
        case ESP_LE_KEY_LLK:
            key_str = "ESP_LE_KEY_LLK";
            break;
        case ESP_LE_KEY_LENC:
            key_str = "ESP_LE_KEY_LENC";
            break;
        case ESP_LE_KEY_LID:
            key_str = "ESP_LE_KEY_LID";
            break;
        case ESP_LE_KEY_LCSRK:
            key_str = "ESP_LE_KEY_LCSRK";
            break;
        default:
            key_str = "INVALID BLE KEY TYPE";
            break;
    }

    return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req) {
    char *auth_str = NULL;
    switch (auth_req) {
        case ESP_LE_AUTH_NO_BOND:
            auth_str = "ESP_LE_AUTH_NO_BOND";
            break;
        case ESP_LE_AUTH_BOND:
            auth_str = "ESP_LE_AUTH_BOND";
            break;
        case ESP_LE_AUTH_REQ_MITM:
            auth_str = "ESP_LE_AUTH_REQ_MITM";
            break;
        case ESP_LE_AUTH_REQ_BOND_MITM:
            auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
            break;
        case ESP_LE_AUTH_REQ_SC_ONLY:
            auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
            break;
        case ESP_LE_AUTH_REQ_SC_BOND:
            auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
            break;
        case ESP_LE_AUTH_REQ_SC_MITM:
            auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
            break;
        case ESP_LE_AUTH_REQ_SC_MITM_BOND:
            auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
            break;
        default:
            auth_str = "INVALID BLE AUTH REQ";
            break;
    }

    return auth_str;
}

static void show_bonded_devices(void) {
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list =
        (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(__FUNCTION__, "Bonded devices number : %d", dev_num);
    ESP_LOGI(__FUNCTION__, "Bonded devices list : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(__FUNCTION__, (void *)dev_list[i].bd_addr,
                           sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(__FUNCTION__, "GAP_EVT, event %d", event);
    CMD_t cmdBuf;
    BaseType_t err;

    switch (event) {
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&spp_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&spp_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // advertising start complete event to indicate advertising start
            // successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__FUNCTION__,
                         "advertising start failed, error status = %x",
                         param->adv_start_cmpl.status);
                break;
            }
            ESP_LOGI(__FUNCTION__, "advertising start success");
            break;
        case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
            /* Call the following function to input the passkey which is
             * displayed on the remote device */
            // esp_ble_passkey_reply(spp_profile_tab[SPP_PROFILE_APP_IDX].remote_bda,
            // true, 0x00);
            break;
        case ESP_GAP_BLE_OOB_REQ_EVT: {
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_OOB_REQ_EVT");
            uint8_t tk[16] = {1};  // If you paired with OOB, both devices need
                                   // to use the same tk
            esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk,
                                  sizeof(tk));
            break;
        }
        case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_IR_EVT");
            break;
        case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_ER_EVT");
            break;
        case ESP_GAP_BLE_NC_REQ_EVT:
            /* The app will receive this evt when the IO has DisplayYesNO
            capability and the peer device IO also has DisplayYesNo capability.
            show the passkey number to the user to confirm it with the number
            displayed by peer device. */
            esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
            ESP_LOGI(
                __FUNCTION__,
                "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32,
                param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            /* send the positive(true) security response to the peer device to
            accept the security request. If not accept the security request,
            should send the security response with negative(false) accept
            value*/
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  /// the app will receive this evt
                                             /// when the IO  has Output
                                             /// capability and the peer device
                                             /// IO has Input capability.
            /// show the passkey number to the user to input it in the peer
            /// device.
            ESP_LOGI(__FUNCTION__, "The passkey Notify number:%06" PRIu32,
                     param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_KEY_EVT:
            // shows the ble key info share with peer device to the user.
            ESP_LOGI(__FUNCTION__, "key type = %s",
                     esp_key_type_to_str(param->ble_security.ble_key.key_type));
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT: {
            esp_bd_addr_t bd_addr;
            memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
                   sizeof(esp_bd_addr_t));
            ESP_LOGI(__FUNCTION__, "remote BD_ADDR: %08x%04x",
                     (bd_addr[0] << 24) + (bd_addr[1] << 16) +
                         (bd_addr[2] << 8) + bd_addr[3],
                     (bd_addr[4] << 8) + bd_addr[5]);
            ESP_LOGI(__FUNCTION__, "address type = %d",
                     param->ble_security.auth_cmpl.addr_type);
            ESP_LOGI(
                __FUNCTION__, "pair status = %s",
                param->ble_security.auth_cmpl.success ? "success" : "fail");
            if (!param->ble_security.auth_cmpl.success) {
                ESP_LOGI(__FUNCTION__, "fail reason = 0x%x",
                         param->ble_security.auth_cmpl.fail_reason);
            } else {
                ESP_LOGI(__FUNCTION__, "auth mode = %s",
                         esp_auth_req_to_str(
                             param->ble_security.auth_cmpl.auth_mode));
            }
            show_bonded_devices();

            cmdBuf.spp_event_id = BLE_AUTH_EVT;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(__FUNCTION__, "Failed to send to queue from ISR");
            }
            break;
        }
        case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
            ESP_LOGD(__FUNCTION__,
                     "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d",
                     param->remove_bond_dev_cmpl.status);
            ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_REMOVE_BOND_DEV");
            ESP_LOGI(__FUNCTION__, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
            esp_log_buffer_hex(__FUNCTION__,
                               (void *)param->remove_bond_dev_cmpl.bd_addr,
                               sizeof(esp_bd_addr_t));
            ESP_LOGI(__FUNCTION__, "------------------------------------");
            break;
        }
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(__FUNCTION__, "DLE result: status=%d rx_len=%d tx_len=%d",
                     param->pkt_data_length_cmpl.status,
                     param->pkt_data_length_cmpl.params.rx_len,
                     param->pkt_data_length_cmpl.params.tx_len);
            break;
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        case ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT:
            ESP_LOGI(__FUNCTION__, "set preferred PHY: status=%d",
                     param->set_perf_phy.status);
            break;
        case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
            ESP_LOGI(__FUNCTION__, "PHY updated: status=%d tx=%d rx=%d (2=2M)",
                     param->phy_update.status, param->phy_update.tx_phy,
                     param->phy_update.rx_phy);
            break;
#endif
        case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
            if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__FUNCTION__,
                         "config local privacy failed, error status = %x",
                         param->local_privacy_cmpl.status);
                break;
            }

            esp_err_t ret = esp_ble_gap_config_adv_data(&spp_adv_config);
            if (ret) {
                ESP_LOGE(__FUNCTION__,
                         "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= ADV_CONFIG_FLAG;
            }

            ret = esp_ble_gap_config_adv_data(&spp_scan_rsp_config);
            if (ret) {
                ESP_LOGE(__FUNCTION__,
                         "config adv data failed, error code = %x", ret);
            } else {
                adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            }

            break;
        default:
            break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle) {
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++) {
        if (handle == spp_handle_table[i]) {
            return i;
        }
    }

    return error;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    CMD_t cmdBuf;
    BaseType_t err;

    ESP_LOGI(__FUNCTION__, "event = %d", event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            // generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB,
                                          SPP_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            break;
        case ESP_GATTS_WRITE_EVT: {
            uint8_t idx = find_char_and_desr_index(param->write.handle);
            if (idx == SPP_IDX_SPP_DATA_RECV_VAL) {
                if (param->write.len > SPP_DATA_MAX_LEN) {
                    ESP_LOGE(__FUNCTION__, "Write data too long: %d > %d",
                             param->write.len, SPP_DATA_MAX_LEN);
                    break;
                }
                cmdBuf.spp_event_id = BLE_WRITE_EVT;
            } else if (idx == SPP_IDX_CTRL_VAL) {
                if (param->write.len != sizeof(ctrl_lines_val)) {
                    ESP_LOGE(__FUNCTION__, "Bad control lines length: %d",
                             param->write.len);
                    break;
                }
                cmdBuf.spp_event_id = BLE_SET_CTRL_EVT;
            } else if (idx == SPP_IDX_LINE_VAL) {
                if (param->write.len != sizeof(line_coding_val)) {
                    ESP_LOGE(__FUNCTION__, "Bad line coding length: %d",
                             param->write.len);
                    break;
                }
                cmdBuf.spp_event_id = BLE_SET_LINE_EVT;
            } else {
                break;
            }

            cmdBuf.length = param->write.len;
            memcpy(cmdBuf.payload, param->write.value, cmdBuf.length);

            err = xQueueSend(xQueueSpp, &cmdBuf, pdMS_TO_TICKS(100));
            if (err != pdTRUE) {
                ESP_LOGW(__FUNCTION__, "SPP queue full, dropping write");
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_MTU_EVT, MTU%d", param->mtu.mtu);
            spp_mtu = param->mtu.mtu;
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_CONNECT_EVT");
            cmdBuf.spp_event_id = BLE_CONNECT_EVT;
            cmdBuf.spp_conn_id = p_data->connect.conn_id;
            cmdBuf.spp_gatts_if = gatts_if;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(TAG, "xQueueSendFromISR Fail");
            }

            ESP_LOGI(__FUNCTION__, "ESP_GATTS_CONNECT_EVT, conn_id = %d",
                     param->connect.conn_id);
            esp_log_buffer_hex(__FUNCTION__, param->connect.remote_bda, 6);

            // --- BLE MTU and connection interval tuning ---
            // Set maximum possible MTU (up to 517 for BLE)
            esp_err_t mtu_ret = esp_ble_gatt_set_local_mtu(517);
            if (mtu_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set local MTU: %s",
                         esp_err_to_name(mtu_ret));
            } else {
                ESP_LOGI(TAG, "Requested BLE MTU 517");
            }

            // Request LE Data Length Extension: without it every link
            // layer packet carries only 27 bytes and large GATT writes
            // crawl (esptool stub upload runs into command timeouts)
            esp_ble_gap_set_pkt_data_len(param->connect.remote_bda, 251);

#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
            // Prefer 2M PHY in both directions; the central falls back to
            // 1M if it doesn't support it
            esp_ble_gap_set_preferred_phy(
                param->connect.remote_bda, 0, ESP_BLE_GAP_PHY_2M_PREF_MASK,
                ESP_BLE_GAP_PHY_2M_PREF_MASK,
                ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
#endif

            // Tune connection interval for higher throughput: ask for
            // 7.5ms, accept up to 15ms (iOS grants 15ms at best)
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda,
                   sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.min_int = 0x06;    // 7.5ms (0x06 * 1.25ms)
            conn_params.max_int = 0x0C;    // 15ms
            conn_params.timeout = 0x0C80;  // 4s
            esp_ble_gap_update_conn_params(&conn_params);

            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(__FUNCTION__,
                     "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",
                     param->disconnect.reason);
            cmdBuf.spp_event_id = BLE_DISCONNECT_EVT;
            err = xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
            if (err != pdTRUE) {
                ESP_LOGE(TAG, "xQueueSendFromISR Fail");
            }
            /* start advertising again when missing the connect */
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(__FUNCTION__, "The number handle =%x",
                     param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(__FUNCTION__,
                         "Create attribute table failed, error code=0x%x",
                         param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
                ESP_LOGE(__FUNCTION__,
                         "Create attribute table abnormally, num_handle (%d) "
                         "doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, SPP_IDX_NB);
            } else {
                memcpy(spp_handle_table, param->add_attr_tab.handles,
                       sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(__FUNCTION__, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if ==
                    ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a
                                           certain gatt_if, need to call every
                                           profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void spp_init_queues(void) {
    size_t queue_len = 256;  // Large queue for bursty serial data
    size_t cmd_size = sizeof(CMD_t);

    // Check if PSRAM is available
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM detected. Allocating queues in PSRAM.");
        // Allocate queue storage in PSRAM
        void *spp_queue_storage = heap_caps_malloc(
            queue_len * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        void *uart_queue_storage = heap_caps_malloc(
            queue_len * cmd_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (spp_queue_storage && uart_queue_storage) {
            xQueueSpp = xQueueCreateStatic(SPP_QUEUE_LEN, sizeof(CMD_t),
                                           spp_queue_storage, NULL);
            if (xQueueSpp == NULL) {
                ESP_LOGE(TAG, "xQueueSpp creation failed!");
                abort();
            }
            xQueueUartTX = xQueueCreateStatic(queue_len, cmd_size,
                                              uart_queue_storage, NULL);
            ESP_LOGI(TAG, "Queues allocated in PSRAM, length=%d", queue_len);
        } else {
            ESP_LOGE(TAG,
                     "Failed to allocate queues in PSRAM, falling back to "
                     "normal RAM.");
            xQueueSpp = xQueueCreate(queue_len / 2, cmd_size);
            xQueueUartTX = xQueueCreate(queue_len / 2, cmd_size);
        }
    } else {
        ESP_LOGW(TAG, "No PSRAM detected. Using internal RAM for queues.");
        xQueueSpp = xQueueCreate(32, cmd_size);
        xQueueUartTX = xQueueCreate(32, cmd_size);
    }
}

void spp_task(void *arg) {
    ESP_LOGI(pcTaskGetName(0), "Start");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize BLE controller
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0))
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize bluedroid failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
#else
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "init bluedroid failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
#endif

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluetooth failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) !=
        ESP_OK) {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_ble_gatts_app_register(ESP_SPP_APP_ID)) != ESP_OK) {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        vTaskDelete(NULL);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key
     * parameters to the stack*/
    esp_ble_auth_req_t auth_req =
        ESP_LE_AUTH_REQ_SC_MITM_BOND;  // bonding with peer device after
                                       // authentication
    esp_ble_io_cap_t iocap =
        ESP_IO_CAP_NONE;    // set the IO capability to No output No input
    uint8_t key_size = 16;  // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    // set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey,
                                   sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                   &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support,
                                   sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which
    types of key of the master should distribute to you, and the response key
    means which key you can distribute to the master; If your BLE device acts as
    a master, the response key means you hope which types of key of the slave
    should distribute to you, and the init key means which key you can
    distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
                                   sizeof(uint8_t));

    CMD_t cmdBuf;
    uint16_t spp_conn_id = 0xffff;
    esp_gatt_if_t spp_gatts_if = 0xff;
    bool connected = false;

    while (1) {
        if (xQueueReceive(xQueueSpp, &cmdBuf, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.spp_event_id=%d connected=%d",
                 cmdBuf.spp_event_id, connected);

        if (cmdBuf.spp_event_id == BLE_CONNECT_EVT) {
            ESP_LOGI(pcTaskGetName(NULL), "BLE_CONNECT_EVT");
            spp_conn_id = cmdBuf.spp_conn_id;
            spp_gatts_if = cmdBuf.spp_gatts_if;
            connected = true;
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_ble = 1;
            xSemaphoreGive(led_sync);
        } else if (cmdBuf.spp_event_id == BLE_AUTH_EVT) {
            ESP_LOGI(pcTaskGetName(NULL), "BLE_AUTH_EVT");
        } else if (cmdBuf.spp_event_id == BLE_DISCONNECT_EVT) {
            ESP_LOGI(pcTaskGetName(NULL), "BLE_DISCONNECT_EVT");
            connected = false;
            spp_mtu = 23;
            // Drop data the departed client left behind: a non-consuming
            // target drains it at seconds per item and blocks fresh
            // sessions for minutes
            xQueueReset(xQueueUartTX);
            xSemaphoreTake(led_sync, portMAX_DELAY);
            led_ble = 0;
            xSemaphoreGive(led_sync);
        } else if (cmdBuf.spp_event_id == BLE_UART_EVT) {
            if (connected) {
                // Coalesce all serial data already waiting in the queue
                // into MTU-sized notifications: underfilled ATT packets
                // are the main throughput killer
                static uint8_t coalesce[SPP_DATA_MAX_LEN];
                uint16_t max_chunk = spp_mtu > 23 ? spp_mtu - 3 : 20;
                if (max_chunk > SPP_DATA_MAX_LEN)
                    max_chunk = SPP_DATA_MAX_LEN;
                uint8_t sync_frame[16];
                uint16_t sync_frame_len = 0;
                int sync_seen = 0;
                uint16_t fill = 0;
                uint16_t offset = 0;
                while (1) {
                    if (sync_pending) {
                        // count sync replies and capture one complete frame
                        for (uint16_t i = 0;
                             i + sizeof(sync_rsp_magic) <= cmdBuf.length; i++) {
                            if (memcmp(cmdBuf.payload + i, sync_rsp_magic,
                                       sizeof(sync_rsp_magic)) != 0)
                                continue;
                            sync_seen++;
                            if (sync_frame_len == 0) {
                                for (uint16_t j = i + 4;
                                     j < cmdBuf.length && j - i < 16; j++) {
                                    if (cmdBuf.payload[j] == 0xC0) {
                                        sync_frame_len = j - i + 1;
                                        memcpy(sync_frame, cmdBuf.payload + i,
                                               sync_frame_len);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    while (offset < cmdBuf.length) {
                        uint16_t n = cmdBuf.length - offset;
                        uint16_t space = max_chunk - fill;
                        if (n > space) n = space;
                        memcpy(coalesce + fill, cmdBuf.payload + offset, n);
                        fill += n;
                        offset += n;
                        if (fill == max_chunk) {
                            esp_err_t ind_err = esp_ble_gatts_send_indicate(
                                spp_gatts_if, spp_conn_id,
                                spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL],
                                fill, coalesce, false);
                            if (ind_err != ESP_OK) {
                                ESP_LOGW(pcTaskGetName(NULL),
                                         "notify %u bytes: %s", fill,
                                         esp_err_to_name(ind_err));
                            }
                            fill = 0;
                        }
                    }
                    // pull more serial data if it is already queued
                    CMD_t peek;
                    if (xQueuePeek(xQueueSpp, &peek, 0) == pdTRUE &&
                        peek.spp_event_id == BLE_UART_EVT) {
                        xQueueReceive(xQueueSpp, &cmdBuf, 0);
                        offset = 0;
                        continue;
                    }
                    break;
                }
                if (fill > 0) {
                    esp_err_t ind_err = esp_ble_gatts_send_indicate(
                        spp_gatts_if, spp_conn_id,
                        spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], fill,
                        coalesce, false);
                    if (ind_err != ESP_OK) {
                        ESP_LOGW(pcTaskGetName(NULL), "notify %u bytes: %s",
                                 fill, esp_err_to_name(ind_err));
                    }
                }

                if (sync_pending && sync_seen > 0 && sync_frame_len > 0) {
                    sync_pending = false;
                    int pad = 8 - sync_seen;
                    if (pad > 0) {
                        ESP_LOGI(pcTaskGetName(NULL),
                                 "sync assist: %d replies seen, padding %d",
                                 sync_seen, pad);
                        uint16_t n = 0;
                        for (int k = 0; k < pad; k++) {
                            memcpy(coalesce + n, sync_frame, sync_frame_len);
                            n += sync_frame_len;
                        }
                        esp_ble_gatts_send_indicate(
                            spp_gatts_if, spp_conn_id,
                            spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], n,
                            coalesce, false);
                    }
                }
            }
        } else if (cmdBuf.spp_event_id == BLE_WRITE_EVT ||
                   cmdBuf.spp_event_id == BLE_SET_CTRL_EVT ||
                   cmdBuf.spp_event_id == BLE_SET_LINE_EVT) {
            if (cmdBuf.spp_event_id == BLE_WRITE_EVT &&
                contains_seq(cmdBuf.payload, cmdBuf.length, sync_cmd_magic,
                             sizeof(sync_cmd_magic))) {
                sync_pending = true;
            }

            // Control writes share the UART queue with data so they apply
            // in the order the client sent them
            BaseType_t send_err = xQueueSend(
                xQueueUartTX, &cmdBuf, pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));
            if (send_err != pdTRUE) {
                ESP_LOGW(TAG, "UART TX queue full, dropping data");
            }
        } else if (cmdBuf.spp_event_id == SERIAL_STATE_EVT) {
            esp_ble_gatts_set_attr_value(spp_handle_table[SPP_IDX_STATE_VAL],
                                         cmdBuf.length, cmdBuf.payload);
            if (connected) {
                esp_ble_gatts_send_indicate(
                    spp_gatts_if, spp_conn_id,
                    spp_handle_table[SPP_IDX_STATE_VAL], cmdBuf.length,
                    cmdBuf.payload, false);
            }
        }
    }  // end while

    // never reach here
    vTaskDelete(NULL);
}
