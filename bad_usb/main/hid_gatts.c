/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class/hid/hid_device.h"
#include "driver/gpio.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <inttypes.h>

#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "tinyusb.h"

#define TUSB_DESC_TOTAL_LEN                                                    \
  (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

#define APP_TAG "BADUSB"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define APP_ID 0x55
#define HEART_RATE_SVC_INST_ID 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static void handle_gatts_write_event(esp_ble_gatts_cb_param_t *param);

const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "ESP",                // 1: Manufacturer
    "BADUSB",             // 2: Product
    "2022DP2892",         // 3: Serials, should use chip ID
    "BADUSB HID",         // 4: HID
};

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE)),
};

static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP
    // In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16,
                       10),
};

uint8_t const conv_table[128][2] = {HID_ASCII_TO_KEYCODE};

enum {
  IDX_SVC,
  IDX_INFORMATION_CHAR,
  IDX_INFORMATION,
  IDX_KEYBOARD_CHAR,
  IDX_KEYBOARD_INPUT,
  IDX_MOUSE_CHAR,
  IDX_MOUSE_INPUT,
  IDX_NB,
};

static uint8_t adv_config_done = 0;

static uint16_t handle_table[IDX_NB];

static uint8_t manufacturer[3] = "ESP";

static const char *device_name = "ESP_BADUSB";

static uint8_t sec_service_uuid[16] = {
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x18, 0x12, 0x00, 0x00,
};

// config adv data
static esp_ble_adv_data_t adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time =
                            // min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time =
                            // max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// config scan response data
static esp_ble_adv_data_t scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(manufacturer),
    .p_manufacturer_data = manufacturer,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
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
static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE,
        },
};

/*
 *  PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// HID Service
static const uint16_t hid_svc = ESP_GATT_UUID_HID_SVC;

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;

static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;

/// HID Service
static const uint16_t information_uuid = ESP_GATT_UUID_HID_INFORMATION;
static const uint8_t information[6] = "BADUSB";
static const uint16_t keyboard_input_uuid = ESP_GATT_UUID_HID_BT_KB_INPUT;
static const uint8_t keyboard_input_data[1] = {0x00};
static const uint16_t mouse_input_uuid = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
static const float mouse_input_data[2] = {0.0, 0.0};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t heart_rate_gatt_db[IDX_NB] = {
    // Heart Rate Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                 {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                  ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(hid_svc),
                  (uint8_t *)&hid_svc}},
    // info
    [IDX_INFORMATION_CHAR] = {{ESP_GATT_AUTO_RSP},
                              {ESP_UUID_LEN_16,
                               (uint8_t *)&character_declaration_uuid,
                               ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                               CHAR_DECLARATION_SIZE,
                               (uint8_t *)&char_prop_read}},
    [IDX_INFORMATION] = {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t *)&information_uuid,
                          ESP_GATT_PERM_READ, sizeof(information),
                          sizeof(information), (uint8_t *)information}},
    // Keyboard
    [IDX_KEYBOARD_CHAR] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16,
                            (uint8_t *)&character_declaration_uuid,
                            ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                            CHAR_DECLARATION_SIZE,
                            (uint8_t *)&char_prop_write}},
    [IDX_KEYBOARD_INPUT] = {{ESP_GATT_AUTO_RSP},
                            {ESP_UUID_LEN_16, (uint8_t *)&keyboard_input_uuid,
                             ESP_GATT_PERM_WRITE, sizeof(uint8_t),
                             sizeof(keyboard_input_data),
                             (uint8_t *)keyboard_input_data}},
    // mouse
    [IDX_MOUSE_CHAR] = {{ESP_GATT_AUTO_RSP},
                        {ESP_UUID_LEN_16,
                         (uint8_t *)&character_declaration_uuid,
                         ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE,
                         CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
    [IDX_MOUSE_INPUT] = {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t *)&mouse_input_uuid,
                          ESP_GATT_PERM_WRITE, sizeof(float) * 2,
                          sizeof(mouse_input_data),
                          (uint8_t *)mouse_input_data}},
};

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
  ESP_LOGI(APP_TAG, "Bonded devices number : %d\n", dev_num);

  ESP_LOGI(APP_TAG, "Bonded devices list : %d\n", dev_num);
  for (int i = 0; i < dev_num; i++) {
    esp_log_buffer_hex(APP_TAG, (void *)dev_list[i].bd_addr,
                       sizeof(esp_bd_addr_t));
  }

  free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();

  esp_ble_bond_dev_t *dev_list =
      (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) {
    esp_ble_remove_bond_device(dev_list[i].bd_addr);
  }

  free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  ESP_LOGV(APP_TAG, "GAP_EVT, event %d\n", event);

  switch (event) {
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~ADV_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start
    // successfully or failed
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(APP_TAG, "advertising start failed, error status = %x",
               param->adv_start_cmpl.status);
      break;
    }
    ESP_LOGI(APP_TAG, "advertising start success");
    break;
  case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
    ESP_LOGI(APP_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
    /* Call the following function to input the passkey which is displayed on
     * the remote device */
    // esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda,
    // true, 0x00);
    break;
  case ESP_GAP_BLE_OOB_REQ_EVT: {
    ESP_LOGI(APP_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
    uint8_t tk[16] = {
        1}; // If you paired with OOB, both devices need to use the same tk
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
    break;
  }
  case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
    ESP_LOGI(APP_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
    break;
  case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
    ESP_LOGI(APP_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
    break;
  case ESP_GAP_BLE_NC_REQ_EVT:
    /* The app will receive this evt when the IO has DisplayYesNO capability and
    the peer device IO also has DisplayYesNo capability. show the passkey number
    to the user to confirm it with the number displayed by peer device. */
    esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
    ESP_LOGI(APP_TAG,
             "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32,
             param->ble_security.key_notif.passkey);
    break;
  case ESP_GAP_BLE_SEC_REQ_EVT:
    /* send the positive(true) security response to the peer device to accept
    the security request. If not accept the security request, should send the
    security response with negative(false) accept value*/
    esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
    break;
  case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: /// the app will receive this evt when the
                                      /// IO  has Output capability and the peer
                                      /// device IO has Input capability.
    /// show the passkey number to the user to input it in the peer device.
    ESP_LOGI(APP_TAG, "The passkey Notify number:%06" PRIu32,
             param->ble_security.key_notif.passkey);
    break;
  case ESP_GAP_BLE_KEY_EVT:
    // shows the ble key info share with peer device to the user.
    ESP_LOGI(APP_TAG, "key type = %s",
             esp_key_type_to_str(param->ble_security.ble_key.key_type));
    break;
  case ESP_GAP_BLE_AUTH_CMPL_EVT: {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
           sizeof(esp_bd_addr_t));
    ESP_LOGI(APP_TAG, "remote BD_ADDR: %08x%04x",
             (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) +
                 bd_addr[3],
             (bd_addr[4] << 8) + bd_addr[5]);
    ESP_LOGI(APP_TAG, "address type = %d",
             param->ble_security.auth_cmpl.addr_type);
    ESP_LOGI(APP_TAG, "pair status = %s",
             param->ble_security.auth_cmpl.success ? "success" : "fail");
    if (!param->ble_security.auth_cmpl.success) {
      ESP_LOGI(APP_TAG, "fail reason = 0x%x",
               param->ble_security.auth_cmpl.fail_reason);
    } else {
      ESP_LOGI(APP_TAG, "auth mode = %s",
               esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
    }
    show_bonded_devices();
    break;
  }
  case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
    ESP_LOGD(APP_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d",
             param->remove_bond_dev_cmpl.status);
    ESP_LOGI(APP_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
    ESP_LOGI(APP_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
    esp_log_buffer_hex(APP_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr,
                       sizeof(esp_bd_addr_t));
    ESP_LOGI(APP_TAG, "------------------------------------");
    break;
  }
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(APP_TAG, "config local privacy failed, error status = %x",
               param->local_privacy_cmpl.status);
      break;
    }

    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_config);
    if (ret) {
      ESP_LOGE(APP_TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= ADV_CONFIG_FLAG;
    }

    ret = esp_ble_gap_config_adv_data(&scan_rsp_config);
    if (ret) {
      ESP_LOGE(APP_TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    }

    break;
  default:
    break;
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  ESP_LOGV(APP_TAG, "event = %x\n", event);
  switch (event) {
  case ESP_GATTS_REG_EVT:
    esp_ble_gap_set_device_name(device_name);
    // generate a resolvable random address
    esp_ble_gap_config_local_privacy(true);
    esp_ble_gatts_create_attr_tab(heart_rate_gatt_db, gatts_if, IDX_NB,
                                  HEART_RATE_SVC_INST_ID);
    break;
  case ESP_GATTS_READ_EVT:
    break;
  case ESP_GATTS_WRITE_EVT:
    handle_gatts_write_event(param);
    break;
  case ESP_GATTS_EXEC_WRITE_EVT:
    break;
  case ESP_GATTS_MTU_EVT:
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
    ESP_LOGI(APP_TAG, "ESP_GATTS_CONNECT_EVT");
    /* start security connect with peer device when receive the connect event
     * sent by the master */
    esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(APP_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",
             param->disconnect.reason);
    /* start advertising again when missing the connect */
    esp_ble_gap_start_advertising(&adv_params);
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
  case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
    ESP_LOGI(APP_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
    if (param->create.status == ESP_GATT_OK) {
      if (param->add_attr_tab.num_handle == IDX_NB) {
        memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
        esp_ble_gatts_start_service(handle_table[IDX_SVC]);
      } else {
        ESP_LOGE(APP_TAG,
                 "Create attribute table abnormally, num_handle (%d) doesn't "
                 "equal to HRS_IDX_NB(%d)",
                 param->add_attr_tab.num_handle, IDX_NB);
      }
    } else {
      ESP_LOGE(APP_TAG, " Create attribute table failed, error code = %x",
               param->create.status);
    }
    break;
  }

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
      profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGI(APP_TAG, "Reg app failed, app_id %04x, status %d\n",
               param->reg.app_id, param->reg.status);
      return;
    }
  }

  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a
                                             certain gatt_if, need to call every
                                             profile cb function */
          gatts_if == profile_tab[idx].gatts_if) {
        if (profile_tab[idx].gatts_cb) {
          profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  // We use only one interface and one HID report descriptor, so we can ignore
  // parameter 'instance'
  return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {}

static void app_send_keystroke(char *keys) {
  ESP_LOGI(APP_TAG, "Sending Keyboard report");
  int len = strlen(keys);

  for (int i = 0; i < len; ++i) {
    int ch = keys[i];
    uint8_t keycode[6] = {0};
    uint8_t modifier = 0;
    if (conv_table[ch][0])
      modifier = KEYBOARD_MODIFIER_LEFTSHIFT;
    keycode[0] = conv_table[ch][1];
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, modifier, keycode);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void handle_gatts_write_event(esp_ble_gatts_cb_param_t *param) {
  ESP_LOGI(APP_TAG, "ESP_GATTS_WRITE_EVT");
  char buffer[1024] = {0};
  memcpy(buffer, param->write.value, param->write.len);
  if (buffer[0] == 'K') {
    app_send_keystroke(buffer + 1);
  }
}

void setup_tiny_usb() {
  const tinyusb_config_t tusb_cfg = {
      .device_descriptor = NULL,
      .string_descriptor = hid_string_descriptor,
      .string_descriptor_count =
          sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
      .external_phy = false,
      .configuration_descriptor = hid_configuration_descriptor,
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(APP_TAG, "TinyUSB initialization DONE");
}

void setup_ble() {
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(APP_TAG, "%s init controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(APP_TAG, "%s enable controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  ESP_LOGI(APP_TAG, "%s init bluetooth", __func__);
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(APP_TAG, "%s init bluetooth failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(APP_TAG, "%s enable bluetooth failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(APP_TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(APP_TAG, "gap register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(APP_ID);
  if (ret) {
    ESP_LOGE(APP_TAG, "gatts app register error, error code = %x", ret);
    return;
  }

  /* set the security iocap & auth_req & key size & init key response key
   * parameters to the stack*/
  esp_ble_auth_req_t auth_req =
      ESP_LE_AUTH_REQ_SC_MITM_BOND; // bonding with peer device after
                                    // authentication
  esp_ble_io_cap_t iocap =
      ESP_IO_CAP_NONE;   // set the IO capability to No output No input
  uint8_t key_size = 16; // the key size should be 7~16 bytes
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
  /* If your BLE device acts as a Slave, the init_key means you hope which types
  of key of the master should distribute to you, and the response key means
  which key you can distribute to the master; If your BLE device acts as a
  master, the response key means you hope which types of key of the slave should
  distribute to you, and the init key means which key you can distribute to the
  slave. */
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
                                 sizeof(uint8_t));
}

void app_main(void) {
  setup_tiny_usb();
  setup_ble();
  /* Just show how to clear all the bonded devices
   * Delay 30s, clear all the bonded devices
   *
   * vTaskDelay(30000 / portTICK_PERIOD_MS);
   * remove_all_bonded_devices();
   */
}
