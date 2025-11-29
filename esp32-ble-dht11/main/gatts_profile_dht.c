#include "gatts_profile_dht.h"

#define DEVICE_NAME "ESP_DHT11"
#define GATTS_SERVICE_UUID_DHT 0x0066
#define GATTS_CHAR_UUID_DHT 0x0166
#define GATTS_DESCR_UUID_DHT 0x0266
#define GATTS_HANDLE_COUNT 2
#define GATTS_DHT11_SVC_INST_ID 0

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

gatts_profile_inst instant;
dht11_t dht11;

static const char *TAG = "gatts-dht11";

static uint8_t sec_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};

static const char manufacturer[] = "ESP";

static esp_ble_adv_data_t dht11_adv_data = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0020,
    .max_interval = 0x0080,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t dht11_scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = 3,
    .p_manufacturer_data = (uint8_t *)manufacturer,
};

static esp_ble_adv_params_t dht11_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const uint16_t dht11_svc = ESP_GATT_UUID_ENVIRONMENTAL_SENSING_SVC;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint16_t character_client_config_uuid =
//     ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// static const uint16_t dht11_temp_meas_uuid =
//     ESP_GATT_UUID_ENV_SENSING_MEASUREMENT_DESCR;
static const uint16_t dht11_temp_meas_uuid = 0x272F;
// static uint8_t dht11_temperature_config[2] = {0x00, 0x00};

// static const uint16_t dht11_humi_meas_uuid =
//     ESP_GATT_UUID_ENV_SENSING_MEASUREMENT_DESCR;
static const uint16_t dht11_humi_meas_uuid = 0x2A6F;

static uint8_t adv_config_done = 0;

const esp_gatts_attr_db_t dht11_gatt_db[DHT11_IDX_NB] = {
    // Service Declaration
    [DHT11_IDX_SVC] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16,
                (uint8_t *)&primary_service_uuid,
                ESP_GATT_PERM_READ,
                sizeof(uint16_t),
                sizeof(dht11_svc),
                (uint8_t *)&dht11_svc,
            },
        },

    // Temperature Characteristic Declaration
    [DHT11_IDX_TEMP_MEAS_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16,
                (uint8_t *)&character_declaration_uuid,
                ESP_GATT_PERM_READ,
                sizeof(uint8_t),
                sizeof(uint8_t),
                (uint8_t *)&char_prop_read,
            },
        },

    // Temperature Measurement Characteristic Value
    [DHT11_IDX_TEMP_MEAS_VAL] =
        {
            {ESP_GATT_RSP_BY_APP},
            {
                ESP_UUID_LEN_16,
                (uint8_t *)&dht11_temp_meas_uuid,
                ESP_GATT_PERM_READ,
                sizeof(dht11.raw.raw),
                0,
                NULL,
            },
        },

    // Humidity Characteristic Declaration
    [DHT11_IDX_HUMI_MEAS_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16,
                (uint8_t *)&character_declaration_uuid,
                ESP_GATT_PERM_READ,
                sizeof(uint8_t),
                sizeof(uint8_t),
                (uint8_t *)&char_prop_read,
            },
        },

    // Humidity Measurement Characteristic Value
    [DHT11_IDX_HUMI_MEAS_VAL] =
        {
            {ESP_GATT_RSP_BY_APP},
            {
                ESP_UUID_LEN_16,
                (uint8_t *)&dht11_humi_meas_uuid,
                ESP_GATT_PERM_READ,
                sizeof(dht11.raw.raw),
                0,
                NULL,
            },
        },
};

static uint16_t handle_table[DHT11_IDX_NB];

void gatts_profile_dht_event_handler(esp_gatts_cb_event_t event,
                                     esp_gatt_if_t gatts_if,
                                     esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT:
    esp_ble_gap_set_device_name(DEVICE_NAME);
    // generate a resolvable random address
    esp_ble_gap_config_local_privacy(true);
    esp_ble_gatts_create_attr_tab(dht11_gatt_db, gatts_if, DHT11_IDX_NB,
                                  GATTS_DHT11_SVC_INST_ID);
    dht11_read_data(&dht11);
    break;
  case ESP_GATTS_READ_EVT:
    ESP_LOGI(TAG,
             "ESP_GATTS_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d",
             param->read.conn_id, param->read.trans_id, param->read.handle);
    if (param->read.handle == handle_table[DHT11_IDX_TEMP_MEAS_VAL] ||
        param->read.handle == handle_table[DHT11_IDX_HUMI_MEAS_VAL]) {
      dht11_read_data(&dht11);
      esp_gatt_rsp_t rsp;
      memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
      rsp.attr_value.handle = param->read.handle;
      rsp.attr_value.len = 2;
      if (param->read.handle == handle_table[DHT11_IDX_TEMP_MEAS_VAL]) {
        rsp.attr_value.value[0] = dht11.raw.raw[2];
        rsp.attr_value.value[1] = dht11.raw.raw[3];
      } else if (param->read.handle == handle_table[DHT11_IDX_HUMI_MEAS_VAL]) {
        rsp.attr_value.value[0] = dht11.raw.raw[0];
        rsp.attr_value.value[1] = dht11.raw.raw[1];
      }
      esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                  param->read.trans_id, ESP_GATT_OK, &rsp);
    }
    break;
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT, write value:");
    esp_log_buffer_hex(TAG, param->write.value, param->write.len);
    break;
  case ESP_GATTS_EXEC_WRITE_EVT:
    ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    break;
  case ESP_GATTS_MTU_EVT:
    break;
  case ESP_GATTS_CONF_EVT:
    ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT");
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
    ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT");
    /* start security connect with peer device when receive the connect event
     * sent by the master */
    esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",
             param->disconnect.reason);
    /* start advertising again when missing the connect */
    esp_ble_gap_start_advertising(&dht11_adv_params);
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
    ESP_LOGI(TAG, "The number handle = %x", param->add_attr_tab.num_handle);
    if (param->create.status == ESP_GATT_OK) {
      if (param->add_attr_tab.num_handle == DHT11_IDX_NB) {
        memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
        for (int i = 0; i < DHT11_IDX_NB; ++i) {
          ESP_LOGI(TAG, "handle %d = %d", i, handle_table[i]);
        }
        esp_ble_gatts_start_service(handle_table[DHT11_IDX_SVC]);
      } else {
        ESP_LOGE(TAG,
                 "Create attribute table abnormally, num_handle (%d) doesn't "
                 "equal to HRS_IDX_NB(%d)",
                 param->add_attr_tab.num_handle, DHT11_IDX_NB);
      }
    } else {
      ESP_LOGE(TAG, " Create attribute table failed, error code = %x",
               param->create.status);
    }
    break;
  }

  default:
    break;
  }
}

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
  ESP_LOGI(TAG, "Bonded devices number : %d\n", dev_num);

  ESP_LOGI(TAG, "Bonded devices list : %d\n", dev_num);
  for (int i = 0; i < dev_num; i++) {
    esp_log_buffer_hex(TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
  }

  free(dev_list);
}

void gap_event_handler(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
  ESP_LOGV(TAG, "GAP_EVT, event %d\n", event);

  switch (event) {
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&dht11_adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~ADV_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&dht11_adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start
    // successfully or failed
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "advertising start failed, error status = %x",
               param->adv_start_cmpl.status);
      break;
    }
    ESP_LOGI(TAG, "advertising start success");
    break;
  case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
    /* Call the following function to input the passkey which is displayed on
     * the remote device */
    // esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda,
    // true, 0x00);
    break;
  case ESP_GAP_BLE_OOB_REQ_EVT: {
    ESP_LOGI(TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
    // If you paired with OOB, both devices need to use the same tk
    uint8_t tk[16] = {6};
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
    break;
  }
  case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
    break;
  case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
    ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
    break;
  case ESP_GAP_BLE_NC_REQ_EVT:
    /* The app will receive this evt when the IO has DisplayYesNO capability and
    the peer device IO also has DisplayYesNo capability. show the passkey number
    to the user to confirm it with the number displayed by peer device. */
    esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
    ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32,
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
    ESP_LOGI(TAG, "The passkey Notify number:%06" PRIu32,
             param->ble_security.key_notif.passkey);
    break;
  case ESP_GAP_BLE_KEY_EVT:
    // shows the ble key info share with peer device to the user.
    ESP_LOGI(TAG, "key type = %s",
             esp_key_type_to_str(param->ble_security.ble_key.key_type));
    break;
  case ESP_GAP_BLE_AUTH_CMPL_EVT: {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
           sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
             (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) +
                 bd_addr[3],
             (bd_addr[4] << 8) + bd_addr[5]);
    ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
    ESP_LOGI(TAG, "pair status = %s",
             param->ble_security.auth_cmpl.success ? "success" : "fail");
    if (!param->ble_security.auth_cmpl.success) {
      ESP_LOGI(TAG, "fail reason = 0x%x",
               param->ble_security.auth_cmpl.fail_reason);
    } else {
      ESP_LOGI(TAG, "auth mode = %s",
               esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
    }
    show_bonded_devices();
    break;
  }
  case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
    ESP_LOGD(TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d",
             param->remove_bond_dev_cmpl.status);
    ESP_LOGI(TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
    ESP_LOGI(TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
    esp_log_buffer_hex(TAG, (void *)param->remove_bond_dev_cmpl.bd_addr,
                       sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG, "------------------------------------");
    break;
  }
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "config local privacy failed, error status = %x",
               param->local_privacy_cmpl.status);
      break;
    }

    esp_err_t ret = esp_ble_gap_config_adv_data(&dht11_adv_data);
    if (ret) {
      ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= ADV_CONFIG_FLAG;
    }

    ret = esp_ble_gap_config_adv_data(&dht11_scan_rsp_data);
    if (ret) {
      ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    }

    break;
  default:
    break;
  }
}

void setup_security() {
  // bonding with peer device after authentication
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  // set the IO capability to No output No input
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  // the key size should be 7~16 bytes
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  // set static passkey
  uint32_t passkey = CONFIG_BLE_SECURITY_PASSKEY;
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
