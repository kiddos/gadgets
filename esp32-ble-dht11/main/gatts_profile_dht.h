#ifndef GATTS_PROFILE_DHT_H
#define GATTS_PROFILE_DHT_H

#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "dht11.h"

enum {
  DHT11_IDX_SVC,

  DHT11_IDX_TEMP_MEAS_CHAR,
  DHT11_IDX_TEMP_MEAS_VAL,

  DHT11_IDX_HUMI_MEAS_CHAR,
  DHT11_IDX_HUMI_MEAS_VAL,

  DHT11_IDX_NB,
};

typedef struct gatts_profile_inst {
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
} gatts_profile_inst;

extern gatts_profile_inst instant;
extern dht11_t dht11;

void gatts_profile_dht_event_handler(esp_gatts_cb_event_t event,
                                     esp_gatt_if_t gatts_if,
                                     esp_ble_gatts_cb_param_t *param);

void gap_event_handler(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param);

void setup_security();

#endif /* end of include guard: GATTS_PROFILE_DHT_H */
