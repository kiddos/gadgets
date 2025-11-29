#ifndef dht11_H
#define dht11_H

#include "driver/gpio.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <inttypes.h>
#include <string.h>

typedef union dht11_raw_data_t {
  int8_t raw[5];
} dht11_raw_data_t;

typedef struct dht11_data_t {
  float rh;
  float t;
} dht11_data_t;

typedef struct dht11_t {
  gpio_num_t dht11_gpio;
  dht11_raw_data_t raw;
  dht11_data_t data;
} dht11_t;

typedef enum dht11_status {
  dht11_OK,
  dht11_TIMEOUT,
  dht11_INVALID_CHECK_SUM,
} dht11_status;

void dht11_init(dht11_t *dht11, gpio_num_t pin);
dht11_status dht11_read_byte(dht11_t *dht11, int8_t *b);
dht11_status dht11_read_data(dht11_t *dht11);
void dht11_convert_raw_values(dht11_t *dht11);

#endif /* end of include guard: dht11_H */
