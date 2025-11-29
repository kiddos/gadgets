#include "dht11.h"

static const char *TAG = "dht11";

void dht11_init(dht11_t *dht11, gpio_num_t pin) {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  dht11->dht11_gpio = pin;
  memset(dht11->raw.raw, 0, sizeof(dht11->raw.raw));
}

// dht1111 begins, the programme of MCU will set Data Single-bus voltage level
// from high to low and this process must take at least 18ms to ensure dht11’s
// detection of MCU's signal, then MCU will pull up voltage and wait 20-40us for
// dht11’s response.
static void start_signal(dht11_t *dht11) {
  gpio_set_direction(dht11->dht11_gpio, GPIO_MODE_OUTPUT);
  gpio_set_level(dht11->dht11_gpio, 0);
  ets_delay_us(20 * 1000);
  gpio_set_level(dht11->dht11_gpio, 1);
  ets_delay_us(40);
  gpio_set_direction(dht11->dht11_gpio, GPIO_MODE_INPUT);
}

// Once dht11 detects the start signal, it will send out a low-voltage-level
// response signal, which lasts 80us. Then the programme of dht11 sets Data
// Single-bus voltage level from low to high and keeps it for 80us for dht11’s
// preparation for sending data.
//
// When dht11 is sending data to MCU, every bit of data begins with the 50us
// low-voltage-level and the length of the following high-voltage-level signal
// determines whether data bit is "0" or "1"
static int16_t wait_for_gpio(dht11_t *dht11, int16_t microseconds, int level) {
  int16_t micros_ticks = 0;
  while (micros_ticks <= microseconds) {
    if (gpio_get_level(dht11->dht11_gpio) == level) {
      break;
    }
    ets_delay_us(1);
    micros_ticks++;
  }
  return micros_ticks;
}

static int read_bit(dht11_t *dht11) {
  int16_t result = wait_for_gpio(dht11, 50, 1);
  if (result > 50) {
    return -1;
  }

  result = wait_for_gpio(dht11, 70, 0);
  if (result > 70) {
    return -1;
  }
  if (result < 31) {
    return 0;
  } else {
    return 1;
  }
}

dht11_status dht11_read_byte(dht11_t *dht11, int8_t *b) {
  *b = 0;
  for (int i = 0; i < 8; ++i) {
    int bit = read_bit(dht11);
    if (bit < 0) {
      return dht11_TIMEOUT;
    }
    *b |= (bit << (7 - i));
  }
  return dht11_OK;
}

dht11_status dht11_read_data(dht11_t *dht11) {
  ESP_LOGI(TAG, "READ DATA");
  start_signal(dht11);

  int16_t t = wait_for_gpio(dht11, 80, 1);
  if (t > 80) {
    ESP_LOGE(TAG, "timeout");
    return dht11_TIMEOUT;
  }

  t = wait_for_gpio(dht11, 80, 0);
  if (t > 80) {
    ESP_LOGE(TAG, "timeout");
    return dht11_TIMEOUT;
  }

  dht11_raw_data_t *raw = &dht11->raw;
  for (int i = 0; i < 5; ++i) {
    dht11_status status = dht11_read_byte(dht11, &raw->raw[i]);
    if (status == dht11_TIMEOUT) {
      ESP_LOGE(TAG, "timeout");
      return dht11_TIMEOUT;
    }
  }

  int8_t sum = 0;
  for (int i = 0; i < 4; ++i) {
    sum += raw->raw[i];
  }
  if (sum != raw->raw[4]) {
    ESP_LOGE(TAG, "invalid check sum");
    return dht11_INVALID_CHECK_SUM;
  }

  dht11_convert_raw_values(dht11);
  return dht11_OK;
}

static float to_decimal(int8_t val) {
  float x = (float)val;
  while (x >= 1) {
    x /= 10;
  }
  return x;
}

void dht11_convert_raw_values(dht11_t *dht11) {
  dht11_raw_data_t *raw = &dht11->raw;
  float rh = (float)raw->raw[0] + to_decimal(raw->raw[1]);
  float t = (float)raw->raw[2] + to_decimal(raw->raw[3]);
  dht11->data.rh = rh;
  dht11->data.t = t;
  ESP_LOGI(TAG, "data read");
  ESP_LOGI(TAG, "rh = %f, t = %f\n", rh, t);
}
