#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "dht.h"
#include "jsmn.h"
#include "json_generator.h"
#include "json_parser.h"
#include "led_strip.h"

#include <math.h>
#include <stdio.h>
#include <sys/time.h>

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define LED_COUNT 30
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "ledstrip";

static esp_chip_info_t chip_info;
static led_strip_handle_t led_strip;
static jsmn_parser parser;
static char info_buffer[1024];
static char dht11_buffer[1024];
static char request_buffer[1024];
static char response_buffer[128];
static int brightness = 0;
static int delta = 6;
static int random_offset[LED_COUNT];

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static httpd_handle_t server = NULL;

static httpd_handle_t start_webserver(void);

enum mode_t { WHITE, RGB, RANDOM, BREATHING };

struct rgb_t {
  int16_t r, g, b;
};

struct hsv_t {
  int16_t h;
  float s, v;
};

typedef struct rgb_t rgb_t;
typedef struct hsv_t hsv_t;

hsv_t hsv = {
    .h = 0,
    .s = 1.0,
    .v = 1.0,
};

mode_t mode = WHITE;

static rgb_t hsv2rgb(int16_t h, float s, float v) {
  int16_t h2 = h / 60;
  float f = (float)h / 60.0 - h2;
  float c = v * s;
  float x = c * (1 - fabs((h2 % 2) + f - 1));
  float r = 0;
  float g = 0;
  float b = 0;
  if (h < 60) {
    r = c;
    g = x;
  } else if (h < 120) {
    r = x;
    g = c;
  } else if (h < 180) {
    g = c;
    b = x;
  } else if (h < 240) {
    g = x;
    b = c;
  } else if (h < 300) {
    r = x;
    b = c;
  } else {
    r = c;
    b = x;
  }

  float m = v - c;
  rgb_t rgb = {
      .r = (r + m) * brightness,
      .g = (g + m) * brightness,
      .b = (b + m) * brightness,
  };
  return rgb;
}

static void display_color(void) {
  if (mode == WHITE) {
    for (int i = 0; i < LED_COUNT; ++i) {
      /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
      led_strip_set_pixel(led_strip, i, brightness, brightness, brightness);
      /* Refresh the strip to send data */
      led_strip_refresh(led_strip);
    }
  } else if (mode == RGB) {
    for (int i = 0; i < LED_COUNT; ++i) {
      rgb_t rgb = hsv2rgb((hsv.h + delta * i + 360) % 360, hsv.s, hsv.v);
      led_strip_set_pixel(led_strip, i, rgb.r, rgb.g, rgb.b);
      led_strip_refresh(led_strip);
    }
    hsv.h = (hsv.h + delta + 360) % 360;
  } else if (mode == RANDOM) {
    for (int i = 0; i < LED_COUNT; ++i) {
      rgb_t rgb = hsv2rgb((hsv.h + random_offset[i]) % 360, hsv.s, hsv.v);
      led_strip_set_pixel(led_strip, i, rgb.r, rgb.g, rgb.b);
      led_strip_refresh(led_strip);
    }
    hsv.h = (hsv.h + delta + 360) % 360;
  } else if (mode == BREATHING) {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL); // Get current time
    // Access seconds and microseconds
    int64_t time_second = (int64_t)tv_now.tv_sec;
    for (int i = 0; i < LED_COUNT; ++i) {
      int b = round((sin(time_second * M_PI) + 1) * 0.5 * brightness);
      led_strip_set_pixel(led_strip, i, b, b, b);
      led_strip_refresh(led_strip);
    }
  }
}

static void configure_led() {
  ESP_LOGI(TAG, "setting up led-strip...");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = CONFIG_LED_GPIO,
      .max_leds = LED_COUNT, // at least one LED on board
      .led_pixel_format = LED_PIXEL_FORMAT_GRB,
      .led_model = LED_MODEL_WS2812,
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

static void board_info() {
  /* Print chip information */
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154)
             ? ", 802.15.4 (Zigbee/Thread)"
             : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  uint32_t flash_size;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return;
  }

  printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());
}

static esp_err_t ledstrip_handler(httpd_req_t *req) {
  memset(request_buffer, 0, sizeof(request_buffer));

  int ret, remaining = req->content_len;
  int status = 0;
  while (remaining > 0) {
    /* Read the data for the request */
    if ((ret = httpd_req_recv(req, request_buffer,
                              MIN(remaining, sizeof(request_buffer)))) <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        /* Retry receiving if timeout occurred */
        continue;
      }
      return ESP_FAIL;
    }
    remaining -= ret;

    /* Log data received */
    ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
    ESP_LOGI(TAG, "%.*s", ret, request_buffer);
    ESP_LOGI(TAG, "====================================");

    jparse_ctx_t jctx;
    int ret = json_parse_start(&jctx, request_buffer, strlen(request_buffer));
    if (ret != OS_SUCCESS) {
      status = 1;
      continue;
    }

    if (json_obj_get_int(&jctx, "brightness", &brightness) == OS_SUCCESS) {
      brightness = MAX(MIN(brightness, 255), 0);
      ESP_LOGI(TAG, "brightness=%d\n", brightness);
    }

    if (json_obj_get_int(&jctx, "delta", &delta) == OS_SUCCESS) {
      delta = MAX(MIN(delta, 10), 0);
      ESP_LOGI(TAG, "delta=%d\n", delta);
    }

    char buf[64];
    memset(buf, 0, sizeof(buf));
    if (json_obj_get_string(&jctx, "mode", buf, sizeof(buf)) == OS_SUCCESS) {
      if (strcmp(buf, "white") == 0) {
        mode = WHITE;
      } else if (strcmp(buf, "rgb") == 0) {
        mode = RGB;
      } else if (strcmp(buf, "random") == 0) {
        mode = RANDOM;
        for (int i = 0; i < LED_COUNT; ++i) {
          random_offset[i] = rand() % 360;
        }
      } else if (strcmp(buf, "breathing")) {
        mode = BREATHING;
      }
    }
  }

  memset(response_buffer, 0, sizeof(response_buffer));
  json_gen_str_t jstr;
  json_gen_str_start(&jstr, response_buffer, sizeof(response_buffer), NULL,
                     NULL);
  json_gen_start_object(&jstr);
  json_gen_obj_set_int(&jstr, "status", status);
  json_gen_end_object(&jstr);
  json_gen_str_end(&jstr);

  httpd_resp_set_hdr(req, "Content-Type", "application/json");

  httpd_resp_send(req, response_buffer, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    if (server == NULL) {
      ESP_LOGI(TAG, "Starting webserver");
      server = start_webserver();
    }
  }
}

esp_err_t wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = CONFIG_ESP_WIFI_SSID,
              .password = CONFIG_ESP_WIFI_PASSWORD,
              /* Authmode threshold resets to WPA2 as default if password
               * matches WPA2 standards (pasword len => 8). If you want to
               * connect the device to deprecated WEP/WPA networks, Please set
               * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set
               * the password with length and format matching to
               * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
               */
              .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
              .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
              .sae_h2e_identifier = "",
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", CONFIG_ESP_WIFI_SSID,
             CONFIG_ESP_WIFI_PASSWORD);
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
  return ESP_FAIL;
}

static const httpd_uri_t ledstrip = {
    .uri = "/api/ledstrip",
    .method = HTTP_POST,
    .handler = ledstrip_handler,
    .user_ctx = NULL,
};

static esp_err_t info_handler(httpd_req_t *req) {
  json_gen_str_t jstr;
  memset(info_buffer, 0, sizeof(info_buffer));

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;

  json_gen_str_start(&jstr, info_buffer, sizeof(info_buffer), NULL, NULL);
  json_gen_start_object(&jstr);
  json_gen_obj_set_int(&jstr, "major_rev", major_rev);
  json_gen_obj_set_int(&jstr, "minor_rev", minor_rev);
  json_gen_obj_set_int(&jstr, "cores", chip_info.cores);
  json_gen_end_object(&jstr);
  json_gen_str_end(&jstr);

  httpd_resp_set_hdr(req, "Content-Type", "application/json");

  httpd_resp_send(req, info_buffer, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static const httpd_uri_t info = {
    .uri = "/api/info",
    .method = HTTP_GET,
    .handler = info_handler,
    .user_ctx = NULL,
};

static int64_t last_read;
static float temperature;
static float humidity;

static esp_err_t dht11_data_handler(httpd_req_t *req) {
  json_gen_str_t jstr;
  memset(dht11_buffer, 0, sizeof(dht11_buffer));

  json_gen_str_start(&jstr, dht11_buffer, sizeof(dht11_buffer), NULL, NULL);
  json_gen_start_object(&jstr);
  json_gen_obj_set_float(&jstr, "temperature", temperature);
  json_gen_obj_set_float(&jstr, "humidity", humidity);
  json_gen_end_object(&jstr);
  json_gen_str_end(&jstr);

  httpd_resp_set_hdr(req, "Content-Type", "application/json");

  httpd_resp_send(req, dht11_buffer, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static const httpd_uri_t dht11_data = {
    .uri = "/api/dht11",
    .method = HTTP_GET,
    .handler = dht11_data_handler,
    .user_ctx = NULL,
};

static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &ledstrip);
    httpd_register_uri_handler(server, &info);
    httpd_register_uri_handler(server, &dht11_data);

    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

static void setup_json_parser() { jsmn_init(&parser); }

static esp_err_t stop_webserver(httpd_handle_t server) {
  // Stop the httpd server
  return httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  httpd_handle_t *server = (httpd_handle_t *)arg;
  if (*server) {
    ESP_LOGI(TAG, "Stopping webserver");
    if (stop_webserver(*server) == ESP_OK) {
      *server = NULL;
    } else {
      ESP_LOGE(TAG, "Failed to stop http server");
    }
  }
}

void read_dht11() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL); // Get current time
  // Access seconds and microseconds
  int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  int64_t diff = time_us - last_read;
  if (diff < 2 * 1000000L) {
    return;
  }

  printf("reading dht11 at %d\n", CONFIG_DHT11_GPIO);
  if (dht_read_float_data(DHT_TYPE_DHT11, CONFIG_DHT11_GPIO, &humidity,
                          &temperature) == ESP_OK) {
    last_read = time_us;
    printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
  }
}

void app_main() {
  board_info();
  configure_led();
  setup_json_parser();

  ESP_ERROR_CHECK(nvs_flash_init());

  while (1) {
    ESP_LOGI(TAG, "wifi init sta...");
    ESP_ERROR_CHECK(wifi_init_sta());

    ESP_LOGI(TAG, "register event handler...");
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    ESP_LOGI(TAG, "start main loop...");
    while (server) {
      display_color();
      read_dht11();
      vTaskDelay(CONFIG_LEDSTRIP_PERIOD / portTICK_PERIOD_MS);
    }
    vTaskDelay(100000 / portTICK_PERIOD_MS);
  }
}
