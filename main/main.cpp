#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fcntl.h>
#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "es8311.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "audio_stream";

/* Wi-Fi Configuration */
#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY 5

/* I2C Configuration for ES8311 */
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

/* I2S Configuration */
#define I2S_MCLK_IO 13
#define I2S_BCLK_IO 12
#define I2S_WS_IO 10
#define I2S_DOUT_IO 9
#define I2S_DIN_IO 11

#define SAMPLE_RATE 16000
#define SAMPLE_BITS 16
#define I2S_CHANNELS 2
#define RECORD_BUFFER_SIZE 1024

/* Wav Header constant */
struct wav_header_t {
  char riff[4];
  uint32_t overall_size;
  char wave[4];
  char fmt_chunk_marker[4];
  uint32_t length_of_fmt;
  uint16_t format_type;
  uint16_t channels;
  uint32_t sample_rate;
  uint32_t byterate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data_chunk_header[4];
  uint32_t data_size;
};

static EventGroupHandle_t s_wifi_event_group;
static i2s_chan_handle_t rx_handle = NULL;
static i2s_chan_handle_t tx_handle = NULL;
static es8311_handle_t es8311_dev = NULL;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

/* Wi-Fi Event Handler */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < ESP_MAXIMUM_RETRY) {
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
  }
}

/* Initialize Wi-Fi */
void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *netif = esp_netif_create_default_wifi_sta();
  esp_netif_set_hostname(netif, CONFIG_DEVICE_HOSTNAME);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = ESP_WIFI_SSID,
              .password = ESP_WIFI_PASS,
              .scan_method = WIFI_FAST_SCAN,
              .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
              .threshold =
                  {
                      .rssi = -127,
                      .authmode = WIFI_AUTH_OPEN,
                  },
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");
}

/* Initialize ES8311 */
static esp_err_t audio_codec_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = I2C_MASTER_FREQ_HZ,
          },
      .clk_flags = 0,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

  es8311_dev = es8311_create(I2C_MASTER_NUM, ES8311_ADDRESS_0);
  ESP_RETURN_ON_FALSE(es8311_dev, ESP_FAIL, TAG, "es8311 create failed");

  es8311_clock_config_t es8311_clock = {
      .mclk_inverted = false,
      .sclk_inverted = false,
      .mclk_from_mclk_pin = true,
      .mclk_frequency = SAMPLE_RATE * 256, // Typical MCLK
      .sample_frequency = SAMPLE_RATE,
  };

  ESP_ERROR_CHECK(es8311_init(es8311_dev, &es8311_clock, ES8311_RESOLUTION_16,
                              ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_microphone_gain_set(es8311_dev, ES8311_MIC_GAIN_30DB));
  ESP_ERROR_CHECK(
      es8311_voice_volume_set(es8311_dev, 60, NULL)); // Just in case

  // Configure Microphone
  ESP_ERROR_CHECK(es8311_microphone_config(es8311_dev, false)); // Analog Mic

  // Enable PA (Power Amplifier) - GPIO 53
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << 53);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)53, 0); // Start with PA OFF

  return ESP_OK;
}

/* Initialize I2S */
static esp_err_t i2s_init(void) {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = (gpio_num_t)I2S_MCLK_IO,
              .bclk = (gpio_num_t)I2S_BCLK_IO,
              .ws = (gpio_num_t)I2S_WS_IO,
              .dout = (gpio_num_t)I2S_DOUT_IO,
              .din = (gpio_num_t)I2S_DIN_IO,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  return ESP_OK;
}

/* HTTP GET Handler */
static esp_err_t stream_get_handler(httpd_req_t *req) {
  struct wav_header_t wav_header = {
      .riff = {'R', 'I', 'F', 'F'},
      .overall_size = 0xFFFFFFFF, // Unknown, max
      .wave = {'W', 'A', 'V', 'E'},
      .fmt_chunk_marker = {'f', 'm', 't', ' '},
      .length_of_fmt = 16,
      .format_type = 1, // PCM
      .channels = I2S_CHANNELS,
      .sample_rate = SAMPLE_RATE,
      .byterate = SAMPLE_RATE * I2S_CHANNELS * (SAMPLE_BITS / 8),
      .block_align = I2S_CHANNELS * (SAMPLE_BITS / 8),
      .bits_per_sample = SAMPLE_BITS,
      .data_chunk_header = {'d', 'a', 't', 'a'},
      .data_size = 0xFFFFFFFF, // Unknown
  };

  httpd_resp_set_type(req, "audio/wav");
  httpd_resp_send_chunk(req, (const char *)&wav_header, sizeof(wav_header));

  size_t bytes_read = 0;
  char *i2s_buffer = (char *)calloc(RECORD_BUFFER_SIZE, 1);
  if (!i2s_buffer) {
    ESP_LOGE(TAG, "Failed to allocate buffer");
    return ESP_FAIL;
  }

  while (1) {
    if (i2s_channel_read(rx_handle, i2s_buffer, RECORD_BUFFER_SIZE, &bytes_read,
                         1000) == ESP_OK) {
      if (bytes_read > 0) {
        if (httpd_resp_send_chunk(req, i2s_buffer, bytes_read) != ESP_OK) {
          break;
        }
      }
    } else {
      ESP_LOGW(TAG, "Read Failed");
    }
    // Minimal delay to prevent watchdog if needed
    vTaskDelay(1);
  }
  free(i2s_buffer);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static const httpd_uri_t stream = {.uri = "/stream",
                                   .method = HTTP_GET,
                                   .handler = stream_get_handler,
                                   .user_ctx = NULL};

/* Simple Linear Resampler Helper */
static int16_t interpolate(int16_t s1, int16_t s2, float frac) {
  return s1 + (int16_t)((s2 - s1) * frac);
}

/* HTTP POST Handler for Playback */
static esp_err_t play_post_handler(httpd_req_t *req) {
  int total_len = req->content_len;
  int cur_len = 0;
  char *buf = ((char *)calloc(RECORD_BUFFER_SIZE, 1));
  int16_t *resample_buf =
      (int16_t *)calloc(RECORD_BUFFER_SIZE, 2); // Larger buf for processing
  int received = 0;

  if (total_len < sizeof(wav_header_t)) {
    free(buf);
    free(resample_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File too small");
    return ESP_FAIL;
  }

  /* Read Header */
  received = httpd_req_recv(req, buf, RECORD_BUFFER_SIZE);
  if (received <= 0) {
    free(buf);
    free(resample_buf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Failed to read header");
    return ESP_FAIL;
  }

  /* Parse WAV Header */
  wav_header_t *header = (wav_header_t *)buf;
  if (memcmp(header->riff, "RIFF", 4) != 0 ||
      memcmp(header->wave, "WAVE", 4) != 0) {
    free(buf);
    free(resample_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid WAV header");
    return ESP_FAIL;
  }

  uint32_t sample_rate = header->sample_rate;
  uint16_t channels = header->channels;
  uint16_t bits = header->bits_per_sample;

  /* Enable PA */
  gpio_set_level((gpio_num_t)53, 1);

  /* Resampling state */
  float step = (float)sample_rate / (float)SAMPLE_RATE;
  float pos = 0.0f;

  /* Initial offset - skip header */
  int data_offset = sizeof(wav_header_t);

  /* Validate supported input formats for conversion */
  /* We only support 16-bit input for simplicity in this demo. 8-bit/24-bit
   * requires more conversion logic */
  if (bits != 16) {
    free(buf);
    free(resample_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                        "Only 16-bit WAV supported for resampling");
    return ESP_FAIL;
  }

  cur_len += received;

  /* Process loops */
  while (true) { /* Loop until file end */

    int bytes_available = received - data_offset;
    int16_t *input_samples = (int16_t *)(buf + data_offset);
    int num_input_samples = bytes_available / 2; // 16-bit samples
    int frames = num_input_samples / channels;

    /* Process buffer */
    int out_idx = 0;

    while (pos < frames - 1) { // -1 for interpolation
      int idx = (int)pos;
      float frac = pos - idx;

      int16_t left_in, right_in;
      int16_t left_next, right_next;

      if (channels == 1) {
        left_in = input_samples[idx];
        right_in = input_samples[idx];
        left_next = input_samples[idx + 1];
        right_next = input_samples[idx + 1];
      } else {
        left_in = input_samples[idx * 2];
        right_in = input_samples[idx * 2 + 1];
        left_next = input_samples[(idx + 1) * 2];
        right_next = input_samples[(idx + 1) * 2 + 1];
      }

      int16_t left_out = interpolate(left_in, left_next, frac);
      int16_t right_out = interpolate(right_in, right_next, frac);

      resample_buf[out_idx++] = left_out;
      resample_buf[out_idx++] = right_out;

      /* Flush if buffer full */
      if (out_idx >= (RECORD_BUFFER_SIZE / 2)) {
        size_t bytes_written = 0;
        i2s_channel_write(tx_handle, resample_buf, out_idx * 2, &bytes_written,
                          1000);
        out_idx = 0;
      }

      pos += step;
    }

    /* Send remaining */
    if (out_idx > 0) {
      size_t bytes_written = 0;
      i2s_channel_write(tx_handle, resample_buf, out_idx * 2, &bytes_written,
                        1000);
    }

    /* Adjust pos for next chunk */
    pos -= (floor(pos));

    /* Note: Cross-chunk interpolation is imperfect here for simplicity (we drop
       the fractional/last sample). To be perfect, we'd save the last sample.
       But for a quick stream, this is okay. */

    /* Read Next Chunk */
    if (cur_len >= total_len)
      break;

    data_offset = 0; // Reset offset for subsequent chunks
    received = httpd_req_recv(req, buf, RECORD_BUFFER_SIZE);
    if (received <= 0)
      break;
    cur_len += received;
  }

  /* Flush with Silence to prevent buzzing */
  memset(resample_buf, 0, RECORD_BUFFER_SIZE * 2);
  size_t bytes_written = 0;
  i2s_channel_write(tx_handle, resample_buf, RECORD_BUFFER_SIZE * 2,
                    &bytes_written, 1000);

  /* Disable PA */
  gpio_set_level((gpio_num_t)53, 0);

  free(buf);
  free(resample_buf);
  httpd_resp_send(req, "Playback Finished", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static const httpd_uri_t play = {.uri = "/play",
                                 .method = HTTP_POST,
                                 .handler = play_post_handler,
                                 .user_ctx = NULL};

/* HTTP POST Handler for Volume Control */
static esp_err_t volume_post_handler(httpd_req_t *req) {
  char buf[32];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char val_str[8];
    if (httpd_query_key_value(buf, "val", val_str, sizeof(val_str)) == ESP_OK) {
      int input_vol = atoi(val_str);
      if (input_vol < 0)
        input_vol = 0;
      if (input_vol > 100)
        input_vol = 100;

      int hw_volume = 0;
      if (input_vol > 0) {
        /* Map 1-100 to 60-80 */
        hw_volume = 60 + ((input_vol - 1) * (80 - 60) / 99);
      }

      ESP_LOGI(TAG, "Setting Volume: Input=%d, HW=%d", input_vol, hw_volume);
      if (es8311_voice_volume_set(es8311_dev, hw_volume, NULL) == ESP_OK) {
        httpd_resp_send(req, "Volume Updated", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
    }
  }
  httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                      "Invalid Volume param (?val=0-100)");
  return ESP_FAIL;
}

static const httpd_uri_t volume = {.uri = "/volume",
                                   .method = HTTP_POST,
                                   .handler = volume_post_handler,
                                   .user_ctx = NULL};

static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  // Increase stack size for the handler
  config.stack_size = 8192;

  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &stream);
    httpd_register_uri_handler(server, &play);
    httpd_register_uri_handler(server, &volume);
    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

extern "C" void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();

  // Wait for Wi-Fi
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s", ESP_WIFI_SSID);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  // Init Audio
  ESP_LOGI(TAG, "Initializing I2C and Codec");
  audio_codec_init();

  ESP_LOGI(TAG, "Initializing I2S");
  i2s_init();

  // Start Server
  ESP_LOGI(TAG, "Starting Webserver");
  start_webserver();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}