#include "app_camera_esp.h"
#include "sdkconfig.h"

#if (CONFIG_TFLITE_USE_BSP)
#include "bsp/esp-bsp.h"
#endif
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1  // Reset no está conectado
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22


static const char *TAG = "app_camera";

int app_camera_init() {
#if ESP_CAMERA_SUPPORTED
#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
  /* IO13, IO14 is designed for JTAG by default,
   * to use it as generalized input,
   * firstly declare it as pullup input */
  gpio_config_t conf;
  conf.mode = GPIO_MODE_INPUT;
  conf.pull_up_en = GPIO_PULLUP_ENABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  conf.pin_bit_mask = 1LL << 13;
  gpio_config(&conf);
  conf.pin_bit_mask = 1LL << 14;
  gpio_config(&conf);
#endif // CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD

#if (CONFIG_TFLITE_USE_BSP)
  bsp_i2c_init();
  camera_config_t config = BSP_CAMERA_DEFAULT_CONFIG;

#else // CONFIG_TFLITE_USE_BSP
  camera_config_t config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_96X96,
    .fb_location = CAMERA_FB_IN_DRAM,
    .jpeg_quality = 12,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
  };
#endif // CONFIG_TFLITE_USE_BSP

  // Pixel format and frame size specific configurations
  config.pixel_format = CAMERA_PIXEL_FORMAT;
  config.frame_size = CAMERA_FRAME_SIZE;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error PAPU 0x%x", err);
    return -1;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1); // Flip it back

  // Initial sensor settings
  if (s->id.PID == OV3660_PID) {
    s->set_brightness(s, 1);  // Increase brightness
    s->set_saturation(s, -2); // Decrease saturation
  }
  return 0;
#else // ESP_CAMERA_SUPPORTED
  ESP_LOGE(TAG, "Camera is not supported for this device!");
  return -1;
#endif // ESP_CAMERA_SUPPORTED
}
