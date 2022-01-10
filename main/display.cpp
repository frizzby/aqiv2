#include <freertos/FreeRTOS.h>
// #include <freertos/queue.h>
// #include <freertos/semphr.h>
#include <esp_log.h>
#include <math.h>
#include <string>
#include "display.h"
#include <u8g2.h>
#include "sdkconfig.h"
#include "shared.h"
#include "u8g2_esp32_hal.h"
#include "u8x8.h"
#include "esp_event.h"
#include <cstring>

namespace {

static const char *TAG = "display";
u8g2_t handle_u8g2_;
bool blinker;

void show(DisplayMessage* info) {
    // std::string m_temp, m_humid, m_press, m_rssi;
    char m_temp[17];
    char m_rssi[5];
    char m_pm25[4];
    //char pm25_offset, pm10_offset;
    u8g2_ClearBuffer(&handle_u8g2_);

    // if (info.temperature != NO_VALUE)
    // m_temp = std::to_string(round(info.temperature));
    // m_humid = std::to_string(round(info.humidity));
    // m_press = std::to_string(round(info.pressure));
    // m_rssi = std::to_string(round(info.rssi));
  //  info = *Flags::getDataHandle();
  if (info->temperature != NO_VALUE) {
    sprintf(m_temp, "%.1f", info->temperature);
  } else {
    sprintf(m_temp, " ");
  }
  if (info->rssi != NO_VALUE) {
    sprintf(m_rssi, "%d", info->rssi);
  } else {
    sprintf(m_rssi, "n/a");
  }
  if (info->pm25 != NO_VALUE) {
    sprintf(m_pm25, "%d", info->pm25);
  } else {
    sprintf(m_pm25, "n/a");
  }
  blinker = !blinker;

  const uint8_t *line1_font, *degree_font;
  int line1_len = strlen(m_temp) + strlen(m_pm25);
  switch (line1_len) {
    case 8:
    case 7:
      // u8g2_SetFont(&handle_u8g2_, u8g2_font_fur20_tr);
      line1_font = u8g2_font_fur20_tr;
      degree_font = u8g2_font_fur20_tf;
      break;
    case 6:
      // u8g2_SetFont(&handle_u8g2_, u8g2_font_fur25_tr);
      line1_font = u8g2_font_fur25_tr;
      degree_font = u8g2_font_fur25_tf;
      break;
    default:
      // u8g2_SetFont(&handle_u8g2_, u8g2_font_fur30_tr);
      line1_font = u8g2_font_fur30_tr;
      degree_font = u8g2_font_fur30_tf;
  }
  u8g2_SetFont(&handle_u8g2_, line1_font);

    //u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2_SetBitmapMode(&handle_u8g2_, 1);
    // u8g2_SetFont(&handle_u8g2_, u8g2_font_balthasar_titling_nbp_tr);
    // pm25_offset = u8g2_DrawStr(&handle_u8g2_, 0, 16*1-1, "pm2.5");
    //u8g2.drawStr(0, 16*2-1, m_str1);
    // pm10_offset = u8g2_DrawStr(&handle_u8g2_, 0, 16*3-1, "pm10");
    //u8g2.drawStr(0, 16*4-1, m_str2);
    // u8g2_SetFont(&handle_u8g2_, u8g2_font_fur20_tr);
    // u8g2_DrawStr(&handle_u8g2_, 63, 32*1-1, m_temp);
    // u8g2_DrawStr(&handle_u8g2_, 63, 32*2, m_rssi);
    auto temp_off = u8g2_DrawStr(&handle_u8g2_, 0, 32*1-1, m_temp);
    auto offset = u8g2_DrawStr(&handle_u8g2_, 129, 32*1, m_pm25);
    u8g2_DrawStr(&handle_u8g2_, 128 - offset - 5, 32*1, m_pm25);

 
    
    // u8g2_SetFont(&handle_u8g2_, u8g2_font_fur30_tr);
    u8g2_SetFont(&handle_u8g2_, u8g2_font_fur30_tr);
    if (blinker) {
      u8g2_DrawGlyph(&handle_u8g2_, 120, 64, 0x2E); // full stop
    }
    u8g2_DrawStr(&handle_u8g2_, 0, 32*2, m_rssi);
  
    u8g2_SetFont(&handle_u8g2_, u8g2_font_fur11_tr);
    u8g2_DrawStr(&handle_u8g2_, 60 + 4, 32*2, "dBm");

    if (info->temperature != NO_VALUE) {
      u8g2_SetFont(&handle_u8g2_, degree_font);
      // u8g2_DrawUTF8(&handle_u8g2_, temp_off + 2, 32*1-1, "°");
      // u8g2_DrawGlyph(&handle_u8g2_, temp_off + 7, 32*1-1, 0xB0); // full stop
      u8g2_DrawGlyph(&handle_u8g2_, temp_off + 1, 32*1-3, 0xB0); // degree
      // u8g2_DrawGlyph(&handle_u8g2_, 0, temp_off + 2, 0xb0); // degree
    }
    
 
    //u8g2_SetFont(&handle_u8g2_, u8g2_font_open_iconic_arrow_2x_t);
    //u8g2_DrawGlyph(&handle_u8g2_, pm25_offset-1 + round((63 - pm25_offset-1) / 2) - 5, 15, 64);

    u8g2_SendBuffer(&handle_u8g2_);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  DisplayMessage* data = (DisplayMessage*)event_data;
  show(data);
}

void init(const esp_event_loop_handle_t loop_handle) {
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_DISPLAY_SDA;
  u8g2_esp32_hal.scl = PIN_DISPLAY_SCL;
  u8g2_esp32_hal.reset = PIN_DISPLAY_RST;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&handle_u8g2_, U8G2_R0, u8g2_esp32_i2c_byte_cb,
                                         u8g2_esp32_gpio_and_delay_cb);
  u8x8_SetI2CAddress(&handle_u8g2_.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&handle_u8g2_);
  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&handle_u8g2_, false);  // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&handle_u8g2_);
  esp_event_handler_register_with(loop_handle, AQI_EVENT_BASE, AQI_EVENT_ID_NEW_DATA, event_handler, NULL);
  // xTaskCreatePinnedToCore(+[](){}, );
}

} // namespace

void start_display(const esp_event_loop_handle_t loop_handle) {
    //esp_event_loop_handle_t loop_handle = *((esp_event_loop_handle_t*)(pvParameters));
    //esp_event_handler_register_with(loop_handle, MY_EVENT_BASE, MY_EVENT_ID, run_on_event, ...);

    init(loop_handle);
}