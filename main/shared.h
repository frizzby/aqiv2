#ifndef AQI_SHARED_H_
#define AQI_SHARED_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_event.h>

// #include <bitset>

#define AQI_DEVICE_ID "white01"

#define ESP_WIFI_SSID      "need-beer"
#define ESP_WIFI_PASS      ""
#define ESP_WIFI_10TH_RETRY_DELAY_MS  10000

#define WIFI_AVAILABLE_BIT (1 << 0) /* BIT0 */
#define HPMA_AVAILABLE_BIT (1 << 1) /* BIT1 */
#define BME_AVAILABLE_BIT (1 << 2) /* BIT2 */
#define RSSI_AVAILABLE_BIT (1 << 3) /* BIT3 */

#define ALL_AVAILABLE_BIT (BME_AVAILABLE_BIT | RSSI_AVAILABLE_BIT | HPMA_AVAILABLE_BIT)

#define NO_VALUE (-100)

// enum MsgType { red, green, blue };

// struct Message {

// };
//static inline const char *TAG = "main.cpp";

ESP_EVENT_DECLARE_BASE(AQI_EVENT_BASE);

enum {
    AQI_EVENT_ID_NEW_DATA
};

struct DisplayMessage {
    double temperature;
    double humidity;
    double pressure;
    int pm25;
    int pm10;
    int8_t rssi;
    char hpma_coeff = 0;
};

class Flags {
    public:
     static void Init() {
         Flags::eg_ = xEventGroupCreate();
     }

    static bool HasWifi() {
         return (xEventGroupWaitBits(eg_, WIFI_AVAILABLE_BIT, pdFALSE, pdFALSE, 0) & WIFI_AVAILABLE_BIT) == 1;
     }

     static void WaitWifi() {
         //ESP_LOGI(TAG, "BMP280: waiting for notification.");
         while((xEventGroupWaitBits(eg_, WIFI_AVAILABLE_BIT, pdFALSE, pdFALSE, portMAX_DELAY) & WIFI_AVAILABLE_BIT) == 0);
     }

     static void SetWifiAvailable() {
         xEventGroupSetBits(eg_, WIFI_AVAILABLE_BIT);
     }
     static void SetWifiUnavailable() {
         xEventGroupClearBits(eg_, WIFI_AVAILABLE_BIT);
     }

    static void SetRSSIAvailable() {
        xEventGroupSetBits(eg_, RSSI_AVAILABLE_BIT);
    }
    static void SetRSSIUnavailable() {
        xEventGroupClearBits(eg_, RSSI_AVAILABLE_BIT);
    }

    static void SetBMEAvailable() {
         xEventGroupSetBits(eg_, BME_AVAILABLE_BIT);
     }
     static void SetBMEUnavailable() {
         xEventGroupClearBits(eg_, BME_AVAILABLE_BIT);
     }

    static void SetHPMAAvailable() {
         xEventGroupSetBits(eg_, HPMA_AVAILABLE_BIT);
     }
     static void SetHPMAUnavailable() {
         xEventGroupClearBits(eg_, HPMA_AVAILABLE_BIT);
     }

    static EventBits_t WaitAllData(const TickType_t deadline) {
        return xEventGroupWaitBits(eg_, ALL_AVAILABLE_BIT, pdTRUE, pdTRUE, deadline);
    }

    static void SetAllUnavailable() {
        xEventGroupClearBits(eg_, ALL_AVAILABLE_BIT);
    }

    static inline bool HasRSSI(const EventBits_t bits) {
        return (bits & RSSI_AVAILABLE_BIT);
    }

    static inline bool HasBME(const EventBits_t bits) {
        // std::bitset<32> x(bits);
        return (bits & BME_AVAILABLE_BIT);
    }

    static inline bool HasHPMA(const EventBits_t bits) {
        return (bits & HPMA_AVAILABLE_BIT);
    }
    static DisplayMessage* getDataHandle() {
         return &data_;
    }

    private:
     static inline EventGroupHandle_t eg_;
     static inline DisplayMessage data_;
     // xEventGroupCreate();
};

int make_hex(const char * text, char * buf);

#endif /* AQI_SHARED_H_ */