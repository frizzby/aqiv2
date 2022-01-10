#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <bmp280.h>
#include <shared.h>
#include "wifi.h"
#include "bme280.h"
#include "esp_err.h"
#include <cstring>

namespace {

static const char *TAG = "bme280";
static bool disabled = false;
bmp280_t dev;

void setup() {
    if (i2cdev_init() != ESP_OK) {
        disabled = true;
        ESP_LOGE(TAG, "BME280: setup failed. Disabling bme readings.");
        return;
    }
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    memset(&dev, 0, sizeof(bmp280_t));

    if (bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, PIN_BME_SDA, PIN_BME_SCL) != ESP_OK) {
        disabled = true;
        ESP_LOGE(TAG, "BME280: setup failed. Disabling bme readings.");
        return;
    }
    if(bmp280_init(&dev, &params) != ESP_OK) {
        disabled = true;
        ESP_LOGE(TAG, "BME280: setup failed. Disabling bme readings.");
        return;
    }

    bool bme280p = dev.id == BME280_CHIP_ID;
    ESP_LOGI(TAG, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void run() {
    float pressure, temperature, humidity;
    uint32_t ulNotifiedValue;

    while (1) {
        while(!xTaskNotifyWait(0x00,       /* Don't clear any notification bits on entry. */
                         ULONG_MAX,        /* Reset the notification value to 0 on exit. */
                         &ulNotifiedValue, /* Notified value pass out in
                                              ulNotifiedValue. */
                         portMAX_DELAY ));  /* Block indefinitely. */
        // ESP_LOGI(TAG, "BMP280: received notification.");

        if (disabled) {
            DisplayMessage* data = Flags::getDataHandle();
            data->temperature = NO_VALUE;
            data->humidity = NO_VALUE;
            data->pressure = NO_VALUE;
            Flags::SetBMEAvailable();
            continue;
        }

        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK) {
            ESP_LOGE(TAG, "Temperature/pressure/humidity reading failed.");
            continue;
        }
        DisplayMessage* data = Flags::getDataHandle();
        data->temperature = temperature;
        data->humidity = humidity;
        data->pressure = pressure;
        Flags::SetBMEAvailable();
        //TODO: Send msg to the event bus.
        // printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        // if (bme280p) {
        //     printf(", Humidity: %.2f\n", humidity);
        // }
    }
    vTaskDelete(NULL);
}

} // namespace

void start_bme280(void *pvParameters) {
    setup();
    run();
}