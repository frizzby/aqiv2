#ifndef AQI_BME280_H_
#define AQI_BME280_H_

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define PIN_BME_SDA (gpio_num_t)4
#define PIN_BME_SCL (gpio_num_t)15

void start_bme280(void *pvParameters);

#endif /* AQI_BME280_H_ */