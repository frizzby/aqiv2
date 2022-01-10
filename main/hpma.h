#ifndef AQI_HPMA_H_
#define AQI_HPMA_H_

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_event.h"


#define PIN_HPMA_RX (gpio_num_t)18
#define PIN_HPMA_TX (gpio_num_t)17
#define PIN_HPMA_RTS  (UART_PIN_NO_CHANGE)
#define PIN_HPMA_CTS  (UART_PIN_NO_CHANGE)

void start_hpma(void *pvParameters);

#endif /* AQI_HPMA_H_ */