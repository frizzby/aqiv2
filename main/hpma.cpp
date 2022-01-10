#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include <memory>
#include "hpma.h"
#include "hpma115s0.h"
#include <shared.h>
#include <esp_log.h>

namespace {

static constexpr int RX_BUF_SIZE = 1024 * 2;
static const char *TAG = "HPMA";

std::unique_ptr<HPMA115S0> setup() {
  /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    // uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    // uart_config.source_clk = UART_SCLK_APB;

    uart_driver_install(UART_NUM_2, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, PIN_HPMA_TX, PIN_HPMA_RX, PIN_HPMA_RTS, PIN_HPMA_CTS);

    // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
 
  vTaskDelay(pdMS_TO_TICKS(2000));
  std::unique_ptr<HPMA115S0> hpma115S0 = std::make_unique<HPMA115S0>(UART_NUM_2);
//   HPMA115S0 hpma115S0(UART_NUM_2);
  hpma115S0->Init();
  hpma115S0->StartParticleMeasurement();
  
  ESP_LOGI(TAG, "Setup complete.");
  
  return hpma115S0;
}

void run(std::unique_ptr<HPMA115S0> hpma115S0) {
    uint32_t ulNotifiedValue;

    ESP_LOGI(TAG, "Starting loop.");
    while (1) {
        while(!xTaskNotifyWait(0x00,       /* Don't clear any notification bits on entry. */
                         ULONG_MAX,        /* Reset the notification value to 0 on exit. */
                         &ulNotifiedValue, /* Notified value pass out in
                                              ulNotifiedValue. */
                         portMAX_DELAY ));  /* Block indefinitely. */
        // ESP_LOGI(TAG, "received notification.");

        // if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK) {
        //     ESP_LOGE(TAG, "Temperature/pressure/humidity reading failed.");
        //     continue;
        // }
        DisplayMessage* data = Flags::getDataHandle();

        // if (data->hpma_coeff != 0) {
        //   hpma115S0->SetCustomerAdjustmentCoefficient(data->hpma_coeff);
        //   data->hpma_coeff = 0;
        // }

        if (hpma115S0->ReadParticleMeasurement(&(data->pm25), &(data->pm10))) {
            Flags::SetHPMAAvailable();
        }
    }
    vTaskDelete(NULL);
}

} // namespace

void start_hpma(void *pvParameters) {
    //esp_event_loop_handle_t loop_handle = *((esp_event_loop_handle_t*)(pvParameters));
    //esp_event_handler_register_with(loop_handle, MY_EVENT_BASE, MY_EVENT_ID, run_on_event, ...);

    run(setup());
}







