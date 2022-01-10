#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>


#include "display.h"
#include "dispatcher.h"
#include "shared.h"
#include <u8g2.h>
#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include "u8x8.h"

#include <esp_system.h>
#include <bme280.h>
#include <shared.h>
#include "wifi.h"
#include "udp.h"
#include "hpma.h"
#include "lwip/apps/sntp.h"
// #include "cmdline.h"


ESP_EVENT_DEFINE_BASE(AQI_EVENT_BASE);


static const char *TAG = "main.cpp";
static constexpr TickType_t kUpdateInterval = pdMS_TO_TICKS(2000);


// /*****************************************************/


// #include "esp_log.h"
// #include "driver/uart.h"
// #include "string.h"
// #include "driver/gpio.h"

// static const int RX_BUF_SIZE = 1024;

// #define TXD_PIN (GPIO_NUM_17)
// #define RXD_PIN (GPIO_NUM_18)

// void init(void) {
//     const uart_config_t uart_config = {
//         .baud_rate = 9600,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     // We won't use a buffer for sending data.
//     uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM_1, &uart_config);
//     uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// int sendData(const char* logName, const char* data, int len)
// {
//     // const int len = strlen(data);
//     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//     ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }

// static void tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     const char cmd[] = {0x68, 0x01, 0x01, 0x96};
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) {
//         sendData(TX_TASK_TAG, cmd, 4);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
// }
// /*****************************************************/




// void bmp280_test(void *pvParamters) {
//     bmp280_params_t params;
//     bmp280_init_default_params(&params);
//     bmp280_t dev;
//     memset(&dev, 0, sizeof(bmp280_t));

//     ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, PIN_SDA, PIN_SCL));
//     ESP_ERROR_CHECK(bmp280_init(&dev, &params));

//     bool bme280p = dev.id == BME280_CHIP_ID;
//     printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

//     float pressure, temperature, humidity;

//     while (1)
//     {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
//         {
//             printf("Temperature/pressure reading failed\n");
//             continue;
//         }

//         /* float is used in printf(). you need non-default configuration in
//          * sdkconfig for ESP8266, which is enabled by default for this
//          * example. see sdkconfig.defaults.esp8266
//          */
//         printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
//         if (bme280p)
//             printf(", Humidity: %.2f\n", humidity);
//         else
//             printf("\n");
//     }
// }




// void task_test_SSD1306i2c() {
//     return;
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// 	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
// 	u8g2_esp32_hal.sda   = PIN_SDA;
// 	u8g2_esp32_hal.scl  = PIN_SCL;
//     u8g2_esp32_hal.reset = PIN_RST;
// 	u8g2_esp32_hal_init(u8g2_esp32_hal);


// 	u8g2_t u8g2; // a structure which will contain all the data for one display
// 	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
// 		&u8g2,
// 		U8G2_R0,
// 		//u8x8_byte_sw_i2c,
// 		u8g2_esp32_i2c_byte_cb,
//         // u8x8_byte_sw_i2c, // ok
// 		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
// 	// u8x8_SetPin_SW_I2C(&u8g2.u8x8, PIN_SCL, PIN_SDA, PIN_RST);
//     u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

// 	ESP_LOGI(TAG, "u8g2_InitDisplay");
// 	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
//     ESP_LOGI(TAG, "Address of tile_buf_ptr is %p\n", (void *)(u8g2.tile_buf_ptr));
//     bool power_on = false;
//     // while (1) {
//         ESP_LOGI(TAG, "u8g2_SetPowerSave");
//         u8g2_SetPowerSave(&u8g2, power_on); // wake up display
//         //power_on = !power_on; 
//         ESP_LOGI(TAG, "u8g2_ClearBuffer");
//         u8g2_ClearBuffer(&u8g2);
//         ESP_LOGI(TAG, "u8g2_DrawBox");
//         u8g2_DrawBox(&u8g2, 0, 26, 80,6);
//         u8g2_DrawFrame(&u8g2, 0,26,100,6);

//         ESP_LOGI(TAG, "u8g2_SetFont");
//         u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//         ESP_LOGI(TAG, "u8g2_DrawStr");
//         u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
//         ESP_LOGI(TAG, "u8g2_SendBuffer");
//         u8g2_SendBuffer(&u8g2);

//         ESP_LOGI(TAG, "All done!");
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     // }

// 	vTaskDelete(NULL);
    
// }

/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/

extern "C" {
    void app_main();
}


// void write(void * pvParamters) {
//     DisplayMessage xMessage;
//     DisplayMessage *pxMessage;
//     pxMessage = &xMessage;
//     ESP_LOGI(TAG, "2 Sending.");
//     QueueHandle_t h = *(QueueHandle_t*)(pvParamters);
//     xQueueSend( h, ( void * ) &pxMessage, ( TickType_t ) 0 );
// }


esp_event_loop_handle_t create_event_loop() {
    esp_event_loop_args_t loop_args = {
        .queue_size = 8,
        .task_name = "data event loop",
        .task_priority = tskIDLE_PRIORITY + 3,
        .task_stack_size = configMINIMAL_STACK_SIZE * 8,
        .task_core_id = APP_CPU_NUM
    };

    esp_event_loop_handle_t loop_handle;
    esp_event_loop_create(&loop_args, &loop_handle);
    return loop_handle;
}


void app_main() {

    // init();
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // return;


    // ESP_ERROR_CHECK(i2cdev_init());
    // xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // //task_test_SSD1306i2c();
    // wifi_connect();

    // Dispatcher dispatcher;
    // QueueHandle_t display_q = xQueueCreate(10, sizeof(DisplayMessage*));
    // dispatcher.AddQueue(display_q);
    // //Display display(display_q);

    // // xTaskCreatePinnedToCore(write, "write", configMINIMAL_STACK_SIZE * 8, (void*)&display_q, 5, NULL, APP_CPU_NUM);

    // TaskHandle_t xDisplayTaskHandle;
    // xTaskCreate(
    //                 Display::Create,                       /* Function that implements the task. */
    //                 "Display",                          /* Text name for the task. */
    //                 configMINIMAL_STACK_SIZE * 8,       /* Stack size in words, not bytes. */
    //                 ( void * ) display_q,               /* Parameter passed into the task. */
    //                 tskIDLE_PRIORITY,                   /* Priority at which the task is created. */
    //                 &xDisplayTaskHandle);               /* Used to pass out the created task's handle. */
    
    // DisplayMessage xMessage;
    // DisplayMessage *pxMessage;
    // pxMessage = &xMessage;
    // xQueueSend( display_q, ( void * ) &pxMessage, ( TickType_t ) 0 );
    // while (true) {
    //     xMessage.temperature += 1;
    //     xMessage.humidity += 1;
    //     // ESP_LOGI(TAG, "2 Sending.");
    //     dispatcher.Send(( void * ) &pxMessage, ( TickType_t ) 0 );
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    Flags::Init();
    TaskHandle_t task_bme;
    TaskHandle_t task_hpma;
    TaskHandle_t task_wifi;
    xTaskCreatePinnedToCore(wifi_connect, "wifi_connect", configMINIMAL_STACK_SIZE * 10, NULL, tskIDLE_PRIORITY + 1, &task_wifi, PRO_CPU_NUM);
    // xTaskCreatePinnedToCore(bmp280_test, "bme", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 5, NULL, APP_CPU_NUM);
    
    xTaskCreatePinnedToCore(start_bme280, "start_bme280", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, &task_bme, APP_CPU_NUM);
    xTaskCreatePinnedToCore(start_hpma, "start_hpma", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, &task_hpma, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(start_udp, "start_udp", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 2, NULL, APP_CPU_NUM);

    esp_event_loop_handle_t loop_handle = create_event_loop();
    start_udp(loop_handle);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Allow sensors to init.
    start_display(loop_handle);
    // start_cmdline();

    for (;;) {
        TickType_t start = xTaskGetTickCount();
        xTaskNotify(task_bme, 0, eNoAction);
        xTaskNotify(task_wifi, 0, eNoAction);
        xTaskNotify(task_hpma, 0, eNoAction);
        EventBits_t bits = Flags::WaitAllData(pdMS_TO_TICKS(kUpdateInterval / 2));
        if (!Flags::HasBME(bits)) {
            ESP_LOGI(TAG, "Didn't get data from BME task in time.");
            Flags::getDataHandle()->temperature = NO_VALUE;
            Flags::getDataHandle()->humidity = NO_VALUE;
            Flags::getDataHandle()->pressure = NO_VALUE;
        }
        if (!Flags::HasRSSI(bits)) {
            ESP_LOGI(TAG, "Didn't get data from WiFi task in time.");
            Flags::getDataHandle()->rssi = NO_VALUE;
        }
        if (!Flags::HasHPMA(bits)) {
            ESP_LOGI(TAG, "Didn't get data from HPMA task in time.");
            Flags::getDataHandle()->pm25 = NO_VALUE;
            Flags::getDataHandle()->pm10 = NO_VALUE;
        }
        esp_event_post_to(loop_handle, AQI_EVENT_BASE, AQI_EVENT_ID_NEW_DATA, Flags::getDataHandle(), sizeof(DisplayMessage), 0);
        Flags::SetAllUnavailable();
        
        TickType_t waitticks = kUpdateInterval - (xTaskGetTickCount() - start);
        // ESP_LOGI(TAG, "Waiting %d of total %d ms per loop.", pdTICKS_TO_MS(waitticks), pdTICKS_TO_MS(kUpdateInterval));
        vTaskDelay(waitticks);
    }
}


