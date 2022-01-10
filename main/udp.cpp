#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include <arpa/inet.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "shared.h"
#include <string>
#include <cstring>


#define HOST_IP_ADDR "255.255.255.255"
#define PORT 5987

static char *TAG = "udp";
static int sock;
sockaddr_in dest_addr;
char payload_c[140];

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        
    if (!Flags::HasWifi()) {
        return;
    }
    DisplayMessage* data = (DisplayMessage*)event_data;

    if (data->temperature == NO_VALUE && data->rssi == NO_VALUE) {
        return;
    }

    // strcpy(payload_c, "airhome ");
    sprintf(payload_c, "%s ", AQI_DEVICE_ID);
    if (data->temperature != NO_VALUE) {
        sprintf(payload_c + strlen(payload_c), "temp=%.2f,humid=%.2f,press=%.2f", data->temperature, data->humidity, data->pressure);
    }
    if (data->rssi != NO_VALUE) {
        sprintf(payload_c + strlen(payload_c), "%srssi=%d", data->temperature != NO_VALUE ? "," : "", data->rssi);
    }
    if (data->pm25 != NO_VALUE) {
        sprintf(payload_c + strlen(payload_c), "%spm25=%d,pm10=%d", data->temperature != NO_VALUE || data->rssi != NO_VALUE ? "," : "", data->pm25, data->pm10);
    }
    // printf("Sending: %s\n", payload_c);
    //std::string payload = "airhome temp=" + std::to_string(data->temperature) + ",humid=" + std::to_string(data->humidity) + ",press=" + std::to_string(data->pressure);
    //airhome temp=-99.99,humid=100.00,press=101169.68,rssi=-100
    
    int err = sendto(sock, payload_c, strlen(payload_c), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d: %s", errno, std::strerror(errno));
        return;
    }
    // ESP_LOGI(TAG, "Message sent");
}

static void init(const esp_event_loop_handle_t loop_handle) {
    Flags::WaitWifi();

    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    int trueflag = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &trueflag, sizeof trueflag) < 0) {
        ESP_LOGE(TAG, "Unable to setsockopt: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
    //TODO: Set up handler.
    esp_event_handler_register_with(loop_handle, AQI_EVENT_BASE, AQI_EVENT_ID_NEW_DATA, event_handler, NULL);
}

// static void udp_client(const DisplayMessage *data) {
//     int addr_family = 0;
//     int ip_protocol = 0;

//     while (1) {
//         Flags::WaitWifi();
//         struct sockaddr_in dest_addr;
//         dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
//         dest_addr.sin_family = AF_INET;
//         dest_addr.sin_port = htons(PORT);
//         addr_family = AF_INET;
//         ip_protocol = IPPROTO_IP;

//         int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//         if (sock < 0) {
//             ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
//             break;
//         }
//         int trueflag = 1;
//         if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &trueflag, sizeof trueflag) < 0) {
//             ESP_LOGE(TAG, "Unable to setsockopt: errno %d", errno);
//             break;
//         }
//         ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

//         while (1) {
//             Flags::WaitWifi();
//             std::string payload = "airhome temp=" + std::to_string(data->temperature) + ",humid=" + std::to_string(data->humidity) + ",press=" + std::to_string(data->pressure);
//             int err = sendto(sock, payload.c_str(), payload.length(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//             if (err < 0) {
//                 ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
//                 break;
//             }
//             ESP_LOGI(TAG, "Message sent");

//             vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }

//         if (sock != -1) {
//             ESP_LOGE(TAG, "Shutting down socket and restarting...");
//             shutdown(sock, 0);
//             close(sock);
//         }
//     }
//     vTaskDelete(NULL);
// }

void start_udp(const esp_event_loop_handle_t loop_handle) {
    //esp_event_loop_handle_t loop_handle = *((esp_event_loop_handle_t*)(pvParameters));
    //esp_event_handler_register_with(loop_handle, MY_EVENT_BASE, MY_EVENT_ID, run_on_event, ...);

    init(loop_handle);
}