#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include "dispatcher.h"
#include <vector>

static const char *TAG = "dispatcher";


Dispatcher::Dispatcher(/* args */){}


void Dispatcher::AddQueue(QueueHandle_t q) {
    all_queues_.push_back(q);
}

void Dispatcher::Send(const void *const pvItemToQueue, TickType_t xTicksToWait) {
    BaseType_t res;
    for(const auto queue: all_queues_) {
        ESP_LOGI(TAG, "Sending.");
        res = xQueueSend(queue, pvItemToQueue, xTicksToWait);
        ESP_LOGI(TAG, "Sent %d", res);
    }
}