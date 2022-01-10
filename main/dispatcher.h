#ifndef AQI_DISPATCHER_H_
#define AQI_DISPATCHER_H_

#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class Dispatcher {

public:
    Dispatcher(/* args */);
    void AddQueue(QueueHandle_t q);
    void Send(const void *const pvItemToQueue, TickType_t xTicksToWait);

private:
    //QueueHandle_t a;
    std::vector<QueueHandle_t> all_queues_;
};

#endif /* AQI_DISPATCHER_H_ */