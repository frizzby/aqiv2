#ifndef AQI_DISPLAY_H_
#define AQI_DISPLAY_H_

#include "esp_event.h"


#define PIN_DISPLAY_SDA (gpio_num_t)4
#define PIN_DISPLAY_SCL (gpio_num_t)15
#define PIN_DISPLAY_RST (gpio_num_t)16

void start_display(const esp_event_loop_handle_t loop_handle);

#endif /* AQI_DISPLAY_H_ */