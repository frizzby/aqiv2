#include <stdio.h>
#include <cstring>
#include "esp_log.h"
#include "shared.h"

static const char *TAG = "shared.c";

int make_hex(const char * text, char * buf) {
    size_t olen = strlen(text);
    int rlen = olen * 3;
    for (int i = 0; i < olen; i++) {
      sprintf(buf + i*3, "%02x:", text[i]);
    }
    buf[rlen-1] = '\r';
    buf[rlen-1] = '\n';
    buf[rlen+1] = '\0';
    ESP_LOGI(TAG, "Hexed: %s", buf);
    ESP_LOG_BUFFER_HEX(TAG, buf, rlen+2);
    return rlen+2;
}