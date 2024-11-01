#ifndef __THINHSBOARD_H__
#define __THINHSBOARD_H__

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mqtt_client.h"


void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

#endif