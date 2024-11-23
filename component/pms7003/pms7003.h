#ifndef PMS7003_H
#define PMS7003_H

#include "driver/uart.h"
#include "esp_err.h"

typedef struct {
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
} pms7003_data_t;

esp_err_t pms7003_init(uart_port_t uart_num);
esp_err_t pms7003_read_data(uart_port_t uart_num, pms7003_data_t *data, bool is_atmospheric);

#endif
