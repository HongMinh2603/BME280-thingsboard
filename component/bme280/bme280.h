#ifndef BME280_H
#define BME280_H

#include "driver/i2c.h"

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} bme280_data_t;

esp_err_t bme280_init(i2c_port_t i2c_port, uint8_t i2c_addr);
esp_err_t bme280_read_data(i2c_port_t i2c_port, uint8_t i2c_addr, bme280_data_t *data);

#endif // BME280_H