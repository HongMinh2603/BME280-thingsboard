#include "bme280.h"
#include "esp_log.h"

#define BME280_TEMP_PRESS_CALIB_DATA_LEN 26
#define BME280_HUMIDITY_CALIB_DATA_LEN 7
#define BME280_PRESSURE_DATA_ADDR 0xF7
#define BME280_TEMPERATURE_DATA_ADDR 0xFA
#define BME280_HUMIDITY_DATA_ADDR 0xFD
#define BME280_MODE_ADDR 0xF4

static const char *TAG = "BME280";

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

static int32_t t_fine; 

static esp_err_t read_register(i2c_port_t i2c_port, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int32_t compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) {
        return 0;  
  }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

static uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + 
                     ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * 
                     ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
                     ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

esp_err_t bme280_init(i2c_port_t i2c_port, uint8_t i2c_addr) {
    uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN];
    esp_err_t err; 
    err = read_register(i2c_port, i2c_addr, 0x88, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data from BME280: %d", err);
        return err;
    }

    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];
    dig_H1 = calib_data[25];

    uint8_t calib_data_h[BME280_HUMIDITY_CALIB_DATA_LEN];
    err = read_register(i2c_port, i2c_addr, 0xE1, calib_data_h, BME280_HUMIDITY_CALIB_DATA_LEN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read humidity calibration data from BME280: %d", err);
        return err;
    }

    dig_H2 = (calib_data_h[1] << 8) | calib_data_h[0];
    dig_H3 = calib_data_h[2];
    dig_H4 = (calib_data_h[3] << 4) | (calib_data_h[4] & 0x0F);
    dig_H5 = (calib_data_h[5] << 4) | (calib_data_h[4] >> 4);
    dig_H6 = calib_data_h[6];

    uint8_t config_data[2];
    config_data[0] = 0xF2;
    config_data[1] = 0x01;  
    err = i2c_master_write_to_device(i2c_port, i2c_addr, config_data, sizeof(config_data), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BME280: %d", err);
        return err;
    }

    config_data[0] = 0xF4;
    config_data[1] = 0x27;  
    err = i2c_master_write_to_device(i2c_port, i2c_addr, config_data, sizeof(config_data), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BME280: %d", err);
        return err;
    }

    config_data[0] = 0xF5;
    config_data[1] = 0xA0;  
    err = i2c_master_write_to_device(i2c_port, i2c_addr, config_data, sizeof(config_data), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BME280: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "BME280 initialized successfully");
    return ESP_OK;
}

esp_err_t bme280_read_data(i2c_port_t i2c_port, uint8_t i2c_addr, bme280_data_t *data) {
    uint8_t raw_data[8];
    esp_err_t err = read_register(i2c_port, i2c_addr, BME280_PRESSURE_DATA_ADDR, raw_data, 8);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from BME280: %d", err);
        return err;
    }

    int32_t adc_T = (int32_t)(((uint32_t)(raw_data[3]) << 12) | ((uint32_t)(raw_data[4]) << 4) | ((uint32_t)(raw_data[5]) >> 4));
    int32_t adc_P = (int32_t)(((uint32_t)(raw_data[0]) << 12) | ((uint32_t)(raw_data[1]) << 4) | ((uint32_t)(raw_data[2]) >> 4));
    int32_t adc_H = (int32_t)((raw_data[6] << 8) | raw_data[7]);

    data->temperature = compensate_temperature(adc_T) / 100.0;
    data->pressure = compensate_pressure(adc_P) / 25600.0;
    data->humidity = compensate_humidity(adc_H) / 1024.0;

    ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f %%", data->temperature, data->pressure, data->humidity);
    return ESP_OK;
}
