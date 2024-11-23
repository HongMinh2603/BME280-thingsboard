#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "wifi.h"

#include "bme280.h"

#include "thingsboard.h"

#include "pms7003.h"

esp_mqtt_client_handle_t client;

#define TIME_DELAY_MS CONFIG_TIME_DELAY_MS
#define IN_ATMOSPHERIC_MODE CONFIG_IN_ATMOSPHERIC_MODE

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO
#define I2C_MASTER_NUM CONFIG_I2C_MASTER_NUM
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQ_HZ
#define BME280_I2C_ADDRESS CONFIG_BME280_I2C_ADDRESS

#define THINGSBOARD_SERVER CONFIG_THINGSBOARD_SERVER
#define THINGSBOARD_PORT CONFIG_THINGSBOARD_PORT
#define THINGSBOARD_TOKEN CONFIG_THINGSBOARD_TOKEN
#define THINGSBOARD_TOPIC "v1/devices/me/telemetry"

static const char *TAG = "bme280_thingsboard";

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

    if (pms7003_init(UART_NUM_1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PMS7003");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_SERVER,
        .credentials.username = THINGSBOARD_TOKEN,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    if (bme280_init(I2C_MASTER_NUM, BME280_I2C_ADDRESS) == ESP_OK) {
        bme280_data_t bme280_data;
        pms7003_data_t pms7003_data;
        bool is_atmospheric = IN_ATMOSPHERIC_MODE; 

        while (1) {
            if (bme280_read_data(I2C_MASTER_NUM, BME280_I2C_ADDRESS, &bme280_data) == ESP_OK) {
                printf("Temperature: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa\n",
                       bme280_data.temperature, bme280_data.humidity, bme280_data.pressure);

                char msg[200];
                snprintf(msg, sizeof(msg), "{\"Temperature\": %.2f, \"Humidity\": %.2f, \"Pressure\": %.2f}",
                         bme280_data.temperature, bme280_data.humidity, bme280_data.pressure);
                esp_mqtt_client_publish(client, THINGSBOARD_TOPIC, msg, 0, 1, 0);
            } else {
                printf("Failed to read data from BME280 sensor\n");
            }

            if (pms7003_read_data(UART_NUM_1, &pms7003_data, is_atmospheric) == ESP_OK) {
                ESP_LOGI(TAG, "PM1.0: %d ug/m3, PM2.5: %d ug/m3, PM10: %d ug/m3",
                         pms7003_data.pm1_0, pms7003_data.pm2_5, pms7003_data.pm10);

                char msg[200];
                snprintf(msg, sizeof(msg), "{\"PM1.0\": %d, \"PM2.5\": %d, \"PM10\": %d}",
                         pms7003_data.pm1_0, pms7003_data.pm2_5, pms7003_data.pm10);
                esp_mqtt_client_publish(client, THINGSBOARD_TOPIC, msg, 0, 1, 0);
            } else {
                ESP_LOGE(TAG, "Failed to read data from PMS7003");
            }

            vTaskDelay(pdMS_TO_TICKS(TIME_DELAY_MS)); 
        }
    } else {
        printf("Failed to initialize BME280 sensor\n");
    }
}
