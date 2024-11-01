#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "wifi.h"

#include "bme280.h"

#include "thingsboard.h"

esp_mqtt_client_handle_t client;
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO
#define I2C_MASTER_NUM CONFIG_I2C_MASTER_NUM
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQ_HZ
#define BME280_I2C_ADDRESS CONFIG_BME280_I2C_ADDRESS

#define SSID CONFIG_WIFI_SSID
#define PASS CONFIG_WIFI_PASSWORD
#define THINGSBOARD_SERVER CONFIG_THINGSBOARD_SERVER
#define THINGSBOARD_PORT CONFIG_THINGSBOARD_PORT
#define THINGSBOARD_TOKEN CONFIG_THINGSBOARD_TOKEN
#define THINGSBOARD_TOPIC "v1/devices/me/telemetry"

static const char *TAG = "bme280_thingsboard";


void app_main(void) {
    nvs_flash_init();
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

   esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_SERVER,
        .credentials.username = THINGSBOARD_TOKEN,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    if (bme280_init(I2C_MASTER_NUM, BME280_I2C_ADDRESS) == ESP_OK) {
        bme280_data_t data;
        while (1) {
            if (bme280_read_data(I2C_MASTER_NUM, BME280_I2C_ADDRESS, &data) == ESP_OK) {
                printf("Temperature: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa\n",
                    data.temperature, data.humidity, data.pressure);

                char msg[100];
                snprintf(msg, sizeof(msg), "{\"Temperature\": %.2f, \"Humidity\": %.2f, \"Pressure\": %.2f}",
                         data.temperature, data.humidity, data.pressure); // Example payload
                esp_mqtt_client_publish(client, THINGSBOARD_TOPIC, msg, 0, 1, 0);
            } else {
                printf("Failed to read data from BME280 sensor\n");
            }
            vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
        }
    } else {
        printf("Failed to initialize BME280 sensor\n");
    }
}
