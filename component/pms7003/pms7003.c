#include "pms7003.h"
#include "esp_log.h"

#define PMS7003_UART_BAUD_RATE CONFIG_PMS7003_UART_BAUD_RATE
#define PMS7003_UART_TX_PIN CONFIG_PMS7003_UART_TX_PIN
#define PMS7003_UART_RX_PIN CONFIG_PMS7003_UART_RX_PIN

static const char *TAG = "PMS7003";

esp_err_t pms7003_init(uart_port_t uart_num) {
    uart_config_t uart_config = {
        .baud_rate = PMS7003_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, PMS7003_UART_TX_PIN, PMS7003_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return uart_driver_install(uart_num, 2048, 0, 0, NULL, 0);
}

esp_err_t pms7003_read_data(uart_port_t uart_num, pms7003_data_t *data, bool is_atmospheric) {
    uint8_t buffer[32];
    int length = uart_read_bytes(uart_num, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
    if (length < 0 || length < sizeof(buffer)) {
        ESP_LOGE(TAG, "Failed to read data from PMS7003");
        return ESP_FAIL;
    }

    if (buffer[0] != 0x42 || buffer[1] != 0x4D) {
        ESP_LOGE(TAG, "Invalid PMS7003 data");
        return ESP_FAIL;
    }

    int start_byte = is_atmospheric ? 10 : 4;

    data->pm1_0 = (buffer[start_byte] << 8) | buffer[start_byte + 1];
    data->pm2_5 = (buffer[start_byte + 2] << 8) | buffer[start_byte + 3];
    data->pm10  = (buffer[start_byte + 4] << 8) | buffer[start_byte + 5];

    ESP_LOGI(TAG, "PM1.0: %d, PM2.5: %d, PM10: %d", data->pm1_0, data->pm2_5, data->pm10);
    return ESP_OK;
}
