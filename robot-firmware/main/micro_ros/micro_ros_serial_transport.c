#include "micro_ros_serial_transport.h"

#include <uxr/client/transport.h>

#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "micro_ros_uart";

#ifndef CONFIG_MICROROS_UART_TXD
#define CONFIG_MICROROS_UART_TXD (-1)
#endif
#ifndef CONFIG_MICROROS_UART_RXD
#define CONFIG_MICROROS_UART_RXD (-1)
#endif
#ifndef CONFIG_MICROROS_UART_RTS
#define CONFIG_MICROROS_UART_RTS (-1)
#endif
#ifndef CONFIG_MICROROS_UART_CTS
#define CONFIG_MICROROS_UART_CTS (-1)
#endif

#ifndef CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE
#define CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE (115200)
#endif

// The default 512-byte RX buffer can overrun at high baud rates (e.g., 921600)
// when the system is busy. Use larger buffers for more robust serial transport.
#define UART_RX_BUFFER_SIZE (2048)
#define UART_TX_BUFFER_SIZE (2048)

bool micro_ros_serial_open(struct uxrCustomTransport *transport)
{
    uart_port_t *uart_port = (uart_port_t *)transport->args;

    ESP_LOGI(TAG,
             "Opening UART transport: port=%d baud=%d TX=%d RX=%d",
             (int)(*uart_port),
             (int)CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE,
             (int)CONFIG_MICROROS_UART_TXD,
             (int)CONFIG_MICROROS_UART_RXD);

    uart_config_t uart_config = {
        .baud_rate = CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(*uart_port, &uart_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed");
        return false;
    }

    if (uart_set_pin(*uart_port,
                     CONFIG_MICROROS_UART_TXD,
                     CONFIG_MICROROS_UART_RXD,
                     CONFIG_MICROROS_UART_RTS,
                     CONFIG_MICROROS_UART_CTS) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return false;
    }

    if (uart_driver_install(*uart_port, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 0, NULL, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return false;
    }

    ESP_LOGI(TAG, "UART transport ready");
    return true;
}

bool micro_ros_serial_close(struct uxrCustomTransport *transport)
{
    uart_port_t *uart_port = (uart_port_t *)transport->args;
    return uart_driver_delete(*uart_port) == ESP_OK;
}

size_t micro_ros_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)err;
    uart_port_t *uart_port = (uart_port_t *)transport->args;
    const int tx_bytes = uart_write_bytes(*uart_port, (const char *)buf, len);
    return (tx_bytes > 0) ? (size_t)tx_bytes : 0;
}

size_t micro_ros_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)err;
    uart_port_t *uart_port = (uart_port_t *)transport->args;
    const int rx_bytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return (rx_bytes > 0) ? (size_t)rx_bytes : 0;
}
