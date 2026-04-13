#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

static const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

/* Simple RX buffer */
static uint8_t rx_buf[256] = {0};

/* UART interrupt callback */
static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, rx_buf, sizeof(rx_buf));

        if (len > 0) {
            LOG_HEXDUMP_INF(rx_buf, len, "RX");
        }
    }
}

/* Correct TX function */
static void uart_send(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    LOG_HEXDUMP_INF(data, len, "TX");
}

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return 0;
    }

    LOG_INF("UART interrupt test started");

    /* Set callback */
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);

    /* Enable RX interrupt */
    uart_irq_rx_enable(uart_dev);

    /* ---- Wakeup ---- */
    LOG_INF("Sending wakeup command");
    uint8_t wakeup[] = {0x55, 0x55, 0x00, 0x00, 0x00};
    uart_send(wakeup, sizeof(wakeup));

    /* ---- SAMConfig ---- */
    LOG_INF("Sending SAMConfig command");
    uint8_t samconfig_cmd[] = {
        0x00, 0xFF, 0x05, 0xFB,
        0xD4, 0x14,
        0x01, 0x14, 0x01,
        0x02,
        0x00
    };
    uart_send(samconfig_cmd, sizeof(samconfig_cmd));

    while (1) {
        k_sleep(K_MSEC(10));
    }

    return 0;
}