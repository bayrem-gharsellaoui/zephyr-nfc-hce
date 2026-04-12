#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

static const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

/* Simple RX buffer */
#define RX_BUF_SIZE 64
static uint8_t rx_buf[RX_BUF_SIZE];

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

    /* Send something */
    const char msg[] = "PING\r\n";

    for (size_t i = 0; i < sizeof(msg) - 1; i++)
    {
        uart_poll_out(uart_dev, msg[i]);
    }

    LOG_INF("TX: %s", msg);

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}