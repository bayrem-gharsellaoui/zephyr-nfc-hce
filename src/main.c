#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return -1;
    }

    while (1) {
        uart_poll_out(uart_dev, 'B');
        k_msleep(1000);
    }

    return 0;
}