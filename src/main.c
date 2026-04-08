#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

static const uint8_t GetFirmwareVersion[] = {
    0x00, 0x00, 0xFF,  // Preamble
    0x02, 0xFE,        // LEN, LCS
    0xD4,              // TFI (host → PN532)
    0x02,              // GetFirmwareVersion
    0x2A,              // DCS
    0x00               // Postamble
};

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return -1;
    }

    /* Send the GetFirmwareVersion command */
    LOG_INF("Sending GetFirmwareVersion command...");
    for (size_t i = 0; i < sizeof(GetFirmwareVersion); i++) {
        uart_poll_out(uart_dev, GetFirmwareVersion[i]);
    }

    /* Read the response */
    LOG_INF("Reading response...");
    
    uint8_t byte = 0;
    while (1) {
        if (uart_poll_in(uart_dev, &byte) == 0) {
            printk("%02X ", byte);
        } else {
            k_msleep(1);
        }
    }

    return 0;
}