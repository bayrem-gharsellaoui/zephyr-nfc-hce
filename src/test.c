#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

static const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

static uint8_t rx_buf[256] = {0};
static volatile size_t rx_len = 0;

/* UART interrupt callback */
static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, rx_buf + rx_len, sizeof(rx_buf) - rx_len);
        if (len > 0) {
            rx_len += len;
            LOG_HEXDUMP_INF(rx_buf, rx_len, "RX");
        }
    }
}

static void uart_send(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    LOG_HEXDUMP_INF(data, len, "TX");
}

static bool wait_for_rx(size_t expected_len, int timeout_ms)
{
    int64_t end = k_uptime_get() + timeout_ms;

    while (k_uptime_get() < end) {
        if (rx_len >= expected_len) {
            return true;
        }
        k_sleep(K_MSEC(1));
    }

    return false;
}

static void rx_reset(void)
{
    rx_len = 0;
}

static bool is_ack(uint8_t *buf)
{
    const uint8_t ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    return memcmp(buf, ack, 6) == 0;
}

static bool pn532_send_command(const uint8_t *cmd, size_t cmd_len)
{
    /* Reset RX buffer */
    rx_reset();

    /* Send command */
    uart_send(cmd, cmd_len);

    /* Wait for ACK */
    if (!wait_for_rx(6, 200)) {
        LOG_ERR("Timeout waiting for ACK");
        return false;
    }

    if (!is_ack(rx_buf)) {
        LOG_ERR("Invalid ACK");
        return false;
    }

    LOG_INF("ACK received");

    /* If response already in buffer */
    if (rx_len > 6) {
        LOG_INF("Response already received");
        return true;
    }

    /* Otherwise wait for more */
    int64_t end = k_uptime_get() + 300;
    while (k_uptime_get() < end) {
        if (rx_len > 6) {
            LOG_INF("Response received");
            return true;
        }
        k_sleep(K_MSEC(1));
    }

    LOG_ERR("Timeout waiting for response");
    return false;
}

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return 0;
    }

    LOG_INF("PN532 test started");

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
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

    if (!pn532_send_command(samconfig_cmd, sizeof(samconfig_cmd))) {
        LOG_ERR("SAMConfig failed");
        return 0;
    }

    /* Check SAMConfig response manually */
    if ((rx_len < 8) || (rx_buf[6] != 0x15)) {
        LOG_ERR("Invalid SAMConfig response");
        return 0;
    }

    LOG_INF("SAMConfig OK");

    /* ---- GetFirmwareVersion ---- */
    LOG_INF("Sending GetFirmwareVersion");
    uint8_t fw_cmd[] = {
        0x00, 0xFF, 0x02, 0xFE,
        0xD4, 0x02,
        0x2A,
        0x00
    };

    if (!pn532_send_command(fw_cmd, sizeof(fw_cmd))) {
        LOG_ERR("GetFirmwareVersion failed");
        return 0;
    }

    /* Parse firmware response manually */
    if (rx_len < 12) {
        LOG_ERR("Response too short");
        return 0;
    }
    /*
     * Expected:
     * ... D5 03 IC Ver Rev Support ...
     * indexes:
     * rx_buf[6] = 0x03
     * rx_buf[7] = IC
     * rx_buf[8] = Ver
     * rx_buf[9] = Rev
     */
    if (rx_buf[6] != 0x03) {
        LOG_ERR("Unexpected response code: 0x%02X", rx_buf[6]);
        return 0;
    }

    uint8_t ic  = rx_buf[7];
    uint8_t ver = rx_buf[8];
    uint8_t rev = rx_buf[9];

    LOG_INF("Found PN5%02X", ic);
    LOG_INF("Firmware version: %d.%d", ver, rev);

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}