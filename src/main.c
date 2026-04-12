#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

void pn532_uart_send(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    LOG_HEXDUMP_INF(data, len, "TX");
}

int pn532_uart_read(uint8_t *buf, size_t max_len, int timeout_ms)
{
    int64_t end = k_uptime_get() + timeout_ms;
    int64_t now;
    size_t idx = 0;

    while (idx < max_len) {
        now = k_uptime_get();
        if (now >= end) {
            break;
        }

        uint8_t c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            buf[idx++] = c;
        } else {
            k_yield();
        }
    }

    if (idx > 0) {
        LOG_HEXDUMP_INF(buf, idx, "RX");
    } else {
        LOG_WRN("RX timeout (%d ms)", timeout_ms);
    }

    return idx;
}

void pn532_uart_flush(void)
{
    uint8_t c;
    while (uart_poll_in(uart_dev, &c) == 0) {
        /* discard */
    }
}

bool pn532_is_ack(uint8_t *buf)
{
    const uint8_t ack[] = {0x00,0x00,0xFF,0x00,0xFF,0x00};
    return memcmp(buf, ack, 6) == 0;
}

void test_fw_version(void)
{
    uint8_t cmd[] = {
        0x00, 0xFF, 0x02, 0xFE,
        0xD4, 0x02,
        0x2A,
        0x00
    };

    pn532_uart_send(cmd, sizeof(cmd));

    uint8_t buf[64];

    // Read ACK
    pn532_uart_read(buf, 6, 100);
    if (!pn532_is_ack(buf)) {
        LOG_ERR("No ACK");
        return;
    }

    // Read response
    int len = pn532_uart_read(buf, sizeof(buf), 200);

    if (len > 0) {
        LOG_INF("Got response");
    }
}

int main(void)
{
    uint8_t buf[64] = {0};
    int len = 0;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return -1;
    }
    LOG_INF("UART ready");

    LOG_INF("PN532 UART test starting...");

    /* ---- Wakeup sequence ---- */
    uint8_t wakeup[] = {0x55, 0x55, 0x00, 0x00, 0x00};
    pn532_uart_send(wakeup, sizeof(wakeup));

    k_msleep(2);  // give PN532 time to wake up
    pn532_uart_flush();

    /* ---- SAMConfig ---- */
    uint8_t samconfig_cmd[] = {
        0x00, 0xFF, 0x05, 0xFB,
        0xD4, 0x14,
        0x01, 0x14, 0x01,
        0x02,
        0x00
    };

    pn532_uart_send(samconfig_cmd, sizeof(samconfig_cmd));

    /* Read ACK */
    len = pn532_uart_read(buf, 6, 200);
    if (len != 6 || !pn532_is_ack(buf)) {
        LOG_ERR("SAMConfig: No ACK");
        return 0;
    }

    LOG_INF("SAMConfig ack OK");

    /* Read response */
    memset(buf, 0, sizeof(buf));
    len = pn532_uart_read(buf, sizeof(buf), 300);
    if (len <= 0) {
        LOG_ERR("SAMConfig: No response");
        return 0;
    }

    /* Optional check */
    if (buf[6] != 0x15) {
        LOG_ERR("SAMConfig failed");
        return 0;
    }

    LOG_INF("SAMConfig response OK");

    /* ---- getFirmwareVersion command ---- */
    uint8_t cmd[] = {
        0x00, 0xFF, 0x02, 0xFE,
        0xD4, 0x02,
        0x2A,
        0x00
    };

    pn532_uart_send(cmd, sizeof(cmd));

    /* ---- Read ACK ---- */
    memset(buf, 0, sizeof(buf));
    len = pn532_uart_read(buf, 6, 200);
    if (len != 6 || !pn532_is_ack(buf)) {
        LOG_ERR("ACK not received");
        return 0;
    }

    LOG_INF("ACK OK");

    /* ---- Read response ---- */
    memset(buf, 0, sizeof(buf));
    len = pn532_uart_read(buf, sizeof(buf), 300);
    if (len <= 0) {
        LOG_ERR("No response");
        return 0;
    }

    LOG_INF("Response received");

    uint8_t ic = buf[7];
    uint8_t ver_major = buf[8];
    uint8_t ver_minor = buf[9];
    LOG_INF("Found chip PN5%02X", ic);
    LOG_INF("Firmware version: %d.%d", ver_major, ver_minor);

    while (1) {
        k_msleep(1000);
    }

    return 0;
}