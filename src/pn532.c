#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pn532, LOG_LEVEL_DBG);

static void uart_send(const struct device *const uart_dev, uint8_t *data, size_t len)
{
    LOG_HEXDUMP_DBG(data, len, "TX");

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

static int uart_read(const struct device *const uart_dev, uint8_t *buf, size_t len, int timeout_ms)
{
    int64_t start = k_uptime_get();
    size_t received = 0;

    while (received < len) {
        if (uart_poll_in(uart_dev, &buf[received]) == 0) {
            received++;
        } else {
            if ((k_uptime_get() - start) > timeout_ms) {
                return -1;
            }
            k_msleep(1);
        }
    }

    LOG_HEXDUMP_DBG(buf, len, "RX");
    return 0;
}

static int uart_wait_data(const struct device *const uart_dev, int timeout_ms)
{
    int64_t start = k_uptime_get();
    uint8_t dummy;

    while ((k_uptime_get() - start) < timeout_ms) {
        if (uart_poll_in(uart_dev, &dummy) == 0) {
            /* put back into flow by returning success */
            return 0;
        }
        k_msleep(1);
    }

    return -1;
}

static size_t pn532_build_frame(uint8_t *cmd, uint8_t cmdlen, uint8_t *frame)
{
    uint8_t len = cmdlen + 1;
    uint8_t lcs = ~len + 1;

    uint8_t sum = 0;

    frame[0] = 0x00;
    frame[1] = 0x00;
    frame[2] = 0xFF;
    frame[3] = len;
    frame[4] = lcs;
    frame[5] = 0xD4;

    sum += 0xD4;

    for (uint8_t i = 0; i < cmdlen; i++) {
        frame[6 + i] = cmd[i];
        sum += cmd[i];
    }

    frame[6 + cmdlen] = ~sum + 1;
    frame[7 + cmdlen] = 0x00;

    return 8 + cmdlen;
}

static int pn532_send_cmd(const struct device *const uart_dev, uint8_t *cmd, uint8_t cmd_len,
                          uint8_t *resp, size_t resp_len)
{
    uint8_t frame[64];
    uint8_t ack[6];

    const uint8_t expected_ack[6] = {
        0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00
    };

    /* 1. Build + send */
    size_t len = pn532_build_frame(cmd, cmd_len, frame);
    uart_send(uart_dev, frame, len);
    k_msleep(100);

    /* 2. Wait for data */
    if (uart_wait_data(uart_dev, 1000) != 0) {
        LOG_ERR("Timeout waiting for ACK");
        return -1;
    }

    /* 3. Read ACK */
    if (uart_read(uart_dev, ack, 6, 1000) != 0) {
        LOG_ERR("Failed to read ACK");
        return -1;
    }

    if (memcmp(ack, expected_ack, 6) != 0) {
        LOG_ERR("Invalid ACK");
        return -1;
    }

    /* 4. Wait for response */
    if (uart_wait_data(uart_dev, 1000) != 0) {
        LOG_ERR("Timeout waiting for response");
        return -1;
    }

    /* 5. Read response */
    if (uart_read(uart_dev, resp, resp_len, 1000) != 0) {
        LOG_ERR("Failed to read response");
        return -1;
    }

    return 0;
}

static uint32_t pn532_get_firmware_version(const struct device *const uart_dev)
{
    uint8_t cmd[] = {0x02};
    uint8_t resp[12] = {0};

    if (pn532_send_cmd(uart_dev, cmd, sizeof(cmd), resp, sizeof(resp)) != 0) {
        return 0;
    }

    /* Basic validation */
    if (!(resp[0] == 0x00 && resp[1] == 0x00 && resp[2] == 0xFF)) {
        LOG_ERR("Invalid response frame");
        return 0;
    }

    int offset = 7;

    uint32_t version = resp[offset++];
    version <<= 8;
    version |= resp[offset++];
    version <<= 8;
    version |= resp[offset++];
    version <<= 8;
    version |= resp[offset++];

    return version;
}
