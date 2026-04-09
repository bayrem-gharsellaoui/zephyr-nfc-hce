#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

RING_BUF_DECLARE(uart_ring_buf, 256U);
K_SEM_DEFINE(uart_rx_sem, 0, 1);

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t buf[32] = {0};

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len > 0) {
            ring_buf_put(&uart_ring_buf, buf, len);
            k_sem_give(&uart_rx_sem);
        }
    }
}

static size_t pn532_build_frame(uint8_t *cmd, uint8_t cmdlen, uint8_t *frame)
{
    uint8_t LEN = cmdlen + 1;
    uint8_t LCS = ~LEN + 1;

    uint8_t sum = 0;

    frame[0] = 0x00;
    frame[1] = 0x00;
    frame[2] = 0xFF;
    frame[3] = LEN;
    frame[4] = LCS;
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

static void pn532_uart_send(uint8_t *frame, size_t len)
{
    LOG_HEXDUMP_DBG(frame, len, "TX");

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, frame[i]);
    }
}

static int pn532_send_cmd(uint8_t *cmd, uint8_t cmd_len,
                          uint8_t *resp, uint8_t *resp_len)
{
    uint8_t cmd_frame[64] = {0};
    uint8_t ack_frame[6] = {0};
    const uint8_t expected_ack_frame[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

    /* 1. Build command frame */
    size_t frame_len = pn532_build_frame(cmd, cmd_len, cmd_frame);

    /* 2. Send frame via UART */
    pn532_uart_send(cmd_frame, frame_len);

    /* 3. Wait for RX indication (for ack) */
    if (k_sem_take(&uart_rx_sem, K_MSEC(100)) != 0) {
        LOG_ERR("Timeout waiting for ACK");
        return -ETIMEDOUT;
    }

    /* 4. Read and verify ACK */
    ring_buf_get(&uart_ring_buf, ack_frame, sizeof(ack_frame));
    LOG_HEXDUMP_DBG(ack_frame, 6, "ACK");
    if (memcmp(ack_frame, expected_ack_frame, 6) != 0) {
        LOG_ERR("ACK invalid");
        return -1;
    }

    /* 5. Wait for RX indication again (but this time for the actual response) */
    if (k_sem_take(&uart_rx_sem, K_MSEC(200)) != 0) {
        LOG_ERR("Timeout waiting for response");
        return -ETIMEDOUT;
    }

    /* TODO: 6. Read response */

    return 0;
}

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return -1;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    LOG_INF("UART ready");

    LOG_INF("Waking up PN532...");
    uint8_t wakeup[] = {0x55, 0x00, 0x00};
    pn532_uart_send(wakeup, sizeof(wakeup));
    k_msleep(100);

    LOG_INF("Getting firmware version...");
    uint8_t cmd[] = {0x02};
    uint8_t resp[32];
    uint8_t resp_len = sizeof(resp);
    if (pn532_send_cmd(cmd, sizeof(cmd), resp, &resp_len) == 0) {
        LOG_INF("Command success");
        /* Decode useful fields */
        if (resp_len >= 6) {
            uint8_t ic = resp[2];
            uint8_t ver = resp[3];
            uint8_t rev = resp[4];

            LOG_INF("PN532 IC: 0x%02X, Firmware: %d.%d", ic, ver, rev);
        }
    } else {
        LOG_ERR("Command failed");
    }

    while (1) {
        k_msleep(1000);
    }

    return 0;
}