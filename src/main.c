#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define PN532_ACK_LEN       6
#define PN532_MAX_RESP_LEN  64

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));
static const uint8_t pn532_ack_pattern[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

static RING_BUF_DECLARE(uart_ringbuf, 256);
static K_SEM_DEFINE(uart_rx_sem, 0, 16);

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t buf[32] = {0};

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len > 0) {
            ring_buf_put(&uart_ringbuf, buf, len);
            k_sem_give(&uart_rx_sem);
        }
    }
}

static int rb_read_exact(uint8_t *buf, uint8_t len, uint16_t timeout_ms)
{
    uint32_t deadline = k_uptime_get_32() + timeout_ms;
    uint8_t  received = 0;

    while (received < len) {
        uint32_t now = k_uptime_get_32();
        if (now >= deadline) {
            return -ETIMEDOUT;
        }
        uint32_t remaining_ms = deadline - now;

        /* Drain whatever is already in the ring buffer */
        received += ring_buf_get(&uart_ringbuf, buf + received, len - received);
        if (received < len) {
            /* Nothing yet — wait for ISR to signal more data */
            k_sem_take(&uart_rx_sem, K_MSEC(remaining_ms));
        }
    }
    return 0;
}

static int pn532_read_ack(uint16_t timeout_ms)
{
    uint8_t buf[PN532_ACK_LEN];
    int ret = rb_read_exact(buf, sizeof(buf), timeout_ms);
    if (ret) {
        LOG_ERR("ACK timeout");
        return ret;
    }
    if (memcmp(buf, pn532_ack_pattern, PN532_ACK_LEN) != 0) {
        LOG_HEXDUMP_ERR(buf, sizeof(buf), "Bad ACK:");
        return -EIO;
    }
    return 0;
}

static int pn532_read_response(uint8_t *resp, uint8_t resp_max,
                               uint8_t *resp_len_out, uint16_t timeout_ms)
{
    /* Read preamble + START1 + START2 + LEN + LCS = 5 bytes */
    uint8_t hdr[5];
    int ret = rb_read_exact(hdr, sizeof(hdr), timeout_ms);
    if (ret) { LOG_ERR("Response header timeout"); return ret; }

    if (hdr[0] != 0x00 || hdr[1] != 0x00 || hdr[2] != 0xFF) {
        LOG_HEXDUMP_ERR(hdr, sizeof(hdr), "Bad response preamble:");
        return -EIO;
    }
    if ((uint8_t)(hdr[3] + hdr[4]) != 0x00) {
        LOG_ERR("LEN/LCS checksum fail: 0x%02x + 0x%02x", hdr[3], hdr[4]);
        return -EIO;
    }

    uint8_t frame_len = hdr[3];      /* TFI(1) + CMD(1) + data(N) */
    if (frame_len < 2) { return -EIO; }

    /* Read body: frame_len bytes + DCS(1) + postamble(1) */
    uint8_t body[PN532_MAX_RESP_LEN + 2];
    uint8_t to_read = frame_len + 2;
    ret = rb_read_exact(body, to_read, timeout_ms);
    if (ret) { LOG_ERR("Response body timeout"); return ret; }

    /* body[0]=TFI(0xD5), body[1]=CMD+1, body[2..frame_len-1]=payload */
    if (body[0] != 0xD5) {
        LOG_ERR("Bad TFI: 0x%02x", body[0]);
        return -EIO;
    }

    /* Verify DCS */
    uint8_t dcs = 0;
    for (uint8_t i = 0; i < frame_len; i++) dcs += body[i];
    dcs = (uint8_t)(~dcs + 1);
    if (dcs != body[frame_len]) {
        LOG_ERR("DCS mismatch: calc=0x%02x got=0x%02x", dcs, body[frame_len]);
        return -EIO;
    }

    /* Copy payload (skip TFI and CMD bytes) */
    uint8_t payload_len = frame_len - 2;
    *resp_len_out = MIN(payload_len, resp_max);
    memcpy(resp, &body[2], *resp_len_out);

    return 0;
}

static int pn532_send_cmd(const uint8_t *cmd, uint8_t cmd_len,
                          uint8_t *resp, uint8_t resp_max,
                          uint8_t *resp_len_out,
                          uint16_t timeout_ms)
{
    /* 1. Build and send the frame */
    pn532_write_command(cmd, cmd_len);   /* your existing function */

    /* 2 + 3. Wait for ACK (PN532 must respond within 15ms per spec) */
    int ret = pn532_read_ack(15);
    if (ret) {
        LOG_ERR("No ACK for cmd 0x%02x", cmd[0]);
        return ret;
    }

    /* 4 + 5. Wait for and parse the response */
    if (resp && resp_len_out) {
        ret = pn532_read_response(resp, resp_max, resp_len_out, timeout_ms);
    }

    return ret;
}

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART not ready");
        return -1;
    }

    /* Enable UART RX interrupt */
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    LOG_INF("UART interrupt enabled");

    /* ================= WAKEUP ================= */
    LOG_INF("Waking up PN532...");
    const uint8_t wakeup[] = {0x55, 0x00, 0x00};
    uart_write(uart_dev, wakeup, sizeof(wakeup));
    k_msleep(200);
    flush_ring_buf();

    /* ================= TEST COMMAND ================= */
    LOG_INF("Sending GetFirmwareVersion...");
    uint8_t cmd[] = {0x02};
    uint8_t resp[8] = {0};
    uint8_t resp_len = 0;
    int err = pn532_send_cmd(cmd, sizeof(cmd), resp, sizeof(resp), &resp_len, 500);
    if (err == 0) {
        LOG_INF("PN532 FW v%u.%u  IC=0x%02x  Support=0x%02x",
                resp[1], resp[2], resp[0], resp[3]);
    }

    while (1) {
        k_msleep(1000);
    }

    return 0;
}