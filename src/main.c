#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(pn532_uart));

RING_BUF_DECLARE(uart_ringbuf, 256);

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    uint8_t buf[32] = {0};

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len > 0) {
            ring_buf_put(&uart_ringbuf, buf, len);
        }
    }
}

static void uart_write(const struct device *uart, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart, data[i]);
    }
}

static void flush_ring_buf(void)
{
    uint8_t dummy;
    while (ring_buf_get(&uart_ringbuf, &dummy, 1) == 1);
}

static void dump_uart_stream(void)
{
    uint8_t byte;

    printk("RX: ");
    while (ring_buf_get(&uart_ringbuf, &byte, 1) == 1) {
        printk("%02X ", byte);
    }
    printk("\n");
}

static void pn532_write_command(uint8_t *cmd, uint8_t cmdlen)
{
    uint8_t frame[64];

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

    uint8_t DCS = ~sum + 1;

    frame[6 + cmdlen] = DCS;
    frame[7 + cmdlen] = 0x00;

    printk("TX: ");
    for (int i = 0; i < 8 + cmdlen; i++) {
        printk("%02X ", frame[i]);
    }
    printk("\n");

    uart_write(uart_dev, frame, 8 + cmdlen);
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

    k_msleep(200); // give PN532 time

    flush_ring_buf();

    /* ================= TEST COMMAND ================= */
    LOG_INF("Sending GetFirmwareVersion...");
    uint8_t cmd[] = {0x02}; // GetFirmwareVersion
    pn532_write_command(cmd, sizeof(cmd));

    /* Wait for response */
    k_msleep(200);

    dump_uart_stream();

    while (1) {
        k_msleep(1000);

        /* continuously print anything incoming */
        dump_uart_stream();
    }

    return 0;
}