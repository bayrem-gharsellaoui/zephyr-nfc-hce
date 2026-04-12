#ifndef PN532_H
#define PN532_H

#include <zephyr/device.h>

uint32_t pn532_get_firmware_version(const struct device *const uart_dev);

#endif /* PN532_H */