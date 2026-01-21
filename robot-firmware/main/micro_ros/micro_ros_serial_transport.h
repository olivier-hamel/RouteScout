#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uxrCustomTransport;

bool micro_ros_serial_open(struct uxrCustomTransport *transport);
bool micro_ros_serial_close(struct uxrCustomTransport *transport);
size_t micro_ros_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t micro_ros_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif
