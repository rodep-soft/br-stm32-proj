/**
 * @file app_log.h
 * @brief Leveled, timestamped, atomic-per-line logging over USART3 (ST-LINK VCP).
 *
 * Line format:   [  12.345][I][zenoh ] message
 *                  uptime   lvl  tag
 *
 * - One mutex-protected UART write per line: lines from different tasks never
 *   interleave.
 * - Bounded UART timeout: a stuck/unplugged UART can never hang the robot.
 * - Safe to call from any task. NOT for ISRs (an ISR call is counted and
 *   dropped, never blocks).
 */
#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_LOG_ERROR = 0,
    APP_LOG_WARN,
    APP_LOG_INFO,
    APP_LOG_DEBUG,
} app_log_level_t;

/** Create the mutex. Call once before the scheduler starts (or right after). */
void app_log_init(void);

void app_log_set_level(app_log_level_t level);
app_log_level_t app_log_get_level(void);

void app_log(app_log_level_t level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/** Single character sink for newlib printf()/puts() (see main.c __io_putchar). */
void app_log_putc(int ch);

/** Lock-free raw write, usable from fault handlers / panic paths. */
void app_log_panic_write(const char *s);

/** Number of lines lost (called from ISR, or UART timeout). */
uint32_t app_log_dropped(void);

#define LOGE(tag, ...) app_log(APP_LOG_ERROR, (tag), __VA_ARGS__)
#define LOGW(tag, ...) app_log(APP_LOG_WARN,  (tag), __VA_ARGS__)
#define LOGI(tag, ...) app_log(APP_LOG_INFO,  (tag), __VA_ARGS__)
#define LOGD(tag, ...) app_log(APP_LOG_DEBUG, (tag), __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
