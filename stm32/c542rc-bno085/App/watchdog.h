#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H
#include <stdint.h>
void watchdog_start(void);
void watchdog_feed(void);
uint32_t stack_unused_bytes(void);
extern volatile uint32_t startup_reset_flags;
#endif
