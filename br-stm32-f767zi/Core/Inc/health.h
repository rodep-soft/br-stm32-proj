/**
 * @file health.h
 * @brief Reset-cause tracking, crash records that survive reset, IWDG control.
 *
 * Design goal: the robot must come back by itself from ANY failure
 * (hard fault, assert, hung task, runaway loop) and tell us afterwards
 * exactly what happened.
 */
#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Call first thing in main() (after HAL_Init): reads the reset cause, loads
 *  the persistent record and starts the independent watchdog. */
void health_early_init(void);

/** Feed the independent watchdog. */
void health_kick(void);

/** Print boot banner, reset cause and the previous crash (if any). Needs app_log. */
void health_log_boot_report(void);

/** One-line summary of persistent counters (boots / watchdog resets / crashes). */
void health_log_summary(void);

/** Record a reason, flush it to the UART and reset the MCU. Never returns. */
void health_reset(const char *reason) __attribute__((noreturn));

/** Entry from HardFault_Handler. Never returns. */
void crash_report_from_fault(uint32_t *stacked_regs, uint32_t exc_lr) __attribute__((noreturn));

/** configASSERT() target. Never returns. */
void app_assert_failed(const char *file, int line) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_H */
