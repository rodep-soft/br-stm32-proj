#include "watchdog.h"
#include "stm32c5xx_ll_iwdg.h"
#include "stm32c5xx_ll_rcc.h"

volatile uint32_t startup_reset_flags;
extern uint32_t __StackLimit;
#define STACK_MARKER 0xa59bc37dU
static uint32_t *stack_mark_end;

void watchdog_start(void)
{
    startup_reset_flags = RCC->RSR;
    LL_RCC_ForceClearResetFlags();
    LL_RCC_ReleaseClearResetFlags();
    /* Enabling IWDG also requests LSI. No option-byte changes or IRQ feeding.
       32 kHz / 128, reload 1999 -> nominal 8 s (LSI tolerance applies).
       Start before system initialization so a stuck clock/HAL init also resets. */
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_128);
    LL_IWDG_SetReloadCounter(IWDG, 1999U);
    while (!LL_IWDG_IsReady(IWDG)) { /* Hardware watchdog bounds this wait. */ }
    LL_IWDG_ReloadCounter(IWDG);
    LL_IWDG_DisableWriteAccess(IWDG);
    /* Paint only unused stack below the live startup frames, before IRQ setup.
       The unpainted guard makes the reported free space conservative. */
    stack_mark_end = (uint32_t *)((__get_MSP() - 128U) & ~3U);
    for (uint32_t *p = &__StackLimit; p < stack_mark_end; ++p) *p = STACK_MARKER;
}

void watchdog_feed(void) { LL_IWDG_ReloadCounter(IWDG); }

uint32_t stack_unused_bytes(void)
{
    uint32_t *p = &__StackLimit;
    while (p < stack_mark_end && *p == STACK_MARKER) ++p;
    return (uint32_t)((uintptr_t)p - (uintptr_t)&__StackLimit);
}
