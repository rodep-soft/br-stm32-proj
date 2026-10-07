/**
 * @file health.c
 * @brief Reset cause, crash records that survive reset, IWDG, fault reporting for STM32H5.
 */
#include "health.h"

#include <string.h>

#include "main.h"
#include "app_log.h"

/* ───────────────────── Persistent record (survives soft reset) ─────────── */

#define PERSIST_MAGIC 0xB0B0C0DEU

typedef struct {
    uint32_t magic;
    uint32_t boots;
    uint32_t iwdg_resets;
    uint32_t fault_resets;
    uint32_t panic_resets;   /* assert / explicit health_reset() */
    uint32_t other_resets;
    uint32_t crash_unread;   /* 1 = details below not yet shown after boot */
    uint32_t kind;           /* 1 = HardFault, 2 = panic/assert */
    uint32_t regs[8];        /* r0 r1 r2 r3 r12 lr pc psr */
    uint32_t exc_lr;
    uint32_t cfsr, hfsr, mmfar, bfar;
    uint32_t uptime_ms;
    char     reason[64];
    uint32_t checksum;
} persist_t;

/* .persistent_ram is marked NOLOAD in STM32H5E4xx_FLASH.ld: neither zeroed nor loaded at startup,
 * so it keeps its content across IWDG / software resets. Validity is guarded by magic+sum. */
static persist_t g_p __attribute__((section(".persistent_ram")));
static const char *s_cause = "unknown";

static uint32_t persist_sum(const persist_t *p) {
    const uint32_t *w = (const uint32_t *)p;
    uint32_t s = 0xA5A5A5A5U;
    for (size_t i = 0; i < (sizeof(*p) / 4U) - 1U; i++) {
        s = (s << 1 | s >> 31) ^ w[i];
    }
    return s;
}

static void persist_seal(void) {
    g_p.checksum = persist_sum(&g_p);
}

static bool persist_valid(void) {
    return g_p.magic == PERSIST_MAGIC && g_p.checksum == persist_sum(&g_p);
}

/* ───────────────────────────── IWDG ────────────────────────────────────── */

/* LSI ~32 kHz / 64 = 500 Hz; 2500 counts = ~5 s. The supervisor feeds it every
 * 500 ms only while every critical task is alive. */
#define IWDG_PRESCALER_64   4U
#define IWDG_RELOAD_5S      2500U

static void iwdg_start(void) {
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP; /* don't reset while halted in a debugger */
    IWDG->KR = 0xCCCC;
    IWDG->KR = 0x5555;
    IWDG->PR = IWDG_PRESCALER_64;
    IWDG->RLR = IWDG_RELOAD_5S;
    uint32_t spin = 0;
    while (IWDG->SR != 0U && ++spin < 1000000U) { }
    IWDG->KR = 0xAAAA;
}

void health_kick(void) {
    IWDG->KR = 0xAAAA;
}

/* ─────────────────────────── Early init ────────────────────────────────── */

void health_early_init(void) {
    const uint32_t rsr = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;

    const bool por = (rsr & RCC_RSR_BORRSTF) != 0U;
    if (por || !persist_valid()) {
        memset(&g_p, 0, sizeof(g_p));
        g_p.magic = PERSIST_MAGIC;
    }

    g_p.boots++;
    if (por) {
        s_cause = "power-on/brown-out";
    } else if (rsr & RCC_RSR_IWDGRSTF) {
        s_cause = "IWDG watchdog";
        g_p.iwdg_resets++;
    } else if (rsr & RCC_RSR_WWDGRSTF) {
        s_cause = "window watchdog";
        g_p.other_resets++;
    } else if (rsr & RCC_RSR_SFTRSTF) {
        s_cause = (g_p.crash_unread && g_p.kind == 1) ? "software reset after HardFault"
                : (g_p.crash_unread)                  ? "software reset (panic/assert)"
                                                      : "software reset (requested)";
        if (!g_p.crash_unread) {
            g_p.other_resets++;
        }
    } else if (rsr & RCC_RSR_PINRSTF) {
        s_cause = "reset pin / debugger";
    } else if (rsr & RCC_RSR_LPWRRSTF) {
        s_cause = "low-power reset";
        g_p.other_resets++;
    }
    persist_seal();
    iwdg_start();
}

void health_log_summary(void) {
    LOGI("health", "persistent: boots=%lu iwdg_resets=%lu fault_resets=%lu panic_resets=%lu other=%lu",
         (unsigned long)g_p.boots, (unsigned long)g_p.iwdg_resets, (unsigned long)g_p.fault_resets,
         (unsigned long)g_p.panic_resets, (unsigned long)g_p.other_resets);
}

void health_log_boot_report(void) {
    LOGI("boot", "==================================================");
    LOGI("boot", " STM32H5E4VKT6 Zenoh-CAN bridge   build %s %s", __DATE__, __TIME__);
    LOGI("boot", " reset cause : %s", s_cause);
    LOGI("boot", " SYSCLK %lu Hz, IWDG ~5 s (fed by supervisor)", (unsigned long)SystemCoreClock);
    LOGI("boot", "==================================================");
    health_log_summary();

    if (g_p.crash_unread) {
        if (g_p.kind == 1) {
            LOGE("crash", "PREVIOUS RUN: HardFault after %lu ms", (unsigned long)g_p.uptime_ms);
            LOGE("crash", "  PC=0x%08lX LR=0x%08lX PSR=0x%08lX EXC_RETURN=0x%08lX",
                 (unsigned long)g_p.regs[6], (unsigned long)g_p.regs[5],
                 (unsigned long)g_p.regs[7], (unsigned long)g_p.exc_lr);
            LOGE("crash", "  R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX",
                 (unsigned long)g_p.regs[0], (unsigned long)g_p.regs[1], (unsigned long)g_p.regs[2],
                 (unsigned long)g_p.regs[3], (unsigned long)g_p.regs[4]);
            LOGE("crash", "  CFSR=0x%08lX HFSR=0x%08lX MMFAR=0x%08lX BFAR=0x%08lX",
                 (unsigned long)g_p.cfsr, (unsigned long)g_p.hfsr,
                 (unsigned long)g_p.mmfar, (unsigned long)g_p.bfar);
            LOGE("crash", "  decode: arm-none-eabi-addr2line -e <elf> -f 0x%08lX 0x%08lX",
                 (unsigned long)g_p.regs[6], (unsigned long)g_p.regs[5]);
        } else {
            LOGE("crash", "PREVIOUS RUN: reset requested after %lu ms: %s",
                 (unsigned long)g_p.uptime_ms, g_p.reason);
        }
        g_p.crash_unread = 0;
        persist_seal();
    }
}

/* ───────────────── Panic path (no ring buffer, no RTOS calls) ──────────── */

static void pw_hex(const char *label, uint32_t v) {
    char b[32];
    size_t n = 0;
    static const char hex[] = "0123456789ABCDEF";
    b[n++] = '0';
    b[n++] = 'x';
    for (int i = 28; i >= 0; i -= 4) {
        b[n++] = hex[(v >> i) & 0xFU];
    }
    b[n++] = ' ';
    b[n] = '\0';
    app_log_panic_write(label);
    app_log_panic_write(b);
}

static void panic_flush_and_reset(void) {
    uint32_t spin = 0;
    while (!(USART3->ISR & USART_ISR_TC) && ++spin < 400000U) { }
    __DSB();
    NVIC_SystemReset();
    for (;;) { }
}

void health_reset(const char *reason) {
    __disable_irq();
    g_p.kind = 2;
    g_p.crash_unread = 1;
    g_p.panic_resets++;
    g_p.uptime_ms = HAL_GetTick();
    memset(g_p.reason, 0, sizeof(g_p.reason));
    if (reason != NULL) {
        strncpy(g_p.reason, reason, sizeof(g_p.reason) - 1U);
    }
    persist_seal();
    app_log_panic_write("\r\n[PANIC] resetting: ");
    app_log_panic_write(g_p.reason);
    app_log_panic_write("\r\n");
    panic_flush_and_reset();
    for (;;) { }
}

void app_assert_failed(const char *file, int line) {
    /* keep only the file name, and build "assert file:line" without printf */
    const char *base = file;
    for (const char *p = file; p && *p; p++) {
        if (*p == '/' || *p == '\\') {
            base = p + 1;
        }
    }
    char msg[64];
    size_t n = 0;
    const char *pre = "assert ";
    while (*pre && n < sizeof(msg) - 1U) { msg[n++] = *pre++; }
    while (base && *base && n < sizeof(msg) - 12U) { msg[n++] = *base++; }
    msg[n++] = ':';
    char digits[12];
    size_t d = 0;
    unsigned v = (unsigned)(line < 0 ? 0 : line);
    do { digits[d++] = (char)('0' + v % 10U); v /= 10U; } while (v && d < sizeof(digits));
    while (d && n < sizeof(msg) - 1U) { msg[n++] = digits[--d]; }
    msg[n] = '\0';
    health_reset(msg);
}

void crash_report_from_fault(uint32_t *s, uint32_t exc_lr) {
    __disable_irq();
    g_p.kind = 1;
    g_p.crash_unread = 1;
    g_p.fault_resets++;
    g_p.uptime_ms = HAL_GetTick();
    if (s != NULL) {
        for (int i = 0; i < 8; i++) {
            g_p.regs[i] = s[i];
        }
    }
    g_p.exc_lr = exc_lr;
    g_p.cfsr  = SCB->CFSR;
    g_p.hfsr  = SCB->HFSR;
    g_p.mmfar = SCB->MMFAR;
    g_p.bfar  = SCB->BFAR;
    memset(g_p.reason, 0, sizeof(g_p.reason));
    strncpy(g_p.reason, "HardFault", sizeof(g_p.reason) - 1U);
    persist_seal();

    app_log_panic_write("\r\n[FAULT] HardFault! ");
    pw_hex("PC=", g_p.regs[6]);
    pw_hex("LR=", g_p.regs[5]);
    pw_hex("CFSR=", g_p.cfsr);
    pw_hex("HFSR=", g_p.hfsr);
    pw_hex("BFAR=", g_p.bfar);
    app_log_panic_write("-> reset\r\n");
    panic_flush_and_reset();
    for (;;) { }
}
