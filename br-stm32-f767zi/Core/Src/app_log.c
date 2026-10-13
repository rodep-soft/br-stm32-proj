/**
 * @file app_log.c
 * @brief Interrupt-driven, non-blocking logger (ring buffer -> USART3 TXE ISR).
 *
 * Why: a blocking 115200-baud printf costs ~8 ms of CPU per 100-char line.
 * Here a log call only formats into a stack buffer and memcpy()s into a ring
 * buffer (microseconds); the USART3 TXE interrupt drains it in the background.
 * If the ring is full the whole line is dropped and counted - logging can
 * never stall CAN handling, Ethernet or Zenoh.
 */
#include "app_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define LOG_RING_SIZE   6144U   /* power of two not required, kept simple */
#define LOG_LINE_MAX    200U

static uint8_t           s_ring[LOG_RING_SIZE];
static volatile uint32_t s_head;      /* written by producers (critical section) */
static volatile uint32_t s_tail;      /* written by the TXE ISR only             */
static volatile uint32_t s_dropped;
static volatile uint32_t s_reported_dropped;
static volatile app_log_level_t s_level = APP_LOG_INFO;

static inline uint32_t ring_used(void) {
    uint32_t h = s_head, t = s_tail;
    return (h >= t) ? (h - t) : (LOG_RING_SIZE - t + h);
}

/* One slot is kept empty to distinguish full from empty. */
static inline uint32_t ring_free(void) {
    return LOG_RING_SIZE - 1U - ring_used();
}

static void tx_kick(void) {
    USART3->CR1 |= USART_CR1_TXEIE;
}

/* USART3 TXE interrupt: move one byte from the ring to the shifter. */
void USART3_IRQHandler(void) {
    if ((USART3->CR1 & USART_CR1_TXEIE) && (USART3->ISR & USART_ISR_TXE)) {
        uint32_t t = s_tail;
        if (t != s_head) {
            USART3->TDR = s_ring[t];
            s_tail = (t + 1U) % LOG_RING_SIZE;
        } else {
            USART3->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    /* Clear overrun so a stray RX byte can never wedge the peripheral. */
    if (USART3->ISR & USART_ISR_ORE) {
        USART3->ICR = USART_ICR_ORECF;
    }
}

void app_log_init(void) {
    s_head = s_tail = 0;
    HAL_NVIC_SetPriority(USART3_IRQn, 8, 0); /* below configMAX_SYSCALL: no RTOS calls in ISR */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void app_log_set_level(app_log_level_t level) { s_level = level; }
app_log_level_t app_log_get_level(void)       { return s_level; }
uint32_t app_log_dropped(void)                { return s_dropped; }

static bool scheduler_running(void) {
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/* Push a block atomically. Before the scheduler starts we wait for the ISR to
 * make room (boot messages must not be lost); afterwards we never wait. */
static void ring_push(const uint8_t *p, size_t len) {
    if (len == 0 || len >= LOG_RING_SIZE) {
        return;
    }
    if (!scheduler_running()) {
        uint32_t spin = 0;
        while (ring_free() < len && ++spin < 20000000U) {
            tx_kick();
        }
    }
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    if (ring_free() >= len) {
        uint32_t h = s_head;
        for (size_t i = 0; i < len; i++) {
            s_ring[h] = p[i];
            h = (h + 1U) % LOG_RING_SIZE;
        }
        s_head = h;
        taskEXIT_CRITICAL_FROM_ISR(saved);
        tx_kick();
    } else {
        s_dropped++;
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

void app_log(app_log_level_t level, const char *tag, const char *fmt, ...) {
    if (level > s_level) {
        return;
    }
    static const char lvl_chr[] = {'E', 'W', 'I', 'D'};
    char line[LOG_LINE_MAX];

    /* Report previously dropped lines once, as its own line. */
    uint32_t dropped = s_dropped;
    if (dropped != s_reported_dropped) {
        s_reported_dropped = dropped;
        char note[64];
        int nn = snprintf(note, sizeof(note), "[LOG] %lu line(s) dropped (ring full)\r\n",
                          (unsigned long)dropped);
        if (nn > 0) {
            ring_push((const uint8_t *)note, (size_t)nn);
        }
    }

    uint32_t ms = HAL_GetTick();
    int n = snprintf(line, sizeof(line), "[%5lu.%03lu][%c][%-6.6s] ",
                     (unsigned long)(ms / 1000U), (unsigned long)(ms % 1000U),
                     lvl_chr[level <= APP_LOG_DEBUG ? level : APP_LOG_DEBUG],
                     tag ? tag : "?");
    if (n < 0 || (size_t)n >= sizeof(line) - 3U) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(line + n, sizeof(line) - (size_t)n - 3U, fmt, ap);
    va_end(ap);
    if (m < 0) {
        return;
    }
    size_t len = (size_t)n + (size_t)m;
    if (len > sizeof(line) - 3U) { /* truncated: mark it */
        len = sizeof(line) - 3U;
        line[len - 1U] = '~';
    }
    line[len++] = '\r';
    line[len++] = '\n';
    ring_push((const uint8_t *)line, len);
}

/* newlib printf() / puts() end up here (see main.c). Single chars. */
void app_log_putc(int ch) {
    uint8_t c = (uint8_t)ch;
    ring_push(&c, 1);
}

/* Fault / assert path: bypass ring and interrupts, bounded busy-wait. */
void app_log_panic_write(const char *s) {
    USART3->CR1 &= ~USART_CR1_TXEIE;
    while (*s) {
        uint32_t spin = 0;
        while (!(USART3->ISR & USART_ISR_TXE) && ++spin < 400000U) { }
        USART3->TDR = (uint8_t)*s++;
    }
}
