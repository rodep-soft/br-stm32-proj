#include "bno_transport.h"
#include "dma_io.h"
#include "mx_gpio_default.h"
#include "mx_spi1.h"
#include "stm32c5xx_ll_exti.h"
#include <string.h>

#define CS(state) HAL_GPIO_WritePin(PA4_PORT, PA4_PIN, (state))
#define RESET(state) HAL_GPIO_WritePin(PA1_PORT, PA1_PIN, (state))
#define WAKE(state) HAL_GPIO_WritePin(PB0_PORT, PB0_PIN, (state))

typedef enum { CLOSED, IDLE, RX_HEADER, RX_BODY, RX_READY, TX_WAKE, TX_DONE, FAULT } transfer_state_t;
static volatile transfer_state_t state = CLOSED;
/* DMA never accesses SH-2 caller storage: it may be reused after read returns 0. */
static uint8_t dummy_tx[SH2_HAL_MAX_TRANSFER_IN];
static uint8_t rx_packet[SH2_HAL_MAX_TRANSFER_IN];
static uint8_t tx_packet[SH2_HAL_MAX_TRANSFER_IN];
static bool writing;
static volatile bool rx_available;
static unsigned rx_length, tx_length;
static uint32_t transfer_started_ms, rx_timestamp_us;
static volatile uint32_t int_timestamp_us;
static volatile bool int_pending, transport_error;
volatile unsigned bno_transport_fault, bno_last_header;
volatile uint32_t bno_dma_reads, bno_dma_writes, bno_max_int_latency_us;

static bool int_low(void)
{
    return HAL_GPIO_ReadPin(PA0_PORT, PA0_PIN) == HAL_GPIO_PIN_RESET;
}

void EXTI0_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_0)) {
        LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_0);
        int_timestamp_us = app_time_us();
        int_pending = true;
    }
}

/* Safe in an ISR: recovery/abort is done later in thread context. */
static int fail_transfer(unsigned fault)
{
    bno_transport_fault = fault;
    transport_error = true;
    state = FAULT;
    CS(HAL_GPIO_PIN_SET);
    WAKE(HAL_GPIO_PIN_SET);
    return -1;
}

static void check_timeout(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    transfer_state_t current = state;
    bool expired = (current == RX_HEADER || current == RX_BODY || current == TX_WAKE) &&
        (uint32_t)(HAL_GetTick() - transfer_started_ms) >= (current == TX_WAKE ? 100U : 20U);
    if (expired) {
        /* Atomically claim timeout versus a DMA completion. The HAL abort must
           run with interrupts restored because its waits use the system tick. */
        (void)fail_transfer(current == TX_WAKE ? 6 : 9);
    }
    __set_PRIMASK(saved);
    if (expired) (void)HAL_SPI_Abort(mx_spi1_gethandle());
    log_service();
}

static int transport_open(sh2_Hal_t *self)
{
    (void)self;
    transport_error = false;
    bno_transport_fault = 0;
    state = CLOSED;
    rx_available = false;
    CS(HAL_GPIO_PIN_SET);
    WAKE(HAL_GPIO_PIN_SET);
    if (HAL_SPI_MASTER_EnableKeepIOState(mx_spi1_gethandle()) != HAL_OK) return fail_transfer(8);
    LL_EXTI_SetEXTISource(LL_EXTI_GPIO_PORTA, LL_EXTI_GPIO_LINE0);
    LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
    /* One clean reset pulse, including when reopening after an error. */
    RESET(HAL_GPIO_PIN_RESET);
    HAL_Delay(10);
    int_pending = false;
    RESET(HAL_GPIO_PIN_SET);
    /* Board-tested startup guard, not a datasheet maximum/ready indication. */
    HAL_Delay(100);
    uint32_t start = HAL_GetTick();
    while (!int_low()) {
        if ((uint32_t)(HAL_GetTick() - start) >= 1000U) return fail_transfer(1);
        __WFI();
    }
    /* PS1 externally HIGH and PS0 HIGH throughout reset and first INT. */
    state = IDLE;
    return 0;
}

static void transport_close(sh2_Hal_t *self)
{
    (void)self;
    state = CLOSED;
    rx_available = false;
    (void)HAL_SPI_Abort(mx_spi1_gethandle());
    CS(HAL_GPIO_PIN_SET);
    WAKE(HAL_GPIO_PIN_SET);
    RESET(HAL_GPIO_PIN_RESET);
    int_pending = false;
}

void HAL_SPI_TxRxCpltCallback(hal_spi_handle_t *spi)
{
    if (spi != mx_spi1_gethandle()) return;
    if (state == RX_HEADER) {
        bno_last_header = rx_packet[0] | ((unsigned)rx_packet[1] << 8) |
                          ((unsigned)rx_packet[2] << 16) | ((unsigned)rx_packet[3] << 24);
        rx_length = (rx_packet[0] | ((unsigned)rx_packet[1] << 8)) & 0x7fffU;
        if ((rx_length && rx_length < 4) || rx_length > sizeof(rx_packet)) {
            (void)fail_transfer(4); return;
        }
        /* SHTP SPI is full duplex. Clock out the longer of both packets and
           preserve a simultaneous hub response even during host writes. */
        unsigned total = writing && tx_length > rx_length ? tx_length : rx_length;
        if (total > 4) {
            state = RX_BODY;
            if (HAL_SPI_TransmitReceive_DMA(spi, writing ? tx_packet + 4 : dummy_tx,
                                           rx_packet + 4, total - 4) != HAL_OK)
                (void)fail_transfer(5);
            return;
        }
    } else if (state != RX_BODY) return;
    CS(HAL_GPIO_PIN_SET);
    WAKE(HAL_GPIO_PIN_SET);
    if (rx_length) bno_dma_reads++;
    if (writing) bno_dma_writes++;
    __DMB();
    rx_available = rx_length != 0;
    state = writing ? TX_DONE : (rx_available ? RX_READY : IDLE);
}

void HAL_SPI_ErrorCallback(hal_spi_handle_t *spi)
{
    if (spi == mx_spi1_gethandle()) (void)fail_transfer(10);
}

static int transport_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity, uint32_t *timestamp_us)
{
    (void)self;
    check_timeout();
    if (state == FAULT) return -1;
    if ((state == RX_READY || state == TX_DONE) && rx_available) {
        __DMB();
        if (capacity < rx_length) return fail_transfer(2);
        memcpy(buffer, rx_packet, rx_length);
        if (timestamp_us) *timestamp_us = rx_timestamp_us;
        unsigned count = rx_length;
        rx_available = false;
        if (state == RX_READY) state = IDLE;
        return (int)count;
    }
    if (state != IDLE || !int_low()) return 0;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint32_t now_us = app_time_us();
    rx_timestamp_us = int_pending ? int_timestamp_us : now_us;
    uint32_t latency = now_us - rx_timestamp_us;
    if (latency > bno_max_int_latency_us) bno_max_int_latency_us = latency;
    int_pending = false;
    transfer_started_ms = HAL_GetTick();
    writing = false;
    state = RX_HEADER;
    CS(HAL_GPIO_PIN_RESET);
    __set_PRIMASK(saved);
    if (HAL_SPI_TransmitReceive_DMA(mx_spi1_gethandle(), dummy_tx, rx_packet, 4) != HAL_OK)
        return fail_transfer(3);
    return 0;
}

static int transport_write(sh2_Hal_t *self, uint8_t *buffer, unsigned length)
{
    (void)self;
    check_timeout();
    if (state == FAULT) return -1;
    if (!buffer || !length || length > SH2_HAL_MAX_TRANSFER_OUT) return -1;
    if (state == TX_DONE) {
        if (rx_available) return 0; /* Let SH-2 consume the duplex response first. */
        state = IDLE; return (int)tx_length;
    }
    if (state == IDLE) {
        memset(tx_packet, 0, sizeof(tx_packet));
        memcpy(tx_packet, buffer, length);
        tx_length = length;
        transfer_started_ms = HAL_GetTick();
        state = TX_WAKE;
        WAKE(HAL_GPIO_PIN_RESET);
    }
    if (state != TX_WAKE || !int_low()) return 0;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    rx_timestamp_us = int_pending ? int_timestamp_us : app_time_us();
    int_pending = false;
    transfer_started_ms = HAL_GetTick();
    writing = true;
    state = RX_HEADER;
    CS(HAL_GPIO_PIN_RESET);
    __set_PRIMASK(saved);
    if (HAL_SPI_TransmitReceive_DMA(mx_spi1_gethandle(), tx_packet, rx_packet, 4) != HAL_OK)
        return fail_transfer(7);
    return 0; /* SH-2 retries while servicing reads until TX_DONE. */
}

static uint32_t transport_time(sh2_Hal_t *self) { (void)self; return app_time_us(); }
sh2_Hal_t *bno_transport_get(void)
{
    static sh2_Hal_t hal = {.open = transport_open, .close = transport_close, .read = transport_read,
                           .write = transport_write, .getTimeUs = transport_time};
    return &hal;
}
bool bno_transport_take_error(void)
{
    bool error = transport_error;
    transport_error = false;
    return error;
}
/* Call with IRQs masked before WFI, so an INT/completion cannot be lost. */
bool bno_transport_has_work(void)
{
    return int_low() || state == RX_READY || state == TX_DONE || state == FAULT;
}
