#include "dma_io.h"
#include "mx_spi1.h"
#include "mx_usart2.h"
#include "stm32c5xx_ll_tim.h"

static hal_dma_handle_t spi_rx_dma, spi_tx_dma, uart_tx_dma;
static uint8_t log_ring[1024];
static volatile unsigned log_head, log_tail, log_inflight;
static volatile bool log_error;
volatile uint32_t log_dropped_bytes, log_dma_errors;

static bool configure_dma(hal_dma_handle_t *dma, hal_dma_channel_t channel,
                          hal_dma_request_source_t request, bool receive)
{
    hal_dma_direct_xfer_config_t config = {
        .request = request,
        .direction = receive ? HAL_DMA_DIRECTION_PERIPH_TO_MEMORY : HAL_DMA_DIRECTION_MEMORY_TO_PERIPH,
        .src_inc = receive ? HAL_DMA_SRC_ADDR_FIXED : HAL_DMA_SRC_ADDR_INCREMENTED,
        .dest_inc = receive ? HAL_DMA_DEST_ADDR_INCREMENTED : HAL_DMA_DEST_ADDR_FIXED,
        .src_data_width = HAL_DMA_SRC_DATA_WIDTH_BYTE,
        .dest_data_width = HAL_DMA_DEST_DATA_WIDTH_BYTE,
        .priority = receive ? HAL_DMA_PRIORITY_HIGH : HAL_DMA_PRIORITY_LOW_WEIGHT_HIGH,
    };
    return HAL_DMA_Init(dma, channel) == HAL_OK &&
           HAL_DMA_SetConfigPeriphDirectXfer(dma, &config) == HAL_OK;
}

bool dma_io_init(void)
{
    /* PCLK1 and TIM2 kernel clock are 144 MHz, no APB prescaling. */
    HAL_RCC_TIM2_EnableClock();
    LL_TIM_SetPrescaler(TIM2, 143);
    LL_TIM_SetAutoReload(TIM2, UINT32_MAX);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    HAL_RCC_LPDMA1_EnableClock();
    if (!configure_dma(&spi_rx_dma, HAL_LPDMA1_CH0, HAL_LPDMA1_REQUEST_SPI1_RX, true) ||
        !configure_dma(&spi_tx_dma, HAL_LPDMA1_CH1, HAL_LPDMA1_REQUEST_SPI1_TX, false) ||
        !configure_dma(&uart_tx_dma, HAL_LPDMA1_CH2, HAL_LPDMA1_REQUEST_USART2_TX, false) ||
        HAL_SPI_SetRxDMA(mx_spi1_gethandle(), &spi_rx_dma) != HAL_OK ||
        HAL_SPI_SetTxDMA(mx_spi1_gethandle(), &spi_tx_dma) != HAL_OK ||
        HAL_UART_SetTxDMA(mx_usart2_uart_gethandle(), &uart_tx_dma) != HAL_OK) return false;
    const IRQn_Type irqs[] = {LPDMA1_CH0_IRQn, LPDMA1_CH1_IRQn, SPI1_IRQn,
                             LPDMA1_CH2_IRQn, USART2_IRQn};
    for (unsigned i = 0; i < sizeof(irqs) / sizeof(irqs[0]); ++i) {
        NVIC_SetPriority(irqs[i], i < 3 ? 2 : 3);
        NVIC_ClearPendingIRQ(irqs[i]);
        NVIC_EnableIRQ(irqs[i]);
    }
    return true;
}

uint32_t app_time_us(void) { return LL_TIM_GetCounter(TIM2); }
void LPDMA1_CH0_IRQHandler(void) { HAL_DMA_IRQHandler(&spi_rx_dma); }
void LPDMA1_CH1_IRQHandler(void) { HAL_DMA_IRQHandler(&spi_tx_dma); }
void LPDMA1_CH2_IRQHandler(void) { HAL_DMA_IRQHandler(&uart_tx_dma); }
void SPI1_IRQHandler(void) { HAL_SPI_IRQHandler(mx_spi1_gethandle()); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(mx_usart2_uart_gethandle()); }

/* Called under an IRQ lock. DMA owns [tail, tail+inflight) until UART TC. */
static void log_kick(void)
{
    if (log_inflight || log_error || log_head == log_tail) return;
    unsigned count = log_head > log_tail ? log_head - log_tail : sizeof(log_ring) - log_tail;
    log_inflight = count;
    if (HAL_UART_Transmit_DMA(mx_usart2_uart_gethandle(), log_ring + log_tail, count) != HAL_OK) {
        log_error = true;
        log_dma_errors++;
    }
}

int __io_putchar(int ch)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    unsigned next = (log_head + 1) % sizeof(log_ring);
    if (next == log_tail) log_dropped_bytes++;
    else { log_ring[log_head] = (uint8_t)ch; log_head = next; }
    log_kick();
    __set_PRIMASK(saved);
    return ch;
}

void HAL_UART_TxCpltCallback(hal_uart_handle_t *uart)
{
    if (uart != mx_usart2_uart_gethandle()) return;
    log_tail = (log_tail + log_inflight) % sizeof(log_ring);
    log_inflight = 0;
    log_kick();
}

void HAL_UART_ErrorCallback(hal_uart_handle_t *uart)
{
    if (uart == mx_usart2_uart_gethandle()) { log_error = true; log_dma_errors++; }
}

void log_service(void)
{
    if (!log_error) return;
    /* Abort is deliberately outside the ISR/IRQ lock (HAL uses timeouts). */
    (void)HAL_UART_AbortTransmit(mx_usart2_uart_gethandle());
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    log_dropped_bytes += log_inflight;
    log_tail = (log_tail + log_inflight) % sizeof(log_ring);
    log_inflight = 0;
    log_error = false;
    log_kick();
    __set_PRIMASK(saved);
}
