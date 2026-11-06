#include <assert.h>
#include <stdio.h>
#include "../App/bno_transport.c"
uint32_t mock_irq_mask;
static hal_spi_handle_t spi;
static uint32_t tick, usec, dma_count, abort_count;
static const uint8_t *dma_tx;
static uint8_t *dma_rx;
static bool dma_active;
static int cs = 1, wake = 1, int_pin, dma_status;
hal_spi_handle_t *mx_spi1_gethandle(void) { return &spi; }
int HAL_SPI_MASTER_EnableKeepIOState(hal_spi_handle_t *s) { (void)s; return HAL_OK; }
int HAL_SPI_Abort(hal_spi_handle_t *s) { (void)s; dma_active = false; abort_count++; return HAL_OK; }
int HAL_SPI_TransmitReceive_DMA(hal_spi_handle_t *s, const void *tx, void *rx, uint32_t count)
{
    (void)s; assert(!dma_active && !cs);
    if (dma_status) return dma_status;
    dma_active = true; dma_tx = tx; dma_rx = rx; dma_count = count; return HAL_OK;
}
uint32_t HAL_GetTick(void) { return tick; }
uint32_t app_time_us(void) { return usec; }
void HAL_Delay(uint32_t ms) { tick += ms; usec += ms * 1000; }
void log_service(void) {}
int HAL_GPIO_ReadPin(int port, int pin) { (void)port; (void)pin; return int_pin; }
void HAL_GPIO_WritePin(int port, int pin, int value)
{ if (port == PA4_PORT && pin == PA4_PIN) cs = value; if (port == PB0_PORT && pin == PB0_PIN) wake = value; }
static void complete(const uint8_t *data)
{
    assert(dma_active); memcpy(dma_rx, data, dma_count); dma_active = false;
    HAL_SPI_TxRxCpltCallback(&spi);
}
static void reopen(void) { transport_close(NULL); int_pin = 0; assert(transport_open(NULL) == 0); }
int main(void)
{
    uint8_t output[1024], header[4], body[1020];
    uint32_t stamp;
    memset(body, 0x5a, sizeof(body)); memset(output, 0xcc, sizeof(output));
    reopen(); EXTI0_IRQHandler(); usec += 25;
    assert(transport_read(NULL, output, sizeof(output), &stamp) == 0 && dma_count == 4);
    assert(output[0] == 0xcc); /* Caller buffer is not owned by DMA. */
    memcpy(header, (uint8_t[]){0, 4, 3, 7}, 4); complete(header);
    assert(!cs && dma_count == 1020); complete(body);
    assert(cs && transport_read(NULL, output, sizeof(output), &stamp) == 1024);
    assert(stamp == usec - 25 && output[1023] == 0x5a && bno_max_int_latency_us == 25);

    uint8_t command[21] = {21, 0, 2, 1, 0xfd, 1};
    assert(transport_write(NULL, command, sizeof(command)) == 0 && dma_count == 4);
    memcpy(header, (uint8_t[]){40, 0, 3, 8}, 4); complete(header);
    assert(dma_count == 36 && !cs);
    for (unsigned i = 17; i < dma_count; ++i) assert(dma_tx[i] == 0); /* Pad beyond TX length. */
    complete(body);
    assert(transport_write(NULL, command, sizeof(command)) == 0); /* Pending duplex RX. */
    assert(transport_read(NULL, output, sizeof(output), &stamp) == 40);
    assert(transport_write(NULL, command, sizeof(command)) == 21 && state == IDLE);

    /* Null read header still needs the rest of the outgoing command clocked. */
    int_pin = 1;
    assert(transport_write(NULL, command, sizeof(command)) == 0 && !wake && !dma_active);
    int_pin = 0;
    assert(transport_write(NULL, command, sizeof(command)) == 0);
    memset(header, 0, sizeof(header)); complete(header);
    assert(dma_count == 17); complete(body);
    assert(transport_write(NULL, command, sizeof(command)) == 21 && wake);

    assert(transport_read(NULL, output, sizeof(output), &stamp) == 0);
    memcpy(header, (uint8_t[]){0xff, 0xff, 0xff, 0xff}, 4); complete(header);
    assert(state == FAULT && cs && bno_transport_take_error());
    assert(!dma_active && transport_read(NULL, output, sizeof(output), &stamp) == -1);
    reopen();
    assert(transport_read(NULL, output, sizeof(output), &stamp) == 0);
    memcpy(header, (uint8_t[]){19, 0, 3, 0}, 4); complete(header);
    unsigned aborted = abort_count; tick += 20;
    assert(transport_read(NULL, output, sizeof(output), &stamp) == -1);
    assert(abort_count == aborted + 1 && !dma_active && cs);
    assert(mock_irq_mask == 0);
    HAL_SPI_TxRxCpltCallback(&spi); /* A late completion cannot revive FAULT. */
    assert(state == FAULT && !rx_available);
    reopen();
    assert(transport_read(NULL, output, sizeof(output), &stamp) == 0);
    memcpy(header, (uint8_t[]){19, 0, 3, 0}, 4); complete(header);
    complete(body); /* Completed just before the timeout check. */
    tick += 20;
    aborted = abort_count;
    assert(transport_read(NULL, output, sizeof(output), &stamp) == 19);
    assert(abort_count == aborted && mock_irq_mask == 0);
    reopen();
    dma_status = HAL_ERROR;
    assert(transport_read(NULL, output, sizeof(output), &stamp) == -1 && cs);
    puts("PASS: async variable-length DMA, duplex RX preservation, wake, errors and timeout cleanup");
}
