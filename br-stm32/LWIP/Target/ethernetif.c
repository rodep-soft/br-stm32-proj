#include "main.h"
#include "ethernetif.h"
#include "lan8742.h"
#include "lwip/etharp.h"
#include "lwip/ethip6.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "lwip/tcpip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stddef.h>
#include <string.h>

#define IFNAME0 's'
#define IFNAME1 't'
#define ETH_RX_BUFFER_SIZE 1536U
#define ETH_INTERFACE_STACK 350U

extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfigTypeDef TxConfig;
extern ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];
extern ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];

typedef struct {
  struct pbuf_custom custom;
  uint8_t buffer[ETH_RX_BUFFER_SIZE] __ALIGNED(32);
} RxBuffer;

static SemaphoreHandle_t rx_semaphore;
static SemaphoreHandle_t tx_semaphore;
static StaticSemaphore_t rx_semaphore_storage;
static StaticSemaphore_t tx_semaphore_storage;
static TaskHandle_t input_task;
static RxBuffer rx_buffers[ETH_RX_DESC_CNT * 2U] __ALIGNED(32);
static uint32_t rx_buffer_index;
static struct netif *ethernet_netif;
static lan8742_Object_t lan8742;

static int32_t phy_init(void);
static int32_t phy_deinit(void);
static int32_t phy_read(uint32_t, uint32_t, uint32_t *);
static int32_t phy_write(uint32_t, uint32_t, uint32_t);
static int32_t phy_tick(void);
static void ethernetif_input(void *argument);
void pbuf_free_custom(struct pbuf *p);
static void rx_allocate(uint8_t **buffer);
static void rx_link(void **start, void **end, uint8_t *buffer, uint16_t length);
static void tx_free(uint32_t *buffer);

static lan8742_IOCtx_t lan8742_io = {
  phy_init, phy_deinit, phy_write, phy_read, phy_tick
};

static void rx_allocate(uint8_t **buffer)
{
  RxBuffer *rx = &rx_buffers[rx_buffer_index++ % (ETH_RX_DESC_CNT * 2U)];
  rx->custom.custom_free_function = pbuf_free_custom;
  *buffer = rx->buffer;
}

static void rx_link(void **start, void **end, uint8_t *buffer, uint16_t length)
{
  struct pbuf *p = (struct pbuf *)(buffer - offsetof(RxBuffer, buffer));
  p->next = NULL;
  p->len = length;
  p->tot_len = length;
  if (*start == NULL) {
    *start = p;
  } else {
    ((struct pbuf *)*end)->next = p;
  }
  *end = p;
}

static void tx_free(uint32_t *buffer)
{
  pbuf_free((struct pbuf *)buffer);
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *handler)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  (void)handler;
  xSemaphoreGiveFromISR(rx_semaphore, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *handler)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  (void)handler;
  xSemaphoreGiveFromISR(tx_semaphore, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *handler)
{
  (void)handler;
  if (rx_semaphore != NULL) {
    xSemaphoreGiveFromISR(rx_semaphore, NULL);
  }
}

static void low_level_init(struct netif *netif)
{
  static uint8_t mac_address[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};
  ethernet_netif = netif;
  netif->hwaddr_len = ETH_HWADDR_LEN;
  memcpy(netif->hwaddr, mac_address, sizeof(mac_address));
  netif->mtu = ETH_MAX_PAYLOAD;
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  rx_semaphore = xSemaphoreCreateBinaryStatic(&rx_semaphore_storage);
  tx_semaphore = xSemaphoreCreateBinaryStatic(&tx_semaphore_storage);

  if (HAL_ETH_RegisterRxAllocateCallback(&heth, rx_allocate) != HAL_OK ||
      HAL_ETH_RegisterRxLinkCallback(&heth, rx_link) != HAL_OK ||
      HAL_ETH_RegisterTxFreeCallback(&heth, tx_free) != HAL_OK ||
      LAN8742_RegisterBusIO(&lan8742, &lan8742_io) != LAN8742_STATUS_OK ||
      LAN8742_Init(&lan8742) != LAN8742_STATUS_OK ||
      HAL_ETH_Start_IT(&heth) != HAL_OK) {
    netif_set_down(netif);
    return;
  }

  xTaskCreate(ethernetif_input, "EthIf", ETH_INTERFACE_STACK, netif,
              configMAX_PRIORITIES - 2U, &input_task);
  netif_set_up(netif);
  netif_set_link_up(netif);
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  ETH_BufferTypeDef buffers[ETH_TX_DESC_CNT] = {0};
  struct pbuf *q;
  uint32_t index = 0;
  (void)netif;

  for (q = p; q != NULL; q = q->next) {
    if (index >= ETH_TX_DESC_CNT) {
      return ERR_IF;
    }
    buffers[index].buffer = q->payload;
    buffers[index].len = q->len;
    if (index > 0) {
      buffers[index - 1U].next = &buffers[index];
    }
    ++index;
  }
  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = buffers;
  TxConfig.pData = p;
  pbuf_ref(p);
  if (HAL_ETH_Transmit_IT(&heth, &TxConfig) != HAL_OK) {
    pbuf_free(p);
    return ERR_BUF;
  }
  return ERR_OK;
}

static void ethernetif_input(void *argument)
{
  struct netif *netif = argument;
  struct pbuf *packet;
  for (;;) {
    if (xSemaphoreTake(rx_semaphore, portMAX_DELAY) == pdTRUE) {
      do {
        packet = NULL;
        if (HAL_ETH_ReadData(&heth, (void **)&packet) == HAL_OK && packet != NULL) {
          if (netif->input(packet, netif) != ERR_OK) {
            pbuf_free(packet);
          }
        }
      } while (packet != NULL);
    }
  }
}

err_t ethernetif_init(struct netif *netif)
{
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif
  netif->linkoutput = low_level_output;
  low_level_init(netif);
  return ERR_OK;
}

void pbuf_free_custom(struct pbuf *p)
{
  (void)p;
}

u32_t sys_now(void)
{
  return HAL_GetTick();
}

static int32_t phy_init(void)
{
  HAL_ETH_SetMDIOClockRange(&heth);
  return 0;
}

static int32_t phy_deinit(void) { return 0; }

static int32_t phy_read(uint32_t dev_addr, uint32_t reg_addr, uint32_t *value)
{
  return HAL_ETH_ReadPHYRegister(&heth, dev_addr, reg_addr, value) == HAL_OK ? 0 : -1;
}

static int32_t phy_write(uint32_t dev_addr, uint32_t reg_addr, uint32_t value)
{
  return HAL_ETH_WritePHYRegister(&heth, dev_addr, reg_addr, value) == HAL_OK ? 0 : -1;
}

static int32_t phy_tick(void) { return (int32_t)HAL_GetTick(); }
