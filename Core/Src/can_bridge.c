/**
 * @file can_bridge.c
 * @brief High-performance, minimal-ISR CAN1 HAL driver with DTCM-RAM & HW Filtering
 */

#include "can_bridge.h"
#include <string.h>

static CAN_HandleTypeDef hcan1;

/* ────────────── DTCM-RAM Allocations (Zero-Wait State Memory) ────────── */

/* Place RX Queue control block & storage in DTCM-RAM (0x20000000) for zero cache-miss jitter */
static uint8_t ucRxQueueStorage[CAN_BRIDGE_RX_QUEUE_SIZE * sizeof(can_frame_t)] __attribute__((section(".dtcmram")));
static StaticQueue_t xRxQueueBuffer __attribute__((section(".dtcmram")));
static QueueHandle_t g_rx_queue = NULL;

static volatile uint32_t stat_rx_frames = 0;
static volatile uint32_t stat_tx_frames = 0;
static volatile uint32_t stat_drop_count = 0;

/* ──────────────────── Hardware Filter Configuration ────────────────── */

static void configure_hardware_filters(const uint32_t *filter_ids, size_t filter_id_count) {
    if (filter_ids == NULL || filter_id_count == 0) {
        /* Fallback: Accept all standard IDs */
        CAN_FilterTypeDef filter = {0};
        filter.FilterBank           = 0;
        filter.FilterMode           = CAN_FILTERMODE_IDMASK;
        filter.FilterScale          = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh         = 0x0000;
        filter.FilterIdLow          = 0x0000;
        filter.FilterMaskIdHigh     = 0x0000;
        filter.FilterMaskIdLow      = 0x0000;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation     = ENABLE;
        filter.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &filter);
        return;
    }

    /*
     * bxCAN 16-bit List Mode:
     * Each filter bank accommodates 4 exact standard 11-bit IDs.
     * Standard ID format in 16-bit register: [15:5] = STID[10:0], [4..0] = 0 (RTR=0, IDE=0)
     */
    size_t bank_idx = 0;
    size_t id_idx = 0;

    while (id_idx < filter_id_count && bank_idx < 14) {
        uint16_t slots[4];
        for (int s = 0; s < 4; s++) {
            if (id_idx < filter_id_count) {
                slots[s] = (uint16_t)((filter_ids[id_idx] & 0x7FF) << 5);
                id_idx++;
            } else {
                /* Fill remaining slots with the last valid ID so no unwanted ID is matched */
                slots[s] = slots[s > 0 ? s - 1 : 0];
            }
        }

        CAN_FilterTypeDef filter = {0};
        filter.FilterBank           = bank_idx;
        filter.FilterMode           = CAN_FILTERMODE_IDLIST;
        filter.FilterScale          = CAN_FILTERSCALE_16BIT;
        filter.FilterIdHigh         = slots[0];
        filter.FilterIdLow          = slots[1];
        filter.FilterMaskIdHigh     = slots[2];
        filter.FilterMaskIdLow      = slots[3];
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation     = ENABLE;
        filter.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(&hcan1, &filter);
        bank_idx++;
    }

    /* Disable any remaining unused filter banks */
    for (; bank_idx < 14; bank_idx++) {
        CAN_FilterTypeDef filter = {0};
        filter.FilterBank           = bank_idx;
        filter.FilterActivation     = DISABLE;
        filter.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &filter);
    }
}

/* ──────────────────────── Initialization ───────────────────────────── */

void can_bridge_init(const uint32_t *filter_ids, size_t filter_id_count) {
    if (g_rx_queue == NULL) {
        g_rx_queue = xQueueCreateStatic(CAN_BRIDGE_RX_QUEUE_SIZE,
                                        sizeof(can_frame_t),
                                        ucRxQueueStorage,
                                        &xRxQueueBuffer);
        configASSERT(g_rx_queue != NULL);
    }

    /*
     * Bit Timing: derived automatically from CONFIG_CAN_BITRATE in app_config.h
     * APB1 clock = 48 MHz, Sample point = 14 / 16 = 87.5%
     */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = CONFIG_CAN_PRESCALER;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;   /* Automatically recover from bus-off error */
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;   /* Ensure delivery on arbitration collision */
    hcan1.Init.ReceiveFifoLocked    = DISABLE;  /* Discard oldest message if overrun */
    hcan1.Init.TransmitFifoPriority = ENABLE;   /* Transmit in chronological FIFO order */

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        configASSERT(0);
    }

    /* Configure hardware filters (exact ID list or all-pass) */
    configure_hardware_filters(filter_ids, filter_id_count);

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        configASSERT(0);
    }

    /* Enable RX FIFO0 message pending notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        configASSERT(0);
    }

    /* Priority 6 (safe for FreeRTOS MAX_SYSCALL=5) */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

QueueHandle_t can_bridge_get_rx_queue(void) {
    return g_rx_queue;
}

BaseType_t can_bridge_send_frame(const can_frame_t *frame, TickType_t timeout) {
    if (!frame) return pdFALSE;

    CAN_TxHeaderTypeDef tx_hdr;
    tx_hdr.StdId              = frame->id & 0x7FF;
    tx_hdr.ExtId              = 0;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = frame->dlc > 8 ? 8 : frame->dlc;
    tx_hdr.TransmitGlobalTime = DISABLE;

    TickType_t start_tick = xTaskGetTickCount();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((xTaskGetTickCount() - start_tick) >= timeout) {
            stat_drop_count++;
            return pdFALSE;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, (uint8_t *)frame->data, &mailbox) != HAL_OK) {
        stat_drop_count++;
        return pdFALSE;
    }

    stat_tx_frames++;
    return pdTRUE;
}

void can_bridge_get_stats(uint32_t *rx_frames, uint32_t *tx_frames, uint32_t *drop_count) {
    if (rx_frames)  *rx_frames  = stat_rx_frames;
    if (tx_frames)  *tx_frames  = stat_tx_frames;
    if (drop_count) *drop_count = stat_drop_count;
}

void can_bridge_get_bus_health(can_bus_health_t *health) {
    if (!health) return;

    uint32_t esr = CAN1->ESR;
    health->tec        = (uint8_t)((esr >> 16) & 0xFF);
    health->rec        = (uint8_t)((esr >> 24) & 0xFF);
    health->lec        = (uint8_t)((esr >> 4)  & 0x07);
    health->is_bus_off = (esr & (1U << 2)) != 0;
    health->is_passive = (esr & (1U << 1)) != 0;
    health->is_warning = (esr & (1U << 0)) != 0;
}

/* ──────────────────── Hardware ISR Context (< 2µs) ──────────────────── */

void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    can_frame_t frame;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, frame.data) == HAL_OK) {
        if (rx_header.IDE == CAN_ID_STD && g_rx_queue != NULL) {
            frame.id  = rx_header.StdId;
            frame.dlc = (uint8_t)rx_header.DLC;

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(g_rx_queue, &frame, &xHigherPriorityTaskWoken) == pdTRUE) {
                stat_rx_frames++;
            } else {
                stat_drop_count++; /* Queue full */
            }
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
