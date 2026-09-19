/**
 * @file can_bridge.c
 * @brief High-performance CAN Bridge Task implementation
 */

#include "can_bridge.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"

CAN_HandleTypeDef hcan1;
static QueueHandle_t g_can_queue = NULL;
static can_bridge_stats_t g_stats = {0};

CAN_HandleTypeDef *can_bridge_get_handle(void) {
    return &hcan1;
}

void can_bridge_get_stats(can_bridge_stats_t *out_stats) {
    if (out_stats) {
        *out_stats = g_stats;
    }
}

bool can_bridge_init(uint32_t baudrate) {
    /* 1. Calculate Prescaler for 48MHz APB1 CAN clock (16 Tq total: 1 Sync + 12 BS1 + 3 BS2) */
    uint32_t prescaler = 48000000U / (16U * baudrate);
    if (prescaler == 0 || prescaler > 1024) {
        printf("[CAN] Invalid baudrate %lu bps\r\n", (unsigned long)baudrate);
        return false;
    }

    /* 2. Enable CAN1 and GPIOB peripheral clocks */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 3. Configure CAN1 GPIO pins (PB8: RX, PB9: TX) */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* 4. Configure CAN1 peripheral */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = prescaler;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        printf("[CAN] Failed to initialize CAN1!\r\n");
        return false;
    }

    /* 4. Configure default filter (accept all IDs into FIFO 0) */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        printf("[CAN] Failed to configure CAN1 filter!\r\n");
        return false;
    }

    /* 5. Start CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        printf("[CAN] Failed to start CAN1!\r\n");
        return false;
    }

    printf("[CAN] CAN1 initialized at %lu bps (prescaler: %lu, sample point: 81.25%%)\r\n",
           (unsigned long)baudrate, (unsigned long)prescaler);
    return true;
}

bool can_bridge_send_frame(const can_msgs_Frame *frame) {
    if (frame == NULL) return false;

    /* Wait for free TX mailbox with timeout */
    uint32_t wait_start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((HAL_GetTick() - wait_start) > CONFIG_CAN_TX_TIMEOUT_MS) {
            return false; /* Timeout */
        }
        osDelay(1);
    }

    CAN_TxHeaderTypeDef tx_header;
    if (frame->is_extended) {
        tx_header.IDE = CAN_ID_EXT;
        tx_header.ExtId = frame->id & 0x1FFFFFFFU;
        tx_header.StdId = 0;
    } else {
        tx_header.IDE = CAN_ID_STD;
        tx_header.StdId = frame->id & 0x7FFU;
        tx_header.ExtId = 0;
    }

    tx_header.RTR = frame->is_rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC = (frame->dlc > 8) ? 8 : frame->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)frame->data, &mailbox);
    return (status == HAL_OK);
}

bool can_bridge_post_frame(const can_msgs_Frame *frame) {
    if (g_can_queue == NULL || frame == NULL) {
        return false;
    }

    if (xQueueSend(g_can_queue, frame, 0) == pdTRUE) {
        g_stats.rx_from_zenoh++;
        return true;
    } else {
        g_stats.dropped_queue_full++;
        return false;
    }
}

static void can_bridge_task(void const *argument) {
    (void)argument;

    printf("[CAN Bridge] Task running (stack: 512 words, queue capacity: %d)\r\n", CAN_BRIDGE_QUEUE_SIZE);

    can_msgs_Frame frame;
    uint32_t report_tick = HAL_GetTick();

    while (1) {
        if (xQueueReceive(g_can_queue, &frame, portMAX_DELAY) == pdTRUE) {
            if (can_bridge_send_frame(&frame)) {
                g_stats.tx_to_can_success++;
            } else {
                g_stats.tx_to_can_fail++;
                /* Check for CAN hardware error and recover if needed */
                uint32_t can_err = HAL_CAN_GetError(&hcan1);
                if (can_err != HAL_CAN_ERROR_NONE) {
                    HAL_CAN_ResetError(&hcan1);
                }
            }
        }

        /* Periodic statistics report if traffic observed */
        if ((HAL_GetTick() - report_tick) > CONFIG_CAN_STATS_PERIOD_MS) {
            report_tick = HAL_GetTick();
            if (g_stats.rx_from_zenoh > 0 || g_stats.tx_to_can_fail > 0) {
                printf("[CAN Bridge] Stats: RX=%lu, TX_OK=%lu, TX_ERR=%lu, DROPPED=%lu\r\n",
                       (unsigned long)g_stats.rx_from_zenoh,
                       (unsigned long)g_stats.tx_to_can_success,
                       (unsigned long)g_stats.tx_to_can_fail,
                       (unsigned long)g_stats.dropped_queue_full);
            }
        }
    }
}

bool can_bridge_start(void) {
    /* Guard against double initialization */
    if (g_can_queue != NULL) {
        return true;
    }

    /* 1. Initialize hardware */
    if (!can_bridge_init(CAN_BRIDGE_DEFAULT_BAUDRATE)) {
        return false;
    }

    /* 2. Create message queue */
    g_can_queue = xQueueCreate(CAN_BRIDGE_QUEUE_SIZE, sizeof(can_msgs_Frame));
    if (g_can_queue == NULL) {
        printf("[CAN Bridge] Error: Failed to create FreeRTOS queue!\r\n");
        return false;
    }

    /* 3. Create FreeRTOS task */
    osThreadDef(canBridgeTask, can_bridge_task, osPriorityNormal, 0, 512);
    osThreadId task_id = osThreadCreate(osThread(canBridgeTask), NULL);
    if (task_id == NULL) {
        printf("[CAN Bridge] Error: Failed to create FreeRTOS thread!\r\n");
        return false;
    }

    return true;
}
