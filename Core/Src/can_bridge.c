/**
 * @file can_bridge.c
 * @brief CAN Bridge Module - HAL driver, ISR, and FreeRTOS queue management
 *
 * This module handles:
 * 1. CAN1 HAL initialization (500 kbps on PB8/PB9)
 * 2. RX interrupt-driven multi-frame message reassembly
 * 3. Lock-free FreeRTOS queue interface for CAN↔Application data flow
 *
 * Performance notes:
 * - ISR directly reassembles multi-frame messages using bitmask tracking
 * - No dynamic allocation, no mutexes in data path
 * - __DSB() after ISR queue operations for Cortex-M7 store buffer consistency
 * - CAN filter accepts all standard IDs; software filtering in ISR
 */

#include "can_bridge.h"
#include <string.h>

/* ─────────────────────── Private State ────────────────────────────── */

static CAN_HandleTypeDef hcan1;

static QueueHandle_t rx_queue;
static QueueHandle_t tx_queue;

/** Statistics counters (updated from ISR and task context) */
static volatile uint32_t stat_rx_count  = 0;
static volatile uint32_t stat_tx_count  = 0;
static volatile uint32_t stat_err_count = 0;

/* ─────────── Multi-Frame Reassembly Buffers (ISR context) ─────────── */

/**
 * Reassembly works by:
 * 1. Each CAN frame sets its bit in the mask and unpacks data into the buffer
 * 2. When mask == complete_mask, the full message is pushed to the RX queue
 * 3. Mask is reset to 0 for the next message cycle
 *
 * This is safe because CAN frames from the same sender arrive in order,
 * and the ISR is not preempted by itself.
 */

static robot_msgs_MotorStatus reassembly_motor_status;
static uint8_t reassembly_motor_status_mask = 0;
#define MOTOR_STATUS_COMPLETE ((1U << CAN_MSG_MOTOR_STATUS_FRAME_COUNT) - 1U)

static robot_msgs_ImuData reassembly_imu_data;
static uint8_t reassembly_imu_data_mask = 0;
#define IMU_DATA_COMPLETE ((1U << CAN_MSG_IMU_DATA_FRAME_COUNT) - 1U)

/* ─────────────────────── Initialization ───────────────────────────── */

void can_bridge_init(void) {
    /* Create FreeRTOS queues */
    rx_queue = xQueueCreate(CAN_BRIDGE_RX_QUEUE_LEN, sizeof(can_bridge_rx_msg_t));
    tx_queue = xQueueCreate(CAN_BRIDGE_TX_QUEUE_LEN, sizeof(can_bridge_tx_msg_t));
    configASSERT(rx_queue != NULL);
    configASSERT(tx_queue != NULL);

    /*
     * CAN1 bit timing for 500 kbps:
     *   APB1 clock = 48 MHz (SYSCLK 96 MHz / APB1_DIV 2)
     *   Bit time   = Prescaler * (1 + BS1 + BS2) / APB1_CLK
     *              = 6 * (1 + 13 + 2) / 48 MHz
     *              = 6 * 16 / 48 MHz = 2 µs → 500 kbps
     *   Sample point = (1 + 13) / 16 = 87.5% (optimal for automotive/robotics)
     */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 6;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;   /* Auto-recover from bus-off */
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;   /* Retry on arbitration loss */
    hcan1.Init.ReceiveFifoLocked    = DISABLE;  /* Overwrite oldest on FIFO full */
    hcan1.Init.TransmitFifoPriority = ENABLE;   /* FIFO order, not ID priority */

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        configASSERT(0);  /* Fatal: CAN init failed */
    }

    /*
     * Filter configuration: Accept all standard CAN IDs.
     * Mask = 0x0000 means "don't care about any ID bits".
     * We do software filtering in the ISR for flexibility.
     */
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

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        configASSERT(0);
    }

    /* Start CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        configASSERT(0);
    }

    /* Enable RX FIFO0 message pending interrupt */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        configASSERT(0);
    }

    /*
     * NVIC priority must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
     * to safely call FreeRTOS *FromISR functions.
     * Using priority 6 (lower than MAX_SYSCALL=5) so FreeRTOS API is safe.
     */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* Also enable TX interrupt for error reporting */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/* ─────────────────────── Queue API ────────────────────────────────── */

BaseType_t can_bridge_receive(can_bridge_rx_msg_t *msg, TickType_t timeout) {
    if (msg == NULL) {
        return pdFALSE;
    }
    return xQueueReceive(rx_queue, msg, timeout);
}

BaseType_t can_bridge_send(const can_bridge_tx_msg_t *msg) {
    if (msg == NULL) {
        return pdFALSE;
    }
    BaseType_t ret = xQueueSend(tx_queue, msg, 0);
    if (ret == pdTRUE) {
        stat_tx_count++;
    } else {
        stat_err_count++;
    }
    return ret;
}

BaseType_t can_bridge_tx_receive(can_bridge_tx_msg_t *msg, TickType_t timeout) {
    if (msg == NULL) {
        return pdFALSE;
    }
    return xQueueReceive(tx_queue, msg, timeout);
}

CAN_HandleTypeDef *can_bridge_get_handle(void) {
    return &hcan1;
}

void can_bridge_get_stats(uint32_t *rx_count, uint32_t *tx_count, uint32_t *err_count) {
    if (rx_count)  *rx_count  = stat_rx_count;
    if (tx_count)  *tx_count  = stat_tx_count;
    if (err_count) *err_count = stat_err_count;
}

/* ─────────────────── Interrupt Handlers ───────────────────────────── */

/**
 * @brief CAN1 RX FIFO0 IRQ handler.
 *        This is the hardware interrupt vector — calls HAL IRQ dispatcher.
 */
void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

/**
 * @brief CAN1 TX IRQ handler (for TX complete / error tracking).
 */
void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

/**
 * @brief HAL callback: new CAN message pending in RX FIFO0.
 *
 * This runs in ISR context. Rules:
 * - No printf, no malloc, no blocking calls
 * - Only use *FromISR FreeRTOS functions
 * - Must call portYIELD_FROM_ISR if waking a higher-priority task
 * - __DSB() for Cortex-M7 write buffer flush
 *
 * Multi-frame reassembly:
 * - CAN ID range determines message type
 * - Frame index = CAN_ID - BASE_ID
 * - Each frame sets a bit in the reassembly mask
 * - When all bits set, push complete message to RX queue
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        stat_err_count++;
        return;
    }

    /* Only process standard ID frames */
    if (rx_header.IDE != CAN_ID_STD) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    can_bridge_rx_msg_t complete_msg;
    bool msg_ready = false;
    uint32_t id = rx_header.StdId;

    /* ── MotorStatus: CAN IDs 0x100..0x103 (4 frames) ── */
    if (id >= CAN_ID_MOTOR_STATUS_BASE &&
        id < (CAN_ID_MOTOR_STATUS_BASE + CAN_MSG_MOTOR_STATUS_FRAME_COUNT)) {

        uint8_t frame_idx = (uint8_t)(id - CAN_ID_MOTOR_STATUS_BASE);
        can_unpack_motor_status(&reassembly_motor_status, frame_idx,
                                rx_data, (uint8_t)rx_header.DLC);
        reassembly_motor_status_mask |= (1U << frame_idx);

        if (reassembly_motor_status_mask == MOTOR_STATUS_COMPLETE) {
            complete_msg.type = CAN_MSG_MOTOR_STATUS;
            memcpy(&complete_msg.data.motor_status, &reassembly_motor_status,
                   sizeof(robot_msgs_MotorStatus));
            reassembly_motor_status_mask = 0;
            msg_ready = true;
        }
    }
    /* ── ImuData: CAN IDs 0x200..0x202 (3 frames) ── */
    else if (id >= CAN_ID_IMU_DATA_BASE &&
             id < (CAN_ID_IMU_DATA_BASE + CAN_MSG_IMU_DATA_FRAME_COUNT)) {

        uint8_t frame_idx = (uint8_t)(id - CAN_ID_IMU_DATA_BASE);
        can_unpack_imu_data(&reassembly_imu_data, frame_idx,
                            rx_data, (uint8_t)rx_header.DLC);
        reassembly_imu_data_mask |= (1U << frame_idx);

        if (reassembly_imu_data_mask == IMU_DATA_COMPLETE) {
            complete_msg.type = CAN_MSG_IMU_DATA;
            memcpy(&complete_msg.data.imu_data, &reassembly_imu_data,
                   sizeof(robot_msgs_ImuData));
            reassembly_imu_data_mask = 0;
            msg_ready = true;
        }
    }

    /* Push to RX queue if a complete message was assembled */
    if (msg_ready) {
        if (xQueueSendFromISR(rx_queue, &complete_msg, &xHigherPriorityTaskWoken) == pdTRUE) {
            stat_rx_count++;
        } else {
            stat_err_count++;  /* Queue full — data dropped */
        }
        __DSB();  /* Cortex-M7: flush store buffer before yield decision */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
