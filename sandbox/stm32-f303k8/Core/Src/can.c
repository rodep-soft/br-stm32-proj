#include "stdio.h"
#include "can.h"
#include "bno085_can.h"
#include "main.h"

uint8_t tx_data[BNO085_CAN_BNO085_DATA_LENGTH] = {0};
uint32_t tx_mailbox;

void can_send(sh2_SensorValue_t *gyro, sh2_SensorValue_t *accel, CAN_HandleTypeDef *hcan)
{
  CAN_TxHeaderTypeDef tx_header = {0};

  HAL_StatusTypeDef status;
  struct bno085_can_bno085_data_t tx_msg;

  // transmit messageを初期化
  bno085_can_bno085_data_init(&tx_msg);
  
  tx_msg.accel_x   = bno085_can_bno085_data_accel_x_encode(accel->un.accelerometer.x);
  tx_msg.accel_y   = bno085_can_bno085_data_accel_y_encode(accel->un.accelerometer.y);
  tx_msg.accel_seq = bno085_can_bno085_data_accel_seq_encode(accel->sequence);
  tx_msg.gyro_z    = bno085_can_bno085_data_gyro_z_encode(gyro->un.gyroscope.z);
  tx_msg.gyro_seq  = bno085_can_bno085_data_gyro_seq_encode(gyro->sequence);

  int ret = bno085_can_bno085_data_pack(tx_data, &tx_msg, BNO085_CAN_BNO085_DATA_LENGTH);

  if (ret <= 0) 
  {
    printf("[NG] bno085_can_bno085_data_pack failed (ret=%d)\r\n", ret);
    return;
  }

  tx_header.StdId = BNO085_CAN_BNO085_DATA_FRAME_ID;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = BNO085_CAN_BNO085_DATA_LENGTH;

//   printf("CAN TX: ID=0x%03lX DLC=%lu DATA=",
//          (unsigned long)tx_header.StdId, (unsigned long)tx_header.DLC);
//   for (uint32_t i = 0; i < tx_header.DLC; i++)
//   {
//     printf("%02X ", tx_data[i]);
//   }
//   printf("\r\n");

  status = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
  if (status != HAL_OK)
  {
    printf("[NG] HAL_CAN_AddTxMessage returned %d\r\n", (int)status);
    return;
  }
    
}
/* ===== CANループバックテスト =====*/
void CAN_LoopbackRecieve(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[BNO085_CAN_BNO085_DATA_LENGTH] = {0};
  HAL_StatusTypeDef status;
  struct bno085_can_bno085_data_t rx_msg;


  /* --- 送信完了待ち --- */
  uint32_t t0 = HAL_GetTick();
  // while (HAL_CAN_IsTxMessagePending(hcan, tx_mailbox))
  // {
  //   if ((HAL_GetTick() - t0) > 100)
  //   {
  //     printf("[NG] TX timeout\r\n");
  //     return;
  //   }
  // }
  // printf("[OK] TX complete (mailbox %lu)\r\n", (unsigned long)tx_mailbox);

  /* --- ループバックされたフレームの受信待ち --- */
  //t0 = HAL_GetTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
  {
    if ((HAL_GetTick() - t0) > 100)
    {
      printf("[NG] RX timeout (no loopback frame)\r\n");
      return;
    }
  }

  status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (status != HAL_OK)
  {
    printf("[NG] HAL_CAN_GetRxMessage returned %d\r\n", (int)status);
    return;
  }

  int ret = bno085_can_bno085_data_unpack(&rx_msg, rx_data, BNO085_CAN_BNO085_DATA_LENGTH);
  (void)ret;
  printf("CAN RX: ID=0x%03lX IDE=%s DLC=%lu DATA=",
         (unsigned long)rx_header.StdId,
         (rx_header.IDE == CAN_ID_STD) ? "STD" : "EXT",
         (unsigned long)rx_header.DLC);
  printf("accel_x=%d accel_y=%d accel_seq=%u gyro_z=%d gyro_seq=%u",
         rx_msg.accel_x, rx_msg.accel_y, rx_msg.accel_seq, rx_msg.gyro_z, rx_msg.gyro_seq);
  
  // for (uint32_t i = 0; i < rx_header.DLC; i++)
  // {
  //   //printf("%02X ", rx_data[i]);
  // }
  printf("\r\n");

  /* --- 送受信一致チェック --- */
  bool match = (rx_header.StdId == BNO085_CAN_BNO085_DATA_FRAME_ID) &&
               (rx_header.IDE == CAN_ID_STD) &&
               (rx_header.DLC == BNO085_CAN_BNO085_DATA_LENGTH);
  for (uint32_t i = 0; (i < BNO085_CAN_BNO085_DATA_LENGTH) && match; i++)
  {
    if (rx_data[i] != tx_data[i])
    {
      match = false;
    }
  }

  printf(match ? "[PASS] loopback data matched\r\n"
               : "[FAIL] TX/RX mismatch\r\n");
}


