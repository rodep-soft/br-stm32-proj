#ifndef __CAN_H
#define __CAN_H

#include "sh2_SensorValue.h"
#include "stm32f3xx_hal.h"


void can_send(sh2_SensorValue_t *gyro, sh2_SensorValue_t *accel, CAN_HandleTypeDef *hcan);
void CAN_LoopbackRecieve(CAN_HandleTypeDef *hcan);

#endif