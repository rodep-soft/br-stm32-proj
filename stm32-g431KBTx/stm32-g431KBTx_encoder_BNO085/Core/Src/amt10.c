#include "main.h"
#include "amt10.h"
#include <stm32g4xx_hal_tim.h>

extern uint16_t pos1;
extern uint16_t pos2;

void get_encoder_count(void)
{
    pos1 = __HAL_TIM_GET_COUNTER(&htim1);
    pos2 = __HAL_TIM_GET_COUNTER(&htim2);
}

float calc_RPM(uint16_t now, uint16_t *prev_val)
{
    int32_t diff = (int32_t)now - (int32_t)(*prev_val);

    *prev_val = now;

    if (diff > 32767) diff -= 65536;

    if (diff < -32768) diff += 65536;

    return (float)diff * (60.0f / 0.01f / ((float)ENCODER_PPR * 4.0f));
}