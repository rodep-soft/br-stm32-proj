#ifndef MOCK_GPIO_H
#define MOCK_GPIO_H
#define PA0_PORT 0
#define PA1_PORT 0
#define PA4_PORT 0
#define PB0_PORT 1
#define PA0_PIN 0
#define PA1_PIN 1
#define PA4_PIN 4
#define PB0_PIN 0
#define HAL_GPIO_PIN_RESET 0
#define HAL_GPIO_PIN_SET 1
int HAL_GPIO_ReadPin(int, int);
void HAL_GPIO_WritePin(int, int, int);
#endif
