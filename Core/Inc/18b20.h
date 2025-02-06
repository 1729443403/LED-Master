#include "main.h"

#define DQ_GPIO_Port GPIOB

#define DQ_Pin GPIO_PIN_5

#define DS18B20_DQ_OUT_HIGH HAL_GPIO_WritePin(DQ_GPIO_Port, DQ_Pin, GPIO_PIN_SET)

#define DS18B20_DQ_OUT_LOW HAL_GPIO_WritePin(DQ_GPIO_Port, DQ_Pin, GPIO_PIN_RESET)

#define DS18B20_DQ_IN HAL_GPIO_ReadPin(DQ_GPIO_Port, DQ_Pin)


uint8_t DS18B20_Init(void);

short DS18B20_Get_Temperature();
