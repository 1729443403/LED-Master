/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "18b20.h"
#define BREATHING_MAX_BRIGHTNESS 100  // 最大亮度值，与count的范围一致
#define BREATHING_STEPS 100           // 呼吸灯的渐变步数，可以根据需要调整
GPIO_InitTypeDef GPIO_InitStruct = {0};
uint8_t mode = 1;
uint8_t green_period = 2;  // LED_GREEN初始周期为2秒
uint8_t red_period = 2;    // LED_RED初始周期为2秒
uint32_t green_last_toggle_time = 0;  // LED_GREEN上次切换时间
uint32_t red_last_toggle_time = 0;    // LED_RED上次切换时间
uint8_t green_state = 0;  // LED_GREEN状态
uint8_t red_state = 0;    // LED_RED状态
uint8_t last_button_state = 1; // 上一次按钮状态，默认为未按下（高电平）
uint8_t button_pressed = 0;   // 按钮是否被按下
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receiveData[50];
uint8_t bleCmdReceived = 0; // 标记是否收到新命令
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3) {
        receiveData[Size] = '\0'; // 添加字符串终止符
        bleCmdReceived = 1;       // 标记有新命令

        // 立即重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,sizeof(receiveData));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
  HAL_Delay(20);
  OLED_Init();
  DS18B20_Init();
  char tellmode[50];
  char redloop[50];
  char greenloop[50];
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  int count=0;
  char message[20];
  char message2[20];
  static uint32_t last_breath_time = 0; // 上一次呼吸灯状态更新的时间
  static int breath_period = 1;        // 呼吸灯周期，初始为1
  static int breath_step = 0;          // 当前呼吸灯的步数
  char breath_message[32];             // 用于存储显示的字符串
  int value=0;
  float voltage=0.0;
  int temperature_threshold = 15;
  float temp=5;
  float threshold_voltage=0.0;
  char color_message[32]; // 用于存储颜色信息的字符串
  uint16_t pwm_value = 0; // PWM占空比
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  typedef enum {
      COLOR_RED,
      COLOR_GREEN,
      COLOR_BLUE,
      COLOR_YELLOW,
      COLOR_CYAN,
      COLOR_MAGENTA,
      COLOR_WHITE,
      COLOR_OFF
  } ColorState;
  const char* color_names[] = {
      "红色",
      "绿色",
      "蓝色",
      "黄色",
      "青色",
      "品红",
      "白色",
      "关闭"
  };
  ColorState current_color = COLOR_RED; // 当前颜色状态
  void LED_Init(void) {
      GPIO_InitTypeDef GPIO_InitStruct = {0};
  }

  void SwitchColor(void) {
      switch (current_color) {
          case COLOR_RED:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              break;
          case COLOR_GREEN:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              break;
          case COLOR_BLUE:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              break;
          case COLOR_YELLOW:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              break;
          case COLOR_CYAN:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              break;
          case COLOR_MAGENTA:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              break;
          case COLOR_WHITE:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              break;
          case COLOR_OFF:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              break;
          default:
              break;
      }
  }
  void ProcessBLECommand(uint8_t cmd[])
  {
      // 模式切换命令
      if (strcmp((char*)cmd, "mode") == 0) {
          mode = (mode % 12) + 1;
      }
      // QR码显示命令
      else if (strcmp((char*)cmd, "QR") == 0) {
          mode = 12;
      }
      // 各模式参数调整
      else {
          switch (mode) {
              case 4: { // 绿色LED周期调整
                  int period;
                  if (sscanf((char*)cmd, "green %d", &period) == 1) {
                      green_period = period;
                  }
              } break;

              case 5: { // 红色LED周期调整
                  int period2;
                  if (sscanf((char*)cmd, "red %d", &period2) == 1) {
                      red_period = period2;
                  }
              } break;

              case 6: { // 亮度调整
                  int brightness;
                  if (sscanf((char*)cmd, "bright %d", &brightness) == 1) {
                      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, brightness);
                  }
              } break;

              case 7: { // 呼吸周期调整
                  int period3;
                  if (sscanf((char*)cmd, "breath %d", &period3) == 1) {
                      breath_period = period3;
                  }
              } break;

              case 8: { // 电压阈值调整
                  float threshold;
                  if (sscanf((char*)cmd, "threshold %f", &threshold) == 1) {
                      threshold_voltage = threshold;
                  }
              } break;
              case 10: { // 温度阈值调整
                  int threshold2;
                  if (sscanf((char*)cmd, "temper %d", &threshold2) == 1) {
                      temperature_threshold = threshold2;
                  }
              } break;
              case 11: { // RGB颜色调整
                  if (strcmp((char*)cmd, "color red") == 0) current_color = COLOR_RED;
                  else if (strcmp((char*)cmd, "color green") == 0) current_color = COLOR_GREEN;
                  else if (strcmp((char*)cmd, "color blue") == 0) current_color = COLOR_BLUE;
                  else if (strcmp((char*)cmd, "color yellow") == 0) current_color = COLOR_YELLOW;
                  else if (strcmp((char*)cmd, "color cyan") == 0) current_color = COLOR_CYAN;
                  else if (strcmp((char*)cmd, "color magenta") == 0) current_color = COLOR_MAGENTA;
                  else if (strcmp((char*)cmd, "color white") == 0) current_color = COLOR_WHITE;
                  else if (strcmp((char*)cmd, "color off") == 0) current_color = COLOR_OFF;
                  SwitchColor();
              } break;
          }
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	    if (bleCmdReceived) {
	        ProcessBLECommand(receiveData);
	        memset(receiveData, 0, sizeof(receiveData));
	        bleCmdReceived = 0;
	    }
	  count = __HAL_TIM_GET_COUNTER(&htim1);
	  if (count>60000){
		  count = 0;
		  __HAL_TIM_SET_COUNTER(&htim1,0);
	  }else if (count>100){
		  count =100;
		  __HAL_TIM_SET_COUNTER(&htim1,100);
	  }
	  sprintf(message,"亮度=%d",count);
	  OLED_NewFrame();
	  sprintf(tellmode,"当前模式: %d",mode);
	  OLED_PrintString(0,0,tellmode,&font16x16,OLED_COLOR_NORMAL);
	  OLED_PrintString(0,18,"用按钮2切换模式",&font16x16,OLED_COLOR_NORMAL);



	  if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
          mode = (mode % 12) + 1;
          HAL_Delay(20);
          while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);  // 等待按键释放
      }

      switch (mode) {

          case 1:

                GPIOA->CRL&=0xf7ffffff;

        	      	                //  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       	                             // GPIO_InitStruct.Pin = LED_BLUE_Pin;
        	                        //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        	                        //  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

              HAL_Delay(20);
              if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              } else {
                  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              }
              break;

          case 2:
        	  HAL_Delay(20);

              if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
              } else {
                  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
              }
              break;

          case 3:
        	  HAL_Delay(20);
              if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                  HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
                  while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){};  // 等待按键释放
              }
              break;

          case 4:
        	  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	  HAL_Delay(20);
        	  sprintf(greenloop,"绿色闪烁周期: %d",green_period*2);
        	  OLED_PrintString(0,36,greenloop,&font16x16,OLED_COLOR_NORMAL);
              if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                  green_period = (green_period % 3) + 1;
                  HAL_Delay(20);
                  while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
              }

              if ((HAL_GetTick() - green_last_toggle_time) >= green_period * 1000) {
                  green_last_toggle_time = HAL_GetTick();
                  green_state = !green_state;
                  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, green_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                  while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){};
              }
              break;

          case 5:
        	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        	  HAL_Delay(20);

        	  // 假设count的范围是1到100，将count映射到闪烁周期范围1秒到10秒
        	  int red_period = (count - 1) * 9 / 99 + 1; // 简单线性映射
        	  int timee=((count - 1) * 9 / 99 + 1) * 2;
        	  sprintf(redloop, "红色闪烁周期: %d", timee);
        	  OLED_PrintString(0, 36, redloop, &font16x16, OLED_COLOR_NORMAL);

        	  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        	      // 假设按键按下时，count在1到100之间循环
        	      count = (count % 100) + 1;
        	      red_period = (count - 1) * 9 / 99 + 1; // 更新周期
        	      HAL_Delay(20);
        	      while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
        	  }

        	  if ((HAL_GetTick() - red_last_toggle_time) >= red_period * 1000) {
        	      red_last_toggle_time = HAL_GetTick();
        	      red_state = !red_state;
        	      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, red_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        	      while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {};
        	  }
        	  break;

          case 6:
        	  GPIOA->CRL&=0xf7ffffff;
        	  GPIOA->CRL|=0x08000000;

//
//        	          	      	                    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        	          	                            GPIO_InitStruct.Pin = LED_BLUE_Pin;
//        	          	                            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        	          	                            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        	  HAL_Delay(20);
        	  OLED_PrintString(0,36,message,&font16x16,OLED_COLOR_NORMAL);
        	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,count);
        	  break;
          case 7:
        	  sprintf(breath_message, "呼吸灯周期: %d秒", breath_period); // 显示呼吸灯周期
        	  OLED_PrintString(0, 36, breath_message, &font16x16, OLED_COLOR_NORMAL);
        	  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        	      // 按键按下时，周期在1到5之间循环
        	      breath_period = (breath_period % 5) + 1;
        	      HAL_Delay(200); // 防抖延时
        	      while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET); // 等待按键释放

        	      // 更新显示的周期信息
        	      sprintf(breath_message, "呼吸灯周期: %d秒", breath_period);
        	      OLED_PrintString(0, 36, breath_message, &font16x16, OLED_COLOR_NORMAL);
        	  }

        	  // 检查是否需要更新呼吸灯状态
        	  if ((HAL_GetTick() - last_breath_time) >= breath_period * 1000 / BREATHING_STEPS) {
        	      last_breath_time = HAL_GetTick(); // 更新时间戳

        	      // 计算当前亮度
        	      int brightness = (int)(0.5 * BREATHING_MAX_BRIGHTNESS * (1 + sin(2 * M_PI * breath_step / BREATHING_STEPS)));

        	      // 设置PWM占空比来控制呼吸灯亮度
        	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, brightness);

        	      // 更新呼吸灯步数
        	      breath_step++;
        	      if (breath_step >= BREATHING_STEPS) {
        	          breath_step = 0; // 重置步数
        	      }
        	  }

        	  break;

          case 8:
        	  GPIOA->CRL&=0xf7ffffff;
        	  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	  HAL_ADC_Start(&hadc1);
        	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        	  value = HAL_ADC_GetValue(&hadc1);
        	  float voltage = (float)value * 3.3 / 4095.0;
        	  sprintf(message, "Voltage:%.2fv", voltage);
        	  sprintf(message2, "电压阈值:%.2fv", threshold_voltage);
        	  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
        	  HAL_Delay(20);
        	  OLED_PrintString(0, 34, message, &font16x16, OLED_COLOR_NORMAL);
              threshold_voltage = (float)count * 3.3 / 100.0;

              // 根据电压与阈值比较控制LED
              if (voltage > threshold_voltage) {
                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);  // 点亮红色LED
              } else {
                  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);  // 熄灭红色LED
              }
              sprintf(message, "Voltage: %.2fV", voltage);
              HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
              OLED_PrintString(0, 28, message, &font16x16, OLED_COLOR_NORMAL);
              OLED_PrintString(0, 44, message2, &font16x16, OLED_COLOR_NORMAL);
              HAL_Delay(40);
        	  break;

          case 9:
        	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        	  GPIOA->CRL&=0xf7ffffff;
        	  GPIOA->CRL|=0x08000000;
        	    HAL_ADC_Start(&hadc2);
        	    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        	    value = HAL_ADC_GetValue(&hadc2);
        	    voltage = (float)value * 3.3 / 4095.0; // 将ADC值转换为电压

        	    // 根据voltage值调整PWM占空比，使亮度随voltage变化
        	    // 假设voltage范围为0到2V，PWM占空比范围为0到1000
        	    pwm_value = (uint16_t)((voltage / 2.0) * 500); // 将voltage映射到PWM占空比
        	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value); // 设置PWM占空比

        	    // 判断声音大小并显示
        	    if (voltage >= 1.5) {
        	        sprintf(message2, "声音大小:大");
        	    } else if (voltage >= 0.7) {
        	        sprintf(message2, "声音大小:中");
        	    } else {
        	        sprintf(message2, "声音大小:小");
        	    }

        	    // 显示电压值和声音大小
        	    sprintf(message, "Voice: %.2fV", voltage);
        	    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
        	    OLED_PrintString(0, 28, message, &font16x16, OLED_COLOR_NORMAL);
        	    OLED_PrintString(0, 44, message2, &font16x16, OLED_COLOR_NORMAL);

        	  break;
          case 10:
  	    	  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	  temp=DS18B20_Get_Temperature();
        	  sprintf(message, "Temp: %.0fC", temp);
        	  sprintf(message2, "阈值: %.dC", temperature_threshold);
        	  OLED_PrintString(0, 34, message, &font16x16, OLED_COLOR_NORMAL);
        	  OLED_PrintString(0, 50, message2, &font16x16, OLED_COLOR_NORMAL);
        	    if (temp > temperature_threshold) {
        	        // 打开灯
        	    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        	    } else {
        	        // 关闭灯
        	    	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        	    }
        	  HAL_Delay(200);
        	  break;

          case 11:
        	  GPIOA->CRL&=0xf7ffffff;
        	  uint8_t current_button_state = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
              // 检测按钮是否被按下
              if (current_button_state == 0 && last_button_state == 1) {
                  button_pressed = 1;
              } else {
                  button_pressed = 0;
              }

              // 如果按钮被按下，切换颜色
              if (button_pressed) {
                  current_color = (ColorState)((current_color + 1) % COLOR_OFF); // 循环切换颜色
                  SwitchColor();
                  HAL_Delay(200); // 防抖延时
              }
              sprintf(color_message, "当前颜色: %s", color_names[current_color]);
              OLED_PrintString(0, 36, color_message, &font16x16, OLED_COLOR_NORMAL);

              last_button_state = current_button_state; // 更新按钮状态
        	  break;

          case 12:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	  HAL_Delay(20);
        	  OLED_NewFrame();
        	  OLED_PrintString(18, 5, "扫描", &font16x16, OLED_COLOR_NORMAL);
        	  OLED_PrintString(10, 25, "二维码", &font16x16, OLED_COLOR_NORMAL);
        	  OLED_PrintString(0, 40, "进入蓝牙", &font16x16, OLED_COLOR_NORMAL);
        	  OLED_DrawImage(64, 0, &qrImg, OLED_COLOR_NORMAL);
        	  break;



          default:
              break;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  OLED_ShowFrame();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
