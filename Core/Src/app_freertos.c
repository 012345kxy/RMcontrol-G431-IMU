/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
LSM6D读取相关
先测试，后期再整理
*/
#include "custom_mems_conf.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "com.h"
#include "usart.h"
#include "spi.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/*
LSM6D读取相关
先测试，后期再整理
*/

// SPI发送数据包装器函数
int32_t SPI2_Send(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    // 在这里，'reg' 可能是寄存器的地址，可以在发送的数据前添加此地址
    uint8_t data[Length + 1];  // 创建一个新的数组来包含寄存器地址和数据
    data[0] = reg;             // 设置寄存器地址为第一个字节
    memcpy(&data[1], pData, Length);  // 复制数据

    // 使用BSP_SPI2_Send发送数据，忽略寄存器地址，因为大多数SPI设备会先发送地址
    return BSP_SPI2_Send(data, Length + 1);
}

// SPI接收数据包装器函数
int32_t SPI2_Recv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    // 发送寄存器地址
    BSP_SPI2_Send(&reg, 1);  // 先发送寄存器地址
    // 接收数据
    return BSP_SPI2_Recv(pData, Length);
}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  if (BSP_SPI2_Init() != HAL_OK) {
      cprintf(&huart3, "SPI2 INIT ERROR");
  }
  // LSM6D 驱动结构体配置
  stmdev_ctx_t reg_ctx;
  reg_ctx.write_reg = SPI2_Send;
  reg_ctx.read_reg = SPI2_Recv;
  reg_ctx.handle = &hspi2;

  // 初始化LSM6DSO传感器
  LSM6DSO_Object_t lsm6dso_obj;
  lsm6dso_obj.Ctx = reg_ctx;
  // if (LSM6DSO_Init(&lsm6dso_obj) != LSM6DSO_OK) {
  //     Error_Handler();
  // }
  

  for(;;)
  {
    uint8_t id = 0;
    // if (lsm6dso_device_id_get(&reg_ctx, &id) == 0) {
    // }
    //SPI TEST
    uint8_t data[3] = {0x01, 0x02, 0x03};
    while(1){
      // SPI2_Send(&hspi2, 0x5A, data, 3);
      HAL_SPI_Transmit(&hspi2, data, 3, 0xFF);
      vTaskDelay(1);
    }
      //闪一次灯
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      vTaskDelay(100);
    // cprintf(&huart3, "%d\n", status);
    // cprintf(&huart3, "ok\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)"ok\n", 4, 0xFFFF);

  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

