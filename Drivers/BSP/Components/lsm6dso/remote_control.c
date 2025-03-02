/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief 遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  * 资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board
  tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

// #include "bsp_usart.h"
#include "bsp_rc.h"
#include "com.h"
#include "string.h"
#include "usart.h"
// #include "detect_task.h"

// 遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void rc_trans(void);
// remote control data
// 遥控器控制变量
RC_ctrl_t rc_ctrl;
// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[50] = {0};

/**
 * @brief          remote control init
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          遥控器初始化
 * @param[in]      none
 * @retval         none
 */
void remote_control_init(void) { RC_Init(SBUS_RX_BUF_NUM); }
/**
 * @brief          get remote control data point
 * @param[in]      none
 * @retval         remote control data point
 */
/**
 * @brief          获取遥控器数据指针
 * @param[in]      none
 * @retval         遥控器数据指针
 */
const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }

void slove_RC_lost(void) { RC_restart(SBUS_RX_BUF_NUM); }
void slove_data_error(void) { RC_restart(SBUS_RX_BUF_NUM); }

// 串口中断
/*
void USART2_IRQHandler(void)
{
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    __HAL_DMA_DISABLE(&hdma_usart2_rx);

    sbus_to_rc(sbus_rx_buf, &rc_ctrl);

    __HAL_DMA_ENABLE(&hdma_usart2_rx);//使能DMA
}
*/

/*
void HAL_UARTEx_RxEventCa11back(UART_HandleTypeDef  *huart, uint16_t Size)
{
    if(huart==&huart2)
    {
        sbus_to_rc(sbus_rx_buf, &rc_ctrl);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buf, sizeof(sbus_rx_buf));
    }
}
*/
/*
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
        // 停止本次DMA传输
    HAL_UART_DMAStop(&huart2);

    sbus_to_rc(sbus_rx_buf, &rc_ctrl);

    HAL_UART_Receive_DMA(&huart2,(uint8_t *)sbus_rx_buf, 50);
}
*/

/**
 * @brief          remote control protocol resolution
 * @param[in]      sbus_buf: raw data point
 * @param[out]     rc_ctrl: remote control data struct point
 * @retval         none
 */
/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: 遥控器数据指
 * @retval         none
 */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
  if (sbus_buf == NULL || rc_ctrl == NULL) {
    return;
  }

  rc_ctrl->rc.ch[0] =
      (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
  rc_ctrl->rc.ch[1] =
      ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;        //!< Channel 1
  rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | //!< Channel 2
                       (sbus_buf[4] << 10)) &
                      0x07ff;
  rc_ctrl->rc.ch[3] =
      ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
  rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);       //!< Switch left
  rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  //!< Switch right
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);    //!< Mouse X axis
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);    //!< Mouse Y axis
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  //!< Mouse Z axis
  rc_ctrl->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press ?
  rc_ctrl->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press ?
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);    //!< KeyBoard value
  rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); // NULL

  rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

  rc_trans();

  cprintf(&huart3, "**********\r\n\
ch0:%d\r\n\
ch1:%d\r\n\
ch2:%d\r\n\
ch3:%d\r\n\
ch4:%d\r\n\
s1:%d\r\n\
s2:%d\r\n\
mouse_x:%d\r\n\
mouse_y:%d\r\n\
mouse_z:%d\r\n\
press_l:%d\r\n\
press_r:%d\r\n\
key:%d\r\n\
**********\r\n",
          rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2],
          rc_ctrl->rc.ch[3], rc_ctrl->rc.ch[4], rc_ctrl->rc.s[0],
          rc_ctrl->rc.s[1], rc_ctrl->mouse.x, rc_ctrl->mouse.y,
          rc_ctrl->mouse.z, rc_ctrl->mouse.press_l, rc_ctrl->mouse.press_r,
          rc_ctrl->key.v);
}

struct rc_data rc_datapack;

void rc_trans(void) {
  rc_datapack.ch0 = rc_ctrl.rc.ch[0];
  rc_datapack.ch1 = rc_ctrl.rc.ch[1];
  rc_datapack.ch2 = rc_ctrl.rc.ch[2];
  rc_datapack.ch3 = rc_ctrl.rc.ch[3];
  rc_datapack.ch4 = rc_ctrl.rc.ch[4];
  rc_datapack.s1 = rc_ctrl.rc.s[0];
  rc_datapack.s2 = rc_ctrl.rc.s[1];
  rc_datapack.mouse_x = rc_ctrl.mouse.x;
  rc_datapack.mouse_y = rc_ctrl.mouse.y;
  rc_datapack.mouse_z = rc_ctrl.mouse.z;
  rc_datapack.mouse_l = rc_ctrl.mouse.press_l;
  rc_datapack.mouse_r = rc_ctrl.mouse.press_r;
  rc_datapack.key = rc_ctrl.key.v;
}
