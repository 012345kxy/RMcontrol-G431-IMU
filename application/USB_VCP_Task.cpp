#include "USB_VCP_Task.hpp"
#include "EKF_fusion.hpp"
#include "LSM6DSO_Task.hpp"

#include "remote_control.h"

USB_VCPTask Usb;

static const RC_ctrl_t *cali_RC;
void send_imu_rc_pack(float *angle, float *angle_v, float *quaternion,
                      float *linear_acceleration);

/*USB接收线程*/
// TODO
// 一个奇怪的bug，必须调用EKF.plot_angle的串口阻塞发送EKF解算才正常，不然yaw疯狂飘
// 现在这里周期性调用它防止yaw飘
void USB_VCP_RX_Task(void *argument) {
  while (1) {
    // cprintf(&huart3, "usb rx ok\n");
    EKF.plot_angle();
    vTaskDelay(100);
  }
}

/*USB发送线程*/
void USB_VCP_TX_Task(void *argument) {
  // 跳过开机时的零数据
  //  while(1)
  //  {
  //      if(IMU.acceleration_mg[0] != 0) break;
  //      vTaskDelay(1000);
  //  }

  while (1) {
    // Usb.imu_angle_send(EKF.Angle_fused, IMU.angular_rate_mdps);
    // Usb.imu_angle_send_vofa(EKF.Angle_fused, IMU.angular_rate_mdps,
    // EKF.Quaternion, EKF.linear_acceleration);
    send_imu_rc_pack(EKF.Angle_fused, IMU.angular_rate_mdps, EKF.Quaternion,
                     EKF.linear_acceleration);

    // CDC_Transmit_FS((uint8_t *)&rc_datapack,sizeof(rc_datapack));

    /*发送hello*/
    // char msg[30] = "hello\n";
    // CDC_Transmit_FS((uint8_t *)msg, sizeof(msg));

    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    // IMU.plot_data();
    // IMU.print_data();
    // EKF.print_angle();
  }
}

// USB接收回调函数
void CDC_Receive_CallBack(uint8_t *pbuf, uint32_t *Len) {
  uint8_t instruc_buf[32];
  uint8_t instruc[32]; // 提取到的指令
  uint8_t len = *Len;
  // 帧头帧尾索引
  uint8_t head_index = 0;
  uint8_t tail_index = 0;
  // 解析到的命令ld

  // 把pbuf的前len个字节拷贝到instruc_buf
  memcpy(instruc_buf, pbuf, len);
  instruc_buf[len] = '\0';

  // 寻找帧头尾
  for (uint8_t i = 0; i < len; i++) {
    if (instruc_buf[i] == 0x5A)
      head_index = i; // 帧头
    if (instruc_buf[i] == 0x9C)
      tail_index = i; // 帧尾
  }
  // 无效帧返回
  if (tail_index == 0)
    return;

  // 提取指令
  memcpy(instruc, instruc_buf + head_index, tail_index - head_index + 1);

  Usb.instruct_decode(instruc, tail_index - head_index + 1);
}

void USB_VCPTask::instruct_decode(uint8_t *buf, uint8_t len) {
  Frame_freq_type imu_set_frame;
  Frame_pwm_type pwm_set_frame;
  Frame_laser_type laser_set_frame;
  Frame_hardware_info_type hardware_info_frame;

  // 根据ID解析指令
  switch (buf[1]) {
  case 0x02: // CMD_IMU_SET_FREQ
    memcpy(&imu_set_frame, buf, len);
    // 输出imu_set_frame全部信息
    cprintf(&huart3, "head:%x, length:%d, cmd:%x, freq:%d, end:%x\n",
            imu_set_frame.head, imu_set_frame.length, imu_set_frame.cmd,
            imu_set_frame.freq, imu_set_frame.end);

    // todo:设置IMU发送频率
    break;

  case 0x03: // CMD_PWM_SET
    memcpy(&pwm_set_frame, buf, len);
    // 输出pwm_set_frame全部信息
    cprintf(&huart3, "head:%x, length:%d, cmd:%x, pwm:%d, %d, %d, %d, end:%x\n",
            pwm_set_frame.head, pwm_set_frame.length, pwm_set_frame.cmd,
            pwm_set_frame.pwm[0], pwm_set_frame.pwm[1], pwm_set_frame.pwm[2],
            pwm_set_frame.pwm[3], pwm_set_frame.end);

    break;

  case 0x04: // CMD_LASER_SET
    memcpy(&laser_set_frame, buf, len);
    // 输出laser_set_frame全部信息
    cprintf(&huart3, "head:%x, length:%d, cmd:%x, laser:%d, end:%x\n",
            laser_set_frame.head, laser_set_frame.length, laser_set_frame.cmd,
            laser_set_frame.laser, laser_set_frame.end);

    break;

  case 0x05: // CMD_HARDWARE_INFO
    // todo:响应查询
    break;
  default:
    break;
  }
}

void USB_VCPTask::imu_angle_send(float *angle, float *angle_v) {
  typedef struct SendPacket {
    uint8_t heaedr;
    float yaw;
    float pitch;
    float roll;
    float roll_v;
    float pitch_v;
    float yaw_v;
    int16_t ch[5];
    char s[2];
  } __attribute__((packed)) SendPacket;

  SendPacket frame;
  frame.heaedr = 0x5A;
  frame.roll = angle[2];
  frame.pitch = angle[1];
  frame.yaw = angle[0];
  frame.roll_v = angle_v[2];
  frame.pitch_v = angle_v[1];
  frame.yaw_v = angle_v[0];
  CDC_Transmit_FS((uint8_t *)&frame, sizeof(frame));
}

void USB_VCPTask::imu_angle_send_vofa(float *angle, float *angle_v,
                                      float *quaternion,
                                      float *linear_acceleration) {
  typedef struct {
    float yaw;
    float pitch;
    float roll;
    float yaw_v;
    float pitch_v;
    float roll_v;
    float quaternion[4];          // 四元数
    float linear_acceleration[3]; // 线加速度

    // int ch0;
    // int ch1;
    // int ch2;
    // int ch3;
    // int ch4;
    // int s1;
    // int s2;
    // int mouse_x;
    // int mouse_y;
    // int mouse_z;
    // int mouse_l;
    // int mouse_r;
    // int key;

    uint8_t tail[4]{0x00, 0x00, 0x80, 0x7f};
  } __attribute__((packed)) Frame_type;
  Frame_type frame;

  frame.yaw = angle[0];
  frame.pitch = angle[1];
  frame.roll = angle[2];
  frame.roll_v = angle_v[2] / 1000;
  frame.pitch_v = angle_v[1] / 1000;
  frame.yaw_v = angle_v[0] / 1000;

  // frame.ch0 = rc_datapack.ch0;
  // frame.ch1 = rc_datapack.ch1;
  // frame.ch2 = rc_datapack.ch2;
  // frame.ch3 = rc_datapack.ch3;
  // frame.ch4 = rc_datapack.ch4;
  // frame.s1 = rc_datapack.s1;
  // frame.s2 = rc_datapack.s2;
  // frame.mouse_x = rc_datapack.mouse_x;
  // frame.mouse_z = rc_datapack.mouse_x;
  // frame.mouse_l = rc_datapack.mouse_l;
  // frame.mouse_r = rc_datapack.mouse_r;
  // frame.key = rc_datapack.key;

  for (int i = 0; i < 4; i++) {
    frame.quaternion[i] = quaternion[i];
  }
  for (int i = 0; i < 3; i++) {
    frame.linear_acceleration[i] = linear_acceleration[i];
  }

  CDC_Transmit_FS((uint8_t *)&frame, sizeof(frame));
}

void send_imu_rc_pack(float *angle, float *angle_v, float *quaternion,
                      float *linear_acceleration) {
  typedef struct {
    uint16_t heaedr;
    uint8_t pkg_id;
    float yaw;
    float pitch;
    float roll;
    float yaw_v;
    float pitch_v;
    float roll_v;
  } __attribute__((packed)) Frame_type;
  Frame_type frame;

  typedef struct {
    uint16_t heaedr;
    uint8_t pkg_id;
    int ch0;
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int s1;
    int s2;
    int mouse_x;
    int mouse_y;
    int mouse_z;
    int mouse_l;
    int mouse_r;
    int key;

  } __attribute__((packed)) Frame_type2;
  Frame_type2 frame2;

  frame.heaedr = 0xAA55;
  frame.pkg_id = 0x1;
  frame.yaw = angle[0];
  frame.pitch = angle[1];
  frame.roll = angle[2];
  frame.roll_v = angle_v[0];
  frame.pitch_v = angle_v[1];
  frame.yaw_v = angle_v[2];
  CDC_Transmit_FS((uint8_t *)&frame2, sizeof(frame2));
  vTaskDelay(1 * 5); // 1khz
  // rc_trans();
  // get_remote_control_point();

  // cali_RC.

  frame2.heaedr = 0xAA55;
  frame2.pkg_id = 0x2;
  frame2.ch0 = rc_datapack.ch0;
  frame2.ch1 = rc_datapack.ch1;
  frame2.ch2 = rc_datapack.ch2;
  frame2.ch3 = rc_datapack.ch3;
  frame2.ch4 = rc_datapack.ch4;
  frame2.s1 = rc_datapack.s1;
  frame2.s2 = rc_datapack.s2;
  frame2.mouse_x = rc_datapack.mouse_x;
  frame2.mouse_z = rc_datapack.mouse_x;
  frame2.mouse_l = rc_datapack.mouse_l;
  frame2.mouse_r = rc_datapack.mouse_r;
  frame2.key = rc_datapack.key;
  CDC_Transmit_FS((uint8_t *)&frame, sizeof(frame));
  vTaskDelay(1 * 5); // 1khz
}
