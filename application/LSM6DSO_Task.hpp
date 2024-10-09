#ifndef LSM6DSO_Task_H
#define LSM6DSO_Task_H

/*CPP���ܲ���*/
#ifdef __cplusplus
//RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//debug serial
#include "com.h"
#include "usart.h"
//lsm6dso driver
#include "custom_mems_conf.h"
#include "lsm6dso_reg.h"
#include "lsm6dso.h"
#include "spi.h"

class LSM6DSO_Handle
{
private:
    /*Ĭ�ϴ���������*/
    lsm6dso_odr_xl_t xl_odr_set = LSM6DSO_XL_ODR_6667Hz;    //���ٶȼƲ�����
    lsm6dso_fs_xl_t xl_fullscale_set = LSM6DSO_2g;          //���ٶȼ��������
    lsm6dso_odr_g_t gyro_odr_set = LSM6DSO_GY_ODR_6667Hz;   //�����ǲ�����
    lsm6dso_fs_g_t gyro_fullscale_set = LSM6DSO_2000dps;    //�������������
public:
    /*�豸����*/
    stmdev_ctx_t reg_ctx;
    LSM6DSO_Object_t lsm6dso_obj;
    /*����������*/
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
    int16_t data_raw_temperature;
    float acceleration_mg[3];
    float angular_rate_mdps[3];
    float temperature_degC;
    uint8_t whoamI, rst;
    
    /*���ߺ���*/
    void print_data();              //�����ǰ�洢������
    void update();                  //��������
    float_t get_temperature();      //��ȡ�¶�
    int8_t ready();                 //�����ݿɶ�
    void begin();                   //��ʼ����
    void reset();                   //����
    uint8_t checkid();              //��ȡIMU ID
    LSM6DSO_Handle(/* args */);
    ~LSM6DSO_Handle();
};




#endif  // __cplusplus


/*FreeRTOS�ĵ��ýӿ�*/
#ifdef __cplusplus
extern "C" {
#endif

void LSM6DSO_Task(void *argument);
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);
int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length);
void freertos_delay(uint32_t ms);
#ifdef __cplusplus
}
#endif

#endif