#ifndef LSM6DSO_Task_H
#define LSM6DSO_Task_H

/*CPP���ܲ���*/
#ifdef __cplusplus
//debug serial
#include "com.h"
#include "usart.h"
//lsm6dso driver


class LSM6DSO_Handle
{
private:
    /* data */
public:
    LSM6DSO_Handle(/* args */);
    ~LSM6DSO_Handle();
};

/*IMU SPI��ʼ��*/
LSM6DSO_Handle::LSM6DSO_Handle(/* args */)
{
}

LSM6DSO_Handle::~LSM6DSO_Handle()
{
}
#endif  // __cplusplus


/*FreeRTOS�ĵ��ýӿ�*/
#ifdef __cplusplus
extern "C" {
#endif

void LSM6DSO_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif