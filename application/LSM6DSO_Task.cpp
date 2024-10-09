#include "LSM6DSO_Task.hpp"


LSM6DSO_Handle IMU;

void LSM6DSO_Task(void *argument)
{
    while (1)
    {
        cprintf(&huart3, "id = %x\n", IMU.checkid());
        vTaskDelay(100);
    }
    
}

//return 1: error
uint8_t LSM6DSO_Handle::checkid()
{
    u_int8_t id=0x00;
    if (LSM6DSO_ReadID(&lsm6dso_obj, &id) != 0) {
        return 1;
    }
    return id;
}

/*IMU SPI��ʼ��*/
LSM6DSO_Handle::LSM6DSO_Handle(/* args */)
{
    BSP_SPI2_Init();
    reg_ctx.write_reg = SPI2_IOSend;
    reg_ctx.read_reg = SPI2_IORecv;
    reg_ctx.handle = &hspi2;
    lsm6dso_obj.Ctx = reg_ctx;
}

LSM6DSO_Handle::~LSM6DSO_Handle()
{
}


/*SPI IO�շ�*/
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    if (Length > 128) {
        return -1;
    }

    uint8_t data[128 + 1];  
    data[0] = (reg | 0x80);            // ���üĴ�����ַΪ��һ���ֽ�
    memcpy(&data[1], pData, Length);   // ��������

    int err = 0;
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    err = BSP_SPI2_Send(data, Length + 1);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

    return err;
}

int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *pData, uint16_t Length) {
    uint8_t dataReg = reg | 0x80; // ���ö�λ������MSB�Ƕ�д����λ
    
    // ���ͼĴ�����ַ
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
    BSP_SPI2_Send(&dataReg, 1);  // �ȷ��ͼĴ�����ַ
    
    // ��������
    int err = 0;
    err = BSP_SPI2_Recv(pData, Length);
    HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
    return err;
}