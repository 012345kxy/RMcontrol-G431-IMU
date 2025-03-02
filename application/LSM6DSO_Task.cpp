#include "LSM6DSO_Task.hpp"

LSM6DSO_Handle IMU;

/******* 不在此处单开线程读取IMU，IMU读取和EKF并入同一个线程 ********/
void LSM6DSO_Task(void *argument) {
  IMU.begin();
  // cprintf(&huart3, "IMU id = %x\n", IMU.checkid());
  // cprintf(&huart3, "temperature:%d\n", (int)IMU.get_temperature());
  /*以1KHZ的频率轮询获取数据*/
  while (1) {
    // if(!IMU.ready()) continue;
    IMU.update();
    // IMU.print_data();
    // IMU.plot_data();
    vTaskDelay(10);
  }
}

void LSM6DSO_Handle::plot_data() {
  typedef struct {
    float v_yaw;
    float v_pitch;
    float v_roll;
    float ax;
    float ay;
    float az;
    uint8_t tail[4]{0x00, 0x00, 0x80, 0x7f};
  } __attribute__((packed)) Frame_type;
  Frame_type frame;

  frame.v_yaw = angular_rate_mdps[0];
  frame.v_pitch = angular_rate_mdps[1];
  frame.v_roll = angular_rate_mdps[2];

  frame.ax = acceleration_mg[0];
  frame.ay = acceleration_mg[1];
  frame.az = acceleration_mg[2];

  HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(frame), HAL_MAX_DELAY);
}

void LSM6DSO_Handle::print_data() {
  cprintf(&huart3, "Gyro:\t%d,\t %d,\t %d\t(mdps)\n",
          (int)IMU.angular_rate_mdps[0], (int)IMU.angular_rate_mdps[1],
          (int)IMU.angular_rate_mdps[2]);
  vTaskDelay(10);
  cprintf(&huart3, "Accel:\t%d,\t %d,\t %d\t(mg)\n",
          (int)IMU.acceleration_mg[0], (int)IMU.acceleration_mg[1],
          (int)IMU.acceleration_mg[2]);
  vTaskDelay(10);
  cprintf(&huart3, "Mag:\t%d,\t %d,\t %d\t(mG)\n", (int)IMU.magnetic_mG[0],
          (int)IMU.magnetic_mG[1], (int)IMU.magnetic_mG[2]);
  vTaskDelay(10);
  cprintf(&huart3, "temperature:%d\n", (int)IMU.temperature_degC);
  // vTaskDelay(10);
  // cprintf(&huart3, "delta time:%d\n", (int)this_timestamp.i32bit);
}

/*更新LSM6DSO数据*/
void LSM6DSO_Handle::update() {
  /*data to fill*/
  // acceleration_mg
  // angular_rate_mdps
  // temperature_degC
  /*临时变量*/
  uint16_t num = 0;
  uint8_t wmflag = 0;
  lsm6dso_fifo_tag_t reg_tag;
  axis3bit16_t dummy;

  /* Read watermark flag */
  lsm6dso_fifo_wtm_flag_get(&reg_ctx, &wmflag);

  if (wmflag > 0) {
    /* Read number of samples in FIFO */
    lsm6dso_fifo_data_level_get(&reg_ctx, &num);

    /* 读出fifo中的数据 */
    while (num--) {
      lsm6dso_fifo_sensor_tag_get(&reg_ctx, &reg_tag);

      /* 读出目标寄存器中的数据 */
      switch (reg_tag) {
      case LSM6DSO_XL_NC_TAG:
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] =
            lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] =
            lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] =
            lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[2]);
        break;

      case LSM6DSO_GYRO_NC_TAG:
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] =
            lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] =
            lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] =
            lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
        break;

      case LSM6DSO_SENSORHUB_SLAVE0_TAG:
        memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_fifo_out_raw_get(&reg_ctx, data_raw_magnetic.u8bit);

        magnetic_mG[0] = normalizeRawMag(data_raw_magnetic.i16bit[0]);
        magnetic_mG[1] = normalizeRawMag(data_raw_magnetic.i16bit[1]);
        magnetic_mG[2] = normalizeRawMag(data_raw_magnetic.i16bit[2]);

        // has_new_mag = true;
        break;

      /* 时间戳获取 */
      case LSM6DSO_TIMESTAMP_TAG:
        memset(this_timestamp.u8bit, 0x00, sizeof(int32_t));
        lsm6dso_fifo_out_raw_get(&reg_ctx, this_timestamp.u8bit);

      default:
        /* Flush unused samples */
        memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_fifo_out_raw_get(&reg_ctx, dummy.u8bit);
        break;
      }
    }

    // 读取温度
    temperature_degC = get_temperature();
  }
}

/*判断数据是否就位*/
int8_t LSM6DSO_Handle::ready() {
  uint8_t data_ready;
  lsm6dso_temp_flag_data_ready_get(&reg_ctx, &data_ready);
  return data_ready;
}

/*读取IMU温度传感器*/
float_t LSM6DSO_Handle::get_temperature() {
  float temperature_degC;
  static int16_t data_raw_temperature;
  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
  lsm6dso_temperature_raw_get(&reg_ctx, &data_raw_temperature);
  temperature_degC = lsm6dso_from_lsb_to_celsius(data_raw_temperature);
  return temperature_degC;
}

/*读取LSM6DSO ID*/
// return 1: error
uint8_t LSM6DSO_Handle::checkid() {
  uint8_t id = 0x00;
  if (LSM6DSO_ReadID(&lsm6dso_obj, &id) != 0) {
    return 1;
  }
  return id;
}

/*IMU 参数初始化&采样开始*/
void LSM6DSO_Handle::begin() {
  LSM6DSO_Handle::reset();
  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&reg_ctx, LSM6DSO_I3C_DISABLE);

  /* Configure magnetometer */
  // setupMag();

  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&reg_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dso_xl_full_scale_set(&reg_ctx, xl_fullscale_set);
  lsm6dso_gy_full_scale_set(&reg_ctx, gyro_fullscale_set);

  // change to fifo
  // /* Configure filtering chain(No aux interface)
  // * Accelerometer - LPF1 + LPF2 path
  // */
  // lsm6dso_xl_hp_path_on_out_set(&reg_ctx, LSM6DSO_LP_ODR_DIV_10);
  // lsm6dso_xl_filter_lp2_set(&reg_ctx, PROPERTY_ENABLE);

  /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  lsm6dso_fifo_watermark_set(&reg_ctx, 1);
  /* Set FIFO batch XL/Gyro ODR */
  lsm6dso_fifo_xl_batch_set(&reg_ctx, LSM6DSO_XL_BATCHED_AT_833Hz);
  lsm6dso_fifo_gy_batch_set(&reg_ctx, LSM6DSO_GY_BATCHED_AT_833Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  /* flush any previous value in FIFO before start */
  lsm6dso_fifo_mode_set(&reg_ctx, LSM6DSO_BYPASS_MODE);
  /* start batching in continuous mode */
  lsm6dso_fifo_mode_set(&reg_ctx, LSM6DSO_STREAM_MODE);
  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_833Hz);
  lsm6dso_gy_data_rate_set(&reg_ctx, LSM6DSO_GY_ODR_833Hz);
  /* 时间戳 */
  // lsm6dso_fifo_timestamp_decimation_set(&reg_ctx, LSM6DSO_DEC_1);

  /* SH Mag */
  /* Enable FIFO batching of Slave0. ODR batching is 104 Hz. */
  lsm6dso_sh_batch_slave_set(&reg_ctx, 0, PROPERTY_ENABLE);
  lsm6dso_sh_data_rate_set(&reg_ctx, LSM6DSO_SH_ODR_104Hz);

  /* Prepare sensor hub to read data from external Slave0 continuously in order
   * to store data in FIFO. */
  lsm6dso_sh_cfg_read_t sh_cfg_read;

  sh_cfg_read.slv_add = SH_IIC_ADDRESS;
  sh_cfg_read.slv_subadd = SH_MAG_XOUT_ADDR;
  sh_cfg_read.slv_len = SH_MAG_XOUT_LEN;

  lsm6dso_sh_slv_cfg_read(&reg_ctx, 0, &sh_cfg_read);

  /* Configure Sensor Hub to read one slave. */
  lsm6dso_sh_slave_connected_set(&reg_ctx, LSM6DSO_SLV_0);
  /* Enable I2C Master. */
  lsm6dso_sh_master_set(&reg_ctx, PROPERTY_ENABLE);
}

void LSM6DSO_Handle::reset() {
  uint8_t rst;

  /* Restore default configuration */
  lsm6dso_reset_set(&reg_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(&reg_ctx, &rst);
  } while (rst);
}

/*IMU SPI初始化*/
LSM6DSO_Handle::LSM6DSO_Handle(/* args */) {
  BSP_SPI2_Init();
  reg_ctx.write_reg = SPI2_IOSend;
  reg_ctx.read_reg = SPI2_IORecv;
  reg_ctx.mdelay = freertos_delay;
  reg_ctx.handle = &hspi2;
  lsm6dso_obj.Ctx = reg_ctx;
}

LSM6DSO_Handle::~LSM6DSO_Handle() {}

/********************SPI IO收发********************/
int32_t SPI2_IOSend(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)bufp, len, 1000);
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
  return 0;
}

int32_t SPI2_IORecv(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  reg |= 0x80;
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &reg, 1, 1000);
  HAL_SPI_Receive(&hspi2, bufp, len, 1000);
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/********************Sensor Hub IO收发********************/
uint8_t LSM6DSO_Handle::SH_IO_Write(uint8_t *buf, uint8_t reg, uint16_t num) {
  int16_t dummy[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;
  lsm6dso_sh_cfg_write_t sh_cfg_write;
  /* Configure Sensor Hub to write sh slave. */
  sh_cfg_write.slv0_add = MMC5603NJ_I2C_ADDRESS;
  sh_cfg_write.slv0_subadd = reg, sh_cfg_write.slv0_data = *buf,
  ret = lsm6dso_sh_cfg_write(&reg_ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_OFF);
  /* Enable I2C Master. */
  lsm6dso_sh_master_set(&reg_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&reg_ctx, dummy);

  do {
    vTaskDelay(10);
    lsm6dso_xl_flag_data_ready_get(&reg_ctx, &drdy);
  } while (!drdy);

  do {
    lsm6dso_sh_status_get(&reg_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dso_sh_master_set(&reg_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_OFF);
  return ret;
}

uint8_t LSM6DSO_Handle::SH_IO_Read(uint8_t *buf, uint8_t reg, uint16_t num) {
  lsm6dso_sh_cfg_read_t sh_cfg_read;
  int16_t dummy[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;
  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_OFF);
  /* Configure Sensor Hub to read sh slave. */
  sh_cfg_read.slv_add = MMC5603NJ_I2C_ADDRESS;
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = num;
  ret = lsm6dso_sh_slv_cfg_read(&reg_ctx, 0, &sh_cfg_read);
  lsm6dso_sh_slave_connected_set(&reg_ctx, LSM6DSO_SLV_0);
  /* Enable I2C Master and I2C master. */
  lsm6dso_sh_master_set(&reg_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&reg_ctx, dummy);

  do {
    vTaskDelay(10);
    lsm6dso_xl_flag_data_ready_get(&reg_ctx, &drdy);
  } while (!drdy);

  do {
    lsm6dso_sh_status_get(&reg_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dso_sh_master_set(&reg_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&reg_ctx, LSM6DSO_XL_ODR_OFF);
  /* Read SensorHub registers. */
  lsm6dso_sh_read_data_raw_get(&reg_ctx, buf, num);
  return ret;
}

void freertos_delay(uint32_t ms) { vTaskDelay(ms); }
