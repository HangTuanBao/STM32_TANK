/*
 * mpu6050.c
 * Thư viện MPU6050 cho STM32 HAL (I2C)
 */

#include "mpu6050.h"

MPU6050_t MPU6050;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
  uint8_t check = 0;
  uint8_t data = 0;

  // Đọc thanh ghi WHO_AM_I với timeout ngắn (10ms)
  HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, 10);

  if (check == 0x68) { // 0x68 là giá trị mặc định của MPU6050
    // Đánh thức MPU6050 (ghi 0x00 vào PWR_MGMT_1)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1,
                      10);

    // Cài đặt Sample Rate Divider = 125Hz
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1,
                      10);

    // Cài đặt Gyroscope Full Scale Range +/- 250 deg/s
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1,
                      10);

    // Cài đặt Accelerometer Full Scale Range +/- 2g
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1,
                      10);

    MPU6050.Pitch = 0.0f;
    MPU6050.Roll = 0.0f;
    MPU6050.Yaw = 0.0f;
    MPU6050.Gyro_X_Offset = 0.0f;
    MPU6050.Gyro_Y_Offset = 0.0f;
    MPU6050.Gyro_Z_Offset = 0.0f;
    MPU6050.IsConnected = 1;

    MPU6050_Calibrate(hi2c);
    return 0; // Thành công
  }

  MPU6050.IsConnected = 0;
  return 1; // Thất bại
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c) {
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float pitch_sum = 0.0f, roll_sum = 0.0f;
  uint8_t Rec_Data[6];

  // Lấy 200 mẫu để tính giá trị trung bình Gyro Offset khi xe đứng yên
  for (int i = 0; i < 200; i++) {
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 10) ==
        HAL_OK) {
      gx_sum += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
      gy_sum += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
      gz_sum += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    }
    HAL_Delay(2);
  }

  MPU6050.Gyro_X_Offset = (float)gx_sum / 200.0f;
  MPU6050.Gyro_Y_Offset = (float)gy_sum / 200.0f;
  MPU6050.Gyro_Z_Offset = (float)gz_sum / 200.0f;

  // Cấu hình hướng lắp đặt: Đi tiến = Trục Y, Sang phải = Trục X, Hướng lên cao
  // = Trục Z
  for (int i = 0; i < 50; i++) {
    MPU6050_Read_All(hi2c, &MPU6050);
    float init_pitch = atan2f(MPU6050.Ay, sqrtf(MPU6050.Ax * MPU6050.Ax +
                                                MPU6050.Az * MPU6050.Az)) *
                       180.0f / (float)M_PI;
    float init_roll = atan2f(MPU6050.Ax, MPU6050.Az) * 180.0f / (float)M_PI;
    pitch_sum += init_pitch;
    roll_sum += init_roll;
    HAL_Delay(2);
  }

  MPU6050.Pitch_Offset = pitch_sum / 50.0f;
  MPU6050.Roll_Offset = roll_sum / 50.0f;
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
  uint8_t Rec_Data[14];

  if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1,
                       Rec_Data, 14, 10) == HAL_OK) {
    DataStruct->IsConnected = 1;
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;

    DataStruct->Gx =
        (DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / 131.0;
    DataStruct->Gy =
        (DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / 131.0;
    DataStruct->Gz =
        (DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / 131.0;
  } else {
    DataStruct->IsConnected = 0; // Đánh dấu mất kết nối nếu I2C lỗi
  }
}

void MPU6050_UpdateAngle(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct,
                         float dt) {
  MPU6050_Read_All(hi2c, DataStruct);

  // Cấu hình hướng lắp đặt: Đi tiến = Trục Y, Sang phải = Trục X, Hướng lên cao
  // = Trục Z Pitch (Ngóc đầu): Xoay quanh trục X (dùng Gyro X và Accel Y)
  float accel_pitch =
      (atan2f(DataStruct->Ay, sqrtf(DataStruct->Ax * DataStruct->Ax +
                                    DataStruct->Az * DataStruct->Az)) *
       180.0f / (float)M_PI) -
      DataStruct->Pitch_Offset;
  // Roll (Nghiêng trái/phải): Xoay quanh trục Y (dùng Gyro Y và Accel X)
  float accel_roll =
      (atan2f(DataStruct->Ax, DataStruct->Az) * 180.0f / (float)M_PI) -
      DataStruct->Roll_Offset;

  // Bộ lọc phối hợp: Pitch dùng Gyro X (Gx), Roll dùng Gyro Y (Gy)
  DataStruct->Pitch =
      0.96f * (DataStruct->Pitch + DataStruct->Gx * dt) + 0.04f * accel_pitch;
  DataStruct->Roll =
      0.96f * (DataStruct->Roll + DataStruct->Gy * dt) + 0.04f * accel_roll;

  // Tích phân Gyro Z tính góc xoay Yaw (Heading)
  if (fabs(DataStruct->Gz) > 0.6f) {
    DataStruct->Yaw += DataStruct->Gz * dt;
  }
}
