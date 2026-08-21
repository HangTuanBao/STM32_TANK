/*
 * mpu6050.h
 * Thư viện MPU6050 cho STM32 HAL (I2C)
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"
#include "i2c.h"
#include <math.h>

#define MPU6050_ADDR 0xD0 // 0x68 << 1 cho STM32 HAL

// Các thanh ghi MPU6050
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Gyro_X_Offset;
    float Gyro_Y_Offset;
    float Gyro_Z_Offset;

    float Pitch_Offset;
    float Roll_Offset;

    float Pitch;
    float Roll;
    float Yaw;
    
    uint8_t IsConnected;
} MPU6050_t;

extern MPU6050_t MPU6050;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_UpdateAngle(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct, float dt);

#endif /* __MPU6050_H__ */
