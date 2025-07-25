/*
 * mpu6050.h
 *
 *  Created on: Jul 13, 2025
 *      Author: Elio
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>

// Dirección I2C del MPU6050 (7 bits = 0x68), se debe desplazar 1 bit a la izquierda
// porque HAL espera la dirección en formato de 8 bits (con R/W en el bit 0).
// AD0 debe estar conectado a GND para que la dirección sea 0x68.
#define MPU6050_ADDR            (0x68 << 1)  // = 0xD0 para escritura, 0xD1 para lectura

// Registro de gestión de energía del MPU6050.
// Es necesario escribir 0x00 en este registro para sacar al sensor del modo de suspensión.
#define MPU6050_RA_PWR_MGMT_1   0x6B

// Dirección del primer registro de datos del acelerómetro.
// Desde esta posición se leen 14 bytes en total (Accel, Temp, Gyro).
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

// Registro WHO_AM_I, usado para verificar la identidad del dispositivo.
// Debe devolver 0x68 si el sensor está correctamente conectado.
#define MPU6050_RA_WHO_AM_I     0x75

// Datos sin escalar
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    int16_t Temperature;
} MPU6050_DataRaw;

// Datos escalados (g, °/s, °C)
typedef struct {
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Temperature;
} MPU6050_DataScaled;

// Estructura principal que agrupa todo
typedef struct {
    MPU6050_DataRaw raw;
    MPU6050_DataScaled scaled;
    float angleX; // Pitch estimado por acelerómetro
    float angleY; // Roll estimado por acelerómetro
    float angleZ; //
} MPU6050_t;

// API
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);

// Funciones auxiliares
float MPU6050_AccelToAngleX(const MPU6050_DataRaw *data);
float MPU6050_AccelToAngleY(const MPU6050_DataRaw *data);
float MPU6050_AccelToAngleZ(const MPU6050_DataRaw *data)
;
#endif /* INC_MPU6050_H_ */
