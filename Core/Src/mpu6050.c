/*
 * mpu6050.c
 *
 *  Created on: Jul 13, 2025
 *      Author: Elio
 */

#include "mpu6050.h"
#include <string.h>

#define ACCEL_SCALE_FACTOR 16384.0f  // ±2g
#define GYRO_SCALE_FACTOR 131.0f     // ±250°/s

/**
 * @brief Inicializa el sensor MPU6050.
 *
 * Esta función verifica si el sensor MPU6050 está conectado correctamente al bus I2C
 * mediante la lectura del registro WHO_AM_I, y luego lo "despierta" saliendo del modo
 * de suspensión escribiendo 0x00 en el registro de gestión de energía.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C configurada (normalmente &hi2c1)
 * @retval HAL_OK si la inicialización fue exitosa
 *         HAL_ERROR si hubo un error en la comunicación o identificación del sensor
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;  // Variable para almacenar el valor leído del registro WHO_AM_I
    uint8_t data;   // Variable temporal para configurar registros

    // Leer el registro WHO_AM_I del MPU6050 para verificar si responde correctamente
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR; // Si falla la lectura, retornar error

    // El valor esperado en WHO_AM_I es 0x68, si no coincide, retornar error
    if (check != 0x68)
        return HAL_ERROR;

    // Si el sensor respondió correctamente, escribir 0x00 en el registro PWR_MGMT_1
    // para salir del modo de suspensión y habilitar el sensor
    data = 0x00;
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Lee todos los registros relevantes del MPU6050 (acelerómetro, giroscopio y temperatura)
 *        y calcula los ángulos de inclinación a partir de los datos del acelerómetro.
 *
 * Esta función realiza una lectura en bloque de 14 bytes comenzando desde el registro ACCEL_XOUT_H
 * y los descompone en valores de aceleración, temperatura y giroscopio crudos (raw), los escala
 * a valores físicos (g, °C, °/s), y finalmente calcula los ángulos de inclinación X e Y.
 *
 * @param hi2c Puntero al manejador de la interfaz I2C (ej. &hi2c1)
 * @param mpu Puntero a la estructura donde se almacenarán los datos del sensor
 * @retval HAL_OK si la lectura fue exitosa, de lo contrario un código de error HAL
 */
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t buffer[14];  // Arreglo donde se almacenan los 14 bytes leídos del sensor

    // Leer 14 bytes comenzando desde ACCEL_XOUT_H
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H,
        1, buffer, 14, HAL_MAX_DELAY
    );

    if (status != HAL_OK) return status;  // Si hubo error, salir con ese estado

    // Obtener valores RAW (datos crudos) combinando los MSB y LSB de cada eje
    mpu->raw.Accel_X     = (int16_t)(buffer[0] << 8 | buffer[1]);
    mpu->raw.Accel_Y     = (int16_t)(buffer[2] << 8 | buffer[3]);
    mpu->raw.Accel_Z     = (int16_t)(buffer[4] << 8 | buffer[5]);
    mpu->raw.Temperature = (int16_t)(buffer[6] << 8 | buffer[7]);
    mpu->raw.Gyro_X      = (int16_t)(buffer[8] << 8 | buffer[9]);
    mpu->raw.Gyro_Y      = (int16_t)(buffer[10] << 8 | buffer[11]);
    mpu->raw.Gyro_Z      = (int16_t)(buffer[12] << 8 | buffer[13]);

    // Escalar los valores RAW a unidades físicas
    mpu->scaled.Accel_X     = mpu->raw.Accel_X / ACCEL_SCALE_FACTOR; // en "g"
    mpu->scaled.Accel_Y     = mpu->raw.Accel_Y / ACCEL_SCALE_FACTOR;
    mpu->scaled.Accel_Z     = mpu->raw.Accel_Z / ACCEL_SCALE_FACTOR;

    mpu->scaled.Temperature = mpu->raw.Temperature / 340.0f + 36.53f; // en °C

    mpu->scaled.Gyro_X      = mpu->raw.Gyro_X / GYRO_SCALE_FACTOR; // en °/s
    mpu->scaled.Gyro_Y      = mpu->raw.Gyro_Y / GYRO_SCALE_FACTOR;
    mpu->scaled.Gyro_Z      = mpu->raw.Gyro_Z / GYRO_SCALE_FACTOR;

    // Calcular ángulos de inclinación en base a los datos del acelerómetro
    mpu->angleX = MPU6050_AccelToAngleX(&mpu->raw);
    mpu->angleY = MPU6050_AccelToAngleY(&mpu->raw);
    // mpu->angleZ = MPU6050_AccelToAngleZ(&mpu->raw); // Si se implementa el cálculo en Z

    return HAL_OK;  // Lectura y procesamiento exitoso
}

/**
 * @brief Calcula el ángulo de inclinación (Pitch) en el eje X usando los datos del acelerómetro.
 *
 * El ángulo se calcula utilizando la función atan2f sobre los valores Y y Z del acelerómetro.
 * Este ángulo representa la inclinación del dispositivo hacia adelante o hacia atrás.
 *
 * @param data Puntero a una estructura con los valores crudos (raw) del acelerómetro
 * @return Ángulo en grados, en el rango [-180°, +180°]
 */
float MPU6050_AccelToAngleX(const MPU6050_DataRaw *data)
{
    // Calcular el ángulo de pitch usando atan2(Y, Z)
    // Esto representa la rotación sobre el eje X (inclinación hacia adelante o hacia atrás)
    float angle = atan2f((float)data->Accel_Y, (float)data->Accel_Z) * 180.0f / M_PI;

    // Retornar el ángulo en grados
    return angle;
}

/**
 * @brief Calcula el ángulo de inclinación (Roll) en el eje Y usando los datos del acelerómetro.
 *
 * Utiliza la función atan2f para determinar la inclinación lateral (roll) del dispositivo
 * a partir de los valores del eje X y Z del acelerómetro.
 *
 * Este ángulo indica cuánto se ha inclinado el dispositivo hacia la izquierda o derecha,
 * respecto a la gravedad, en el plano lateral.
 *
 * @param data Puntero a la estructura que contiene los valores crudos del acelerómetro (MPU6050_DataRaw)
 * @return Ángulo en grados, en el rango [-180°, +180°]
 */
float MPU6050_AccelToAngleY(const MPU6050_DataRaw *data)
{
    // Calcular el ángulo de roll (rotación lateral) usando atan2(-X, Z)
    // Se invierte el signo de Accel_X para que los ángulos sigan una convención estándar
    float angle = atan2f(-(float)data->Accel_X, (float)data->Accel_Z) * 180.0f / M_PI;

    // Retornar el ángulo en grados
    return angle;
}

