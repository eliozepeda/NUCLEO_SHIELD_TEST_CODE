/*
 * at24c128c.h
 *
 *  Created on: Jul 13, 2025
 *      Author: Elio
 */

#ifndef INC_AT24C128C_H_
#define INC_AT24C128C_H_

#include "stm32f4xx_hal.h"

// Dirección base de la EEPROM AT24C128C (7 bits): 0x50 (binario: 1010000)
// Al utilizar la HAL de STM32, se debe desplazar 1 bit a la izquierda
// para formar la dirección de 8 bits que incluye el bit de R/W (lectura/escritura)
#define EEPROM_I2C_ADDR         (0x50 << 1)  ///< Dirección I2C de 8 bits usada por la HAL

// Tamaño de página de la EEPROM en bytes.
// Este valor indica cuántos bytes se pueden escribir de forma continua
// en una sola operación de página sin sobrepasar los límites internos del dispositivo.
#define EEPROM_PAGE_SIZE        64           ///< Tamaño de página (64 bytes)

// Dirección máxima válida para esta EEPROM (128 Kbit = 16 KB = 16384 bytes).
// Rango válido de direcciones: 0x0000 a 0x3FFF (16383)
#define EEPROM_MAX_ADDRESS      16384        ///< Capacidad total en bytes (16 KB)

HAL_StatusTypeDef AT24C128C_WriteByte(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t data);
HAL_StatusTypeDef AT24C128C_ReadByte(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data);
HAL_StatusTypeDef AT24C128C_WriteBuffer(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data, uint16_t len);
HAL_StatusTypeDef AT24C128C_ReadBuffer(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data, uint16_t len);


#endif /* INC_AT24C128C_H_ */
