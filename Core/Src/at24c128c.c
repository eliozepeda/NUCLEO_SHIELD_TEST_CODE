/*
 * at24c128c.c
 *
 *  Created on: Jul 13, 2025
 *      Author: Elio
 */


#include "at24c128c.h"
#include "string.h"

/**
 * @brief Espera a que la EEPROM AT24C128C complete su ciclo interno de escritura.
 *
 * Cuando se escribe en la EEPROM, ésta entra en un ciclo de escritura interno (típicamente 5 ms),
 * durante el cual no responde a comandos I2C. Esta función implementa "Acknowledge Polling":
 * intenta comunicar con la EEPROM hasta que responda con un ACK, indicando que está lista.
 *
 * @param hi2c Puntero al handle de I2C usado para comunicarse con la EEPROM.
 * @retval HAL_OK si la EEPROM respondió antes de que se agotara el tiempo.
 * @retval HAL_TIMEOUT si no respondió dentro del tiempo límite.
 */
static HAL_StatusTypeDef AT24C128C_WaitForWrite(I2C_HandleTypeDef *hi2c)
{
    // Establece un tiempo máximo de espera de 10 ms desde el momento actual
    uint32_t timeout = HAL_GetTick() + 10;

    // Mientras no haya pasado el tiempo de espera...
    while (HAL_GetTick() < timeout)
    {
        // Intenta comunicar con la EEPROM (dirección, 1 intento, sin esperar más tiempo)
        if (HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR, 1, HAL_MAX_DELAY) == HAL_OK)
            return HAL_OK;  // La EEPROM ya está lista
    }

    // Si se agotó el tiempo y nunca respondió, retorna error de timeout
    return HAL_TIMEOUT;
}

/**
 * @brief Escribe un solo byte en una dirección específica de la EEPROM AT24C128C.
 *
 * Esta función prepara un buffer de 3 bytes: los dos primeros son la dirección
 * (en formato big-endian) y el tercero es el dato a escribir. Luego envía estos bytes
 * usando I2C y espera a que la EEPROM termine su ciclo interno de escritura.
 *
 * @param hi2c Puntero al handle de I2C configurado para la EEPROM.
 * @param memAddress Dirección de memoria en la EEPROM donde se escribirá el byte.
 * @param data Dato de 8 bits que se desea escribir.
 *
 * @retval HAL_OK si la escritura fue exitosa y la EEPROM respondió correctamente.
 * @retval HAL_ERROR si hubo un problema en la transmisión I2C.
 * @retval HAL_TIMEOUT si la EEPROM no respondió dentro del tiempo de espera tras la escritura.
 */
HAL_StatusTypeDef AT24C128C_WriteByte(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t data)
{
    uint8_t buf[3];

    // Dividir la dirección de memoria en dos bytes: MSB primero, luego LSB
    buf[0] = memAddress >> 8;     // Byte alto de la dirección
    buf[1] = memAddress & 0xFF;   // Byte bajo de la dirección

    // El tercer byte es el dato a escribir
    buf[2] = data;

    // Enviar los 3 bytes por I2C: dirección de la EEPROM, buffer, longitud 3 bytes, sin timeout
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR, buf, 3, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status; // Si falló la transmisión, devolver el error

    // Esperar a que la EEPROM finalice su ciclo interno de escritura (polling)
    return AT24C128C_WaitForWrite(hi2c);
}

/**
 * @brief Lee un solo byte desde una dirección específica de la EEPROM AT24C128C.
 *
 * Esta función primero transmite la dirección de memoria deseada (2 bytes),
 * y luego solicita un byte de datos desde esa dirección de la EEPROM.
 *
 * @param hi2c      Puntero al handle del bus I2C.
 * @param memAddress Dirección de memoria desde la cual se desea leer.
 * @param data      Puntero a una variable donde se almacenará el byte leído.
 *
 * @retval HAL_OK       Si la lectura fue exitosa.
 * @retval HAL_ERROR    Si hubo un fallo al enviar la dirección.
 * @retval HAL_TIMEOUT  Si hubo un timeout en la recepción del byte.
 */
HAL_StatusTypeDef AT24C128C_ReadByte(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data)
{
    // Preparar la dirección de memoria en formato big-endian (MSB primero)
    uint8_t addr[2] = {
        memAddress >> 8,        // Byte alto de la dirección
        memAddress & 0xFF       // Byte bajo de la dirección
    };

    // Enviar la dirección al dispositivo para posicionar el puntero interno
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR, addr, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status; // Error al enviar dirección

    // Leer un byte desde la dirección especificada
    return HAL_I2C_Master_Receive(hi2c, EEPROM_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Escribe múltiples bytes en la EEPROM AT24C128C a partir de una dirección específica.
 *
 * Esta función realiza escrituras por páginas, asegurándose de no cruzar los límites
 * de página del dispositivo EEPROM (cada página tiene un tamaño fijo de 64 bytes).
 * Divide automáticamente el buffer de entrada en bloques válidos según la alineación
 * de página y realiza múltiples transmisiones I2C si es necesario.
 *
 * @param hi2c       Puntero al handle del bus I2C.
 * @param memAddress Dirección inicial de memoria donde se comenzará a escribir.
 * @param data       Puntero al buffer con los datos a escribir.
 * @param len        Longitud del buffer de datos (en bytes).
 *
 * @retval HAL_OK       Si la escritura completa fue exitosa.
 * @retval HAL_ERROR    Si ocurrió un error durante alguna transmisión I2C.
 * @retval HAL_TIMEOUT  Si alguna operación I2C excedió el tiempo de espera.
 */
HAL_StatusTypeDef AT24C128C_WriteBuffer(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data, uint16_t len)
{
    while (len > 0)
    {
        // Calcular el desplazamiento dentro de la página actual
        uint8_t page_offset = memAddress % EEPROM_PAGE_SIZE;

        // Calcular cuántos bytes se pueden escribir sin cruzar el límite de página
        uint8_t chunk = EEPROM_PAGE_SIZE - page_offset;
        if (chunk > len) chunk = len;  // Ajustar si quedan menos datos que el espacio disponible

        // Preparar buffer de escritura: 2 bytes para dirección + datos a escribir
        uint8_t buf[2 + EEPROM_PAGE_SIZE];
        buf[0] = memAddress >> 8;       // Byte alto de la dirección
        buf[1] = memAddress & 0xFF;     // Byte bajo de la dirección
        memcpy(&buf[2], data, chunk);   // Copiar los datos al buffer

        // Transmitir dirección + datos al dispositivo
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR, buf, 2 + chunk, HAL_MAX_DELAY);
        if (status != HAL_OK)
            return status;  // Error durante la transmisión

        // Esperar a que la EEPROM complete la escritura interna (puede demorar varios ms)
        HAL_Delay(5); // Alternativamente, usar polling de ACK con AT24C128C_WaitForWrite()
        status = AT24C128C_WaitForWrite(hi2c);
        if (status != HAL_OK)
            return status;  // Fallo al esperar escritura

        // Avanzar puntero y dirección para la próxima página (si queda más por escribir)
        memAddress += chunk;
        data += chunk;
        len -= chunk;
    }

    return HAL_OK; // Escritura exitosa
}

/**
 * @brief Lee múltiples bytes desde una dirección específica de la EEPROM AT24C128C.
 *
 * Esta función primero transmite la dirección de memoria deseada (2 bytes),
 * y luego solicita `len` bytes de datos desde esa dirección en adelante.
 *
 * @param hi2c        Puntero al handle del bus I2C.
 * @param memAddress  Dirección de memoria desde la cual se desea comenzar la lectura.
 * @param data        Puntero al buffer donde se almacenarán los datos leídos.
 * @param len         Cantidad de bytes a leer desde la EEPROM.
 *
 * @retval HAL_OK       Si la lectura fue exitosa.
 * @retval HAL_ERROR    Si hubo un fallo al enviar la dirección o al leer los datos.
 * @retval HAL_TIMEOUT  Si alguna operación I2C excedió el tiempo de espera.
 */
HAL_StatusTypeDef AT24C128C_ReadBuffer(I2C_HandleTypeDef *hi2c, uint16_t memAddress, uint8_t *data, uint16_t len)
{
    // Preparar la dirección de memoria en formato big-endian (MSB primero)
    uint8_t addr[2] = {
        memAddress >> 8,        // Byte alto de la dirección
        memAddress & 0xFF       // Byte bajo de la dirección
    };

    // Enviar la dirección al dispositivo para posicionar el puntero interno
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, EEPROM_I2C_ADDR, addr, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status; // Error al enviar dirección

    // Leer `len` bytes desde la dirección especificada
    return HAL_I2C_Master_Receive(hi2c, EEPROM_I2C_ADDR, data, len, HAL_MAX_DELAY);
}
