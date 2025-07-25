/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include"at24c128c.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*TASKS TIME DEFINES Y BANDERAS*/
#define CIEN_MS 1000
#define QUINIENTOS_MS 5000
#define UN_SEGUNDO 10000
uint16_t CIEN_MS_FLAG = 0;
uint16_t QUINIENTOS_MS_FLAG = 0;
uint16_t UN_SEGUNDO_FLAG = 0;

/*Bandera para control de tiempo de tareas*/
uint16_t time = 0;

/********************/

// Variable global para guardar el último tiempo válido de pulsación
uint32_t lastButtonPressTick = 0;

// Bandera para leer datos de la EEPROM, activada con button 1
uint8_t Leer_datos = 0;

/*VARIABLES ADC*/
#define NUM_OF_ADC_CHANNELS 2
uint16_t CH9;
uint16_t CH14;

uint16_t *ADC_CHANNELS[NUM_OF_ADC_CHANNELS] = {&CH9,&CH14};
/**************/

/* Private defines -----------------------------------------------------------*/
#define EEPROM_HEADER_ADDR   0x0000  // Dirección del encabezado de muestras
#define EEPROM_DATA_START    0x0002  // Dirección de inicio de datos
#define EEPROM_MAX_SAMPLES   1024    // Límite de muestras válidas (4 KB máx)

/* Global variables ----------------------------------------------------------*/
// Manejador de la interfaz I2C1, utilizado para comunicar tanto con el MPU6050 como con la EEPROM AT24C128C
I2C_HandleTypeDef hi2c1;

// Estructura que contiene los datos del sensor MPU6050 (ángulos, aceleración, giroscopio, etc.)
MPU6050_t mpu;

// Dirección actual de escritura en la EEPROM. Se inicializa desde el header al arrancar el sistema.
uint16_t eepromAddr = 0;

// Último valor leído del ángulo Y (Roll), usado para detectar transiciones (por ejemplo, paso del umbral)
float lastAngleY = 0.0f;

// Bandera (flag) que indica si ya se detectó una transición de evento para evitar guardar duplicados
uint8_t triggered = 0;

// Arreglo de 4 bytes utilizado para almacenar temporalmente una muestra (float) antes de escribir o después de leer de EEPROM
uint8_t data[4];

// Variable para almacenar el valor flotante recuperado desde EEPROM
float readValue;

/*FUNCIONES*/
void Read_ADC_Channels(void);
void Init_MPU6050(void);
void Load_EEPROM_Header(void);
void Read_EEPROM(void);
void Print_Stored_Events(uint16_t numSamples);
void Store_Event_Samples(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /*Inicio de conteo del TIMER 2, este timer controla la periodicidad de las tareas*/
  HAL_TIM_Base_Start(&htim1);

  Init_MPU6050();
  Load_EEPROM_Header();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*Actualización de la variable de tiempo*/
	  time = __HAL_TIM_GET_COUNTER(&htim1);

	  /* 100 ms TASK */
	  if(time - CIEN_MS_FLAG == CIEN_MS)
	  {
		  if(Leer_datos == 1)
		  {
			  Read_EEPROM();
		  }

		  CIEN_MS_FLAG += 1000;//Actualizamos bandera para cumplir con el tiempo
	  }

	  /* 500 ms TASK */
	  if(time - QUINIENTOS_MS_FLAG == QUINIENTOS_MS)
	  {
		  Read_ADC_Channels();

		  // Leer todos los datos del MPU6050 (acelerómetro y giroscopio)
		  if (MPU6050_ReadAll(&hi2c1, &mpu) == HAL_OK)
		  {
		      // Extraer el ángulo Y (inclinación lateral o "Roll")
		      float angle = mpu.angleY;

		      // Imprimir los ángulos actuales
		      printf("Pitch: %.2f | Roll: %.2f\r\n", mpu.angleX, angle);

		      // Detectar transición: cuando el ángulo pasa de ≤20 a >20
		      if (!triggered && lastAngleY <= 20.0f && angle > 20.0f)
		      {
		          // Llamar función para guardar las muestras del evento en EEPROM
		          Store_Event_Samples();

		          // Marcar que ya se registró el evento, evitar duplicados
		          triggered = 1;
		      }
		      // Detectar que el ángulo volvió a un valor bajo (≤20)
		      else if (triggered && angle <= 20.0f)
		      {
		          // Permitir registrar un nuevo evento la próxima vez que suba
		          triggered = 0;
		      }

		      // Guardar el valor actual como referencia para la próxima iteración
		      lastAngleY = angle;
		  }

		  QUINIENTOS_MS_FLAG += 5000;//Actualizamos bandera para cumplir con el tiempo
	  }

	  /* 1 sec TASK */
	  if(time - UN_SEGUNDO_FLAG == UN_SEGUNDO)
	  {


		  //streaming de canal 9
		  printf("VALOR ADC CANAL 9: %d \r\n", CH9);
		  //streaming de canal 14
		  printf("VALOR ADC CANAL 14: %d \r\n", CH14);
		  printf("-----------------------------------\r\n");



		  __HAL_TIM_SET_COUNTER(&htim1,0); //Reseteamos contador
		  //Reseteamos banderas de control de tiempo
		  CIEN_MS_FLAG = 0;
		  QUINIENTOS_MS_FLAG = 0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8400 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10500 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin BUTTON3_Pin BUTTON4_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin|BUTTON3_Pin|BUTTON4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*Read_ADC_Channels lee los canales de ADC uno por uno*/
void Read_ADC_Channels(void)
{
	for(int i = 0 ; i < NUM_OF_ADC_CHANNELS ; i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		*ADC_CHANNELS[i] = HAL_ADC_GetValue(&hadc1);
	}
}

/**
 * @brief Inicializa el sensor MPU6050 y detiene el programa si falla.
 */
void Init_MPU6050(void)
{
    if (MPU6050_Init(&hi2c1) != HAL_OK)
    {
        printf("Error initializing MPU6050\r\n");
        while (1);
    }
    printf("MPU6050 initialized!\r\n");
}

/**
 * @brief  Carga el encabezado de la EEPROM y actualiza la dirección actual de escritura.
 *         También imprime los eventos almacenados si el encabezado es válido.
 *
 * Esta función lee los primeros 2 bytes de la EEPROM, que contienen el número total de muestras guardadas.
 * Si el valor leído está corrupto (por ejemplo, 0xFFFF o fuera de rango), se reinicia la dirección de escritura y se limpia el encabezado.
 * Si es válido, se calcula la dirección de escritura y se llama a `Print_Stored_Events()` para mostrar los datos guardados.
 *
 * @param  Ninguno
 * @retval Ninguno
 */
void Load_EEPROM_Header(void)
{
    uint8_t header[2]; // Arreglo para almacenar los 2 bytes del encabezado

    // Leer los 2 bytes iniciales que representan el número total de muestras guardadas
    if (AT24C128C_ReadBuffer(&hi2c1, EEPROM_HEADER_ADDR, header, 2) == HAL_OK)
    {
        // Reconstruir el número de muestras a partir de los dos bytes
        uint16_t numSamples = (header[0] << 8) | header[1];

        // Validar si el valor leído es inválido o está fuera del rango esperado
        if (numSamples == 0xFFFF || numSamples > EEPROM_MAX_SAMPLES)
        {
            printf("⚠️ Header inválido o EEPROM corrupta: numSamples = %u. Reiniciando...\r\n", numSamples);

            numSamples = 0; // Reiniciar el contador de muestras
            eepromAddr = EEPROM_DATA_START; // Volver al inicio de la sección de datos

            // Limpiar el encabezado escribiendo ceros
            uint8_t clean[2] = {0x00, 0x00};
            AT24C128C_WriteBuffer(&hi2c1, EEPROM_HEADER_ADDR, clean, 2);
        }
        else
        {
            // Si el número de muestras es válido, calcular la próxima dirección de escritura
            eepromAddr = EEPROM_DATA_START + numSamples * 4;
        }

        // Mostrar por consola cuántas muestras hay almacenadas y hasta qué dirección
        printf("EEPROM contiene %u muestras hasta addr: 0x%04X\r\n", numSamples, eepromAddr);

        // Imprimir los eventos guardados en grupos de 3 muestras
        Print_Stored_Events(numSamples);
    }
    else
    {
        // Si falló la lectura del encabezado, iniciar desde cero
        eepromAddr = EEPROM_DATA_START;
    }
}


/**
 * @brief  Imprime por consola los eventos almacenados en la EEPROM.
 *
 * Cada evento consta de 3 muestras (floats), las cuales se leen desde EEPROM
 * y se imprimen en formato hexadecimal de dirección y valor decimal.
 * La función se asegura de no leer fuera del límite de la memoria EEPROM.
 *
 * @param  numSamples  Número total de muestras almacenadas (debe ser múltiplo de 3 para eventos completos).
 * @retval Ninguno
 */
void Print_Stored_Events(uint16_t numSamples)
{
    // Recorrer las muestras de 3 en 3 (cada grupo representa un evento)
    for (uint16_t i = 0; i < numSamples; i += 3)
    {
        // Imprimir encabezado del evento
        printf("Evento %u:\r\n", (i / 3) + 1);

        // Leer hasta 3 muestras por evento, asegurándose de no pasar el total
        for (uint8_t j = 0; j < 3 && (i + j) < numSamples; j++)
        {
            // Calcular la dirección de la EEPROM para esta muestra
            uint16_t addr = EEPROM_DATA_START + (i + j) * 4;

            // Verificar que la dirección no exceda el tamaño máximo de la EEPROM (128Kbits = 16 KB)
            if (addr + 4 > (128 * 1024 / 8)) break;

            // Leer 4 bytes (un float) desde la EEPROM
            if (AT24C128C_ReadBuffer(&hi2c1, addr, data, 4) == HAL_OK)
            {
                // Copiar los bytes leídos en la variable tipo float
                memcpy(&readValue, data, sizeof(float));

                // Imprimir solo si el valor no es NaN (por seguridad)
                if (!isnan(readValue))
                {
                    printf("  [0x%04X] = %.2f\r\n", addr, readValue);
                }
            }
        }
    }

    // Breve retraso al final para dar tiempo a leer los datos en consola
    HAL_Delay(3500);
}

/**
 * @brief  Guarda tres muestras consecutivas del ángulo Y en la EEPROM como un evento.
 *
 * Esta función se llama cuando se detecta una inclinación que supera el umbral (>20°).
 * Captura tres muestras del ángulo `mpu.angleY`, las almacena en EEPROM y actualiza el encabezado
 * con el número total de muestras registradas.
 *
 * @param  Ninguno (usa variables globales `mpu`, `eepromAddr`, `hi2c1`, `data`)
 * @retval Ninguno
 */
void Store_Event_Samples(void)
{
    // Mensaje indicativo en consola
    printf("\u26a1 Umbral superado! Registrando evento en EEPROM...\r\n");

    // Repetir 3 veces para almacenar 3 muestras del evento
    for (int i = 0; i < 3; i++)
    {
        float sample = mpu.angleY; // Tomar valor actual de inclinación (ángulo Y)

        // Copiar el valor float en un arreglo de 4 bytes para escribirlo
        memcpy(data, &sample, sizeof(float));

        // Escribir los 4 bytes del float en la dirección actual de EEPROM
        if (AT24C128C_WriteBuffer(&hi2c1, eepromAddr, data, 4) == HAL_OK)
        {
            // Confirmar en consola
            printf("  Guardado #%d: %.2f en addr 0x%04X\r\n", i + 1, sample, eepromAddr);

            // Avanzar dirección de escritura
            eepromAddr += 4;

            // Calcular el nuevo número total de muestras almacenadas
            uint16_t numSamples = (eepromAddr - EEPROM_DATA_START) / 4;

            // Crear nuevo encabezado con el contador actualizado
            uint8_t newHeader[2] = {
                (uint8_t)(numSamples >> 8),     // MSB
                (uint8_t)(numSamples & 0xFF)    // LSB
            };

            // Escribir el nuevo encabezado en la EEPROM
            AT24C128C_WriteBuffer(&hi2c1, EEPROM_HEADER_ADDR, newHeader, 2);
        }
        else
        {
            // Si falla la escritura, reportar el error y salir del ciclo
            printf("  Error escribiendo muestra #%d\r\n", i + 1);
            break;
        }

        // Pequeño retardo para permitir variación entre muestras
        HAL_Delay(100);

        // Leer nuevamente el sensor para obtener un nuevo valor de muestra
        MPU6050_ReadAll(&hi2c1, &mpu);
    }
}

void Read_EEPROM(void)
{
	printf("Botón presionado: leyendo eventos desde EEPROM...\r\n");

	uint8_t header[2];

	// Leer los primeros 2 bytes de la EEPROM donde se almacena el número total de muestras
	if (AT24C128C_ReadBuffer(&hi2c1, EEPROM_HEADER_ADDR, header, 2) == HAL_OK)
	{
		// Combinar los dos bytes en un entero de 16 bits
		uint16_t numSamples = (header[0] << 8) | header[1];

		// Verificar que el número de muestras no exceda el límite permitido
		if (numSamples <= EEPROM_MAX_SAMPLES)
		{
			// Imprimir todos los eventos almacenados en EEPROM
			Print_Stored_Events(numSamples);
		}
	}

	Leer_datos = 0;

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTick = HAL_GetTick(); // Tiempo actual en ms

    // Si han pasado al menos 150 ms desde la última pulsación
    if ((currentTick - lastButtonPressTick) < 150)
        return; // Rechazar evento por rebote (debounce)

    lastButtonPressTick = currentTick;

    if(GPIO_Pin == BUTTON1_Pin)
    {
    	HAL_GPIO_TogglePin(GPIOA, LED1_Pin);

    	Leer_datos = 1;

    }
    else if(GPIO_Pin == BUTTON2_Pin)
    {
    	HAL_GPIO_TogglePin(GPIOA, LED2_Pin);
    }
    else if(GPIO_Pin == BUTTON3_Pin)
    {
    	HAL_GPIO_TogglePin(GPIOA, LED3_Pin);
    }
    else if(GPIO_Pin == BUTTON4_Pin)
    {
    	HAL_GPIO_TogglePin(GPIOA, LED4_Pin);
    }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
