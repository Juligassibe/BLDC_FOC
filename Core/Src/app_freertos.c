/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "usart.h"
#include "general.h"

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
/* USER CODE BEGIN Variables */

QueueHandle_t cola_consola;
QueueHandle_t cola_errores;

/* USER CODE END Variables */
/* Definitions for tarea_consola */
osThreadId_t tarea_consolaHandle;
const osThreadAttr_t tarea_consola_attributes = {
	.name = "tarea_consola",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void consola(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	cola_consola = xQueueCreate(5, sizeof(mensaje_cli_t));
	cola_errores = xQueueCreate(3, sizeof(return_codes_e));

	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of tarea_consola */
	tarea_consolaHandle = osThreadNew(consola, NULL, &tarea_consola_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_consola */
/**
  * @brief  Tarea para dar interfaz CLI al controlador
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_consola */
void consola(void *argument) {
	/* USER CODE BEGIN consola */

	mensaje_cli_t mensaje = {0};

	/* Infinite loop */
	while (1) {
		xQueueReceive(cola_consola, &mensaje, portMAX_DELAY);

		switch (mensaje.mensaje[0]) {
			case 'e':
				estado_sistema();
				break;

			case 'i':
				iniciar_lazos();
				break;

			case 'p':
				parar_lazos();
				break;

			case 'm':
				char *fin = &mensaje.mensaje[2];
				float angulo = strtof(&mensaje.mensaje[2], &fin);

				// VERIFICO QUE EL COMANDO "m ..." CONTENGA UN ANGULO VALIDO QUE SE PUEDA LEER COMO FLOAT
				if (*fin == mensaje.mensaje[2]) {
					HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Verificar angulo enviado\r\n", 26);
					break;
				}

				mover(angulo);
				break;

			case 'c':
				mover_a_cero();
				break;

			case 'l':
				leer_posicion();
				break;

			case 'a':
				calibrar_adcs();
				break;

			default:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Desconocido\r\n", 13);
				break;
		}
	}
	/* USER CODE END consola */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
