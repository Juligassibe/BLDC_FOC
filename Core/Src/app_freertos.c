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
#include <string.h>

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
QueueHandle_t cola_estados;
estados_e estado = INIT;

/* USER CODE END Variables */
/* Definitions for tarea_consola */
osThreadId_t tarea_consolaHandle;
const osThreadAttr_t tarea_consola_attributes = {
	.name = "tarea_consola",
	.priority = (osPriority_t)osPriorityNormal,
	.stack_size = 512 * 4
};
/* Definitions for tarea_sm */
osThreadId_t tarea_smHandle;
const osThreadAttr_t tarea_sm_attributes = {
	.name = "tarea_sm",
	.priority = (osPriority_t)osPriorityHigh4,
	.stack_size = 512 * 4
};
/* Definitions for tarea_errores */
osThreadId_t tarea_erroresHandle;
const osThreadAttr_t tarea_errores_attributes = {
	.name = "tarea_errores",
	.priority = (osPriority_t)osPriorityHigh,
	.stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void consola(void *argument);
void maquina_estados(void *argument);
void handler_errores(void *argument);

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
	cola_estados = xQueueCreate(3, sizeof(estados_e));

	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of tarea_consola */
	tarea_consolaHandle = osThreadNew(consola, NULL, &tarea_consola_attributes);

	/* creation of tarea_sm */
	tarea_smHandle = osThreadNew(maquina_estados, NULL, &tarea_sm_attributes);

	/* creation of tarea_errores */
	tarea_erroresHandle = osThreadNew(handler_errores, NULL, &tarea_errores_attributes);

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

	xQueueReceive(cola_consola, &mensaje, portMAX_DELAY);

	/* Infinite loop */
	while (1) {
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"\r\n>> ", 5);

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
				mover(0.0f);
				break;

			case 'l':
				leer_posicion();
				break;

			case 'a':
				calibrar_adcs();
				break;

			case 'h':
				char msj[] = "Ayuda:"
						"	e: Estado del sistema\r\n"
						"	i: Iniciar lazos de control\r\n"
						"	p: Parar lazos de control\r\n"
						"	m X: Mover a posicion X\r\n"
						"	c: Mover a cero\r\n"
						"	l: Leer posicion\r\n"
						"	a: Calibrar ADCs\r\n";

				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msj, strlen(msj));
				break;

			default:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Comando desconocido (h para ayuda)\r\n", 36);
				break;
		}
	}
	/* USER CODE END consola */
}

/* USER CODE BEGIN Header_maquina_estados */
/**
* @brief Function implementing the task_sm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_maquina_estados */
void maquina_estados(void *argument) {
	/* USER CODE BEGIN maquina_estados */

	/* Infinite loop */
	while (1) {
		switch (estado) {
			case INIT:
				init_sistema();
				break;

			case IDLE:
				mensaje_cli_t mensaje;
				xQueueSend(cola_consola, &mensaje, 100);
				break;

			case PARADA:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Parada emergencia\r\n", 15);
				enviar_cola_estados(FALLA);
				break;

			case CONTROL:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Control iniciado\r\n", 18);
				break;

			case FALLA:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Reiniciar sistema...\r\n", 29);
				break;
		}

		xQueueReceive(cola_estados, &estado, portMAX_DELAY);
	}
	/* USER CODE END maquina_estados */
}

/* USER CODE BEGIN Header_handler_errores */
/**
* @brief Function implementing the tarea_errores thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handler_errores */
void handler_errores(void *argument) {
	/* USER CODE BEGIN handler_errores */

	return_codes_e error;

	/* Infinite loop */
	while (1) {
		xQueueReceive(cola_errores, &error, portMAX_DELAY);

		switch (error) {
			case ADC1_CAL:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Cal ADC1\r\n", 17);
				break;

			case ADC2_CAL:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Cal ADC2\r\n", 17);
				break;

			case ADC2_START:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Start ADC2\r\n", 19);
				break;

			case ADC_MULTIMODE:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: ADC Multimode\r\n", 22);
				break;

			case ADC_OFF:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Lecturas ADCs nulas\r\n", 28);
				break;

			case PWM_CH1:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: PWM CH1\r\n", 16);
				break;

			case PWM_CH2:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: PWM CH2\r\n", 16);
				break;

			case PWM_CH3:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: PWM CH3\r\n", 16);
				break;

			case TIM_POS_START:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: TIM posicion\r\n", 21);
				break;

			case TIM_ENC_START:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: TIM encoder\r\n", 20);
				break;

			case ENCODER_CONFIG:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Configuracion encoder\r\n", 30);
				break;

			case ENCODER_ZERO:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Zero Encoder\r\n", 21);
				break;

			default:
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Error: Desconocido\r\n", 20);
				break;
		}

		enviar_cola_estados(FALLA);
	}
	/* USER CODE END handler_errores */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void enviar_cola_estados(estados_e estado) {
	estados_e estado_nuevo = estado;
	xQueueSend(cola_estados, &estado_nuevo, 100);
}

void enviar_cola_fallas(return_codes_e error) {
	return_codes_e error_nuevo = error;
	xQueueSend(cola_errores, &error_nuevo, 100);
}

/* USER CODE END Application */
