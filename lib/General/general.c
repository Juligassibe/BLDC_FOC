#include "general.h"
#include "adc.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "mt6835.h"
#include "posicion.h"
#include "corriente.h"
#include "interpolador.h"
#include "user_constants.h"

return_codes_e init_pwm() {
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
		return PWM_CH1;
	}

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
		return PWM_CH2;
	}

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) {
		return PWM_CH3;
	}

	return INIT_OK;
}

return_codes_e init_adcs() {
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
		return ADC1_CAL;
	}


	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
		return ADC2_CAL;
	}

	if (HAL_ADC_Start(&hadc2) != HAL_OK) {
		return ADC2_START;
	}

	if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, &raw_adcs, 1) != HAL_OK) {
		return ADC_MULTIMODE;
	}

	set_adc_offsets();

	if ((raw_adcs & 0xFFFF) == 0 ||
		((raw_adcs >> 16) & 0xFFFF) == 0) {
		return ADC_OFF;
	}

	return INIT_OK;
}

return_codes_e init_timer_pos() {
	if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
		return TIM_POS_START;
	}

	return INIT_OK;
}

return_codes_e init_timer_encoder() {
	if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
		return TIM_ENC_START;
	}

	return INIT_OK;
}

return_codes_e alinear_rotor() {
	// Alineo eje d con fase a
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0.03*__HAL_TIM_GET_AUTORELOAD(&htim3));
	HAL_Delay(100);

	encoder_t encoder_conf = {0};

	if (init_encoder(&hspi1, CS_MT6835_GPIO_Port, CS_MT6835_Pin, &encoder_conf, ENCODER_PPR / 2,
					 MT6835_ABZ_NO_SWAP,
					 MT6835_ZRE,
					 MT6835_Z_WIDTH_1LSB,
					 MT6835_Z_ARE,
					 MT6835_CCW_AB) != HAL_OK) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		return ENCODER_CONFIG;
	}

	// Hago 0 en encoder mientras eje d esta alineado con fase a
	if (mt6835_set_zero(&hspi1, CS_MT6835_GPIO_Port, CS_MT6835_Pin) != MT6835_ZERO_OK) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		return ENCODER_ZERO;
	}

	// Pongo CNT del timer en encoder mode en INT32_MAX
	__HAL_TIM_SetCounter(&htim2, ENCODER_CNT_ZERO);
	HAL_Delay(100);

	// Una vez alineado el rotor y puesto en 0 el encoder dejo de alimentar la fase A
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

	return INIT_OK;
}

void init_sistema() {
	return_codes_e init = INIT_OK;

	init = init_pwm();

	if (init != INIT_OK) {
		xQueueSend(cola_errores, &init, 100);
		return;
	}

	init = init_adcs();

	if (init != INIT_OK) {
		xQueueSend(cola_errores, &init, 100);
		return;
	}

	init = init_timer_pos();

	if (init != INIT_OK) {
		xQueueSend(cola_errores, &init, 100);
		return;
	}

	init = init_timer_encoder();

	if (init != INIT_OK) {
		xQueueSend(cola_errores, &init, 100);
		return;
	}

	init = alinear_rotor();

	if (init != INIT_OK) {
		xQueueSend(cola_errores, &init, 100);
		return;
	}

	uart_receive_dma(&huart4);

	/*
	 * Si el sistema inicializa bien paso a estado IDLE
	 * Cualquier falla es manejada en la tarea de fallas y pasa a estado FALLA
	*/
	enviar_cola_estados(IDLE);
}

void estado_sistema() {
	HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Estado\r\n", 8);
}

void iniciar_lazos() {
	switch (estado) {
		case IDLE:
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
			__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

			__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
			__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

			enviar_cola_estados(CONTROL);
			break;

		case CONTROL:
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Lazos de control ya iniciados\r\n", 31);
			break;

		default:
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Reiniciar sistema\r\n", 19);
			break;
	}
}

void parar_lazos() {
	switch (estado) {
		case CONTROL:
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
			__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);

			enviar_cola_estados(IDLE);
			break;

		case IDLE:
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Lazos de control ya detenidos\r\n", 31);
			break;

		default:
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Reiniciar sistema\r\n", 19);
			break;
	}
}

void parada_emergencia() {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	enviar_cola_estados(PARADA);
}

void mover(float angulo) {
	if (estado == CONTROL) {
		consigna_nueva(angulo);
	} else {
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Iniciar lazos de control\r\n", 26);
	}
}

void leer_posicion() {
	if (estado == CONTROL || estado == IDLE) {
		/*
		 * Si NO declaro como static al terminar la ejecucion de la funcion, mientras el DMA lee el buffer para transmitir los
		 * fatos por UART puede leer basura.
		*/
		static char cadena[16] = {0};
		uint8_t n = 0;

		n = snprintf(cadena, sizeof(cadena), "%.2f", get_posicion());
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)cadena, n);
	} else {
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Reiniciar sistema\r\n", 19);
	}
}

void calibrar_adcs() {
	if (estado == IDLE) {
		set_adc_offsets();
	} else {
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"Reiniciar sistema\r\n", 19);
	}
}