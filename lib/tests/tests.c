#include "tests.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "tim.h"
#include "usart.h"
#include "clark_park.h"

static char cadena[128] = {0};
static uint8_t n = 0;
float angulo_recibido = 0.0f;

void test_lut_vs_math_h() {
	volatile float angulo = 0.0f;
	volatile float temp = 0.0f;
	uint32_t fin = 0;

	// COSENO CON LUT
	tim5_of = 0;
	__HAL_TIM_SetCounter(&htim5, 0);
	for (int i = 0; i < 360; i++) {
		temp = coseno(angulo);
		angulo += 1.0f;
	}
	fin = __HAL_TIM_GET_COUNTER(&htim5);

	n = snprintf(cadena, sizeof(cadena), "LUT: %lu\r\n", fin+tim5_of*109999999);
	HAL_UART_Transmit(&huart4, (uint8_t*)cadena, n, 1000);

	// COSENO math.h
	angulo = 0.0f;
	tim5_of = 0;
	__HAL_TIM_SetCounter(&htim5, 0);
	for (int i = 0; i < 360; i++) {
		temp = cosf(angulo);
		angulo += 1.0f;
	}
	fin = __HAL_TIM_GET_COUNTER(&htim5);

	n = snprintf(cadena, sizeof(cadena), "math.h: %lu\r\n", fin+tim5_of*109999999);
	HAL_UART_Transmit(&huart4, (uint8_t*)cadena, n, 1000);
}

void test_coseno() {
	static float prev = 0.0f;

	if (angulo_recibido != prev) {
		float coseno_calculado = coseno(angulo_recibido);

		n = snprintf(cadena, sizeof(cadena), "%f\r\n", coseno_calculado);
		HAL_UART_Transmit(&huart4, (uint8_t*)cadena, n, 1000);
		prev = angulo_recibido;
	}
}

void test_seno() {
	static float prev = 0.0f;

	if (angulo_recibido != prev) {
		float seno_calculado = seno(angulo_recibido);

		n = snprintf(cadena, sizeof(cadena), "%f\r\n", seno_calculado);
		HAL_UART_Transmit(&huart4, (uint8_t*)cadena, n, 1000);
		prev = angulo_recibido;
	}
}