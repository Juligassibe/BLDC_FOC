#include "corriente.h"

#include <string.h>

#include "adc.h"
#include "tim.h"
#include "posicion.h"
#include "current_sense.h"
#include "user_constants.h"

static const float inv_Kt = 1.0f / (1.5f * PP * LAMBDA);
static const float invDti = 1.0f / DTi;
static const float inv_VCC = 1.0f / VCC;

static controlador_corriente_t controlador = {
	.Pq = POLO_CORRIENTE * LQ,
	.Pd = POLO_CORRIENTE * LD
};

static motor_specs_t motor = {
	.Rs = R_FASE,
	.Lq = LQ,
	.Ld = LD,
	.lambda = LAMBDA
};

void lazo_corriente() {
	static float prev_tita_m = 0;

	// Sampleo corrientes
	magnitud_abc_t corrientes_fase = {0};
	get_corrientes_fase(&corrientes_fase);

	// Check sobrecorriente
	if (corrientes_fase.a > I_LIM || corrientes_fase.a < -I_LIM ||
		corrientes_fase.b > I_LIM || corrientes_fase.b < -I_LIM ||
		corrientes_fase.c > I_LIM || corrientes_fase.c < -I_LIM) {

		// Envio error al handler de errores
		error_codes_e error = OVERCURRENT;

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(cola_errores, (void *)&error, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		return;
	}

	float tita_m = get_posicion_deg();
	float wm = (tita_m - prev_tita_m) * invDti * DEG2RAD;
	float tita_e = tita_m * PP;

	clark_park_T(&corrientes_fase, &controlador.corrientes_qd0, tita_e);

	// i_q*[n] = Tm*[n] / Kt
	controlador.consigna_corrientes_qd0.q = get_consigna_torque() * inv_Kt;

	// e_iq[n] = i_q*[n] - i_q[n]
	float error_iq = controlador.consigna_corrientes_qd0.q - controlador.corrientes_qd0.q;

	// vq*[n] = Pq * e_iq[n] + caida ohmica + desacople id + caida BEMF
	controlador.consigna_tension_qd0.q = controlador.Pq * error_iq +
						controlador.corrientes_qd0.q * motor.Rs +				// Caida ohmica
						PP * wm * motor.Ld * controlador.corrientes_qd0.d +		// Desacople id
						PP * motor.lambda * wm;									// Caida BEMF

	/*
	 * vd*[n] = Pd * (-id[n]) + caida ohmica + desacople iq
	 * ed[n] = -id[n] ya que la consigna de id es 0
	 */

	controlador.consigna_tension_qd0.d = -controlador.Pd * controlador.corrientes_qd0.d +
										  controlador.corrientes_qd0.d * motor.Rs -
										  PP * wm * motor.Lq * controlador.corrientes_qd0.q;

	inv_clark_park_T(&controlador.consigna_tension_qd0, &controlador.consigna_tension_fase, tita_e);

	float duty_a = 0.5f + controlador.consigna_tension_fase.a * inv_VCC;
	float duty_b = 0.5f + controlador.consigna_tension_fase.b * inv_VCC;
	float duty_c = 0.5f + controlador.consigna_tension_fase.c * inv_VCC;

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint32_t)(duty_a * __HAL_TIM_GetAutoreload(&htim3)));
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (uint32_t)(duty_b * __HAL_TIM_GetAutoreload(&htim3)));
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (uint32_t)(duty_c * __HAL_TIM_GetAutoreload(&htim3)));

	prev_tita_m = tita_m;
}

void cur_set_Pq(float Pq) {
	controlador.Pq = Pq;
}

void cur_set_Pd(float Pd) {
	controlador.Pd = Pd;
}