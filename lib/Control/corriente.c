#include "corriente.h"
#include "adc.h"
#include "tim.h"
#include "clark_park.h"
#include "posicion.h"

static const float adc_scale = 3.0f / (ADC_3A - ADC_0A);
static const float inv_Kt = 1.0f / (1.5f * PP * LAMBDA);
static const float invDti = 1.0f / DTi;
static const float inv_VCC = 1.0f / VCC;

static int16_t offset_adc1 = 0;
static int16_t offset_adc2 = 0;

static magnitud_abc_t corrientes_fase = {0};
static magnitud_qd0_t corrientes_qd0 = {0};
static magnitud_abc_t cons_tension_fase = {0};
static magnitud_qd0_t cons_tension_qd0 = {0};

static motor_specs_t motor = {
	.Rs = R_FASE,
	.Lq = LQ,
	.Ld = LD,
	.lambda = LAMBDA
};

static controlador_corriente_t controlador_corriente = {
	.Pq = POLO_CORRIENTE * LQ,
	.Pd = POLO_CORRIENTE * LD,
	.consigna_iq = 0,
	.consigna_id = 0
};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint16_t adc1 = (uint16_t)(raw_adcs & 0xFFFF);
	uint16_t adc2 = (uint16_t)((raw_adcs >> 16) & 0xFFFF);

	float xn1 = (float)(adc1 + offset_adc1 - ADC_0A) * adc_scale;
	float xn2 = (float)(adc2 + offset_adc2 - ADC_0A) * adc_scale;

	corrientes_fase.a = corrientes_fase.a + IIR_ALPHA * (xn1 - corrientes_fase.a);
	corrientes_fase.b = corrientes_fase.b + IIR_ALPHA * (xn2 - corrientes_fase.b);
	corrientes_fase.c = -(corrientes_fase.a + corrientes_fase.b);
}

void set_adc_offsets() {
	// Delay para dejar que se estabilicen los ADCs
	HAL_Delay(200);
	uint32_t sum_adc1 = 0;
	uint32_t sum_adc2 = 0;

	for (int i = 0; i < 64; i++) {
		sum_adc1 += (uint16_t)(raw_adcs & 0xFFFF);
		sum_adc2 += (uint16_t)((raw_adcs >> 16) & 0xFFFF);
		HAL_Delay(2);
	}

	offset_adc1 = ADC_0A - (sum_adc1 >> 6);
	offset_adc2 = ADC_0A - (sum_adc2 >> 6);
}

void lazo_corriente() {
	static float prev_tita_m = 0;

	if (corrientes_fase.a > I_MAX || corrientes_fase.a < I_MIN ||
		corrientes_fase.b > I_MAX || corrientes_fase.b < I_MIN ||
		corrientes_fase.c > I_MAX || corrientes_fase.c < I_MIN) {

		// LUEGO CAMBIAR POR HANDLER ADECUADO
		HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
		__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
	}

	float tita_m = get_posicion();
	float wm = (tita_m - prev_tita_m) * invDti;
	float tita_e = tita_m * PP;

	clark_park_T(&corrientes_fase, &corrientes_qd0, tita_e);

	// i_q*[n] = Tm*[n] / Kt
	float consigna_iq = get_consigna_torque() * inv_Kt;

	// e_iq[n] = i_q*[n] - i_q[n]
	float error_iq = consigna_iq - corrientes_qd0.q;

	// vq*[n] = Pq * e_iq[n] + caida ohmica + desacople id + caida BEMF
	cons_tension_qd0.q = controlador_corriente.Pq * error_iq +
						corrientes_qd0.q * motor.Rs +				// Caida ohmica
						PP * wm * motor.Ld * corrientes_qd0.d +		// Desacople id
						PP * motor.lambda * wm;						// Caida BEMF

	/*
	 * vd*[n] = Pd * (-id[n]) + caida ohmica + desacople iq
	 * ed[n] = -id[n] ya que la consigna de id es 0
	 */

	cons_tension_qd0.d = -controlador_corriente.Pd * corrientes_qd0.d +
						corrientes_qd0.d * motor.Rs +					// Caida ohmica
						PP * wm * motor.Lq * corrientes_qd0.q;			// Desacople iq

	inv_clark_park_T(&cons_tension_qd0, &cons_tension_fase, tita_e);

	float duty_a = 0.5f + cons_tension_fase.a * inv_VCC;
	float duty_b = 0.5f + cons_tension_fase.b * inv_VCC;
	float duty_c = 0.5f + cons_tension_fase.c * inv_VCC;

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint32_t)(duty_a * __HAL_TIM_GetAutoreload(&htim3)));
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (uint32_t)(duty_b * __HAL_TIM_GetAutoreload(&htim3)));
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (uint32_t)(duty_c * __HAL_TIM_GetAutoreload(&htim3)));
}