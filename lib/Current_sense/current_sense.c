#include "current_sense.h"
#include "adc.h"

static int16_t offset_adc1 = 0;
static int16_t offset_adc2 = 0;

static const float adc_scale = 3.0f / (ADC_3A - ADC_0A);
static volatile magnitud_abc_t corrientes_fase = {0};

uint32_t raw_adcs = 0;

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

void get_corrientes_fase(magnitud_abc_t *out) {
	out->a = corrientes_fase.a;
	out->b = corrientes_fase.b;
	out->c = corrientes_fase.c;
}