#include "current_sensing.h"

static const float adc_scale = 3.0f / (ADC_3A - ADC_0A);
static int16_t offset_adc1 = 0;
static int16_t offset_adc2 = 0;
static currents_t corrientes = {0};

void set_adc_offsets() {
	HAL_Delay(200);
	uint16_t adc1 = (uint16_t)(raw_adcs & 0xFFFF);
	uint16_t adc2 = (uint16_t)((raw_adcs >> 16) & 0xFFFF);

	offset_adc1 = ADC_0A - adc1;
	offset_adc2 = ADC_0A - adc2;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint16_t adc1 = (uint16_t)(raw_adcs & 0xFFFF);
	uint16_t adc2 = (uint16_t)((raw_adcs >> 16) & 0xFFFF);

	float xn1 = (float)(adc1 + offset_adc1 - ADC_0A) * adc_scale;
	float xn2 = (float)(adc2 + offset_adc2 - ADC_0A) * adc_scale;

	corrientes.cur1 = corrientes.cur1 + IIR_ALPHA * (xn1 - corrientes.cur1);
	corrientes.cur2 = corrientes.cur2 + IIR_ALPHA * (xn2 - corrientes.cur2);
	corrientes.cur3 = -(corrientes.cur1 + corrientes.cur2);
}