#ifndef CURRENT_SENSING_H
#define CURRENT_SENSING_H

#include <stdio.h>
#include <stdint.h>

#include "usart.h"
#include "fixed_point.h"

#define ADC_0A			3103
#define ADC_3A			3792
#define IIR_ALPHA		0.015

typedef struct {
	float cur1;
	float cur2;
	float cur3;
} currents_t;

void set_adc_offsets();

#endif //CURRENT_SENSING_H
