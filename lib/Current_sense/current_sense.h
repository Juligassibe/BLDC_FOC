#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>
#include "clark_park.h"

#define ADC_0A			3103
#define ADC_3A			3792
#define IIR_ALPHA		0.015f

extern uint32_t raw_adcs;

void set_adc_offsets();
void get_corrientes_fase(magnitud_abc_t *out);

#endif //CURRENT_SENSE_H
