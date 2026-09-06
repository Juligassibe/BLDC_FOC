#ifndef CORRIENTE_H
#define CORRIENTE_H

#include <stdint.h>

#define ADC_0A			3103
#define ADC_3A			3792
#define IIR_ALPHA		0.015f
#define DTi				0.0001f	// Periodo de lazo de corriente
#define DEG2RAD			0.0174533f

#define VCC 12.0f
#define I_MAX 12.0f
#define I_LIM 13.0f

#define POLO_CORRIENTE 5000		// s = -5000

#define PP				7		// Pares de polos
#define R_FASE 0.1				// Rs = 0.1 ohms
#define LQ 0.00003052			// Lq = 0.3 mH
#define LD 0.00002289			// Ld = 0.2 mH
#define LAMBDA 0.0001f	// Flujo concatenado

typedef struct {
	float Pq;
	float Pd;
} controlador_corriente_t;

typedef struct {
	float Rs;
	float Lq;
	float Ld;
	float lambda;
} motor_specs_t;

extern uint32_t raw_adcs;

void set_adc_offsets();
void lazo_corriente();

#endif //CORRIENTE_H
