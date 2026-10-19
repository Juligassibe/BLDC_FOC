#ifndef CORRIENTE_H
#define CORRIENTE_H

#include <stdint.h>

#include "clark_park.h"

#define DTi				0.0001f	// Periodo de lazo de corriente
#define DEG2RAD			0.0174533f

#define VCC 12.0f
#define I_MAX 12.0f
#define I_LIM 13.0f

typedef struct {
	float Rs;
	float Lq;
	float Ld;
	float lambda;
} motor_specs_t;

typedef struct {
	// Constantes controlador
	float Pq;
	float Pd;

	magnitud_qd0_t corrientes_qd0;
	magnitud_qd0_t consigna_corrientes_qd0;
	magnitud_qd0_t consigna_tension_qd0;
	magnitud_abc_t consigna_tension_fase;
} controlador_corriente_t;

void lazo_corriente();
void cur_set_Pq(float Pq);
void cur_set_Pd(float Pd);

#endif //CORRIENTE_H
