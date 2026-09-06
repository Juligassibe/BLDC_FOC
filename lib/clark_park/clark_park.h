#ifndef CLARK_PARK_H
#define CLARK_PARK_H

#include <stdint.h>

typedef struct {
	float a;
	float b;
	float c;
} magnitud_abc_t;

typedef struct {
	float q;
	float d;
} magnitud_qd0_t;

uint32_t get_indice_LUT(float angulo);
float coseno(float angulo);
float seno(float angulo);
void clark_park_T(volatile magnitud_abc_t *in, volatile magnitud_qd0_t *out, float tita_e);
void inv_clark_park_T(volatile magnitud_qd0_t *in, volatile magnitud_abc_t *out, float tita_e);

#endif //CLARK_PARK_H
