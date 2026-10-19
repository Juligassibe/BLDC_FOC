#ifndef POSICION_H
#define POSICION_H

#include <stdint.h>

#define ENCODER_CNT_ZERO	INT32_MAX

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float consigna_torque;
	float consigna;
} controlador_posicion_t;

float get_posicion();
float get_posicion_deg();
void lazo_posicion();
float get_consigna_torque();
void pos_set_P(float P);
void pos_set_I(float I);
void pos_set_D(float D);

#endif //POSICION_H
