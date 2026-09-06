#ifndef POSICION_H
#define POSICION_H

#include <stdint.h>

#define PID_P			0.0001f
#define PID_I			0.01f
#define PID_D			0.00001f
#define ENCODER_CNT_ZERO	INT32_MAX

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float consigna_torque;
	float consigna;
} controlador_posicion_t;

float get_posicion();
void lazo_posicion();
float get_consigna_torque();

#endif //POSICION_H
