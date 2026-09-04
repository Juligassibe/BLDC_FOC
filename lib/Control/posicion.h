#ifndef POSICION_H
#define POSICION_H

#define PID_P			0.001f
#define PID_I			0.1f
#define PID_D			0.0001f

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
