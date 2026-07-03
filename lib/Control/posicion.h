#ifndef POSICION_H
#define POSICION_H

#define PP				7		// Pares de polos
#define LAMBDA			0.0001	// Flujo concatenado
#define Rs				0.2		// Resistencia de fase
#define PID_P			0.001
#define PID_I			0.1
#define PID_D			0.0001

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float consigna_torque;
} controlador_posicion_t;

float get_posicion();
void lazo_posicion();
float get_consigna_torque();

#endif //POSICION_H
