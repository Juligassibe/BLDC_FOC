#ifndef INTERPOLADOR_H
#define INTERPOLADOR_H

#include <stdint.h>

typedef struct {
	// Config
	float dt;				// Delta de tiempo
	float v_max;				// Velocidad max rad/s
	float a_max;				// Aceleracion max rad/s^2

	// Perfil actual
	float x0;					// Posicion cuando llega una consigna de posicion nueva
	float xf;					// Posicion final deseada
	float dir;					// Sentido de movimiento
	float distancia;			// Abs(desplazamiento)
	float v_pico;				// Velocidad pico para un desplazamiento dado, v_pico <= v_max
	float d_acc;
	float t_acc;				// Tiempo de aceleracion + tiempo desaceleracion
	float t_const;				// Tiempo a velocidad constante
	float t_total;				// Tiempo total para llegar a la consigna: 2*t_acc + t_const

	uint32_t n;					// Tiempo actual relativo al inicio de la interpolacion
	float consigna;	// Consigna de posicion interpolada para alimentar controlador
} interpolador_t;

void interpolador_init(interpolador_t *ip, float dt, float v_max, float a_max);
void interpolador_nueva_xf(interpolador_t *ip, float x0, float xf);
float interpolador_get_consigna(interpolador_t *ip);
void set_v_max(interpolador_t *ip, float v_max);
void set_a_max(interpolador_t *ip, float a_max);

#endif //INTERPOLADOR_H
