#include "interpolador.h"
#include <math.h>			// Para sqrtf()

#include "user_constants.h"

void interpolador_init(interpolador_t *ip, float dt, float v_max, float a_max) {
	ip->dt       = dt;
	ip->v_max    = v_max * MY_PI / 180.0f;
	ip->a_max    = a_max * MY_PI / 180.0f;
	ip->consigna = 0.0f;
	ip->n        = 0;
}

void interpolador_nueva_xf(interpolador_t *ip, float x0, float xf) {
	const float d = xf - x0;

	ip->x0 = x0;
	ip->xf = xf;
	ip->distancia = (d >= 0.0f) ? d : -d;
	ip->dir = (d >= 0.0f) ? 1.0f : -1.0f;
	ip->consigna = x0;

	// Distancia de acelerado + distancia de frenado
	const float d_acc_fren = ip->v_max * ip->v_max / ip->a_max;

	if (d_acc_fren >= ip->distancia) {	// Perfil triangular, v_pico < v_max
		ip->t_acc = sqrtf(ip->distancia / ip->a_max);
		ip->t_const = 0.0f;
		ip->v_pico = ip->a_max * ip->t_acc;
	} else {							// Perfil trapezoidal, v_pico = v_max
		ip->t_acc = ip->v_max / ip->a_max;
		ip->t_const = (ip->distancia - d_acc_fren) / ip->v_max;
		ip->v_pico = ip->v_max;
	}

	ip->d_acc = 0.5f * ip->a_max * ip->t_acc * ip->t_acc;
	ip->t_total = 2.0f * ip->t_acc + ip->t_const;
	ip->n = 0;
}

float interpolador_get_consigna(interpolador_t *ip) {
	ip->n++;
	const float t = (float)ip->n * ip->dt;
	float dx;

	if (t <= ip->t_acc) {						// Aceleracion de 0 a v_pico
		// dx[n] = 0.5*a*t[n]^2
		dx = 0.5f * ip->a_max * t * t;
	} else if (t <= ip->t_acc + ip->t_const) {	// Velocidad constante
		// dx[n] = 0.5*a*t_acc^2 + v*(t[n]-t_acc)
		dx = ip->d_acc + ip->v_pico * (t - ip->t_acc);
	} else if (t < ip->t_total) {				// Desaceleracion de v_pico a 0
		// dx[n] = abs(desplazamiento) - 0.5*a*(t_total-t[n])^2
		const float td = ip->t_total - t;
		dx = ip->distancia - 0.5f * ip->a_max * td * td;
	} else {
		ip->consigna = ip->xf;
		return ip->consigna;
	}

	ip->consigna = ip->x0 + dx * ip->dir;
	return ip->consigna;
}

void set_v_max(interpolador_t *ip, float v_max) {
	ip->v_max = v_max * MY_PI / 180.0f;
}

void set_a_max(interpolador_t *ip, float a_max) {
	ip->a_max = a_max * MY_PI / 180.0f;
}