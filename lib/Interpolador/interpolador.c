#include "interpolador.h"
#include <math.h>			// Para sqrtf()
#include "main.h"			// Para handle de perifericos
#include "posicion.h"		// Para get_posicion()

static interpolador_t interpolador = {
	.v_max = V_MAX,
	.a_max = A_MAX,
	.dt	   = DT
};

static const float distancia_acc = V_MAX*V_MAX/A_MAX;

void consigna_nueva(const float posicion_final) {
	interpolador.posicion_inicial = get_posicion();
	float desplazamiento = posicion_final - interpolador.posicion_inicial;
	interpolador.distancia_total = (desplazamiento > 0) ? desplazamiento : -desplazamiento;

	// Perfil triangular
	if (distancia_acc >= interpolador.distancia_total) {
		interpolador.t_acc = sqrtf(interpolador.distancia_total/interpolador.a_max);
		interpolador.t_const = 0;
		interpolador.v_max = interpolador.a_max * interpolador.t_acc;
	}

	// Perfil trapezoidal
	else {
		interpolador.t_acc = V_MAX / A_MAX;
		interpolador.v_max = V_MAX;
		interpolador.t_const = (interpolador.distancia_total - distancia_acc) / V_MAX;
	}

	interpolador.t = 0;															// Reseteo tiempo relativo
	interpolador.t_total = 2*interpolador.t_acc + interpolador.t_const;
	interpolador.v_max *= (desplazamiento*interpolador.v_max > 0) ? 1 : -1;		// Verifico que sign(v)=sign(desp)
	interpolador.a_max *= (desplazamiento*interpolador.a_max > 0) ? 1 : -1;		// Verifico que sign(a)=sign(desp)
	interpolador.posicion_final = posicion_final;
}

float interpolar() {
	interpolador.t += interpolador.dt;

	// Aceleracion
	if (interpolador.t <= interpolador.t_acc) {
		HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);
		// x[n] = x[0] + 0.5*a*t[n]^2
		interpolador.consigna_posicion = interpolador.posicion_inicial + 0.5f * interpolador.a_max * interpolador.t * interpolador.t;
	}

	// Velocidad constante
	else if (interpolador.t <= interpolador.t_acc + interpolador.t_const) {
		HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET);
		// x[n] = x[0] + 0.5*a*t_acc^2 + v*(t[n]-t_acc)
		interpolador.consigna_posicion = interpolador.posicion_inicial + 0.5f * interpolador.a_max * interpolador.t_acc * interpolador.t_acc +
										 interpolador.v_max * (interpolador.t - interpolador.t_acc);
	}

	// Desaceleracion
	else if (interpolador.t <= interpolador.t_total) {
		HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, GPIO_PIN_SET);
		// x[n] = x[final] - 0.5*a*(t_total-t[n])^2
		interpolador.consigna_posicion = interpolador.posicion_final - 0.5f * interpolador.a_max * (interpolador.t_total - interpolador.t) *
																								   (interpolador.t_total - interpolador.t);
	}

	return interpolador.consigna_posicion;
}