#ifndef INTERPOLADOR_H
#define INTERPOLADOR_H

#define V_MAX	360.0f
#define A_MAX	1080.0f
#define DT		0.001f

typedef struct {
	float v_max;				// Velocidad max °/s
	float a_max;				// Aceleracion max °/s^2
	const float dt;				// Delta de tiempo
	float t;					// Tiempo actual relativo al inicio de la interpolacion
	float t_acc;				// Tiempo de aceleracion + tiempo desaceleracion
	float t_const;				// Tiempo a velocidad constante
	float t_total;				// 2*t_acc + t_const
	float posicion_inicial;		// Posicion cuando llega una consigna de posicion final nueva
	float posicion_final;		// Consigna de posicion final
	float distancia_total;		// abs(desplazamiento)
	float consigna_posicion;	// Consigna de posicion interpolada para alimentar controlador
} interpolador_t;

void consigna_nueva(float posicion_final);
float interpolar();
float misqrt(float x);

#endif //INTERPOLADOR_H
