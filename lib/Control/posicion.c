#include "posicion.h"
#include "tim.h"
#include "interpolador.h"
#include "corriente.h"

static const float invDT = 1.0f/DT;
static const float KT = 1.5f * PP * LAMBDA;
static const float PID_MAX = KT * I_MAX;

static controlador_posicion_t controlador = {
	.Kp = PID_P,
	.Ki = PID_I,
	.Kd = PID_D
};

float get_posicion() {
	/*
	 * NO DEJAR HARDCODEADO EL 0.18f DE RESOLUCION DEL ENCODER
	 * PONER EN FUNCION DE LOS ppr PUESTOS EN EL ENCODER
	 */
	return (float)((int32_t)(__HAL_TIM_GetCounter(&htim2) - INT32_MAX)) * 0.18f;
}

void lazo_posicion() {
	static float prev_position = 0.0f;
	static float prev_error = 0.0f;
	static float integral = 0.0f;

	// Sampleo posicion
	float posicion = get_posicion();
	float wm = (posicion - prev_position) * invDT;

	// Interpolo nueva consigna
	float consigna_posicion = interpolar();

	// Error de posicion
	float error = consigna_posicion - posicion;

	// P[n] = Kp*error[n]
	float proporcional = controlador.Kp * error;

	// I[n] = Ki*dt*(error[n]+error[n-1]) + I[n-1]
	float tempI = controlador.Ki * DT * (error + prev_error);
	integral += tempI;

	// D[n] = Kd*(-wm[n]) (No uso error en PI+D)
	float derivativa = controlador.Kd * (-wm);

	// Antiwindup
	float integral_max = 0.0f;
	float integral_min = 0.0f;

	// Verifico que no haya saturacion positiva solo con parte P+D, si satura Imax=0
	if (PID_MAX > proporcional + derivativa) {
		integral_max = PID_MAX - proporcional - derivativa;
	} else {
		integral_max = 0;
	}

	// Verifico que no haya saturacion negativa solo con parte P+D, si satura Imin=0
	if (-PID_MAX < proporcional + derivativa) {
		integral_min = -PID_MAX - proporcional - derivativa;
	} else {
		integral_min = 0;
	}

	// Checkeo que parte integral este en el rango [Imin, Imax]
	if (integral > integral_max) {
		integral = integral_max;
	} else if (integral < integral_min) {
		integral = integral_min;
	}

	// T*[n] = P[n] + I[n] + D[n]
	controlador.consigna_torque = proporcional + integral + derivativa;

	// Verifico que la consigna de torque no sea mayor que PID_MAX
	if (controlador.consigna_torque > PID_MAX) {
		controlador.consigna_torque = PID_MAX;
	} else if (controlador.consigna_torque < -PID_MAX) {
		controlador.consigna_torque = -PID_MAX;
	}
}

float get_consigna_torque() {
	return controlador.consigna_torque;
}