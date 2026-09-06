#include "posicion.h"
#include "tim.h"
#include "interpolador.h"
#include "corriente.h"
#include "user_constants.h"

static const float invDT = 1.0f / DT;
static const float KT = 1.5f * PP * LAMBDA;
static const float PID_MAX = KT * I_MAX;
static const float STEP2DEG = 360.0f / ENCODER_PPR;

static volatile controlador_posicion_t controlador_posicion = {
	.Kp = PID_P,
	.Ki = PID_I,
	.Kd = PID_D,
	.consigna_torque = 0,
	.consigna = 0
};
static volatile float posicion = 0;

float get_posicion() {
	/*
	 * NO DEJAR HARDCODEADO EL 0.18f DE RESOLUCION DEL ENCODER
	 * PONER EN FUNCION DE LOS ppr PUESTOS EN EL ENCODER
	 */
	return (float)((int32_t)(__HAL_TIM_GetCounter(&htim2) - ENCODER_CNT_ZERO)) * STEP2DEG;
}

void lazo_posicion() {
	static float prev_position = 0.0f;
	static float prev_error = 0.0f;
	static float integral = 0.0f;

	// Sampleo posicion
	posicion = get_posicion();
	float wm = (posicion - prev_position) * invDT;

	// Interpolo nueva consigna
	controlador_posicion.consigna = interpolar();

	// Error de posicion
	float error = controlador_posicion.consigna - posicion;

	// P[n] = Kp*error[n]
	float proporcional = controlador_posicion.Kp * error;

	// I[n] = Ki*dt*(error[n]+error[n-1])/2 + I[n-1]
	float tempI = controlador_posicion.Ki * DT * (error + prev_error) * 0.5f;
	integral += tempI;

	// D[n] = Kd*(-wm[n]) (No uso error en PI+D)
	float derivativa = controlador_posicion.Kd * (-wm);

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
	controlador_posicion.consigna_torque = proporcional + integral + derivativa;

	// Verifico que la consigna de torque no sea mayor que PID_MAX
	// Ya que el controlador puede saturar solo con P+D
	if (controlador_posicion.consigna_torque > PID_MAX) {
		controlador_posicion.consigna_torque = PID_MAX;
	} else if (controlador_posicion.consigna_torque < -PID_MAX) {
		controlador_posicion.consigna_torque = -PID_MAX;
	}

	prev_position = posicion;
	prev_error = error;
}

float get_consigna_torque() {
	return controlador_posicion.consigna_torque;
}