#ifndef GENERAL_H
#define GENERAL_H

typedef enum {
	INIT_OK = 0,
	ADC1_CAL,
	ADC2_CAL,
	ADC2_START,
	ADC_MULTIMODE,
	ADC_OFF,
	PWM_CH1,
	PWM_CH2,
	PWM_CH3,
	TIM_POS_START,
	TIM_ENC_START,
	ENCODER_CONFIG,
	ENCODER_ZERO,
	OVERCURRENT
} error_codes_e;

typedef enum {
	INIT = 0,
	IDLE,
	PARADA,
	CONTROL,
	FALLA
} estados_e;

// INICIALIZACION PERIFERICOS
error_codes_e init_pwm();
error_codes_e init_adcs();
error_codes_e init_timer_pos();
error_codes_e init_timer_encoder();
error_codes_e alinear_rotor();
void init_sistema();

// INTERFAZ CONSOLA
void estado_sistema();
void iniciar_lazos();
void parar_lazos();
void parada_emergencia();
void mover(float angulo);
void leer_posicion();
void calibrar_adcs();

void handle_sobrecorriente();

#endif