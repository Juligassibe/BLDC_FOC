#ifndef USER_CONSTANTS_H
#define USER_CONSTANTS_H

#define MY_PI 3.14159265358979323846f

// ASEGURARSE QUE PPR SEA PAR YA QUE AL CONFIGURAR ENCODER SE DIVIDE POR 2 PORQUE EL TIMER CUENTA FLANCO DE SUBIDA Y DE BAJADA
#define ENCODER_PPR 2000

#define POLO_CORRIENTE 5000		// s = -5000

#define PID_P			0.005f
#define PID_I			0.01f
#define PID_D			0.000001f

#define PP				7		// Pares de polos
#define R_FASE 0.07f				// Rs = 0.07 ohms
#define LQ 0.0002f			// Lq = 0.6 mH
#define LD 0.00016f			// Ld = 0.5 mH
#define LAMBDA 0.0001f		// Flujo concatenado

#define DT		0.001f
#define V_MAX	360.0f
#define A_MAX	1080.0f

#endif //USER_CONSTANTS_H
