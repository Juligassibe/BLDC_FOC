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
#define LQ 0.0006f			// Lq = 0.6 mH
#define LD 0.0005f			// Ld = 0.5 mH
#define LAMBDA 0.0001f		// Flujo concatenado

#endif //USER_CONSTANTS_H
