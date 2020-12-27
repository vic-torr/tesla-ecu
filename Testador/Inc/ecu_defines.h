/*
 * ecu_defines.h
 * Configura parametros gerais do ECU.
 *
 */

#define num_amostras_filtro 	            10			//Amostra media movel acelerador
#define TORQUE_MAX 			                1000			//em decimos de percentagem do torque nom
#define torq_frenagem 			            750				//Em decimo de percentagem
#define vel_max_rpm 		                4000
#define	vetorizacao_torque	             	0
#define	frenagem_regenerativa	            1
#define	modo_aceleracao	              		0
#define	modo_enduro			                0
#define zona_morta_acelerador          		10
#define TORQUE_GAIN 1
#define ZERO_VOLANTE 2240
#define GAIN_VOLANTE 16.4
#define GAIN_ARRANC 1.1
#define INC_VELOC 50

// defines rampa de torque
#define INC_TORQUE 250						// inclinacao 2a parte da rampa
#define INC_TORQUE_INIT 100					// inclinacao 1a parte
#define TORQUE_INIT_LIMITE 150				// ponto de transicao entre as rampas

// defines controle de tracao
#define DECR_OVER 50						// decrescimo de torque em caso de overshoot de wheel slip
#define DECR_UNDER 10						// decrescimo de torque para reduzir overshoot de wheel slip
#define UPPER_THRESHOLD 18					// valor que determina overshoot
#define LOWER_THRESHOLD 13					// valor que determina undershoot
#define UNDER_MAX_LIMIT 12					// limite superior para undershoot
#define UNDER_MIN_LIMIT	9					// limite inferior para undershoot

/////////////////defines de pid//////////////////////////////////
#define KP 2
#define KI 0
#define KD 0
#define GAIN_PID 1
#define MAX_INTEGRAL_ERROR 10
#define SAMPLE_TIME 20 //em ms
