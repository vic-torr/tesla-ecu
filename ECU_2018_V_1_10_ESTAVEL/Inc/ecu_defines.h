/*
 * ecu_defines.h
 * Configura parametros gerais do ECU.
 *
 */

#define SET_WDG 0xCCCC
#define REFRESH_WDG 0xAAAA
#define PR_WDG 1 //na verdade 1
#define RELOAD_WDG 2500

#define num_amostras_filtro 	            10
#define TORQUE_MAX 			                1000
#define torq_frenagem 			            25
#define vel_max_rpm 		                4000
#define	vetorizacao_torque	             	0
#define	frenagem_regenerativa	            0
#define	modo_aceleracao	              		0
#define	modo_enduro			                0
#define zona_morta_acelerador          		10
#define TORQUE_GAIN 1
#define ZERO_VOLANTE 2240
#define GAIN_VOLANTE 16.4
#define GAIN_ARRANC 1.1
#define INC_TORQUE 250
#define INC_TORQUE_INIT 300
#define TORQUE_INIT_LIMITE 150
#define INC_VELOC 50

/////////////////Valores do ad nas seleções de modo///////////////////////////////////////
#define SELECAO_1 410
#define SELECAO_2 1230
#define SELECAO_3 2050
#define SELECAO_4 2870

/////////////////defines de pid//////////////////////////////////
#define KP 2
#define KI 0
#define KD 0
#define GAIN_PID 1
#define MAX_INTEGRAL_ERROR 10
#define SAMPLE_TIME 20 //em ms

typedef struct //struct de modo
{
	int tor_max; //torque maximo (de 0 a 4000)
	int vel_max; //velocidade maxima (de 0 a 9000)
	int freio_regen; //frenagem regenerativa (1 para ativada, 0 para desativada)
	int dif_elt; //diferencial eletronico (1 ativo, 0 desat)
	int arranc_control; //controle de arrancada (1 ativo, 0 desat)
	int bat_safe; //redução de consumo de bateria se em niveis criticos (1 ativo, 0 desat)
	int torq_gain; //ganho de torque, aconselhavel q seja proporcional ao torque maximo ( de 0 a 40)
	int mode; // 1 acel, 2 skid, 3 autox, 4 enduro
} modos;

