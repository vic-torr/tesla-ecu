/*
 * ecu.h
 *
 *  Created on: 10 de Ago de 2018
 *      Author: r0tc
 */

#ifndef ECU_H_
#define ECU_H_

#include "sys.h"
#include "ecu_defines.h"

#define		MOTOR_ESQ 				0x1
#define		MOTOR_DIR 				0x0
#define		MOTOR_ESQ_DIR 			0x2
#define		_5_kmph_rpm 			459
#define		ACIONA 					0x03
#define		DESABILITA 				0x0
#define		maxADC_APPS1   			3905
#define		minADC_APPS1   			2232
#define		FREIA					0x1
#define		id_msg_controle_dir 		0x301
#define		id_msg_controle_esq 		0x300
#define		pedal_acel_min 			5
#define		timeout_poll_adc		5

void inicializa_perifericos();
void seta_leds(uint8_t led_config);
void inicializa_modos();
void inicializa_adc_dma();
uint16_t le_acelerador(uint8_t* flag_error);
uint16_t le_volante();
void controle();
void comando_inversor();
void transmite_inversores_datalogger(uint16_t, uint8_t*);
void mensagem_inversor_recebida(uint16_t id,uint8_t* data);
void transmite_ecu_datalogger();
void acel_datalogger();
void skid_datalogger();
void autox_datalogger();
void enduro_datalogger();
void init_datalogger();
void start_datalogger();
void stop_datalogger();
void print_can(uint8_t id, uint16_t* data);
void mensagem_CANSPI_recebida(uint16_t id, uint8_t* data);
int16_t PID_control(int16_t setpoint, int16_t medida);
void Vel_Calc();
void Dist_Calc();
void Diferencial();
void Diferencial_2();
int16_t Funcao_Dif();
void inicializa_tab_dif();
void Controle_arrancada();
void actual_datalogger();

#endif /* ECU_LIB_H_ */
