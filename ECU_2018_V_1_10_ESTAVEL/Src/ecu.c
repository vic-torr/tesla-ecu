#include "ecu.h"
#include "CANSPI.h"
#include "MCP2515.h"


/*
 * To do:
 * Plausabilidade, utilizar os dois sensores.
 *
 */

//		Variaveis globais
uint8_t			vetTx[8];
uint16_t		volante, volante_cru, freio, status_ecu, veloc_total,
refVeloc[2], refTorque[2], refTorqueNeg[2], APPS1, valor_APPS[2], ADC_DMA[6], refVeloc_ant[2], refTorque_ant[2];
uint16_t paramControl = 0;
uint16_t		vel_motor[2], torque[2], potencia[2], corr_torque[2], energia_consumida[2],
valor_sobrecarga[2], temp1_mosf[2], temp2_mosf[2], temp1_motor[2],
temp2_motor[2], msg_perdidas[2], cont_busoff[2], estado_can[2],
estado_inv[2], falha_atual[2], alarme_atual[2];
bool			habilita = false, runstop = false, freiar = false, status_comun= true, rev = false;
uint8_t flag_rtds = 0;
uint32_t buffer[6];
uint16_t acelerador;
int16_t integral_error = 0, error_area;
int16_t integral_buffer[10]= {0,0,0,0,0,0,0,0,0,0};
uint8_t integral_ant = 0;
int16_t last_error = 0;
int16_t media_diant=0;
int16_t media_calc=0;
uint16_t vel_roda[4], vel_calculada[2], vel_calc_motor[2];
uint16_t acel_ant=0;
modos aceleracao, skidpad, autox, enduro, erro;
uint8_t flag_dtl=0;
uint16_t prop_dif[25];
uint8_t roda_interna;
int16_t dist_calc=0;

extern int16_t tempo_teste;
extern uint8_t dist_pr;
extern uint16_t time_speed_refresh;
extern uint8_t speed_t_flag[4];
extern uint32_t speed_t_total[4];
extern uint8_t apps_t_flag;
extern modos selecionado;
uint32_t time_init=0, time_actual=0 ;

uint16_t msg_debug[4]= { 0x0, 0x1, 0x2, 0x3};

void seta_leds(uint8_t led_config)//bit 0 azul, bit 1 verde, bit 2 vermelho
{
	if(0b100 == (led_config & 0b100))
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

	if(0b010 == (led_config & 0b010))
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

	if(0b001 == (led_config & 0b001))
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void inicializa_modos()
{
	aceleracao.tor_max = 3500;
	aceleracao.vel_max = 9000;
	aceleracao.freio_regen = 0;
	aceleracao.dif_elt = 0;
	aceleracao.arranc_control = 0;
	aceleracao.bat_safe = 0;
	aceleracao.torq_gain = 35;
	aceleracao.mode = 4;

	skidpad.tor_max = 3200;
	skidpad.vel_max = 9000;
	skidpad.freio_regen = 0;
	skidpad.dif_elt = 0;
	skidpad.arranc_control = 0;
	skidpad.bat_safe = 0;
	skidpad.torq_gain = 32;
	skidpad.mode = 4;

	autox.tor_max = 3000;
	autox.vel_max = 9000;
	autox.freio_regen = 0;
	autox.dif_elt = 0;
	autox.arranc_control = 0;
	autox.bat_safe = 0;
	autox.torq_gain = 25;
	autox.mode = 4;

	enduro.tor_max = 1500;
	enduro.vel_max = 6000;
	enduro.freio_regen = 0;
	enduro.dif_elt = 0;
	enduro.arranc_control = 0;
	enduro.bat_safe = 0;
	enduro.torq_gain = 25;
	enduro.mode = 4;

	erro.tor_max = 0;
	erro.vel_max = 0;
	erro.freio_regen = 0;
	erro.dif_elt = 0;
	erro.arranc_control = 0;
	erro.bat_safe = 0;
	erro.torq_gain = 0;
	erro.mode = 0;
}

void mensagem_CANSPI_recebida(uint16_t id,uint8_t* data){
	if(id == 0x000){
		flag_rtds = 1;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int i = 0; i < 6; i++){
		buffer[i] &= 0x000FFFF;
		ADC_DMA[i] = (uint16_t) buffer[i];
	}
}



void inicializa_adc_dma(){
	HAL_ADC_Start_DMA(&hadc1, buffer, 6);
}

//	Le acelerador: adc le os dois sensores por n = num_am ostras vezes,
//	faz uma filtragem de media, verifica plausabilidade,
//	calcula valor do acelerador.
//	Retorna acelerador
uint16_t le_acelerador(uint8_t *flag_error)
{
	uint16_t APPS1 = ADC_DMA[0];
	uint16_t APPS2 = ADC_DMA[3];
	uint16_t apps1_calc;
	uint16_t BSE =  ADC_DMA[1];
	if(APPS2 < 260)
	{	acelerador = 0;}

	if( APPS2 >= 260 && APPS2 < 467)
	{	acelerador = (uint16_t)(1.162*APPS2 - 342.9);}

	if( APPS2 >= 467 && APPS2 < 1065)
	{	acelerador = (uint16_t)(0.3344*APPS2 + 43.8);}

	if( APPS2>= 1065 && APPS2 < 2253)
	{	acelerador = (uint16_t)(0.1684*APPS2 + 220.7);}

	if( APPS2 >= 2253 && APPS2 < 3211)
	{	acelerador = (uint16_t)(0.2087*APPS2 + 129.9);}

	if( APPS2 >= 3211 && APPS2 < 3720)
	{	acelerador = (uint16_t)(0.6598*APPS2 - 1319);}
	if( APPS2 >= 3720)
		acelerador = 0;

	if( acelerador >= 0 && acelerador < 200)
		apps1_calc = 2212;
	if(acelerador >= 200 && acelerador < 400)
		apps1_calc= (uint16_t)(1.679*acelerador + 1876);
	if(acelerador >= 400 && acelerador < 600)
		apps1_calc= (uint16_t)(2.621*acelerador + 1499);
	if(acelerador >= 600 && acelerador < 800)
		apps1_calc= (uint16_t)(2.212*acelerador + 1745);
	if(acelerador >= 800 && acelerador < 1135)
		apps1_calc= (uint16_t)(1.515*acelerador + 2302);

	if(APPS1 > 3900 || APPS1 < 1802.24)
		acelerador = 0;
	if(APPS1 < apps1_calc*0.9 || APPS1 > apps1_calc*1.1)
		acelerador = 0;
	if(acelerador > 1000)
		acelerador = 1000;

	if(acelerador < 250 && BSE < 2200)
	{
		*flag_error = 0;
		apps_t_flag = 1;
	}

	if(BSE > 2200)
		freio = 1;
	else
		freio = 0;

	if(((acelerador > 0 && BSE > 2200) ||(BSE < 600))){
		*flag_error = 1;
		acelerador = 0;
	}
	if(*flag_error == 1){
		acelerador = 0;
	}
	//////////rampa de torque///////////////////////////

	if(acelerador > 0)
		runstop = 1;
	else
		runstop = 0;


	return (acelerador);
}


uint16_t le_volante()
{
	volante_cru = ADC_DMA[2];
	if(volante_cru > ZERO_VOLANTE)
	{
		volante = volante_cru - ZERO_VOLANTE;
		roda_interna = 1;
	}
	else
	{
		volante = ZERO_VOLANTE - volante_cru;
		roda_interna = 0;
	}
	if(volante < 60)
		volante = 0;
	else
		volante = volante / GAIN_VOLANTE;
	return(volante);
}

//Calcula referencia de torque e velocidade baseado na dinamica e sensores do carro
//Parametros:	acelerador, speed_t_total[4], sens_freio, frenagem_regenerativa
//Retorna:		habilita, refTorque[2], refTorque_neg[2], refveloc
void controle()
{
	//veloc_total = (speed_t_total[0] + speed_t_total[1] + speed_t_total[2] + speed_t_total[3])/4;

//	if(selecionado.dif_elt == 1 && volante > 60)
//		Diferencial_2();
//	else if(selecionado.arranc_control == 1 && volante < 15)
//		Controle_arrancada();
//	else
//	{
		refTorque[MOTOR_DIR] = (uint16_t) (selecionado.torq_gain * acelerador)/10;
		refTorque[MOTOR_ESQ] = (uint16_t) (selecionado.torq_gain * acelerador)/10;
		refTorqueNeg[MOTOR_DIR] =  0;
		refTorqueNeg[MOTOR_ESQ] =  0;
		refVeloc[MOTOR_DIR] = selecionado.vel_max;
		refVeloc[MOTOR_ESQ] = selecionado.vel_max;
//	}


	if(refTorque_ant[MOTOR_DIR] > TORQUE_INIT_LIMITE)
	{
		if(refTorque[MOTOR_DIR] > refTorque_ant[MOTOR_DIR] + INC_TORQUE)
		{
			refTorque[MOTOR_DIR] = refTorque_ant[MOTOR_DIR] + INC_TORQUE;
		}
	}
	else
	{
		if(refTorque[MOTOR_DIR] > refTorque_ant[MOTOR_DIR] + INC_TORQUE_INIT)
		{
			refTorque[MOTOR_DIR] = refTorque_ant[MOTOR_DIR] + INC_TORQUE_INIT;
		}
	}
	refTorque_ant[MOTOR_DIR]= refTorque[MOTOR_DIR];


	if(refTorque_ant[MOTOR_ESQ] > TORQUE_INIT_LIMITE)
	{
		if(refTorque[MOTOR_ESQ] > refTorque_ant[MOTOR_ESQ] + INC_TORQUE)
		{
			refTorque[MOTOR_ESQ] = refTorque_ant[MOTOR_ESQ] + INC_TORQUE;
		}
	}
	else
	{
		if(refTorque[MOTOR_ESQ] > refTorque_ant[MOTOR_ESQ] + INC_TORQUE_INIT)
		{
			refTorque[MOTOR_ESQ] = refTorque_ant[MOTOR_ESQ] + INC_TORQUE_INIT;
		}
	}
	refTorque_ant[MOTOR_ESQ]= refTorque[MOTOR_ESQ];

/*	if(selecionado.arranc_control == 1)
	{
		if(refVeloc[MOTOR_DIR] > refVeloc_ant[MOTOR_DIR] + INC_VELOC)
		{
			refVeloc[MOTOR_DIR] = refVeloc_ant[MOTOR_DIR] + INC_VELOC;
		}
		refVeloc_ant[MOTOR_DIR]= refVeloc[MOTOR_DIR];

		if(refVeloc[MOTOR_ESQ] > refVeloc_ant[MOTOR_ESQ] + INC_VELOC)
		{
			refVeloc[MOTOR_ESQ] = refVeloc_ant[MOTOR_ESQ] + INC_VELOC;
		}
		refVeloc_ant[MOTOR_ESQ]= refVeloc[MOTOR_ESQ];



		if(refVeloc[MOTOR_DIR] < refVeloc_ant[MOTOR_DIR] - INC_VELOC)
		{
			refVeloc[MOTOR_DIR] = refVeloc_ant[MOTOR_DIR] - INC_VELOC;
		}
		refVeloc_ant[MOTOR_DIR]= refVeloc[MOTOR_DIR];

		if(refVeloc[MOTOR_ESQ] < refVeloc_ant[MOTOR_ESQ] - INC_VELOC)
		{
			refVeloc[MOTOR_ESQ] = refVeloc_ant[MOTOR_ESQ] - INC_VELOC;
		}
		refVeloc_ant[MOTOR_ESQ]= refVeloc[MOTOR_ESQ];
	}*/


	/*if (frenagem_regenerativa == true &&
		  (vel_motor[MOTOR_DIR] > _5_kmph_rpm || vel_motor[MOTOR_ESQ] > _5_kmph_rpm))
  {
	habilita = true;
	freiar = 1;
	refTorque[MOTOR_DIR] =  0;
	refTorque[MOTOR_ESQ] =  0;
	refTorqueNeg[MOTOR_DIR] =  10 * torq_frenagem;
	refTorqueNeg[MOTOR_ESQ] =  10 * torq_frenagem;
	refVeloc = 0;
  }
  else
  {
	habilita = false;
	refTorque[MOTOR_DIR] = 0;
	refTorque[MOTOR_ESQ] = 0;
	refTorqueNeg[MOTOR_DIR] =  0;
	refTorqueNeg[MOTOR_ESQ] =  0;
	refVeloc = vel_max_rpm;
  }*/
}

//Envia a mensagem de controle para o barramento can1, para ser armazenado no datalloger
//Parametros: vetTx, vetor da mensagem de controle
//Retorna: nada
void transmite_ecu_datalogger(){

	if(selecionado.mode ==1)
		acel_datalogger();
	else if(selecionado.mode ==2)
		skid_datalogger();
	else if(selecionado.mode ==3)
		autox_datalogger();
	else if(selecionado.mode ==4)
		enduro_datalogger();
	/*
	bool ok = CANSPI_Transmit(id_msg_controle_dir, 8, vetTx);
	if( ok == false)	print_can(0xf,msg_debug);
	vetTx[0] = veloc_total; // data 0 e 3 para ACIONA
	vetTx[1] = veloc_total>>8;
	vetTx[2] = acelerador; // data 2 e 3 para velocidade
	vetTx[3] = acelerador>>8;
	vetTx[4] = volante;
	vetTx[5] = volante>>8;
	vetTx[6] = freio;
	vetTx[7] = freio >> 8;
	ok = CANSPI_Transmit(0x301, 8, vetTx);
	if( ok == false)	print_can(0xf, msg_debug);
	vetTx[0] = status_ecu; // data 0 e 3 para ACIONA
	vetTx[1] = status_ecu >> 8;
	vetTx[2] = 0;
	vetTx[3] = 0;
	vetTx[4] = 0;
	vetTx[5] = 0;
	vetTx[6] = 0;
	vetTx[7] = 0;
	ok = CANSPI_Transmit(0x302, 8, vetTx);
	if( ok == false)	print_can(0xf,msg_debug);

	//  if( ok == false) Error_Handler();
	 */
}

void acel_datalogger()
{
	if(flag_dtl==2)
	{
		vetTx[0] = vel_roda[0];
		vetTx[1] = vel_roda[0] >> 8;
		vetTx[2] = vel_roda[1];
		vetTx[3] = vel_roda[1] >> 8;
		vetTx[4] = vel_roda[2];
		vetTx[5] = vel_roda[2] >> 8;
		vetTx[6] = vel_roda[3];
		vetTx[7] = vel_roda[3] >> 8;
		CANSPI_Transmit(0x301, 8, vetTx);
		flag_dtl= 3;
	}
	else if(flag_dtl==3)
	{
		vetTx[0] = torque[MOTOR_DIR];
		vetTx[1] = torque[MOTOR_DIR] >> 8;
		vetTx[2] = torque[MOTOR_ESQ];
		vetTx[3] = torque[MOTOR_ESQ] >> 8;
		vetTx[4] = refTorque[MOTOR_DIR];
		vetTx[5] = refTorque[MOTOR_DIR] >> 8;
		vetTx[6] = refTorque[MOTOR_ESQ];
		vetTx[7] = refTorque[MOTOR_ESQ] >> 8;
		CANSPI_Transmit(0x302, 8, vetTx);
		flag_dtl= 2;
	}
}

void skid_datalogger()
{
	if(flag_dtl==2)
	{
		vetTx[0] = vel_roda[0];
		vetTx[1] = vel_roda[0] >> 8;
		vetTx[2] = vel_roda[1];
		vetTx[3] = vel_roda[1] >> 8;
		vetTx[4] = vel_roda[2];
		vetTx[5] = vel_roda[2] >> 8;
		vetTx[6] = vel_roda[3];
		vetTx[7] = vel_roda[3] >> 8;
		CANSPI_Transmit(0x301, 8, vetTx);
		flag_dtl= 3;
	}
	else if(flag_dtl==3)
	{
		vetTx[0] = torque[MOTOR_DIR];
		vetTx[1] = torque[MOTOR_DIR] >> 8;
		vetTx[2] = torque[MOTOR_ESQ];
		vetTx[3] = torque[MOTOR_ESQ] >> 8;
		vetTx[4] = volante_cru;
		vetTx[5] = volante_cru >> 8;
		vetTx[6] = 0;
		vetTx[7] = 0;
		CANSPI_Transmit(0x303, 8, vetTx);
		flag_dtl= 2;
	}
}
void autox_datalogger()
{

}
void enduro_datalogger()
{
	if(flag_dtl==2)
	{
		vetTx[0] = media_diant;
		vetTx[1] = media_diant >> 8;
		vetTx[2] = (torque[MOTOR_DIR] + torque[MOTOR_ESQ]) >> 1;
		vetTx[3] = (torque[MOTOR_DIR] + torque[MOTOR_ESQ]) >> 9;
		vetTx[4] = volante_cru;
		vetTx[5] = volante_cru >> 8;
		vetTx[6] = tempo_teste;
		vetTx[7] = tempo_teste >> 8;
		CANSPI_Transmit(0x304, 8, vetTx);
		flag_dtl= 3;
	}
	else if(flag_dtl==3)
	{
		vetTx[0] = energia_consumida[MOTOR_DIR];
		vetTx[1] = energia_consumida[MOTOR_DIR] >> 8;
		vetTx[2] = energia_consumida[MOTOR_ESQ];
		vetTx[3] = energia_consumida[MOTOR_ESQ] >> 8;
		vetTx[4] = corr_torque[MOTOR_DIR];
		vetTx[5] = corr_torque[MOTOR_DIR] >> 8;
		vetTx[6] = corr_torque[MOTOR_ESQ];
		vetTx[7] = corr_torque[MOTOR_ESQ] >> 8;

		CANSPI_Transmit(0x305, 8, vetTx);
		flag_dtl= 4;
	}
	else if(flag_dtl==4)
	{
		vetTx[0] = temp1_mosf[MOTOR_DIR];
		vetTx[1] = temp1_mosf[MOTOR_DIR] >> 8;
		vetTx[2] = temp2_mosf[MOTOR_DIR];
		vetTx[3] = temp2_mosf[MOTOR_DIR] >> 8;
		vetTx[4] = temp1_mosf[MOTOR_ESQ];
		vetTx[5] = temp1_mosf[MOTOR_ESQ] >> 8;
		vetTx[6] = temp2_mosf[MOTOR_ESQ];
		vetTx[7] = temp2_mosf[MOTOR_ESQ] >> 8;
		CANSPI_Transmit(0x306, 8, vetTx);
		flag_dtl= 2;
	}
}

void init_datalogger()
{
	vetTx[0] = selecionado.mode;
	vetTx[1] = 0;
	vetTx[2] = 0;
	vetTx[3] = 0;
	vetTx[4] = 0;
	vetTx[5] = 0;
	vetTx[6] = 0;
	vetTx[7] = 0;
	CANSPI_Transmit(0x310, 8, vetTx);
}

void start_datalogger()
{
	vetTx[0] = selecionado.mode;
	vetTx[1] = 0;
	vetTx[2] = 1;
	vetTx[3] = 0;
	vetTx[4] = 0;
	vetTx[5] = 0;
	vetTx[6] = 0;
	vetTx[7] = 0;
	CANSPI_Transmit(0x310, 8, vetTx);
}

void stop_datalogger()
{
	vetTx[0] = selecionado.mode;
	vetTx[1] = 0;
	vetTx[2] = 0;
	vetTx[3] = 0;
	vetTx[4] = 1;
	vetTx[5] = 0;
	vetTx[6] = 0;
	vetTx[7] = 0;
	CANSPI_Transmit(0x310, 8, vetTx);
}

//Envia a mesnagem de controle para o barramento can2, para controlar os inversores
//Parametros: hailita, refTorque, refTorq_neg, refVeloc
void comando_inversor()
{
	//paramControl = (habilita == true )?  ACIONA : DESABILITA;
	paramControl = (habilita) + (runstop <<1) + (freiar <<2) + (status_comun <<3) + (rev <<4);
	//paramControl = (habilita == true )?  ACIONA : DESABILITA;

	vetTx[0] = paramControl; // data 0 e 3 para ACIONA
	vetTx[1] = 0;
	vetTx[2] = refTorque[MOTOR_DIR] & 0xff; // data 2 e 3 para velocidade
	vetTx[3] = refTorque[MOTOR_DIR]>>8 & 0xff;
	vetTx[4] = 1; //refTorqueNeg[MOTOR_DIR] & 0xff;
	vetTx[5] = 0; //refTorqueNeg[MOTOR_DIR]>>8 & 0xff;
	vetTx[6] = refVeloc[MOTOR_DIR] & 0xff;
	vetTx[7] = refVeloc[MOTOR_DIR] >>8 & 0xff;

	CAN_Transmit(vetTx, id_msg_controle_dir);
	delay_ms_ecu(5);

	vetTx[0] = paramControl; // data 0 e 3 para ACIONA
	vetTx[1] = 0;
	vetTx[2] = refTorque[MOTOR_ESQ] & 0xff; // data 2 e 3 para velocidade
	vetTx[3] = refTorque[MOTOR_ESQ]>>8 & 0xff;
	vetTx[4] = 1; //refTorqueNeg[MOTOR_DIR] & 0xff;
	vetTx[5] = 0; //refTorqueNeg[MOTOR_DIR]>>8 & 0xff;
	vetTx[6] = refVeloc[MOTOR_ESQ] & 0xff;
	vetTx[7] = refVeloc[MOTOR_ESQ] >>8 & 0xff;

	CAN_Transmit(vetTx, id_msg_controle_esq);


}

//Retransmite as mensagens do barramento can2 para o barramento can1
//Chamada no can recieve handler
//Parametros: data, vetor de dados da ultima mensagem recebida
//Retorna: nada
void transmite_inversores_datalogger(uint16_t id,uint8_t* data){
	bool ok = CANSPI_Transmit(id, 8, data);
	//	 if ( ok == false) 	 Error_Handler();
}

// Armazena dados da ultima mensagem recebida dos inversores nas respectivas variaveis e retransmite para can1
void mensagem_inversor_recebida(uint16_t id,uint8_t* data){
	uint16_t* data_word = (uint16_t*) data;
	switch (id){
	//INV_ME
	case 0x100:
		vel_motor[MOTOR_ESQ] =							 	data_word[0];
		torque[MOTOR_ESQ] = 								data_word[1];
		potencia[MOTOR_ESQ] =								data_word[2];
		corr_torque[MOTOR_ESQ] =							data_word[3];
		break;
	case 0x101:
		energia_consumida[MOTOR_ESQ] =						data_word[0];
		valor_sobrecarga[MOTOR_ESQ] =						data_word[1];
		temp1_mosf[MOTOR_ESQ] =	 							data_word[2];
		temp2_mosf[MOTOR_ESQ] =	 							data_word[3];
		break;
	case 0x102:
		msg_perdidas[MOTOR_ESQ] =							data_word[0];
		cont_busoff[MOTOR_ESQ] =							data_word[1];
		estado_can[MOTOR_ESQ] =						 		data_word[2];
		break;
	case 0x103:
		estado_inv[MOTOR_ESQ] =						 		data_word[0];
		falha_atual[MOTOR_ESQ] =	 						data_word[1];
		alarme_atual[MOTOR_ESQ] =						 	data_word[2];
		break;

		//INV_MD
	case 0x200:
		vel_motor[MOTOR_DIR] =		 						data_word[0];
		torque[MOTOR_DIR] =							 		data_word[1];
		potencia[MOTOR_DIR] =								data_word[2];
		corr_torque[MOTOR_DIR] =							data_word[3];
		break;
	case 0x201:
		energia_consumida[MOTOR_DIR] =						data_word[0];
		valor_sobrecarga[MOTOR_DIR] =						data_word[1];
		temp1_mosf[MOTOR_DIR] =	 							data_word[2];
		temp2_mosf[MOTOR_DIR] =						 		data_word[3];
		break;
	case 0x202:
		msg_perdidas[MOTOR_DIR] =							data_word[0];
		cont_busoff[MOTOR_DIR] =							data_word[1];
		estado_can[MOTOR_DIR] =	 							data_word[2];
		break;
	case 0x203:
		estado_inv[MOTOR_DIR] =						 		data_word[0];
		falha_atual[MOTOR_DIR] =	 						data_word[1];
		alarme_atual[MOTOR_DIR] =	 						data_word[2];
		break;
	}
}

//envia um vetor data[4] para o barramento can1, pra debug
void print_can(uint8_t id, uint16_t* data){
	CANSPI_Transmit(id, 8, (uint8_t*)data);
}

int16_t PID_control(int16_t setpoint, int16_t medida)
{
	uint16_t error, dif_error, out;

	error= setpoint - medida;

	error_area = error * time_speed_refresh/40; //time_speed_refrash dividido por 40 para dar valor em ms
	if(integral_ant==9)
		integral_ant = 0;
	else
		integral_ant= integral_ant + 1;
	integral_error = integral_error - integral_buffer[integral_ant] + error_area;
	integral_buffer[integral_ant] = error_area;
	if(integral_error > MAX_INTEGRAL_ERROR)
		integral_error = MAX_INTEGRAL_ERROR;

	if(time_speed_refresh == 0)
		dif_error = 0;
	else
		dif_error = (error - last_error)*40/time_speed_refresh; //time_speed_refrash dividido por 40 para dar valor em ms
	last_error = error;

	out= GAIN_PID * (KP * error + KI * integral_error - KD * dif_error);

	return (out);
}

void Vel_Calc() //calcula velocidades em rpm, ou decimos de km/h
{
	if(speed_t_total[0] == 0)
		vel_roda[0] = 0;
	else
		vel_roda[0] = (uint16_t) (160000/speed_t_total[0]);

	if(speed_t_total[1] == 0)
		vel_roda[1] = 0;
	else
		vel_roda[1] = (uint16_t) (160000/speed_t_total[1]);

	if(speed_t_total[2] == 0)
		vel_roda[2] = 0;
	else
		vel_roda[2] = (uint16_t) (160000/speed_t_total[2]);

	if(speed_t_total[3] == 0)
		vel_roda[3] = 0;
	else
		vel_roda[3] = (uint16_t) (160000/speed_t_total[3]);

	media_diant = (vel_roda[0] + vel_roda[1])>>1;
}

void Dist_Calc()//calcula distancia percorrida desde o inicio do codigo em metros
{
	if(dist_pr >= 10)
	{
		dist_pr = 0;
		dist_calc = dist_calc + media_diant/36;
	}

}

void Diferencial()
{
	int16_t setpoint, medida, media_tras, pid_return;
	Vel_Calc();

	if(media_diant == 0)
		setpoint = 0;
	else
		setpoint =(int16_t) 100* (vel_roda[0] - vel_roda[1])/ media_diant; //se positivo roda direita mais rapida. Se negativo, roda esq mais rapida

	media_tras = (vel_roda[2] + vel_roda[3])>>1;
	if(media_tras == 0)
		medida = 0;
	else
		medida =(int16_t)  100* (vel_roda[2] - vel_roda[3])/ media_tras; //se positivo roda direita mais rapida. Se negativo, roda esq mais rapida
	if(media_diant > 10)
		pid_return = PID_control(setpoint, medida);
	else
	{
		PID_control(setpoint, medida);
		pid_return=0;
	}


	if(pid_return > TORQUE_GAIN * acelerador)
	{
		refTorque[MOTOR_DIR] = 0;
		refTorque[MOTOR_ESQ] = 2* selecionado.torq_gain * acelerador;
	}
	else if((pid_return * (-1)) > (int16_t) selecionado.torq_gain * acelerador)
	{
		refTorque[MOTOR_DIR] = 2* selecionado.torq_gain * acelerador;
		refTorque[MOTOR_ESQ] = 0;
	}
	else
	{
		refTorque[MOTOR_DIR] = (uint16_t) (selecionado.torq_gain * (acelerador - pid_return))/10;
		refTorque[MOTOR_ESQ] = (uint16_t) (selecionado.torq_gain * (acelerador + pid_return))/10;
	}

	if(refTorque[MOTOR_DIR] > selecionado.torq_gain * 1000)
		refTorque[MOTOR_DIR]= selecionado.torq_gain * 1000;
	if(refTorque[MOTOR_ESQ] > selecionado.torq_gain * 1000)
		refTorque[MOTOR_ESQ]= selecionado.torq_gain * 1000;


	/*vetTx[0] = vel_roda[0] & 0xff;
			vetTx[1] = vel_roda[0]>>8 & 0xff;
			vetTx[2] = vel_roda[1] & 0xff; // data 2 e 3 para velocidade
			vetTx[3] = vel_roda[1]>>8 & 0xff;
			vetTx[4] = refTorque[MOTOR_DIR]& 0xff;
			vetTx[5] = refTorque[MOTOR_DIR]>>8 & 0xff;
			vetTx[6] = refTorque[MOTOR_ESQ] & 0xff;
			vetTx[7] = refTorque[MOTOR_ESQ] >>8 & 0xff;
		CAN_Transmit(vetTx, id_msg_controle_esq);*/
}


void Diferencial_2()
{
	int16_t setpoint, medida, pid_return;
	Vel_Calc();
	Funcao_Dif();

	if(media_diant == 0)
		medida = 0;
	else
		medida =(int16_t) (1000* (vel_roda[0] - vel_roda[1]))/ media_diant; //se positivo roda direita mais rapida. Se negativo, roda esq mais rapida

	if(roda_interna == 1)
		setpoint = (-1)*Funcao_Dif();
	else
		setpoint = Funcao_Dif();

	if(media_diant > 10)
		pid_return = PID_control(setpoint, medida);
	else
	{
		PID_control(setpoint, medida);
		pid_return=0;
	}


	if(pid_return > TORQUE_GAIN * acelerador)
	{
		refTorque[MOTOR_DIR] = 0;
		refTorque[MOTOR_ESQ] = 2* selecionado.torq_gain * acelerador;
	}
	else if((pid_return * (-1)) > (int16_t) selecionado.torq_gain * acelerador)
	{
		refTorque[MOTOR_DIR] = 2* selecionado.torq_gain * acelerador;
		refTorque[MOTOR_ESQ] = 0;
	}
	else
	{
		refTorque[MOTOR_DIR] = (uint16_t) (selecionado.torq_gain * (acelerador - pid_return))/10;
		refTorque[MOTOR_ESQ] = (uint16_t) (selecionado.torq_gain * (acelerador + pid_return))/10;
	}

	if(refTorque[MOTOR_DIR] > selecionado.torq_gain * 1000)
		refTorque[MOTOR_DIR]= selecionado.torq_gain * 1000;
	if(refTorque[MOTOR_ESQ] > selecionado.torq_gain * 1000)
		refTorque[MOTOR_ESQ]= selecionado.torq_gain * 1000;

	refVeloc[MOTOR_DIR] = selecionado.vel_max;
	refVeloc[MOTOR_ESQ] = selecionado.vel_max;
}

void inicializa_tab_dif()
{

	prop_dif[0]= 0;
	prop_dif[1]= 16;
	prop_dif[2]= 33;
	prop_dif[3]= 50;
	prop_dif[4]= 68;
	prop_dif[5]= 86;
	prop_dif[6]= 104;
	prop_dif[7]= 123;
	prop_dif[8]= 142;
	prop_dif[9]= 162;
	prop_dif[10]= 182;
	prop_dif[11]= 203;
	prop_dif[12]= 224;
	prop_dif[13]= 246;
	prop_dif[14]= 269;
	prop_dif[15]= 293;
	prop_dif[16]= 317;
	prop_dif[17]= 343;
	prop_dif[18]= 369;
	prop_dif[19]= 397;
	prop_dif[20]= 426;
	prop_dif[21]= 456;
	prop_dif[22]= 488;
	prop_dif[23]= 521;
	prop_dif[24]= 556;
}

int16_t Funcao_Dif()
{
	int16_t proporcao, vol_norm;
	if(volante < 5)
	{
		vol_norm = 0;
		//		vel_calculada[0] = media_diant;
		//		vel_calculada[1] = media_diant;
	}
	else if (volante > 120)
	{
		proporcao = prop_dif[23];
		vol_norm = 120/5;
	}
	else
		vol_norm = volante/5;

	proporcao = prop_dif[vol_norm];

	return(proporcao);
	//	media_calc = (vel_calculada[0] + vel_calculada[1])>>1;
}

void Controle_arrancada()
{

	if(vel_roda[1] <50)
		vel_calc_motor[MOTOR_DIR]=50*9*GAIN_ARRANC;
	else
		vel_calc_motor[MOTOR_DIR]= vel_roda[1]*9*GAIN_ARRANC;
	if(vel_roda[0] <50)
		vel_calc_motor[MOTOR_ESQ]=50*9*GAIN_ARRANC;
	else
		vel_calc_motor[MOTOR_ESQ]= vel_roda[0]*9*GAIN_ARRANC;

	if(vel_calc_motor[MOTOR_DIR] > selecionado.vel_max )
		refVeloc[MOTOR_DIR] = selecionado.vel_max;
	else
		refVeloc[MOTOR_DIR] = vel_calc_motor[MOTOR_DIR];

	if(vel_calc_motor[MOTOR_ESQ] > selecionado.vel_max )
		refVeloc[MOTOR_ESQ] = selecionado.vel_max;
	else
		refVeloc[MOTOR_ESQ] = vel_calc_motor[MOTOR_ESQ];
}


void actual_datalogger() {
	//error_log[11] = 0;

	if (flag_dtl == 2) //variável "flag_dtl" assume o valor "2" no final da main
	{
		vetTx[0] = vel_roda[0];
		vetTx[1] = vel_roda[0] >> 8;
		vetTx[2] = vel_roda[1];
		vetTx[3] = vel_roda[1] >> 8;
		vetTx[4] = vel_roda[2];
		vetTx[5] = vel_roda[2] >> 8;
		vetTx[6] = vel_roda[3];
		vetTx[7] = vel_roda[3] >> 8;
		CANSPI_Transmit(0x301, 8, vetTx);

		flag_dtl = 3;
	}

	else if(flag_dtl == 3)
	{
		time_actual = (HAL_GetTick() - time_init) / 100; //Calcula a diferenca de tempo e transforma para decimos de segundo
		vetTx[0] = media_diant;
		vetTx[1] = media_diant >> 8;
		vetTx[2] = (torque[MOTOR_DIR] + torque[MOTOR_ESQ]) >> 1;
		vetTx[3] = (torque[MOTOR_DIR] + torque[MOTOR_ESQ]) >> 9;
		vetTx[4] = volante_cru;
		vetTx[5] = volante_cru >> 8;
		vetTx[6] = (uint16_t) time_actual;	//Converte para um int 16
		vetTx[7] = (uint16_t) time_actual >> 8;

		CANSPI_Transmit(0x304, 8, vetTx);
		flag_dtl = 4;

	}

	else if (flag_dtl == 4)
	{

		vetTx[0] = torque[MOTOR_DIR];
		vetTx[1] = torque[MOTOR_DIR] >> 8;
		vetTx[2] = torque[MOTOR_ESQ];
		vetTx[3] = torque[MOTOR_ESQ] >> 8;
		vetTx[4] = refTorque[MOTOR_DIR];
		vetTx[5] = refTorque[MOTOR_DIR] >> 8;
		vetTx[6] = refTorque[MOTOR_ESQ];
		vetTx[7] = refTorque[MOTOR_ESQ] >> 8;

		CANSPI_Transmit(0x302, 8, vetTx);

		flag_dtl = 5;
	}

	else if (flag_dtl == 5)
	{

		vetTx[0] = torque[MOTOR_DIR];
		vetTx[1] = torque[MOTOR_DIR] >> 8;
		vetTx[2] = torque[MOTOR_ESQ];
		vetTx[3] = torque[MOTOR_ESQ] >> 8;
		vetTx[4] = acelerador;
		vetTx[5] = acelerador >> 8;
		vetTx[6] = freio;
		vetTx[7] = freio >> 8;

		CANSPI_Transmit(0x303, 8, vetTx);

		flag_dtl = 6;
	}

	else if (flag_dtl == 6)
	{
		vetTx[0] = energia_consumida[MOTOR_DIR];
		vetTx[1] = energia_consumida[MOTOR_DIR] >> 8;
		vetTx[2] = energia_consumida[MOTOR_ESQ];
		vetTx[3] = energia_consumida[MOTOR_ESQ] >> 8;
		vetTx[4] = corr_torque[MOTOR_DIR];
		vetTx[5] = corr_torque[MOTOR_DIR] >> 8;
		vetTx[6] = corr_torque[MOTOR_ESQ];
		vetTx[7] = corr_torque[MOTOR_ESQ] >> 8;

		CANSPI_Transmit(0x305, 8, vetTx);
		flag_dtl = 7;
	}

	else if (flag_dtl == 7)
	{
		vetTx[0] = temp1_mosf[MOTOR_DIR];
		vetTx[1] = temp1_mosf[MOTOR_DIR] >> 8;
		vetTx[2] = temp2_mosf[MOTOR_DIR];
		vetTx[3] = temp2_mosf[MOTOR_DIR] >> 8;
		vetTx[4] = temp1_mosf[MOTOR_ESQ];
		vetTx[5] = temp1_mosf[MOTOR_ESQ] >> 8;
		vetTx[6] = temp2_mosf[MOTOR_ESQ];
		vetTx[7] = temp2_mosf[MOTOR_ESQ] >> 8;
		CANSPI_Transmit(0x306, 8, vetTx);

		flag_dtl = 2;
	}
}
