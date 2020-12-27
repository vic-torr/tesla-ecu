/*      Formula Tesla UFMG
 *     ECU_2018_V_0_2
 */

#include "main.h"
#include "ecu.h"
#include "sys.h"


//    Variaveis globais

extern uint16_t refVeloc, refTorque;
extern uint8_t* vetTx;
extern uint8_t freiar, paramControl;
extern uint16_t velocEncoder;
extern bool habilita;
extern uint8_t speed_t_flag[4];
extern uint32_t speed_t_total[4];
extern uint8_t apps_t_flag;
extern uint8_t flag_rtds;
extern uint16_t ADC_DMA[6];
extern uint8_t refresh_speed;
extern uint16_t vel_roda[4];
extern uint8_t roda_interna;
extern uint8_t flag_dtl;
extern uint8_t conta_t_dtl;
extern int16_t dist_calc, tempo_teste;
extern uint8_t dist_pr;
extern uint32_t time_init;

uint8_t vet[8];
uint8_t aux_vel[8];
uint8_t led_conf;
uint32_t id_msg = 0x666;
uint16_t volante_main;
modos selecionado;
extern modos aceleracao, skidpad, autox, enduro, erro;


int main(void)
{
	//      Init
	uint8_t flag_error=0, RTD_OK=0;
	uint16_t bot_selec;
	uint16_t acel_pedal=0, aux_1, aux_2;
	inicializa_perifericos();
	//      RTDS
	inicializa_modos();
	inicializa_tab_dif();
	seta_leds(0b100);
	while(RTD_OK==0){
		IWDG->KR = REFRESH_WDG;
		//este if deve considerar a flag do contator, BSE e Botao HABILITA
		if(ADC_DMA[1] > 2200 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) ==1){

			bot_selec = ADC_DMA[5];
			if(bot_selec <= SELECAO_1)
			{
				selecionado = aceleracao;
				led_conf = 0b101;
			}
			else if(bot_selec <= SELECAO_2)
			{
				selecionado = skidpad;
				led_conf = 0b011;
			}
			else if(bot_selec <= SELECAO_3)
			{
				selecionado = autox;
				led_conf = 0b001;
			}
			else if(bot_selec <= SELECAO_4)
			{
				selecionado = enduro;
				led_conf = 0b010;
			}
			else
			{
				selecionado = erro;
				led_conf = 0b100;
			}

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			delay_ms_ecu(100);
			//delay_ms(1500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			RTD_OK = 1;
			time_init = HAL_GetTick();
		}
	}
	habilita = true;
	seta_leds(led_conf);
	init_datalogger();
	//      Loop
	while (true)
	{
		IWDG->KR = REFRESH_WDG;
		acel_pedal = le_acelerador(&flag_error);
		volante_main = le_volante();
		Vel_Calc();
		Dist_Calc();
		controle();

		comando_inversor();

		transmite_ecu_datalogger();

		/*aux_1 = vel_roda[0];
		aux_2 = vel_roda[1];

		Vel_Calc();
		aux_vel[0] = aux_1;
		aux_vel[1] = aux_1>>8;
		aux_vel[2] =  aux_2;
		aux_vel[3] =  aux_2>>8;
		aux_vel[4] = (uint8_t)(0xff) & (acel_pedal);//(uint8_t)(0xff) & vel_roda[2];
		aux_vel[5] = (uint8_t)(0xff) & (acel_pedal>>8);//(uint8_t)(0xff) & (vel_roda[2]>>8);
		aux_vel[6] = (uint8_t)(0xff) & (volante_main);//(uint8_t)(0xff) & vel_roda[3];
		aux_vel[7] = (uint8_t)(0xff) & (volante_main>>8);//(uint8_t)(0xff) & (vel_roda[3]>>8);
*/
		//CANSPI_Transmit(0x303, 8, aux_vel);
		//delay_ms(20);

	/*	aux_vel[0] = 0;//led_conf;
		aux_vel[1] = 0;
		aux_vel[2] = 0; //(uint8_t)(0xff) & (acel_pedal);
		aux_vel[3] = 0; //(uint8_t)(0xff) & (acel_pedal>>8);
		aux_vel[6] = 0;//(uint8_t)(0xff) & (volante_main);
		aux_vel[7] = 0;//(uint8_t)(0xff) & (volante_main>>8);
		aux_vel[4] = 0;
		aux_vel[5] = 0;*/

//		CANSPI_Transmit(0x302, 8, aux_vel);
	//	delay_ms(20);

		/*volante = ADC_DMA[2];
	  if(volante > ZERO_VOLANTE)
	  {
		  volante = volante - ZERO_VOLANTE;
		  aux_vel[0] = (uint8_t)(0xff) & (volante);
	  		aux_vel[1] = (uint8_t)(0xff) & (volante>>8);
	  		aux_vel[4] = 0;
	  		aux_vel[6] = 0;
	  		aux_vel[2] = 0;
	  		aux_vel[3] = 0;
	  		aux_vel[5] = 0;
	  		aux_vel[7] = 0;
	  }
	  else
	  {
		  volante = ZERO_VOLANTE - volante;
		  aux_vel[2] = (uint8_t)(0xff) & (volante);
		  	  		aux_vel[3] = (uint8_t)(0xff) & (volante>>8);
		  	  		aux_vel[4] = 0;
		  	  		aux_vel[6] = 0;
		  	  		aux_vel[0] = 0;
		  	  		aux_vel[1] = 0;
		  	  		aux_vel[5] = 0;
		  	  		aux_vel[7] = 0;
	  }*/


		//delay_ms_ecu(50);
		//transmite_ecu_datalogger();
		//print_can(msg_debug, 0x0);
		if(flag_dtl==0)
			if(acel_pedal > 0)
			{
				start_datalogger();
				flag_dtl=2;
				dist_calc = 0;
				tempo_teste=0;
				dist_pr=0;
			}
		if(conta_t_dtl > 10 && selecionado.mode != 4)
				{
					stop_datalogger();
					flag_dtl=1;
					conta_t_dtl=0;
				}
		actual_datalogger();
	}

}
