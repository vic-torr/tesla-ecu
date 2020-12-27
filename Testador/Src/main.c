/*      Formula Tesla UFMG
 *     ECU_2018_V_2_00
 *     Marco 2019
 */

#include "main.h"
#include "ecu.h"
#include "sys.h"

extern uint32_t id_temp;
extern uint8_t vet_temp[8];
extern uint16_t temp_can;
int main(void)
{
	//      Init
	inicializa_perifericos();
	temp_can = 400;

	for (int it=0; it<8; ++it)
		vet_temp[it] = 0;
	id_temp=0x3;
	//      Loop
	while (true)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		atualiza_watchdog();
		delay_ms_ecu(100);
	}

}
