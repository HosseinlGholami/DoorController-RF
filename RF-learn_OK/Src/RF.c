#include "RF.h"

extern 	TIM_HandleTypeDef htim1,htim2;
extern 	UART_HandleTypeDef huart1;
//extern osTimerId myTimer01Handle;


extern	BUF RF_pck ;	//struct for reciving uart
extern char str[100]; //buffer for transmit UART
	
void RF_LEARN(void){
//bufer data ro bayad inja sefr koni
	
	// tajhian tabeh pas bede 
	
	for(int i=0;i<BUF_max_size;i++)
		RF_pck.BUF[i]=0;
		RF_pck.len=0;
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
			
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	osDelay(Timer_Learn);
	
		if(htim1.State==HAL_TIM_STATE_RESET){
			sprintf(str, "recive End2\n");
			HAL_UART_Transmit(&huart1,(unsigned char*)str ,strlen(str),100);
			HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
	//		osTimerStop(myTimer01Handle);
		}

	
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}
void RF_TRANSMIT(void){
	HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_Delay(30);
}
