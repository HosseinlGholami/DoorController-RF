
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"


#define BUF_max_size      64U
#define Timer_RTOS     	  4000
#define Timer_Learn    	  10000

typedef struct{
	uint8_t		len;
	volatile uint16_t	BUF[BUF_max_size];		 	
}BUF;


void RF_LEARN(void);
