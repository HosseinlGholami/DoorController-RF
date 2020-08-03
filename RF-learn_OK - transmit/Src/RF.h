
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"


#define BUF_max_size      64U
#define Timer_RTOS     	  10
#define Timer_Learn    	  4000

typedef enum
{
  RF_IDLE       = 0x00U,
  RF_OK     = 0x01U,
  TIMEOUT  = 0x03U
} RF_StatusTypeDef;

typedef struct{
	uint8_t		len;
	volatile uint16_t	BUF[BUF_max_size];		 	
}BUF;


void RF_LEARN(void);
void RF_TRANSMIT(void);

