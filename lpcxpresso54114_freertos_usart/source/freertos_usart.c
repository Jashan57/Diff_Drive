#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC54114_cm4.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "fsl_usart_freertos.h"
#include "fsl_usart.h"
#include "fsl_debug_console.h"
#include "fsl_ctimer.h"




/* TODO: insert other include files here. */

/* FreeRTOS kernel includes. */


/* TODO: insert other definitions and declarations here. */

#define CTIMER CTIMER0                 /* Timer 0 */
#define L_Mn kCTIMER_Match_0			//	J1[19]
#define L_Mp kCTIMER_Match_2			//  J1[16]
#define R_Mn kCTIMER_Match_1			//  J2[18]
#define R_Mp kCTIMER_Match_3			//  J2[17]

#define DEMO_USART USART0
#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler
#define DEMO_USART_IRQn FLEXCOMM0_IRQn


/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define USART_NVIC_PRIO 5

static void uart_task(void *pvParameters);
static void Diff_Drive(void *pvParameters);

void Motor_Init();

void Motor_Init()
{
		ctimer_config_t config;
		 uint32_t srcClock_Hz;
		 srcClock_Hz = CLOCK_GetFreq(kCLOCK_BusClk);



	    CTIMER_GetDefaultConfig(&config);


	    CTIMER_Init(CTIMER, &config);
	    CTIMER_Init(CTIMER1, &config);
	    CTIMER_SetupPwm(CTIMER,L_Mn,0,20000,srcClock_Hz,NULL);
	    CTIMER_SetupPwm(CTIMER,L_Mp,0,20000,srcClock_Hz,NULL);
	    CTIMER_SetupPwm(CTIMER,R_Mn,0,20000,srcClock_Hz,NULL);
	    CTIMER_SetupPwm(CTIMER1,R_Mp,0,20000,srcClock_Hz,NULL);
	    CTIMER_StartTimer(CTIMER);
	    CTIMER_StartTimer(CTIMER1);
}


uint8_t background_buffer[32];
uint8_t recv_buffer[1];

usart_rtos_handle_t handle;
struct _usart_handle t_handle;


struct rtos_usart_config usart_config = {
    .baudrate    = 9600,
    .parity      = kUSART_ParityDisabled,
    .stopbits    = kUSART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

QueueHandle_t queue1= NULL;


int main(void)

{

	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
	SYSCON->ASYNCAPBCTRL = 1;
	//CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);

	// ctimer_config_t config;
	// uint32_t srcClock_Hz;


    BOARD_InitBootPins();

    BOARD_InitBootPeripherals();
    Motor_Init();

    queue1=xQueueCreate(1,sizeof(char));

    if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 10, NULL, 2, NULL) != pdPASS)
         {
             PRINTF("Task creation failed!.\r\n");
             while (1);
         }


    if (xTaskCreate(Diff_Drive, "Differential_Driving_Task", configMINIMAL_STACK_SIZE + 10, NULL,2, NULL) != pdPASS)
                {
                    PRINTF("Task creation failed!.\r\n");
                    while (1);
                }

         vTaskStartScheduler();
         for (;;)
             ;
     }





static void uart_task(void *pvParameters)
 {
     char error;
     size_t n;
     usart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
     usart_config.base   = DEMO_USART;

     NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);

     USART_RTOS_Init(&handle, &t_handle, &usart_config);

     /* Receive user input and send it back to terminal. */
     while(1)
     {
         USART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
         if (n > 0)
         {
       		error=recv_buffer[0];
       		xQueueSend(queue1,&error,10);
         }
     }

 }



 static void Diff_Drive(void *pvParameters)
 {
	  char recv;

	  	while(1){
	  		xQueueReceive(queue1,&recv,10);
	  		if(recv=='M')
	  		{
	  			CTIMER_UpdatePwmDutycycle(CTIMER, L_Mn,80);
	  			CTIMER_UpdatePwmDutycycle(CTIMER, L_Mp, 0);


	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, R_Mn, 80);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER1, R_Mp, 0);

	  		 	printf("Moving\n");
	  			recv='n';
	  		}
	  		else if(recv=='L')
	  		{

	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mn, 0);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mp, 0);


	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, R_Mn,70);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER1, R_Mp,0);

	  			printf("Left\n");
	  			recv='n';
	  		}
	  		else if(recv=='R')
	  		{

	    		CTIMER_UpdatePwmDutycycle(CTIMER, L_Mn, 70);
	  	   	  	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mp, 0);


	  		  	CTIMER_UpdatePwmDutycycle(CTIMER, R_Mn, 0);
	     	  	CTIMER_UpdatePwmDutycycle(CTIMER1, R_Mp, 0);

	     	  	printf("Right\n");
	  			recv='n';
	  		}
	  		else if(recv=='S')
	  		{
	  			CTIMER_UpdatePwmDutycycle(CTIMER, L_Mn,0);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mp,0);

	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, R_Mn,0);
	  			CTIMER_UpdatePwmDutycycle(CTIMER1,R_Mp,0);

	  			printf("Stop\n");
	  			recv='n';


	  		}
	  		else if(recv=='B')
	  		{
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mn, 0);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, L_Mp, 100);

	  		 	CTIMER_UpdatePwmDutycycle(CTIMER, R_Mn, 0);
	  		 	CTIMER_UpdatePwmDutycycle(CTIMER1,R_Mp, 100);

	  			 printf("Backward\n");
	  			 recv='n';
	  		}





	  	}
 }










