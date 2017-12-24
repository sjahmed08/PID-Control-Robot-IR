/*
*********************************************************************************************************
*
*                                        
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                           STM3240G-EVAL  
*                                         Evaluation Board
*		Modified by Syed J. Ahmed, for the STM32F4-Discovery Board, September 2017.
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : EHS
*                 DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <app_cfg.h>
#include <includes.h>
#include <ucos_ii.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f4_discovery.h>
#include <inttypes.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

                                                                /* ----------------- APPLICATION GLOBALS ------------------ */
#define UpdateRate			0x01
static  OS_STK          AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_STK          sensorTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_STK          pidControlTaskStk[APP_CFG_TASK_START_STK_SIZE];
static	OS_STK			motorControlStk[APP_CFG_TASK_START_STK_SIZE];
static	OS_STK			pid_controllerStk[APP_CFG_TASK_START_STK_SIZE];
static	OS_STK			UpdateRatesStk[APP_CFG_TASK_START_STK_SIZE];


OS_EVENT *CommMbox1;												/* Create COMM mailbox																	*/
OS_EVENT *CommMbox2;												/* Create COMM mailbox																	*/
OS_EVENT *CommMbox3;												/* Create COMM mailbox																	*/
OS_EVENT *CommMbox4;												/* Create COMM mailbox																	*/
OS_EVENT *DispSem1;
INT16U CommRxBuf[10];

__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;

OS_FLAG_GRP *UpdateRates_flag;										/* 																					   */													

INT16U integral = 0;
INT16U derivative = 0; 
INT16U last_error = 0;
INT16U current_position;
INT16U error;
INT16U pwm;
INT16U pid;
INT32U CCR1_Val;

double kp = 0.5;
double ki = 0.00;
double kd = .000;



/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppEventCreate            (void); 
static  void  AppTaskStart              (void        *p_arg);
static  void  AppTaskCreate             (void);

static  void    pidControlTask(void);         
static	void	inputTask(void);
static	void	UpdateRates(void);		
static 	void 	TM_PWM_Init(void);
static	void	TM_TIMER_Init(void);
static	void	motorDrive_task(void);
static  void    sensorDistance_task(void);
static 	void	TIM_Config(void);
static	void	PWM_Config(void);
static	void 	ADC_Conv(void);
static	void	controller_PID(void);


static 	void	update_flag_isr(void)
{
	INT8U err;
	err = OSFlagPost(UpdateRates_flag, UpdateRate, OS_FLAG_SET, &err);
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void init_usart(void);


/*
*********************************************************************************************************
*                                                main()
*
* Description : The main function in this program is responsible to initiating multi-tasking. It calls
*               on OSTaskCreateExt which is linked to AppTaskStart() which starts the tasks based on 
*				the priority that is given. Interrupts and BSP calls are temporarily disabled here 
*				to prevent reentrance situations.
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int main(void)
{	
#if (OS_TASK_NAME_EN > 0)
    CPU_INT08U  err;
	INT8U	flgerr;
		
#endif  
		
						
		TIM_Config();											/* Configure Timer                  					*/
		TM_PWM_Init();											/* Configure PWM channels           					*/
		
		PWM_Config();											/* Configure PWM duty cycle         					*/
		ADC_Conv();												/* Configure ADC                    					*/
	
		GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_6);			/* Set GPIOB Pin 4 and Pin 6 to 1   					*/
		GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_7);			/* Set GPIOB Pin 5 and 7 to 0      					 	*/
	
    BSP_IntDisAll();                                            /* Disable all interrupts.     	    					*/
		
    OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel"          */
		
		UpdateRates_flag = OSFlagCreate(0x00, &flgerr);
		
		CommMbox1 = OSMboxCreate((void *)0);					/* Initialize Mailbox               					*/
		CommMbox2 = OSMboxCreate((void *)0);
		
    OSTaskCreateExt((void (*)(void *)) AppTaskStart,           /* Create the start task                                	*/
                    (void           *) 0,
                    (OS_STK         *)&AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
                    (INT8U           ) APP_CFG_TASK_START_PRIO,
                    (INT16U          ) APP_CFG_TASK_START_PRIO,
                    (OS_STK         *)&AppTaskStartStk[0],
                    (INT32U          ) APP_CFG_TASK_START_STK_SIZE,
                    (void           *) 0,
                    (INT16U          )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_CFG_TASK_START_PRIO, "Start", &err);
#endif

    OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II)   */  
    return (1);
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
	void *msg;
	INT16U pwm_value = 1000;
	INT16U semaVal1;
	INT8U errSem;
	INT16U ledStat;
	char snum[5];
	
   (void)p_arg;
		

     BSP_Init();                                                /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */
    
    BSP_Tick_Init();                                            /* Start Tick Initialization                            */

    Mem_Init();                                                 /* Initialize memory managment module                   */
    Math_Init();                                                /* Initialize mathematical module                       */


#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                               /* Determine CPU capacity                               */
#endif


    
#if (APP_CFG_SERIAL_EN == DEF_ENABLED)    
    App_SerialInit();                                           /* Initialize Serial communication for application ...  */
#endif

		STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
		BSP_IntVectSet(BSP_INT_ID_EXTI0, update_flag_isr);

    APP_TRACE_INFO(("Creating Application Events...\n\r"));
    AppEventCreate();                                          /* Create Application Events                            */

    APP_TRACE_INFO(("Creating Application Tasks...\n\r"));
    AppTaskCreate();   										   /* Create application tasks                             */

  /* Turn on LED4, LED3, LED5 and LED6 */
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);

//	ADC_SoftwareStartConv(ADC3);
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */								
			
		App_SerPrintf ("Value: %" PRIu32 "\n", ADC3ConvertedVoltage); /* Print calibrated IR sensor value to terminal   */
		OSTimeDlyHMSM(0, 0, 0, 100);							/* task delay 											*/
	}
}

/*
*********************************************************************************************************
*                                      AppTaskEventCreate()
*
* Description : Create the application Events
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TasStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppEventCreate (void)
{
}


/*
*********************************************************************************************************
*                                      AppTaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TasStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
	
OSTaskCreate((void (*)(void *)) motorDrive_task,           		/* Motor Drive Task 		                             */
                    (void           *) 0,							// argument
                    (OS_STK         *)&motorControlStk[APP_CFG_TASK_START_STK_SIZE - 1],
                    (INT8U           ) 7 );  						// Task Priority	

OSTaskCreate((void (*)(void *)) sensorDistance_task,           	/* Measure sensor Distance Task                          */
                    (void           *) 0,							// argument
                    (OS_STK         *)&sensorTaskStk[APP_CFG_TASK_START_STK_SIZE - 1],
                    (INT8U           ) 5 );  						// Task Priority	
					
OSTaskCreate((void (*)(void *)) controller_PID,           		/* PID calculation Task                               	 */
                    (void           *) 0,							// argument
                    (OS_STK         *)&pid_controllerStk[APP_CFG_TASK_START_STK_SIZE - 1],
                    (INT8U           ) 6 );  						// Task Priority

}	

/*
*********************************************************************************************************
*                                      sensorDistance_task()
*
* Description : Sensor distance task converts ADC value from IR sensor to binary and sends the calibrated
*				value to PID control for PID feedback
*
* Argument(s) : none.
*
* Return(s)   : Calibrated IR sensor value
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

void sensorDistance_task(void)
{	
	ADC_SoftwareStartConv(ADC3);								/* Convert ADC value to binary 							*/
	while(1)
	{		
		ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF; /*  Calibrate ADC value 									*/
		OSMboxPost(CommMbox2,(void *)ADC3ConvertedValue);      /*  Send calibrated value through mailbox to PID task    */
		OSTimeDlyHMSM(0, 0, 0, 300);						   /*  Task Delay											*/
	}
}

/*
*********************************************************************************************************
*                                      motorDrive_task()
*
* Description : Motor drive task receives message from PID controller then adjusts the speed of motor
*				by adjusting PWM based on pid coefficients. Message is received through comboBox1 
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/
	
void motorDrive_task(void)
{
	void *msg;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//get message from combox1
	while(1){
	
	msg = OSMboxAccept(CommMbox1);	
		
	if (msg != (void*)0)
	{	
			
			CCR1_Val = (3300*(INT16U)msg / 100);
		
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel2 */
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
			
			TIM_OC2Init(TIM3, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
			
			TIM_ARRPreloadConfig(TIM3, ENABLE);
			TIM_Cmd(TIM3, ENABLE);
			OSTimeDlyHMSM(0, 0, 0, 150);
	}
  }	
}

/*
*********************************************************************************************************
*                                      TM_PWM_Init()
*
* Description : Configures timer for Pulse Width Modulation with a system clock of 168Mhz frequency and
* 				period of 8399. 
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

void TM_PWM_Init(void) {
	
	 /* TIM3 clock enable */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR1_Val = 0;
	uint16_t CCR2_Val = 0;
	uint16_t PrescalerValue = 0;
	uint32_t SystemCoreClock = 168000000;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 8399;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

/*
*********************************************************************************************************
*                                      controller_PID()
*
* Description : controller_PID method which returns the value of PWM frequency based on IR sensor 
*				feedback. PID function compares the IR sensor position to target position and makes 
*				adjustment to pwm coefficient based on coefficient calculation and applied pid 
*				control theory algorithm P, I, D coefficients are set as global variables. Once PWM
*				coefficient is calculated it is sent to motorDrive_task() for PWM adjustment. 
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/


void controller_PID(void)
{
	//use mailbox to get IR sensor position
		
	void *msg;
	INT8U err;
	INT16U target_position = 2800;	

	while(1)
	{		
		msg = OSMboxAccept(CommMbox2);							/*  Obtain message from sensorDistance_task 			*/
		
		if (msg != (void*)0)
		{
		current_position = (INT16U)msg; 						/*	Update current_position								*/
		//Calculate the error
		error = target_position - current_position;				/*	Calculate between target_position and current_position */
		
		integral = integral + error;							/*	integral is accumalated by taking sum of past integrals and errors 	*/						
		
		derivative = error - last_error;						/*	derivative is difference of current error to past error	*/
		
		pid = (kp * error) + (ki * integral) + (kd * derivative);	/*	calculate pid coefficient 						*/
		//100% between 500 and 1000
		if ((pid > 500) && (pid < 1000))						/*	set threshold values for pid control				*/
			pwm = 100;
		//80% between 400 and 500
		else if ((pid > 400) && (pid < 500))
			pwm = 80;
		//60% between 300 and 400
		else if ((pid > 300) && (pid < 400))
			pwm = 60;
		//40% between 200 and 300
		else if ((pid > 200) && (pid < 300))
			pwm = 40;
		//20% betwen 100 and 200
		else if ((pid > 100) && (pid < 200))
			pwm = 20;
		//0% betwen 0 and 100
		else if ((pid > 0) && (pid < 100))
			pwm = 0;
		else
			pwm = 0;
		
		OSMboxPost(CommMbox1,(void *)pwm);					/*	Post message using CommMbox1						*/
		last_error = error; 
		OSTimeDlyHMSM(0, 0, 0, 10);
		}
	}
}

/*
*********************************************************************************************************
*                                      TIM_Config()
*
* Description : Configuring timer pin outs 
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/	
	
void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

/*
*********************************************************************************************************
*                                      PWM_Config()
*
* Description : Configuring PWM pin outs utilizing Port B pin 4, 5, 6 and 7
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/	

void PWM_Config(void)
{
	GPIO_InitTypeDef GPIOB_InitStructure;

	GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//Output type push-pull
	GPIOB_InitStructure.GPIO_OType = GPIO_OType_PP;
	//Without pull resistors
	GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//50MHz pin speed
	GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOB_InitStructure);
}

/*
*********************************************************************************************************
*                                      ADC_Conv()
*
* Description : Configuring ADC conversion and selecting ADC pin which is Port C pin 12
*				
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

void ADC_Conv(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}



