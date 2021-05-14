//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)
/*LCD Function Set */
//#define myLCD_FUNCTION_SET ((uint16_t)0x0038)
/*LCD Display On */
//#define myLCD_DISPLAY_ON ((uint16_t)0x000C)
/*LCD Entry Mode Set */
//#define myLCD_ENTRY_MODE_SET ((uint16_t)0x0006)
/*LCD Clear Display */
//#define myLCD_CLEAR_DISPLAY ((uint16_t)0x0001)
/*ASCII codes*/
#define ASCII_F ((uint8_t)0x46)
#define ASCII_H ((uint8_t)0x48)
#define ASCII_R ((uint8_t)(0x52))
#define ASCII_COL ((uint8_t)0x3A)
#define ASCII_z ((uint8_t)0x7A)
#define ASCII_O ((uint8_t)0x4F)
#define ASCII_h ((uint8_t)0x68)
#define ASCII_0 ((uint8_t)0x30)
#define ASCII_1 ((uint8_t)0x31)
#define ASCII_2 ((uint8_t)0x32)
#define ASCII_3 ((uint8_t)0x33)
#define ASCII_4 ((uint8_t)0x34)
#define ASCII_5 ((uint8_t)0x35)
#define ASCII_6 ((uint8_t)0x36)
#define ASCII_7 ((uint8_t)0x37)
#define ASCII_8 ((uint8_t)0x38)
#define ASCII_9 ((uint8_t)0x39)
/*LCD DDRAM Addresses*/
#define LCD_LINE_1 ((uint8_t)0x80)
#define LCD_LINE_2 ((uint8_t)0x40)
/*ADC Scaling Factor*/
#define ADC_CONV_FACTOR 1.25


void myGPIOB_Init(void);
void myLCD_Init(void);
void myLCD_Handshake(void);
void myLCD_Update(void);
void myDAC_Init(void);
void myGPIOC_Init(void);
void myADC_Init(void);
void myGPIOA_Init(void);
void myTIM2_Init(void);
//can use this for a delay for the LCD or whatever delay he said we may need to introduce
//void myTIM3_Init(void);
void myEXTI_Init(void);

//Global variable to hold what is read from ADC
volatile uint16_t ADC_Data = ((uint16_t)0x0000);
//Global variable to hold what is read from DAC
volatile uint16_t DAC_Data = ((uint16_t)0x0000);
//Global variable to hold calculated frequency from TIM2
volatile uint16_t TIM2_Frequency = ((uint16_t)0x0000);

//will most likely need a global variable here to store the data from the ADC to be sent to the LCD
//this global variable will tell us what numbers we need to send to the LCD and we can implement this in
//an if/else type manner
//this will be in the LCD handshake or something like that

// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.
int timerTriggered = 0;

int
main(int argc, char* argv[])
{

	trace_printf("This is The Final Project!...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA (for Timer Output and Optocoupler)*/
	myGPIOB_Init();		/* Initialize I/O port PB (for LCD) */
	myGPIOC_Init();		/* Initialize I/O port PC (for ADC and POT)  */
	myTIM2_Init();		/* Initialize timer TIM2 */
	//myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */
	myLCD_Init();		/* Initialize LCD */
	myADC_Init();
	myDAC_Init();

	//Will need to use this for the handshake with the LCD (for the LCD though I think that this can just be including
	//into the ADC part - the ADC can then just wait to make sure the handshake is complete and then it can send to the LCD)
	//as well as for polling the ADC (which will wait for an input from the POT on PC1) and for the DAC (which will wait
	//for an input from the ADC)


	while (1)
	{
		//If the ADC conversion is complete, read the data register
		if((ADC1->ISR & ADC_ISR_EOC) != 0 ){
			//Read data register ADC->DR bits [11:0]
			ADC_Data = (ADC1->DR & ADC_DR_DATA);
			//trace_printf("The data reads: %u \n", (uint16_t)((ADC_Data)*ADC_CONV_FACTOR));
			//DAC register in use DAC->DHR12R1 (12 bit register for right-aligned) DAC_DHR12R1_DACC1DHR
			//Read from ADC1_DR and then send to DAC_DHR12R1 (i.e. DAC->DH12R1 = ADC1->DR)
			//trace_printf("The DAC Register: %x \n", (uint16_t)(DAC->DHR12R1));
			//ADC_Data = ADC_Data*1.9;

			//DAC->DHR12R1 = (ADC_Data & DAC_DHR12R1_DACC1DHR); -
			DAC->DHR12R1 = (ADC_Data & DAC_DHR12R1_DACC1DHR);
			DAC_Data = (DAC->DHR12R1);
			//DAC_Data =
			//trace_printf("The data passed from ADC1 to DHR12R1 is: %u \n", (uint16_t)(DAC_Data));
			myLCD_Update();

		}

	}

	return 0;

}

void myADC_Init()
{
   /*Enable clock for the ADC*/
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
   //Make sure all the bits in the control register are zero (Bit [0]: ADEN can only be set if all bits of ADC_CR are 0)
   ADC1->CR &= 0x00000000;
   //Enable the ADC - Bit [0]: ADEN set to 1 (Note: this bit is cleared by hardware after ADDIS bit is set - ADDIS is a disable bit - i.e ADDIS=1=DISABLE)
   //Another note on ADDIS - once set to 1, the ADC will begin to shut down... once shut down it will clear ADEN and ADDIS will be cleared by hardware as a result
   ADC1->CR |= ADC_CR_ADEN;

   //Set the SMP[2:0] all to 1 - a sampling time to an intermediate value of 239 1/2 ADC clock cycles - Again, software can write these bits only when ADSTART=0
   ADC1->SMPR |= ADC_SMPR_SMP;
   //Set the channel selection register to channel 11 (since it is internally connected to PC1)
   ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

   //Bit alignment is by default set to 0 which is right data alignment (this is what we want so do nothing)
   //Resolution is by default set to RES=00 which is exactly what we want so no need to change (do nothing)

   //Set the overrun management mode to 1: ADC_DR contents are preserved/overwritten when an overrun is detected and thus we will always read the freshest value
   ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;

   //Set Bit[13]: CONT (continuous conversion mode - everytime it finishes converting it will continue running)
   ADC1->CFGR1 |= ADC_CFGR1_CONT;

   //Now, the ADC will continue converting due to the configurations made. The EOC flag will become 1 when a channel conversion is complete
   //thus EOC is the bit that needs to be polled by software - harware will clear this bit when ADC_DR is read and thus we will always read the freshest value
   //with no confusion overrun stuff (since overrun mode is set) ADC_ISR_EOC

   //Wait for the ADC ready flag (Bit[0]: ADRDY) - this will tell us that initialization is complete
   while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
   //trace_printf("ADRDY is now 1 and the ADC IS READY!\n");

   //Once ADRDY is set 1, the ADC is ready to start converting
   /*should maybe include this stuff in the main?
   //Make sure ADDIS set to zero to allow ADSTART to be set and start the conversion process (most likely an unessecary step due to previous AND operation)
   ADC1->CR &= ~(ADC_CR_ADDIS);
   //Start the conversion process - this will stay 1 since we will just let it keep converting
   //Note: ADSTART is cleared by hardware when the conversion process is finished or stopped (but again.. we will let it keep converting)*/
   ADC1->CR |= ADC_CR_ADSTART;
}

//Here I need to initialize GPIOB for use with the LCD
void myGPIOB_Init()
{
   /*Enable clock for GPIOB peripheral */
   //Relevant register: RCC->AHBENR
   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
   //Configure PB7 as an input
   GPIOB->MODER &= ~(GPIO_MODER_MODER7);
   /* Ensure no pull-up/pull-down for PB7 */
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
   //Configure PB4-6 and PB8-15 as output
   GPIOB->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER8_0 |
		   GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 |
		   GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
   /* Ensure no pull-up/pull-down for PB4-6 and PB8-15 - I think this is correct for this */
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR8 |
		   GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12 | GPIO_PUPDR_PUPDR13 |
		   GPIO_PUPDR_PUPDR14 | GPIO_PUPDR_PUPDR15);
   /* Ensure high-speed mode for PC8 and PC9 */
   //GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR8 |
   //   GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12 |
   //		   GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15);
   /* Ensure push-pull mode selected for data pins*/
   //GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 |
   //GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_15);
}

//Here I need to initialize GPIOC for use with the ADC and POT
void myGPIOC_Init()
{
   /*Enable clock for GPIOC peripheral */
   //Relevant register: RCC->AHBENR
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
   //Configure PC1 as analog
   GPIOC->MODER |= GPIO_MODER_MODER1;
   /*Ensure no pull-up/pull-down for PC1 - this should be correct for this I think */
   GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}

//Master-Slave Handshake
void myLCD_Handshake(){

	//Note that: #define GPIO_ODR_4        ((uint32_t)0x00000010)
	//and #define GPIO_ODR_7               ((uint32_t)0x00000080) and thus,
	//trace_printf will output 9
	//Assert master enable (PB[4]=1)
	GPIOB->ODR |= GPIO_ODR_4;
	//trace_printf("GPIOB I/O Data Registers Write Enable: %x \n", (uint16_t)GPIOB->IDR );
	//Wait for LCD (slave) to assert done (PB[7]=1)
	while((GPIOB->IDR & GPIO_IDR_7) == 0);
	//trace_printf("GPIOB I/O Data Registers LCD Slave assert done : %x \n", (uint16_t)GPIOB->IDR );
	//De-assert master enable (PB[4]=0)
	GPIOB->ODR &= ~(GPIO_ODR_4);
	//trace_printf("GPIOB I/O Data Registers De-assert Enable: %x \n", (uint16_t)GPIOB->IDR );
	//Wait for LCD (slave) to de-assert done (PB[7]=0)
	while((GPIOB->IDR & GPIO_IDR_7) !=0);
	//trace_printf("GPIOB I/O Data Registers LCD Slave De-assert Done : %x \n", (uint16_t)GPIOB->IDR );

}

//Refresh the LCD with new values
void myLCD_Update()
{
	//Divide the ADC data into ones, tens, hundreds, thousands for display
	uint16_t resistance = ADC_Data*ADC_CONV_FACTOR;
	uint16_t resistance_thousands = (resistance)/1000;
	uint16_t resistance_hundreds = ((resistance)-(resistance_thousands*1000))/100;
	uint16_t resistance_tens = ((resistance)-(resistance_thousands*1000)-(resistance_hundreds*100))/10;
	uint16_t resistance_ones = ((resistance)-(resistance_thousands*1000)-(resistance_hundreds*100)-(resistance_tens*10))/1;
    //Divid the DAC data into ones, tens, hundreds, thousands for display
	unsigned int frequency = TIM2_Frequency;
	unsigned int frequency_thousands = (frequency)/1000;
    unsigned int frequency_hundreds = ((frequency)-(frequency_thousands*1000))/100;
	unsigned int frequency_tens = ((frequency)-(frequency_thousands*1000)-(frequency_hundreds*100))/10;
	unsigned int frequency_ones = ((frequency)-(frequency_thousands*1000)-(frequency_hundreds*100)-(frequency_tens*10))/1;


   //Set the LCD's DDRAM 7 bit address you will write to (here RS=0 and R/W=0, DB7=1 and DB6-0=A[6:0] - A for address)

   //Write 8-bits of data D[7:0] into LCD's DDRAM (here RS=1 and R/W=0 and DB7-0=D[7:0](ASCII code)) - each ASCII is written seperately
   //Note: cannot read from DDRAM (R/W is always 0)
   //Note: DDRAM address is incremented for you every time - only have to set the address twice (once for each line)

	//Set DDRAM address for line 1 PB5=RS=0, PB6=R/W=0, PB15=DB7=1, PB8,PB9,...,PB14=D0,D1,...,D6=0
  GPIOB->ODR = (~(GPIO_ODR_5 | GPIO_ODR_6 )&(GPIO_ODR_15));
  myLCD_Handshake();
  //trace_printf("GPIO -> ODR Line 1 Address: %x \n", (uint16_t)GPIOB->ODR);

  //Write data to DDRAM
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_F<<8));
  //trace_printf("GPIO -> ODR Write H : %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_COL<<8));
  //trace_printf("GPIO -> ODR Write F : %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((frequency_thousands+48)<<8));
  //trace_printf("GPIO -> ODR Write 0: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((frequency_hundreds+48)<<8));
  //trace_printf("GPIO -> ODR Write 1: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((frequency_tens+48)<<8));
  //trace_printf("GPIO -> ODR Write 2: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((frequency_ones+48)<<8));
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_H<<8));
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_z<<8));
  myLCD_Handshake();

  //Set DDRAM address for line 2 PB5=RS=0, PB6=R/W=0, PB15=DB7=1, PB14=DB6=1 PB8,PB9,...,PB14=D0,D1,...,D6=0
  GPIOB->ODR = (~(GPIO_ODR_5 | GPIO_ODR_6 )&(GPIO_ODR_15|GPIO_ODR_14));
  myLCD_Handshake();
  //trace_printf("GPIO -> ODR Line 2 Address: %x \n", (uint16_t)GPIOB->ODR);
  //Write data to DDRAM
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_R<<8));
  //trace_printf("GPIO -> ODR Write H : %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_COL<<8));
  //trace_printf("GPIO -> ODR Write F : %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((resistance_thousands+48)<<8));
  //trace_printf("GPIO -> ODR Write 0: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((resistance_hundreds+48)<<8));
  //trace_printf("GPIO -> ODR Write 1: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((resistance_tens+48)<<8));
  //trace_printf("GPIO -> ODR Write 2: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|((resistance_ones+48)<<8));
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_O<<8));
  myLCD_Handshake();
  GPIOB->ODR = (((GPIO_ODR_5)&(~GPIO_ODR_6))|(ASCII_h<<8));
  myLCD_Handshake();
}

void myLCD_Init()
{

  //Function set - PB13=DB5=1, PB12=DB4=1, PB11=DB3=1, PB10=DB2=0
  GPIOB->ODR = ((GPIO_ODR_13 | GPIO_ODR_12 | GPIO_ODR_11)&(~(GPIO_ODR_10)));
  //trace_printf("GPIO -> ODR LCD Function Set : %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  //Display on - PB11=DB3=1, PB10=D2=1, PB9=DB1=0, PB8=DB0=0
  GPIOB->ODR = ((GPIO_ODR_11 | GPIO_ODR_10 )&(~(GPIO_ODR_9 | GPIO_ODR_8 )));
  //trace_printf("GPIO -> ODR LCD Display On: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  //Entry mode set - PB10=DB=1, PB9=DB1=1, PB8=DB0=0
  GPIOB->ODR = ((GPIO_ODR_10 | GPIO_ODR_9)&(~(GPIO_ODR_8)));
  //trace_printf("GPIO -> ODR LCD Entry Mode Set: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();
  //Clear display - PB8=DB0=1
  GPIOB->ODR = GPIO_ODR_8;
  //trace_printf("GPIO -> ODR LCD Clear Display: %x \n", (uint16_t)GPIOB->ODR);
  myLCD_Handshake();

}


//this will be used to measure the frequency of the timer output
//will also need to initialize PA4 for the DAC
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/*Configure PA4 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER4;
	/*Ensure no pull-up/pull-down for PA4*/
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	//Ensure push-pull
	//GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	//Here, you need to initialize pin 4 for the DAC
	//GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
}

void myDAC_Init()
{
	/* Enable clock for the DAC*/
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	//DACoutput=VDDA*DOR/4096
	//DAC register in use DAC->DHR12R1 (12 bit register for right-aligned) DAC_DHR12R1_DACC1DHR
	//Read from ADC1_DR and then send to DAC_DHR12R1 (i.e. DAC->DH12R1 = ADC1->DR)
	//DAC control register (DAC->CR) - Bit[0]:EN1 (Channel1 enable) - must configure PA4 as analog before you enable (RCC too)
	DAC->CR &= 0x00000000;
	DAC->CR |= DAC_CR_EN1;
	//Bit[1]:BBOFF1=0/1 (channel1 tri-state buffer is enabled/disabled) - need this set to 0 (should already be 0 by default)
	DAC->CR &= ~(DAC_CR_BOFF1);
	//Bit[2]:TEN1 (channel1 trigger enable)=0/1: - need this set to 0 (should already be 0 by default)
	DAC-> CR &= ~(DAC_CR_TEN1);
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
	/* Start counting timer pulses */
	TIM2->CR1 |= TIM_CR1_CEN;

}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	//** I dont know if this is how we fill EXTI1 with zeros
	//SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
	SYSCFG->EXTICR[0]|= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;
	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;
	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Declare/initialize your local variables here...
	//timerTriggered += (int) EXTI_PR_PR1;
	uint16_t T = 0;
	uint16_t f = 0;
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		if (timerTriggered == 0){
			//	- Clear count register (TIM2->CNT).
			TIM2->CNT &= ~(myTIM2_PERIOD);
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			//    Else (this is the second edge):
			timerTriggered++;
		} else{
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//	- Read out count register (TIM2->CNT).
			T = (TIM2->CNT-1)/48;
			//	- Calculate signal period and frequency.
			f = 1000000/T -1;
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
			//TIM2_Frequency = 0x0000;
			TIM2_Frequency = f;
			trace_printf("Period: %u microseconds, Frequency:%u Hz\n", (uint16_t)T, (uint16_t)f);
			//
			// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
			// NOTE: A pending register (PR) bit is cleared
			// by writing 1 to it.
			timerTriggered--;
		}

		EXTI->PR |= (EXTI_PR_PR1);


	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
