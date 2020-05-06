#include "initialisation.h"

#define PLL_M 8
#define PLL_N 360
#define PLL_P 2		//  Main PLL (PLL) division factor for main system clock can be 2 (PLL_P = 0), 4 (PLL_P = 1), 6 (PLL_P = 2), 8 (PLL_P = 3)
#define PLL_Q 7

void SystemClock_Config(void) {

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;			// Enable Power Control clock
	PWR->CR |= PWR_CR_VOS_0;					// Enable VOS voltage scaling - allows maximum clock speed

	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));// CPACR register: set full access privileges for coprocessors

	RCC->CR |= RCC_CR_HSEON;					// HSE ON
	while ((RCC->CR & RCC_CR_HSERDY) == 0);		// Wait till HSE is ready
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;			// HCLK = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;			// PCLK2 = HCLK / 2
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;			// PCLK1 = HCLK / 4
	RCC->CR |= RCC_CR_PLLON;					// Enable the main PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait till the main PLL is ready

	// Configure Flash prefetch, Instruction cache, Data cache and wait state
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

}

void InitSysTick()
{

	// Register macros found in core_cm4.h
	SysTick->CTRL = 0;									// Disable SysTick
	SysTick->LOAD = 0xFFFF - 1;						// Set reload register to maximum 2^24

	// Set priority of Systick interrupt to least urgency (ie largest priority value)
	NVIC_SetPriority (SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	SysTick->VAL = 0;									// Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;		// Select processor clock: 1 = processor clock; 0 = external clock
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;			// Enable SysTick interrupt
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;			// Enable SysTick
}

void InitLCDHardware(void) {
	//	Enable GPIO and SPI clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;			// reset and clock control - advanced high performance bus - GPIO port C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;			// reset and clock control - advanced high performance bus - GPIO port D
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;			// reset and clock control - advanced high performance bus - GPIO port F
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

	// Init DC (Data/Command) pin PD13
	GPIOD->MODER |= GPIO_MODER_MODER13_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13_0;	// Medium  - 00: Low speed; 01: Medium speed; 10: High speed; 11: Very high speed

	// Init CS pin PC2
	GPIOC->MODER |= GPIO_MODER_MODER2_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_0;		// Medium  - 00: Low speed; 01: Medium speed; 10: High speed; 11: Very high speed

	// Init RESET pin PD12
	GPIOD->MODER |= GPIO_MODER_MODER12_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR12_0;			// Pull up - 00: No pull-up, pull-down; 01 Pull-up; 10 Pull-down; 11 Reserved

	// Setup SPI pins -  PF7: SPI5_SCK;  PF8: SPI5_MISO;  PF9: SPI5_MOSI [all alternate function AF5 for SPI5]
	GPIOF->MODER |= GPIO_MODER_MODER7_1;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;		// V High  - 00: Low speed; 01: Medium speed; 10: High speed; 11: Very high speed
	GPIOF->AFR[0] |= 0b0101 << 28;					// 0b0101 = Alternate Function 5 (SPI5); 28 is position of Pin 7

	GPIOF->MODER |= GPIO_MODER_MODER8_1;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;		// V High  - 00: Low speed; 01: Medium speed; 10: High speed; 11: Very high speed
	GPIOF->AFR[1] |= 0b0101 << 0;					// 0b0101 = Alternate Function 5 (SPI5); 0 is position of Pin 8

	GPIOF->MODER |= GPIO_MODER_MODER9_1;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;		// V High  - 00: Low speed; 01: Medium speed; 10: High speed; 11: Very high speed
	GPIOF->AFR[1] |= 0b0101 << 4;					// 0b0101 = Alternate Function 5 (SPI5); 4 is position of Pin 9

	// Configure SPI
	SPI5->CR1 |= SPI_CR1_SSM;						// Software slave management: When SSM bit is set, NSS pin input is replaced with the value from the SSI bit
	SPI5->CR1 |= SPI_CR1_SSI;						// Internal slave select
	SPI5->CR1 |= SPI_CR1_BR_0;						// Baud rate control prescaler: 0b001: fPCLK/4; 0b100: fPCLK/32
	SPI5->CR1 |= SPI_CR1_MSTR;						// Master selection

	SPI5->CR1 |= SPI_CR1_SPE;						// Enable SPI

	// Configure DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// Initialise TX stream - Stream 6 = SPI5_TX; Stream 5 = SPI5_RX; Manual p308
	DMA2_Stream6->CR |= DMA_SxCR_CHSEL;				// 0b111 is DMA_Channel_7
	DMA2_Stream6->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA2_Stream6->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA2_Stream6->CR |= DMA_SxCR_DIR_0;				// data transfer direction: 00: peripheral-to-memory; 01: memory-to-peripheral; 10: memory-to-memory
	DMA2_Stream6->CR |= DMA_SxCR_PL_1;				// Set to high priority
	DMA2_Stream6->PAR = (uint32_t) &(SPI5->DR);		// Configure the peripheral data register address
}



//	Setup Timer 5 to count time between bounces
void InitDebounceTimer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// Enable Timer
	TIM5->PSC = 10000;
	TIM5->ARR = 65535;
}

void InitEncoders() {
	// Encoder L: Button on PB4, up/down on PE10 and PE11; Encoder R: button on PA7, up/down on PB6 and PB7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// reset and clock control - advanced high performance bus - GPIO port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;			// reset and clock control - advanced high performance bus - GPIO port E
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;			// Enable system configuration clock: used to manage external interrupt line connection to GPIOs

	// configure PA7 button to fire on an interrupt
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA;	// Select Pin PA7 which uses External interrupt 2
	EXTI->RTSR |= EXTI_RTSR_TR7;					// Enable rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR7;					// Enable falling edge trigger
	EXTI->IMR |= EXTI_IMR_MR7;						// Activate interrupt using mask register

	// configure PB4 button to fire on an interrupt
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;	// Select Pin PB4 which uses External interrupt 2
	EXTI->RTSR |= EXTI_RTSR_TR4;					// Enable rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR4;					// Enable falling edge trigger
	EXTI->IMR |= EXTI_IMR_MR4;						// Activate interrupt using mask register

	// L Encoder using timer functionality - PE9, PE11
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;			// reset and clock control - advanced high performance bus - GPIO port B

	GPIOE->PUPDR |= GPIO_PUPDR_PUPDR9_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOE->MODER |= GPIO_MODER_MODER9_1;			// Set alternate function
	GPIOE->AFR[1] |= 1 << 4;						// Alternate function 1 is TIM1_CH1

	GPIOE->PUPDR |= GPIO_PUPDR_PUPDR11_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOE->MODER |= GPIO_MODER_MODER11_1;			// Set alternate function
	GPIOE->AFR[1] |= 1 << 12;						// Alternate function 1 is TIM1_CH2

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;				// Enable Timer 4
	TIM1->PSC = 0;									// Set prescaler
	TIM1->ARR = 0xFFFF; 							// Set auto reload register to max
	TIM1->SMCR |= TIM_SMCR_SMS_0 |TIM_SMCR_SMS_1;	// SMS=011 for counting on both TI1 and TI2 edges
	TIM1->SMCR |= TIM_SMCR_ETF;						// Enable digital filter
	TIM1->CNT = 32000;								// Start counter at mid way point
	TIM1->CR1 |= TIM_CR1_CEN;

	// R Encoder using timer functionality - PB6, PB7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B

	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOB->MODER |= GPIO_MODER_MODER6_1;			// Set alternate function
	GPIOB->AFR[0] |= 2 << 24;						// Alternate function 2 is TIM4_CH1

	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOB->MODER |= GPIO_MODER_MODER7_1;			// Set alternate function
	GPIOB->AFR[0] |= 2 << 28;						// Alternate function 2 is TIM4_CH2

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;				// Enable Timer 4
	TIM4->PSC = 0;									// Set prescaler
	TIM4->ARR = 0xFFFF; 							// Set auto reload register to max
	TIM4->SMCR |= TIM_SMCR_SMS_0 |TIM_SMCR_SMS_1;	// SMS=011 for counting on both TI1 and TI2 edges
	TIM4->SMCR |= TIM_SMCR_ETF;						// Enable digital filter
	TIM4->CNT = 32000;								// Start counter at mid way point
	TIM4->CR1 |= TIM_CR1_CEN;


	NVIC_SetPriority(EXTI4_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(EXTI9_5_IRQn);
/*
	NVIC_SetPriority(EXTI15_10_IRQn, 4);			// Lower is higher priority
	NVIC_EnableIRQ(EXTI15_10_IRQn);
*/
}

void InitCAN() {
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	// CAN1_TX Pins (AF9): PD1 (also PB9, PA12)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;			// reset and clock control - advanced high performance bus - GPIO port D
	GPIOD->MODER |= GPIO_MODER_MODER1_1;			// Set alternate function
	GPIOD->AFR[0] |= 9 << 4;						// Alternate function 9 is CAN1_TX
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1;

	// CAN_RX Pins (AF9): PA11, PB8, PD0
	GPIOD->MODER |= GPIO_MODER_MODER0_1;			// Set alternate function
	GPIOD->AFR[0] |= 9 << 0;						// Alternate function 9 is CAN1_RX


	CAN1->MCR &= ~CAN_MCR_SLEEP;					// This bit is cleared by software to exit Sleep mode
	CAN1->MCR |= CAN_MCR_INRQ;						// Request the CAN hardware enter initialization mode
	while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

	/* Timing p1093
	BaudRate = 1/NominalBitTime
	NominalBitTime = tq + tBS1 + tBS2				// for 500kBaud = 0.000002‬s or 2us

	tBS1 = tq x (TS1[3:0] + 1),
	tBS2 = tq x (TS2[2:0] + 1),
	tq = (BRP[9:0] + 1) x tPCLK

	tPCLK = 1/AP1 Clock = 1/45MHz = 2.22 x 10^-8 = 22.22ns
	ie 90 AP1 clocks for per Bit Time; set tq = (9 + 1) * tPCLK gives 9 time quanta per bit
	 tq + tBS1 + tBS2 = 1 + 4 + 4
	*/
	CAN1->BTR |= CAN_BTR_BRP_Msk & 9;				// Baud rate prescaler. These bits define the length of a time quanta.
	CAN1->BTR &= ~CAN_BTR_TS1;
	CAN1->BTR |= CAN_BTR_TS1 & (3 << 16);			// number of time quanta in Time Segment 1
	CAN1->BTR &= ~CAN_BTR_TS2;
	CAN1->BTR |= CAN_BTR_TS2 & (3 << 20);			// number of time quanta in Time Segment 2
	CAN1->BTR |= CAN_BTR_LBKM;						// Loopback mode for testing

	// CAN Settings
	CAN1->MCR &= ~CAN_MCR_DBF;						// 0: CAN working during debug	1: CAN reception/transmission frozen during debug
	CAN1->MCR |= CAN_MCR_ABOM;						// Automatic bus-off management:  The Bus-Off state is left automatically by hardware once 128 occurrences of 11 recessive	bits have been monitored
	CAN1->MCR |= CAN_MCR_TXFP;						// Transmit FIFO priority: 1: Priority driven by the request order (chronologically)
	CAN1->MCR &= ~CAN_MCR_RFLM;						// Overrun: Last message stored in the FIFO will be overwritten by the new incoming message
	CAN1->MCR |= CAN_MCR_AWUM; 						// AWUM: Automatic wakeup mode:  The Sleep mode is left automatically by hardware on CAN message detection


	/* Filter settings - in 32 bit mode FR1 contains the ID, FR2 contains the mask

	CAN_FMR_CAN2SB:  CAN2 start bank (defaults to 14) - ie 0-13 are CAN1 filters, 14-27 are CAN2 filters

	CAN receive FIFO 0 register CAN_RF0R - FMP0[1:0]: FIFO 0 messages pending
	Interrupt request is generated if the FMPIE bit in the CAN_IER register is set.

	*/
/*
	CAN1->FS1R |= 1 << CAN_FS1R_FSC0_Pos;			// Filter scale: 0: Dual 16-bit scale configuration; 1: Single 32-bit scale configuration
	CAN1->FS1R |= 1 << CAN_FS1R_FSC1_Pos;			// Filter scale: 0: Dual 16-bit scale configuration; 1: Single 32-bit scale configuration
	CAN1->FM1R &= ~CAN_FM1R_FBM0;					// Filter mode register 0: Two 32-bit registers of filter bank x are in Identifier Mask mode. 1: Two 32-bit registers of filter bank x are in Identifier List mode.
	CAN1->FM1R |= CAN_FM1R_FBM1;					// Filter mode register 0: Two 32-bit registers of filter bank x are in Identifier Mask mode. 1: Two 32-bit registers of filter bank x are in Identifier List mode.
	CAN1->FA1R |= CAN_FA1R_FACT0;					// Filter activation register
	CAN1->FA1R |= CAN_FA1R_FACT1;					// Filter activation register
	CAN1->sFilterRegister[0].FR1 = 0x3BC << 21;		// Filter bank 0 register 1: In 32 bit mode bits [31:21] are std ID
	CAN1->sFilterRegister[0].FR2 = 0x4AB << 21;		// Filter bank 0 register 2: In 32 bit mode bits [31:21] are std ID
*/

	CAN1->FM1R &= ~CAN_FM1R_FBM0;					// Filter mode 0: masked filter mode
	//CAN1->FM1R = CAN_FM1R_FBM1;					// Filter mode 1: Exact ID matching
	CAN1->FA1R |= CAN_FA1R_FACT0;					// Filter activation register
	//CAN1->FA1R |= CAN_FA1R_FACT1;					// Filter activation register

	// Clear all filters
	for (uint8_t f = 0; f < 28; ++f) {
		CAN1->sFilterRegister[f].FR1 = 0;
		CAN1->sFilterRegister[f].FR2 = 0;
	}

	// in masked mode identifier is at bit 5, mask is at bit 21
	CAN1->sFilterRegister[0].FR1 = 0x100 << 5;		// Filter bank 0 register 1: Bits [15:5] ID - ie only IDs that start 0x1XX
	CAN1->sFilterRegister[0].FR1 |= 0x100 << 21;	// Filter bank 0 register 2: Bits [31:21] ID Mask
	CAN1->sFilterRegister[0].FR2 = 0x0 << 5;		// Filter bank 1 register 1: Disable filter
	CAN1->sFilterRegister[0].FR2 |= 0x7FF << 21;	// Filter bank 1 register 2: Disable filter mask

	CAN1->FMR &= ~CAN_FMR_FINIT;					// 0=Active filters mode; 1=Initialization mode for the filters.

	CAN1->IER |= CAN_IER_FMPIE0;					// FIFO message pending interrupt enable
	NVIC_SetPriority(CAN1_RX0_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
/*
	NVIC_SetPriority(CAN1_RX1_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(CAN1_RX1_IRQn);
	NVIC_SetPriority(CAN1_SCE_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(CAN1_SCE_IRQn);
*/

	CAN1->MCR &= ~CAN_MCR_INRQ;						// Switch the hardware into normal mode. Hardware signals ready by clearing the INAK bit in the CAN_MSR register.
}

void SendCAN(uint16_t canID, uint32_t lowData, uint32_t highData) {

	// Send CAN Data p1083
	CAN1->sTxMailBox[0].TIR = (uint32_t)0;
	CAN1->sTxMailBox[0].TIR |= canID << CAN_TI0R_STID_Pos;		//  Standard identifier
	CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE;		// Identifier extension: 0=Standard identifier; 1=Extended identifier
	CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;		// 1 RTR: Remote transmission request	0=Data frame; 1=Remote frame

	CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_TGT;		// 0: Time stamp TIME[15:0] is not sent. 1: Time stamp TIME[15:0] value is sent in the last two data bytes of the 8-byte message
	CAN1->sTxMailBox[0].TDTR |= (8 & CAN_TDT0R_DLC);// Data length code
	CAN1->sTxMailBox[0].TDLR = lowData;				// CAN mailbox data low register
	CAN1->sTxMailBox[0].TDHR = highData;			// CAN mailbox data high register
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;		// Set by software to request the transmission for the corresponding mailbox

}


void InitUART() {
	// F427 Discovery STLink connects virtual COM port to PA9 (USART1_TX) and PA10 (USART1_RX) AF7

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;			// UART4 clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// GPIO port A

	GPIOA->MODER |= GPIO_MODER_MODER9_1;			// Set alternate function on PA9
	GPIOA->AFR[1] |= 7 << 4;						// Alternate function on PA9 for UAART1_TX is AF7
	GPIOA->MODER |= GPIO_MODER_MODER10_1;			// Set alternate function on PA10
	GPIOA->AFR[1] |= 7 << 8;						// Alternate function on PA10 for UAART1_RX is AF7

	int Baud = (SystemCoreClock / 4) / (8 * 230400);
	USART1->BRR |= Baud << 4;						// Baud Rate (called USART_BRR_DIV_Mantissa) = (Sys Clock: 180MHz / APB1 Prescaler DIV4: 45MHz) / (8 * 230400) = 24
	USART1->CR1 &= ~USART_CR1_M;					// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	USART1->CR1 |= USART_CR1_RE;					// Receive enable
	USART1->CR1 |= USART_CR1_TE;					// Transmitter enable

	// Set up interrupts
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 3);				// Lower is higher priority
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1 |= USART_CR1_UE;					// USART Enable
}
