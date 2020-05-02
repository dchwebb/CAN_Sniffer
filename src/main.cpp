#include "initialisation.h"
//#include "lcd.h"


extern uint32_t SystemCoreClock;
volatile uint32_t SysTickVal = 0;

//LCD lcd;

extern "C"
{
//	#include "interrupts.h"
}



int main(void) {

	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL - NB Currently done in SystemInit but will need updating for production board
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
//	InitLCDHardware();
//	InitSysTick();

//	lcd.Init();								// Initialize ILI9341 LCD

	while (1) {


	}
}
