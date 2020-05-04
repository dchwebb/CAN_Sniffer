#include "initialisation.h"
#include "lcd.h"
#include "can.h"


extern uint32_t SystemCoreClock;
volatile uint32_t SysTickVal = 0;

LCD lcd;
CANHandler can;

uint32_t canDataLow = 0, canDataHigh = 0;
uint16_t canID = 0;
uint32_t dummyData = 0;

extern "C"
{
	#include "interrupts.h"
}



int main(void) {

	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL - NB Currently done in SystemInit but will need updating for production board
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitLCDHardware();
//	InitSysTick();

	lcd.Init();								// Initialize ILI9341 LCD
	InitCAN();
	//lcd.DrawString(10, 10, "Hello", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	while (1) {
		dummyData++;
		can.ProcessCAN();

		if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			SendCAN(0x4AB, 0x1000FF00, dummyData);
		}
		for (int t = 0; t < 10000; t++) {}

		//can.ProcessCAN();
		if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			SendCAN(0x3BC, 0xAABBCCDD, ~dummyData);
		}
		for (int t = 0; t < 10000; t++) {}

		//can.ProcessCAN();
		if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			SendCAN(0x222, 0x99887766, dummyData * 2);
		}
		for (int t = 0; t < 10000; t++) {}

		/*
		lcd.DrawString(10, 10, "ID:" + intToHexString(canID), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(10, 30, "Low:" + intToHexString(canDataLow), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(10, 50, "High:" + intToHexString(canDataHigh), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
*/
	}
}
