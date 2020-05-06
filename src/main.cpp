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
volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[50];
volatile bool uartCmdRdy = false;
bool pageDown = false;

extern "C"
{
	#include "interrupts.h"
}



int main(void) {

	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL - NB Currently done in SystemInit but will need updating for production board
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitLCDHardware();
	InitUART();
	InitSysTick();

	lcd.Init();								// Initialize ILI9341 LCD
	InitCAN();

	while (1) {
		dummyData++;

		for (uint16_t tempId = 0x100; tempId < 0x12E; ++tempId) {
			if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
				SendCAN(tempId, 0x1000FF00, dummyData * tempId);
			}
			for (int t = 0; t < 10000; t++) {}
			can.ProcessCAN();
		}

		if (uartCmdRdy) {
			std::stringstream ss;

			for (uint8_t c = 0; c < 22; ++c) {
				if (uartCmd[c] == 10) {
					can.pendingCmd = ss.str();
					break;
				}
				else
					ss << uartCmd[c];
			}

			uartCmdRdy = false;
		}
		//USART1->DR = 'B';



	}
}
