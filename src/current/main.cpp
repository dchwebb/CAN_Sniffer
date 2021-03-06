#include "initialisation.h"
#include "lcd.h"
#include "can.h"

extern uint32_t SystemCoreClock;
volatile uint32_t SysTickVal = 0;

LCD lcd;
CANHandler can;

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[50];
volatile bool uartCmdRdy = false;


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


//	CANUpdateFilters(0x700, 0x700);							// FIXME - should be 0x7E0 ?
//	uint32_t start = SysTickVal;
//	uint8_t q = can.QueueSize;
//	can.SendCAN(0x7DF, 0xCC020902, 0xCCCCCCCC);
//	while (q == can.QueueSize) {}
//
//	q = can.QueueSize;
//	if (false) {
//		can.SendCAN(0x7E0, 0xCC000030, 0xCCCCCCCC);
//	} else {
//		can.SendCAN(0x7E0, 0xCC100030, 0xCCCCCCCC);
//	}
//	uint32_t waitCount = 0;
//	while (waitCount < 0xFFFFFFFF && q == can.QueueSize) {
//		waitCount++;
//	}
//	uint32_t end = SysTickVal;
//	uint32_t time = end - start;
//
//	int x = 0;

	while (1) {


		// Check if a UART command has been received
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

		// If in one of the query modes send the specified query command
		if (can.Mode == OBDMode::Info) {
			can.ProcessOBD();
			for (int t = 0; t < 10000; t++) {}
		} else {
			if (can.Mode == OBDMode::Query) {
				can.SendCAN(0x7DF, can.OBDCmd, 0xCCCCCCCC);			// Generic command
				for (int t = 0; t < 80000; t++) {}
			}
			can.ProcessQueue();
		}


	}

}
