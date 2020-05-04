#include "initialisation.h"
#include "lcd.h"


extern uint32_t SystemCoreClock;
volatile uint32_t SysTickVal = 0;

LCD lcd;

uint32_t canDataLow = 0, canDataHigh = 0;
uint16_t canID = 0;

extern "C"
{
	#include "interrupts.h"
}

std::string floatToString(float f, bool smartFormat) {
	std::string s;
	std::stringstream ss;

	if (smartFormat && f > 10000) {
		ss << (int16_t)std::round(f / 100);
		s = ss.str();
		s.insert(s.length() - 1, ".");
		s+= "k";
	} else if (smartFormat && f > 1000) {
		ss << (int16_t)std::round(f);
		s = ss.str();
	} else	{
		ss << (int32_t)std::round(f * 10);
		s = ss.str();
		s.insert(s.length() - 1, ".");
	}
	return s;
}


std::string intToString(uint32_t v) {
	std::stringstream ss;
	ss << v;
	return ss.str();
}

std::string intToHexString(uint32_t v) {
	std::stringstream ss;
	ss << "0x";
	ss << std::hex << v;
	return ss.str();
}

int main(void) {

	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL - NB Currently done in SystemInit but will need updating for production board
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitLCDHardware();
//	InitSysTick();

	lcd.Init();								// Initialize ILI9341 LCD
	InitCAN();
	lcd.DrawString(10, 10, "Hello", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	while (1) {
		if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			SendCAN(0x4AB, 0x1000FF00, 0x452038F1);
		}
		for (int t = 0; t < 10000; t++) {}
		if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			SendCAN(0x3BC, 0xAABBCCDD, 0x99887766);
		}
		for (int t = 0; t < 10000; t++) {}
		lcd.DrawString(10, 10, "ID:" + intToHexString(canID), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(10, 30, "Low:" + intToHexString(canDataLow), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(10, 50, "High:" + intToHexString(canDataHigh), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	}
}
