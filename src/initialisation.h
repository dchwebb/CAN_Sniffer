#pragma once

#include "stm32f4xx.h"
#include <cmath>
#include <string>
#include <sstream>
#include <queue>
#include <deque>
#include <array>
#include <vector>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <cstdlib>
#include <functional>

#define TESTMODE

// Coverage profiler macros using timer 4 to count clock cycles / 10
#define CP_ON		TIM9->EGR |= TIM_EGR_UG; TIM9->CR1 |= TIM_CR1_CEN; coverageTimer=0;
#define CP_OFF		TIM9->CR1 &= ~TIM_CR1_CEN;
#define CP_CAP		TIM9->CR1 &= ~TIM_CR1_CEN; coverageTotal = (coverageTimer * 65536) + TIM9->CNT;

// Button debounce timer
#define DB_ON		TIM5->EGR |= TIM_EGR_UG; TIM5->CR1 |= TIM_CR1_CEN;
#define DB_OFF		TIM5->CR1 &= ~TIM_CR1_CEN;

//	Define encoder pins and timers for easier reconfiguring
#ifdef STM32F446xx
#define L_ENC_CNT	TIM4->CNT
#elif STM32F429xx
#define L_ENC_CNT	TIM1->CNT
#else
#define L_ENC_CNT	TIM8->CNT
#endif

#ifdef STM32F446xx
#define R_ENC_CNT	TIM8->CNT
#else
#define R_ENC_CNT	TIM4->CNT
#endif

#ifdef STM32F446xx
#define L_BTN_NO(a, b) a ## 10 ## b
#define L_BTN_GPIO	GPIOA
#else
#define L_BTN_NO(a, b) a ## 4 ## b
#define L_BTN_GPIO	GPIOB
#endif

#ifdef STM32F446xx
#define R_BTN_NO(a, b) a ## 13 ## b
#elif STM32F429xx
#define R_BTN_NO(a, b) a ## 7 ## b
#else
#define R_BTN_NO(a, b) a ## 2 ## b
#endif

#ifdef STM32F446xx
#define R_BTN_GPIO	GPIOB
#else
#define R_BTN_GPIO	GPIOA
#endif

// Define LCD DMA and SPI registers
#ifdef STM32F429xx
#define LCD_DMA_STREAM			DMA2_Stream6
#define LCD_SPI 				SPI5
#define LCD_CLEAR_DMA_FLAGS		DMA2->HIFCR = DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6;
#else
#define LCD_DMA_STREAM			DMA1_Stream5
#define LCD_SPI 				SPI3
#define LCD_CLEAR_DMA_FLAGS		DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5;
#endif

// Define macros for setting and clearing GPIO SPI pins
#ifdef STM32F446xx
#define LCD_RST_RESET	GPIOC->BSRR |= GPIO_BSRR_BR_14
#define LCD_RST_SET 	GPIOC->BSRR |= GPIO_BSRR_BS_14
#define LCD_DCX_RESET	GPIOC->BSRR |= GPIO_BSRR_BR_13
#define LCD_DCX_SET		GPIOC->BSRR |= GPIO_BSRR_BS_13
#else
#define LCD_RST_RESET	GPIOD->BSRR |= GPIO_BSRR_BR_12
#define LCD_RST_SET 	GPIOD->BSRR |= GPIO_BSRR_BS_12
#define LCD_DCX_RESET	GPIOD->BSRR |= GPIO_BSRR_BR_13
#define LCD_DCX_SET		GPIOD->BSRR |= GPIO_BSRR_BS_13
#endif

extern volatile uint32_t SysTickVal;

void SystemClock_Config();
void InitSysTick();
void InitLCDHardware();
void InitDebounceTimer();
void InitEncoders();
void CANUpdateFilters(const uint16_t& id, const uint16_t& mask);
void InitCAN();
void SendCAN(const uint16_t& canID, const uint32_t& lowData, const uint32_t& highData, const bool& rtr = false);
void InitUART();
void uartSendChar(const char& c);
void uartSendString(const std::string& s);

