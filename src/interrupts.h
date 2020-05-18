// CAN1 Message received
void CAN1_RX0_IRQHandler(void) {
	//The software reads out the mailbox content and releases it by setting the RFOM bit in the	CAN_RFR register
/*
	canDataLow = CAN1->sFIFOMailBox[0].RDLR;
	canDataHigh = CAN1->sFIFOMailBox[0].RDHR;
	canID = CAN1->sFIFOMailBox[0].RIR >> 21;
*/

	can.Queue[can.QueueWrite].id = CAN1->sFIFOMailBox[0].RIR >> 21;
	can.Queue[can.QueueWrite].dataLow = CAN1->sFIFOMailBox[0].RDLR;
	can.Queue[can.QueueWrite].dataHigh = CAN1->sFIFOMailBox[0].RDHR;

	CAN1->RF0R |= CAN_RF0R_RFOM0;		// Mark the contents of the FIFO as read

	can.LogMsg(can.Queue[can.QueueWrite].id, can.Queue[can.QueueWrite].dataLow, can.Queue[can.QueueWrite].dataHigh);

	can.QueueSize++;
	can.QueueWrite = (can.QueueWrite + 1) % CANQUEUESIZE;

}

// USART Decoder
void USART1_IRQHandler(void) {
	if (USART1->SR | USART_SR_RXNE && !uartCmdRdy) {
		uartCmd[uartCmdPos] = USART1->DR; 				// accessing DR automatically resets the receive flag
		if (uartCmd[uartCmdPos] == 10) {
			uartCmdRdy = true;
			uartCmdPos = 0;
		} else {
			uartCmdPos++;
		}
	}
}

/*
#ifdef STM32F446xx

void EXTI15_10_IRQHandler(void) {

	// Left Encoder Button
	if (EXTI->PR & L_BTN_NO(EXTI_PR_PR,)) {
		if (!(L_BTN_GPIO->IDR & L_BTN_NO(GPIO_IDR_IDR_,))) 			// Encoder button pressed - L_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_2
			DB_ON													// Enable debounce timer
		if (L_BTN_GPIO->IDR & L_BTN_NO(GPIO_IDR_IDR_,) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
			encoderBtnL = true;
			DB_OFF													// Disable debounce timer
		}
		EXTI->PR |= L_BTN_NO(EXTI_PR_PR,);							// Clear interrupt pending
	}

	if (EXTI->PR & R_BTN_NO(EXTI_PR_PR,)) {
		// Right Encoder Button
		if (!(R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_,))) 			// Encoder button pressed - R_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_7
			DB_ON													// Enable debounce timer
		if (R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_,) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
			encoderBtnR = true;
			DB_OFF													// Disable debounce timer
		}
		EXTI->PR |= R_BTN_NO(EXTI_PR_PR,);							// Clear interrupt pending
	}
}



#else

// Left Encoder Button
void L_BTN_NO(EXTI, _IRQHandler)(void) {

	if (!(L_BTN_GPIO->IDR & L_BTN_NO(GPIO_IDR_IDR_,))) 			// Encoder button pressed - L_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_2
		DB_ON													// Enable debounce timer
	if (L_BTN_GPIO->IDR & L_BTN_NO(GPIO_IDR_IDR_,) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
		encoderBtnL = true;
		DB_OFF													// Disable debounce timer
	}
	EXTI->PR |= L_BTN_NO(EXTI_PR_PR,);							// Clear interrupt pending
}

// Right Encoder Button
void EXTI9_5_IRQHandler(void) {

	if (!(R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_,))) 			// Encoder button pressed - R_BTN_NO() adds number of encoder button eg GPIO_IDR_IDR_7
		DB_ON													// Enable debounce timer
	if (R_BTN_GPIO->IDR & R_BTN_NO(GPIO_IDR_IDR_,) && TIM5->CNT > 100) {	// Encoder button released - check enough time has elapsed to ensure not a bounce. A quick press if around 300, a long one around 8000+
		encoderBtnR = true;
		DB_OFF													// Disable debounce timer
	}
	EXTI->PR |= R_BTN_NO(EXTI_PR_PR,);							// Clear interrupt pending
}

#endif

//	Coverage timer
void TIM1_BRK_TIM9_IRQHandler(void) {
	TIM9->SR &= ~TIM_SR_UIF;									// clear UIF flag
	coverageTimer ++;
}
*/
void SysTick_Handler(void) {
	SysTickVal++;
}

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
