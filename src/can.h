#pragma once

#include "initialisation.h"
#include <lcd.h>

#define CANQUEUESIZE 30
#define CANDRAWHEIGHT 17
#define CANPAGEITEMS 11
extern LCD lcd;

struct CANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
};

// Holds raw CAN events as they fire interrupts
struct rawCANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
};

class CANHandler {
public:
	void ProcessCAN();
	rawCANEvent Queue[CANQUEUESIZE];
	uint8_t QueueRead = 0;
	uint8_t QueueWrite = 0;
	uint8_t QueueSize = 0;
	std::string pendingCmd;
private:
	uint8_t CANPos = 0;
	uint16_t pageNo = 0;
	bool viewIDMode = false;
	std::deque<CANEvent>::iterator viewID;
	std::deque<CANEvent> CANEvents;
	void DrawEvent(const CANEvent& event);
	void DrawUI();
	void QueueInc();
};
