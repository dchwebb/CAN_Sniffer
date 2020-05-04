#pragma once

#include "initialisation.h"
#include <lcd.h>

#define CANQUEUESIZE 20
#define CANDRAWHEIGHT 17
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
private:
	uint8_t CANPos = 0;
	std::deque<CANEvent> CANEvents;
	void DrawEvent(const CANEvent& event);
	void QueueInc();
};
