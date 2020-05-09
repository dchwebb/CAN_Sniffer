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
	uint32_t updated;
	uint32_t hits;
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
	bool sendTestData = false;
private:
	uint8_t CANPos = 0;
	uint16_t pageNo = 0;
	const uint8_t CANDrawHeight = 205;
	bool viewIDMode = false;
	std::deque<CANEvent>::iterator viewID;
	std::deque<CANEvent> CANEvents;
	void DrawList(const CANEvent& event);
	std::string CANWordToBytes(const uint32_t& w);
	std::string CANIdToHex(const uint16_t& v);
	void DrawId();
	void DrawUI();
	void QueueInc();
};
