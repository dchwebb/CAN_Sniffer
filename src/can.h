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

struct OBD2Pid {
	uint32_t pid;
};

enum class OBD2Mode { Off, Query, Info };
enum class OBD2State { Start, PIDQuery00, PIDQuery20, PIDQuery40, PIDQuery60, Update };

class CANHandler {
public:
	void ProcessCAN();
	rawCANEvent Queue[CANQUEUESIZE];
	uint8_t QueueRead = 0;
	uint8_t QueueWrite = 0;
	uint8_t QueueSize = 0;
	std::string pendingCmd;
	uint32_t OBD2Cmd;
	bool sendTestData = false;
	OBD2Mode Mode  = OBD2Mode::Off;

	void OBD2Info();
private:
	uint8_t CANPos = 0;
	uint16_t pageNo = 0;
	const uint8_t CANDrawHeight = 205;
	bool viewIDMode = false;
	bool freeze = false;
	OBD2State OBD2InfoState = OBD2State::Start;
	std::vector<CANEvent>::iterator viewID;
	std::vector<CANEvent> CANEvents;
	std::vector<OBD2Pid> OBD2AvailablePIDs;

	void DrawList(const CANEvent& event);
	std::string CANWordToBytes(const uint32_t& w);
	std::string CANIdToHex(const uint16_t& v);
	std::string IntToString(const uint32_t& v);
	std::string HexToString(const uint32_t& v);
	std::string HexByte(const uint16_t& v);
	void DrawId();
	void DrawUI();
	bool ProcessCmd();
	void OBD2QueryMode(const std::string& s);
};

