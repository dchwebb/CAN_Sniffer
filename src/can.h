#pragma once

#include "initialisation.h"
#include <lcd.h>

#define CANQUEUESIZE 30
#define CANDRAWHEIGHT 17
#define CANPAGEITEMS 11

extern LCD lcd;

// 0xCC0C0102
struct CANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
	uint32_t updated;
	uint32_t hits;

	uint8_t PID() const {
		return (dataLow & 0xFF0000) >> 16;
	}
	uint8_t A() const {
		return (dataLow & 0xFF000000) >> 24;
	}
	uint8_t B() const {
		return (dataHigh & 0xFF);
	}
	uint8_t C() const {
		return (dataHigh & 0xFF00) >> 8;
	}
	uint8_t D() const {
		return (dataHigh & 0xFF0000) >> 16;
	}
	uint8_t AB() const {
		return ((dataLow & 0xFF000000) >> 16) + (dataHigh & 0xFF);
	}
};


// Holds raw CAN events as they fire interrupts
struct rawCANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
};

struct OBD2Pid {
	uint16_t pid;
};

enum class PIDCalc { AB, ABl40, APercent, ABdiv4 };

struct PIDItem {
	uint8_t id;
	std::string name;
	PIDCalc calc;
};

enum class OBD2Mode { Off, Query, Info };
enum class OBD2State { Start, PIDQuery, List, Update };

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
	uint16_t PIDCounter = 0;
	uint16_t PIDQueryErrors = 0;

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

	const std::vector<PIDItem> PIDLookup {
		{0x4, "Engine load", PIDCalc::APercent },
		{0xC, "RPM", PIDCalc::ABdiv4 }
	};
};

