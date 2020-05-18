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

	/* Example response data:
	0x7E8, 0x001F4104, 0x00000067	Single frame packet
	0x7E8, 0x02491410, 0x30465701	Multi frame packet 02=PID; 49=service; 014=size; 1=first frame of multi frame packet
	*/
	bool SingleFrame() const { return ((dataLow & 0xF0) >> 4) == 0; }
	uint8_t ByteCount() const { return ((dataLow & 0xFF00) >> 8) + ((dataLow & 0x0F) << 8); }		// NB currently only works for multi-frame packets

	// PID may be in different positions for single frame (0 in 0xF0) and multi frame packets (1-3 in 0xF0)
	uint8_t PID() const { return SingleFrame() ? ((dataLow & 0xFF0000) >> 16) : ((dataLow & 0xFF000000) >> 24);	}

	// Service may be in different positions for single frame (0 in 0xF0) and multi frame packets (1-3 in 0xF0)
	uint8_t Service() const { return SingleFrame() ? ((dataLow & 0xF00) >> 8) : ((dataLow & 0xF0000) >> 16); }

	uint8_t A() const { return (dataLow & 0xFF000000) >> 24; }
	uint8_t B() const { return (dataHigh & 0xFF); }
	uint8_t C() const { return (dataHigh & 0xFF00) >> 8; }
	uint8_t D() const { return (dataHigh & 0xFF0000) >> 16; }
	uint16_t AB() const { return ((dataLow & 0xFF000000) >> 16) + (dataHigh & 0xFF); }
	uint32_t ABCD() const { return (dataLow & 0xFF000000) + ((dataHigh & 0xFF) << 16) + (dataHigh & 0xFF00) + ((dataHigh & 0xFF0000) >> 16); }
};

// Holds raw CAN events as they fire interrupts
struct rawCANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
};

enum class PIDCalc { A, AB, ABCD };
struct OBDPid;			// forward declaration

// Holds const lookup of PID names, codes and calculation types
struct PIDItem {
	uint16_t id;
	bool noUpdate;
	std::string name;
	PIDCalc calc;
	std::function<std::string(const OBDPid& o, const uint32_t& v)> calcn;
};

const std::vector<PIDItem> PIDLookup;		// forward declaration

enum class OBDUpdate { noData, hasData, PartialData };

// Holds list of available OBD PIDs
struct OBDPid {
	uint8_t service;
	uint16_t pid;
	std::vector<PIDItem>::const_iterator info;
	uint32_t calcVal;
	uint32_t valMax = 0x00000000;
	uint32_t valMin = 0xFFFFFFFF;
	uint32_t rawData;
	uint32_t dataLow;
	uint32_t dataHigh;
	uint32_t updated;
	uint32_t hits;
	OBDUpdate updateState = OBDUpdate::noData;
	std::vector<uint8_t> multiFrameData;

	uint32_t ABCD() const { return (dataLow & 0xFF000000) + ((dataHigh & 0xFF) << 16) + (dataHigh & 0xFF00) + ((dataHigh & 0xFF0000) >> 16); }

	bool UpdateValues(const std::vector<CANEvent>& events) {
		// Find latest event data - note if the service type is 3 (DTC query) do not check the pid which is actually part of the error code)
		auto event = std::find_if(events.begin(), events.end(), [&] (CANEvent ce)
				{ return (ce.id & 0xF00) == 0x700 && ce.Service() == service && (service == 3 || ce.PID() == pid); } );
		if (event == events.end()) {
			return false;
		}
		dataLow = event->dataLow;
		dataHigh = event->dataHigh;
		updated = event->updated;
		hits = event->hits;

		// Check if we have lookup info for the OBD item
		if (info != PIDLookup.end()) {
			// Store the value used to generate calculations (usually the A or AB bytes)
			if (info->calc == PIDCalc::A)			calcVal = event->A();
			else if (info->calc == PIDCalc::AB)		calcVal = event->AB();
			else 									calcVal = ABCD();

			// capture minimum and maximum values
			if (calcVal > valMax)	valMax = calcVal;
			if (calcVal < valMin)	valMin = calcVal;

			updateState = OBDUpdate::hasData;
		}
		return true;
	}

	void AddToMultiFrame(const uint32_t& d, uint8_t start) {
		for (; start < 4; ++start) {
			multiFrameData.push_back((d >> (start * 8)) & 0xFF);
		}
	}
};

enum class OBDMode { Off, Query, Info };
enum class OBDState { Start, PIDQuery, List, Update };

class CANHandler {
public:
	rawCANEvent Queue[CANQUEUESIZE];
	uint8_t QueueRead = 0;
	uint8_t QueueWrite = 0;
	uint8_t QueueSize = 0;
	std::string pendingCmd;
	uint32_t OBDCmd;
#ifndef TESTMODE
	OBDMode Mode  = OBDMode::Off;
#else
	OBDMode Mode  = OBDMode::Info;
#endif
	void ProcessQueue();
	void ProcessOBD();
	void SendCAN(const uint16_t& canID, const uint32_t& dataLow, const uint32_t& dataHigh);
	void LogMsg(const uint16_t& canID, const uint32_t& dataLow, const uint32_t& dataHigh);
private:
	uint8_t CANPos = 0;
	uint16_t pageNo = 0;
	const uint8_t CANDrawHeight = 205;
	bool viewIDMode = false;
	bool freeze = false;
	bool viewRaw = false;
	bool OBDCmdPending = false;
	OBDState OBDInfoState = OBDState::Start;
	uint16_t PIDCounter = 0;
	uint8_t ServiceCounter = 0;
	uint16_t PIDQueryErrors = 0;

	std::vector<CANEvent> CANEvents;
	std::vector<CANEvent>::iterator viewEvent;
	std::vector<OBDPid> OBDAvPIDs;
	std::vector<OBDPid>::iterator viewPid;

	void DrawPids(OBDPid& pid);
	void DrawPid();
	void DrawEvents(const CANEvent& event);
	void DrawEvent();
	void DrawUI();
	bool ProcessCmd();
	void OBDQueryMode(const std::string& s);
	std::vector<PIDItem>::const_iterator GetPIDLookup(const uint8_t& service, const uint16_t& id);
	void InjectTestData();
	void testInsert(const uint16_t& id, const uint32_t& dataLow, const uint32_t& dataHigh);
	void RandTestData(const OBDPid& pid);

	std::string FloatToString(const float& f, const bool& smartFormat);
	std::string CANWordToBytes(const uint32_t& w);
	std::string CANIdToHex(const uint16_t& v);
	std::string DTCCode(const uint16_t& c);
	std::string IntToString(const int32_t& v);
	std::string HexToString(const uint32_t& v, const bool& spaces = false);
	std::string HexByte(const uint16_t& v);

	const std::vector<PIDItem> PIDLookup {
		{0x104, false, "Engine load",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55, false) + "%  "; } },
		{0x105, false, "Coolant temp",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x10B, false, "Manifold Prs",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " kPa  "; } },
		{0x10C, false, "RPM",			PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v / 4.0) + " rpm   "; } },
		{0x10D, false, "Speed",			PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " km/h   "; } },
		{0x10F, false, "In Air Temp",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x110, false, "Air flow",		PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 100.0, false) + " g/s   "; } },
		{0x111, false, "Throttle pos",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55, false) + "%  "; } },
		{0x112, false, "Sec air stat",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x11C, true,  "OBD standard",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x11F, false, "Run time",		PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " s"; } },
		{0x121, false, "Dist w Error",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " km"; } },
		{0x123, false, "Fuel Rail Pr",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v * 10) + " kPa   "; } },
		{0x14f, false, "Misc max val",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x300, false, "DTC Codes",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return DTCCode((v & 0xFFFF0000) >> 16) + " " + DTCCode(v & 0xFFFF); } },
		{0x902, true,  "VIN",			PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return std::string(o.multiFrameData.cbegin() + 1, o.multiFrameData.cend()); } },
		{0x904, true,  "CALIB ID",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return std::string(o.multiFrameData.cbegin() + 1, o.multiFrameData.cend()); } }
	};
};

