#pragma once

#include "initialisation.h"
#include <lcd.h>

#define CANQUEUESIZE 50
#define CANTIMEOUT 100000
extern LCD lcd;


struct CANEvent {
	uint16_t id;
	uint32_t dataLow;
	uint32_t dataHigh;
	uint32_t updated = 0;
	uint32_t hits = 0;

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

enum class PIDCalc { A, AB, B, ABCD };
struct OBDPid;			// forward declaration

// Holds const lookup of PID names, codes and calculation types
struct PIDItem {
	uint16_t id;
	bool noUpdate;
	std::string name;
	PIDCalc calc;
	std::function<std::string(const OBDPid& o, const uint32_t& v)> calcn;
};

// Holds list of available OBD PIDs
struct OBDPid : CANEvent {
	uint8_t service;
	uint16_t pid;
	std::vector<PIDItem>::const_iterator info;
	uint32_t calcVal = 0;
	uint32_t valMax = 0x00000000;
	uint32_t valMin = 0xFFFFFFFF;
	uint32_t rawData = 0;
	std::vector<uint8_t> multiFrameData;

	OBDPid(const uint8_t& service, const uint16_t& pid, const std::vector<PIDItem>::const_iterator& info)
		: service{service}, pid{pid}, info{info} {}

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
	const uint8_t DrawHeight = 208;		// Bottom of main list/display area
	const uint8_t RowHeight = 18;			// Row height of list items
	const uint8_t RowCount = 11;			// Number of items in display lists
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
	void TestMultiFrame(const OBDPid& pid);
	void TestInsert(const uint16_t& id, const uint32_t& dataLow, const uint32_t& dataHigh);
	void RandTestData(const OBDPid& pid);

	std::string FloatToString(const float& f, const bool& smartFormat = false);
	std::string CANWordToBytes(const uint32_t& w);
	std::string CANIdToHex(const uint16_t& v);
	std::string DTCCode(const uint16_t& c);
	std::string IntToString(const int32_t& v);
	std::string HexToString(const uint32_t& v, const bool& spaces = false);
	std::string HexByte(const uint16_t& v);
	std::string BinToString(const uint8_t& b);
	std::string MultiFrameToString(const std::vector<uint8_t>& mf, const uint8_t& start);
	uint32_t HexStringToOBD(const std::string& s);

	const std::vector<PIDItem> PIDLookup {
		{0x101, false, "Errors/DTC",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x104, false, "Engine load",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55, false) + "%  "; } },
		{0x105, false, "Coolant temp",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x10B, false, "Manifold prs",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " kPa  "; } },
		{0x10C, false, "RPM",			PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v / 4.0) + " rpm   "; } },
		{0x10D, false, "Speed",			PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " km/h   "; } },
		{0x10F, false, "In air temp",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x110, false, "Air flow",		PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 100.0) + " g/s   "; } },
		{0x111, false, "Throttle pos",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55) + "%  "; } },
		{0x112, false, "Sec air stat",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x113, false, "Oxygen sens",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return "B1:" + BinToString(v & 0xF) + " B2:" + BinToString((v & 0xF0) >> 8); } },
		{0x11C, true,  "OBD standard",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v); } },
		{0x11F, false, "Run time",		PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " s"; } },
		{0x121, false, "Dist w error",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " km"; } },
		{0x123, false, "Fuel rail pr",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v * 10) + " kPa   "; } },
		{0x130, false, "Warm ups",		PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v); } },
		{0x131, false, "Dist s w ups",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return IntToString(v); } },
		{0x133, false, "Bar Pressure",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v) + " kPa  "; } },
		{0x134, false, "Ox Sensor 1",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },		// FIXME Two part encoding
		{0x141, false, "Monitor stat",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },		// FIXME Bit Encoded
		{0x142, false, "Ctl module V",	PIDCalc::AB,	[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 1000.0); } },
		{0x145, false, "Rel throttle",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55) + "%  "; } },
		{0x146, false, "Amb air temp",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x149, false, "Accelr pos D",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55) + "%  "; } },
		{0x14A, false, "Accelr pos E",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55) + "%  "; } },
		{0x14C, false, "C throttle a",	PIDCalc::A,		[&](const OBDPid& o, const uint32_t& v){ return FloatToString((float)v / 2.55) + "%  "; } },
		{0x14F, false, "Misc max val",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x167, false, "Eng cool tmp",	PIDCalc::B,		[&](const OBDPid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },		// FIXME - this is a guess
		{0x169, false, "EGR Error",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return "Encoded"; } },
		{0x177, false, "CAC temp",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return "Encoded"; } },
		{0x178, false, "E Gas temp 1",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return o.multiFrameData.size() > 0 ? "Supported: " + BinToString(o.multiFrameData[0] & 0xF) : ""; } },	// FIXME subsequent bytes are temperatures
		{0x300, false, "DTC codes",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return DTCCode((v & 0xFFFF0000) >> 16) + " " + DTCCode(v & 0xFFFF); } },
		{0x902, false, "VIN",			PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return MultiFrameToString(o.multiFrameData, 1); } },
		{0x904, false, "Calib ID",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return MultiFrameToString(o.multiFrameData, 1); } },
		{0x906, false, "Calib Ver N",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x90A, false, "ECU name",		PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return MultiFrameToString(o.multiFrameData, 1); } },
		{0x90B, false, "Perf track",	PIDCalc::ABCD,	[&](const OBDPid& o, const uint32_t& v){ return HexToString(v, true); } }
	};
};


