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

	uint8_t PID() const { return (dataLow & 0xFF0000) >> 16; }
	uint8_t Service() const { return (dataLow & 0xF00) >> 8; }
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
struct OBD2Pid;			// forward declaration

// Holds const lookup of PID names, codes and calculation types
struct PIDItem {
	uint16_t id;
	std::string name;
	PIDCalc calc;
	std::function<std::string(const OBD2Pid& o, const uint32_t& v)> calcn;
};

const std::vector<PIDItem> PIDLookup;		// forward declaration

// Holds list of available OBD2 PIDs
struct OBD2Pid {
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

		// Check if we have lookup info for the OBD2 item
		if (info != PIDLookup.end()) {
			// Store the value used to generate calculations (usually the A or AB bytes)
			if (info->calc == PIDCalc::A)			calcVal = event->A();
			else if (info->calc == PIDCalc::AB)		calcVal = event->AB();
			else 									calcVal = ABCD();

			// capture minimum and maximum values
			if (calcVal > valMax)	valMax = calcVal;
			if (calcVal < valMin)	valMin = calcVal;
		}
		return true;
	}
};

enum class OBD2Mode { Off, Query, Info };
enum class OBD2State { Start, PIDQuery, List, Update };

class CANHandler {
public:
	rawCANEvent Queue[CANQUEUESIZE];
	uint8_t QueueRead = 0;
	uint8_t QueueWrite = 0;
	uint8_t QueueSize = 0;
	std::string pendingCmd;
	uint32_t OBD2Cmd;
#ifndef TESTMODE
	OBD2Mode Mode  = OBD2Mode::Off;
#else
	OBD2Mode Mode  = OBD2Mode::Info;
#endif
	void ProcessCAN();
	void OBD2Info();
private:
	uint8_t CANPos = 0;
	uint16_t pageNo = 0;
	const uint8_t CANDrawHeight = 205;
	bool viewIDMode = false;
	bool freeze = false;
	OBD2State OBD2InfoState = OBD2State::Start;
	uint16_t PIDCounter = 0;
	uint16_t ServiceCounter = 0;
	uint16_t PIDQueryErrors = 0;

	std::vector<CANEvent> CANEvents;
	std::vector<CANEvent>::iterator viewEvent;
	std::vector<OBD2Pid> OBD2AvailablePIDs;
	std::vector<OBD2Pid>::iterator viewPid;

	std::string FloatToString(const float& f, const bool& smartFormat);
	std::string CANWordToBytes(const uint32_t& w);
	std::string CANIdToHex(const uint16_t& v);
	std::string DTCCode(const uint16_t& c);
	std::string IntToString(const uint32_t& v);
	std::string HexToString(const uint32_t& v, const bool& spaces = false);
	std::string HexByte(const uint16_t& v);

	void DrawPids(OBD2Pid& pid);
	void DrawPid();
	void DrawEvents(const CANEvent& event);
	void DrawEvent();
	void DrawUI();
	bool ProcessCmd();
	void OBD2QueryMode(const std::string& s);
	std::vector<PIDItem>::const_iterator GetPIDLookup(const uint8_t& service, const uint16_t& id);

	const std::vector<PIDItem> PIDLookup {
		{0x104, "Engine load",	PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return FloatToString((float)v / 2.55, false) + "%  "; } },
		{0x105, "Coolant temp",	PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x10B, "Manifold Prs",	PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v) + " kPa  "; } },
		{0x10C, "RPM",			PIDCalc::AB,	[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v / 4.0) + " rpm   "; } },
		{0x10D, "Speed",		PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v) + " km/h   "; } },
		{0x10F, "In Air Temp",	PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v - 40) + " C  "; } },
		{0x110, "Air flow",		PIDCalc::AB,	[&](const OBD2Pid& o, const uint32_t& v){ return FloatToString((float)v / 100.0, false) + " g/s   "; } },
		{0x111, "Throttle pos",	PIDCalc::A,		[&](const OBD2Pid& o, const uint32_t& v){ return FloatToString((float)v / 2.55, false) + "%  "; } },
		{0x112, "Sec air stat",	PIDCalc::ABCD,	[&](const OBD2Pid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x11C, "OBD standard",	PIDCalc::ABCD,	[&](const OBD2Pid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x11F, "Run time",		PIDCalc::AB,	[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v) + " s"; } },
		{0x121, "Dist w Error",	PIDCalc::AB,	[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v) + " km"; } },
		{0x123, "Fuel Rail Pr",	PIDCalc::AB,	[&](const OBD2Pid& o, const uint32_t& v){ return IntToString(v * 10) + " kPa   "; } },
		{0x14f, "Misc max val",	PIDCalc::ABCD,	[&](const OBD2Pid& o, const uint32_t& v){ return HexToString(v, true); } },
		{0x300, "DTC Codes",	PIDCalc::ABCD,	[&](const OBD2Pid& o, const uint32_t& v){ return DTCCode((v & 0xFFFF0000) >> 16) + " " + DTCCode(v & 0xFFFF); } }
	};
};

