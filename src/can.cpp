#include <can.h>

std::string CANHandler::FloatToString(const float& f, const bool& smartFormat) {
	std::string s;
	std::stringstream ss;

	if (f == 0) {
		s = "0";
	} else if (smartFormat && f > 10000) {
		ss << (int16_t)std::round(f / 100);
		s = ss.str();
		s.insert(s.length() - 1, ".");
		s+= "k";
	} else if (smartFormat && f > 1000) {
		ss << (int16_t)std::round(f);
		s = ss.str();
	} else	{
		ss << (int32_t)std::round(f * 10);
		s = ss.str();
		s.insert(s.length() - 1, ".");
	}
	return s;
}

std::string CANHandler::IntToString(const uint32_t& v) {
	std::stringstream ss;
	ss << v;
	return ss.str();
}

std::string CANHandler::HexToString(const uint32_t& v, const bool& spaces) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << v;
	if (spaces) {
		//std::string s = ss.str();
		return ss.str().insert(2, " ").insert(5, " ").insert(8, " ");
	}
	return ss.str();
}

std::string CANHandler::HexByte(const uint16_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << v;
	return ss.str();
}

std::string CANHandler::CANIdToHex(const uint16_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(3) << std::hex << v;
	return ss.str();
}

std::string CANHandler::DTCCode(const uint16_t& c) {
	std::stringstream ss;
	// First two bits of A are error type code
	switch((0xC000 & c) >> 14) {
		case 0b00: ss << "P"; break;
		case 0b01: ss << "C"; break;
		case 0b10: ss << "B"; break;
		case 0b11: ss << "U"; break;
	}
	//	remaining bits are BCD
	ss << std::hex << ((0x3000 & c) >> 12);
	ss << std::hex << ((0x0F00 & c) >> 8);
	ss << std::hex << ((0x00F0 & c) >> 4);
	ss << std::hex << (0x000F & c);

	return ss.str();
}

std::string CANHandler::CANWordToBytes(const uint32_t& w) {
	std::stringstream ss;

	for (uint8_t c = 0; c < 4; ++c) {
		ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (w >> (8 * (c % 4)) & 0xFF);
		if (c < 3)
			ss << ' ';
	}
	return ss.str();
}

// Process incoming CAN messages
void CANHandler::ProcessCAN() {

	while (QueueSize > 0) {
		bool edited = false;
		rawCANEvent nextEvent = Queue[QueueRead];
		QueueSize--;
		QueueRead = (QueueRead + 1) % CANQUEUESIZE;

		//	Overwrite last event if the id is the same
		if (!CANEvents.empty()) {

			auto event = CANEvents.begin();
			if (Mode == OBD2Mode::Info) {
				// Check if first three bytes match
				event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)
						{ return ce.id == nextEvent.id && (ce.dataLow & 0xFFFFFF) == (nextEvent.dataLow & 0xFFFFFF); } );
			} else {
				event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce) { return ce.id == nextEvent.id; } );
			}

			//if (event->id == nextEvent.id) {
			if (event != CANEvents.end()) {
				if (event->dataLow != nextEvent.dataLow || event->dataHigh != nextEvent.dataHigh)
					event->updated = SysTickVal;
				event->dataLow = nextEvent.dataLow;
				event->dataHigh = nextEvent.dataHigh;
				event->hits++;
				edited = true;
			}
		}

		if (!edited)
			CANEvents.push_back({nextEvent.id, nextEvent.dataLow, nextEvent.dataHigh, SysTickVal, 0});

	}

	// Draw CAN events one at a time
	if (CANEvents.size() > 0) {
		if (viewIDMode) {
			if (Mode == OBD2Mode::Info)
				DrawPid();
			else
				DrawEvent();

			DrawUI();
		} else {
			if (CANPos >= CANPAGEITEMS || (uint16_t)(CANPos + (pageNo * CANPAGEITEMS)) >= (uint16_t)(Mode == OBD2Mode::Info ? OBD2AvailablePIDs.size() : CANEvents.size())) {
				CANPos = 0;
				DrawUI();
			} else {
				uint16_t i = CANPos + (pageNo * CANPAGEITEMS);
				if (Mode == OBD2Mode::Info)
					DrawPids(OBD2AvailablePIDs[i]);
				else
					DrawEvents(CANEvents[i]);
				CANPos++;
			}
		}
	} else {
		DrawUI();
	}

}


std::vector<PIDItem>::const_iterator CANHandler::GetPIDLookup(const uint8_t& service, const uint16_t& id) {
	return std::find_if(PIDLookup.cbegin(), PIDLookup.cend(), [&] (PIDItem pi) { return pi.id == (service << 8) + id; } );
}


void CANHandler::DrawPids(OBD2Pid& obd2Item) {
	// Draw list of SAE OBD2 standard diagnostics (also updates the available PIDs vector with current, min, max and raw values)
	if (freeze) return;

	uint8_t top = (CANDRAWHEIGHT * CANPos) + 5;

	// Find latest event data and update values in Available PIDs vector
	bool dataFound = obd2Item.UpdateValues(CANEvents);
	lcd.DrawString(10, top, CANIdToHex((obd2Item.service << 8) + obd2Item.pid), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Check if we have lookup info for the OBD2 item
	if (obd2Item.info != PIDLookup.end()) {
		// If the found PID is in the lookup use the friendly name and calculation lambda
		lcd.DrawString(50, top, obd2Item.info->name, &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(190, top, (dataFound ? obd2Item.info->calcn(obd2Item, obd2Item.calcVal) : ""), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);

	} else if (dataFound) {
		// We have data but no lookup information - show bytes
		lcd.DrawString(50, top, HexToString(obd2Item.ABCD(), true), &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	}
}


void CANHandler::DrawPid() {
	// Draw detail for a single PID item
	if (freeze) return;

	viewPid->UpdateValues(CANEvents);

	bool infoAv = (viewPid->info != PIDLookup.end());

	lcd.DrawString(10, 5, "Service:" + IntToString(viewPid->service) + " PID:" + HexByte(viewPid->pid), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);
	lcd.DrawString(10, 30, "Diagnostic:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(150, 30, (infoAv ? viewPid->info->name : "Unknown"), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	lcd.DrawString(10, 55, "Current:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(10, 75, "Maximum:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(10, 95, "Minimum:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	if (infoAv) {
		lcd.DrawString(150, 55, viewPid->info->calcn(*viewPid, viewPid->calcVal), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
		lcd.DrawString(150, 95, viewPid->info->calcn(*viewPid, viewPid->valMin), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
		lcd.DrawString(150, 75, viewPid->info->calcn(*viewPid, viewPid->valMax), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	}

	lcd.DrawString(10, 120, "Raw ABCD:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(150, 120, HexToString(viewPid->ABCD(), true), &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	// print out last update time and number of hits
	lcd.DrawString(10, 145, "Updated:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(150, 145, IntToString(std::round((float)(SysTickVal - viewPid->updated) / 10)) + "ms  ", &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
	lcd.DrawString(10, 168, "Hits:", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(150, 168, IntToString(viewPid->hits), &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
}


void CANHandler::DrawEvents(const CANEvent& event) {
	// Draw list of CAN packets passively sniffed
	if (freeze) return;

	uint8_t top = (CANDRAWHEIGHT * CANPos) + 5;

	lcd.DrawString(10, top, CANIdToHex(event.id), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Draw bytes as hex values in alternating colours
	for (uint8_t c = 0; c < 8; ++c) {
		lcd.DrawString(60 + (c * 29), top, HexByte(((c < 4 ? event.dataLow : event.dataHigh) >> (8 * (c % 4))) & 0xFF), &lcd.Font_Large, (c % 2 ? LCD_YELLOW : LCD_ORANGE), LCD_BLACK);
	}
}


void CANHandler::DrawEvent() {
	// Draw detail for a single sniffed CAN packet
	if (freeze) return;

	lcd.DrawString(10, 5, "ID: 0x" + CANIdToHex(viewEvent->id) + " (" + IntToString(viewEvent->id) + ")", &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Print out high low bytes in hex
	lcd.DrawString(10, 30, "L:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	lcd.DrawString(40, 30, CANWordToBytes(viewEvent->dataLow), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 30, "(" + IntToString(viewEvent->dataLow) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	lcd.DrawString(10, 50, "H:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	lcd.DrawString(40, 50, CANWordToBytes(viewEvent->dataHigh), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 50, "(" + IntToString(viewEvent->dataHigh) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	// print out last update time and number of hits
	lcd.DrawString(10, 75, "Updated: " + IntToString(std::round((float)(SysTickVal - viewEvent->updated) / 10)) + "ms  ", &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
	lcd.DrawString(10, 95, "Hits: " + IntToString(viewEvent->hits), &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
}


uint32_t StringToOBD2(const std::string s) {
	uint32_t id;
	std::stringstream ss;
	ss << std::hex << s;
	ss >> id;
	return id;
}

void CANHandler::OBD2QueryMode(const std::string& s){
	/* Send OBD2 SAE standard query:
	 * byte 0: No. data bytes (set to 2)
	 * byte 1: Service code (01 = show current data, 02 = freeze frame;
	 * byte 2: PID code	(e.g.: 0C = Engine RPM)
	 * bytes 3 - 7: not used (ISO 15765-2 suggests 0xCC)
	 * eg 0xCC050102 0xCCCCCCCC
	*/

	// Passed in a service + PID code Eg o010C to return current value (01) of RPM (0C) - command is 0xCC0C0102 where 02 is number of bytes
	if (s.length() == 3 || s.length() == 5) {
		uint8_t b0 = s.length() == 3 ? 1 : 2;
		uint8_t b1 = StringToOBD2(pendingCmd.substr(1, 2));
		uint8_t b2 = s.length() == 5 ? StringToOBD2(pendingCmd.substr(3, 2)) : 0xCC;
		OBD2Cmd = b0 + (b1 << 8) + (b2 << 16) + (0xCC << 24);
	} else {
		OBD2Cmd = 0xCC0C0102;
	}
}


void CANHandler::OBD2Info(){
	//	State machine to query which PIDs are available and build a list; then enter update state which cycles through available PIDs querying latest data
	switch (OBD2InfoState) {
		case OBD2State::Start :
			CANUpdateFilters(0x700, 0x700);
			OBD2AvailablePIDs.clear();
			OBD2AvailablePIDs.push_back({ 3, 0, GetPIDLookup(3, 0) });		// Add item for DTC errors
			QueueSize = 0;
			CANEvents.clear();
			OBD2Cmd = 0xCC000102;
			PIDCounter = 0;
			ServiceCounter = 1;
			PIDQueryErrors = 0;
			OBD2InfoState = OBD2State::PIDQuery;
			break;

		case OBD2State::PIDQuery: {

#ifdef TESTMODE
			// FIXME - dummy code to simulate actual PIDs
			CANEvents.push_back({0x7E8, 0xC4024306, 0x0001C015, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x98004106, 0x0013C03B, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0xA0204106, 0x00010000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x00404106, 0x00000002, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x02014106, 0x0000E80E, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x6D044103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x41054103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x480B4103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x0C0C4104, 0x000000BE, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x000D4103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x370F4103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x03104104, 0x00000080, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x00114103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x04124103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x061C4103, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x001F4104, 0x00000031, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x00214104, 0x00000000, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x0B234104, 0x0000009A, SysTickVal, 0});
			CANEvents.push_back({0x7E8, 0x004F4106, 0x00230000, SysTickVal, 0});

			CANEvents.push_back({0x7E8, 0x54004906, 0x00000000, SysTickVal, 0});
#endif

			auto event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)			// check if data returned yet
					{ return ce.id == 0x7E8 && (ce.dataLow & 0xFFFF00) == static_cast<uint32_t>((PIDCounter << 16) + (ServiceCounter << 8) + 0x004000); } );

			if (event != CANEvents.end()) {
				// Create bit mask showing which PIDs car supports and add to available PIDs (ignoring the last PID which is used where another page of PID commands is available)
				uint32_t idMask = event->ABCD();
				for (uint8_t id = 0; id < 31; ++id) {
					if (0x80000000 >> id & idMask) {
						uint16_t pidId = id + 1 + PIDCounter;
						OBD2AvailablePIDs.push_back({ ServiceCounter, pidId, GetPIDLookup(ServiceCounter, pidId) });
					}
				}

				// check if there are no further PID commands available
				if (ServiceCounter == 9) {
					OBD2InfoState = OBD2State::List;
				} else if ((idMask & 1) == 0) {
					ServiceCounter = 9;
					PIDCounter = 0;
					OBD2Cmd = 0xCC000902 + (PIDCounter << 16);
				} else {
					PIDCounter += 0x20;
					OBD2Cmd = 0xCC000102 + (PIDCounter << 16);
				}
			} else {
				PIDQueryErrors++;
				if (PIDQueryErrors > 100)
					OBD2InfoState = OBD2State::List;
			}
		}
		break;

		case OBD2State::List: {
			std::stringstream ss;
			ss << "Available PIDs: ";
			for (auto p : OBD2AvailablePIDs) {
				ss << HexByte(p.pid) << ", ";
			}
			uartSendString(ss.str() + '\n');
			PIDCounter = 0;
			OBD2InfoState = OBD2State::Update;
		}
		break;

		case OBD2State::Update: {
			// Cycle through available PIDs generating queries
			OBD2Pid pid = OBD2AvailablePIDs[PIDCounter];
			OBD2Cmd = 0xCC000002 + (pid.pid << 16) + (pid.service << 8);
			PIDCounter ++;
			if (PIDCounter >= OBD2AvailablePIDs.size())
				PIDCounter = 0;
		}
		break;

	}
}



bool CANHandler::ProcessCmd() {
	// Checks if a valid command has been sent via UART and processes accordingly

	if (pendingCmd.empty())
		return false;

	bool cmdValid = true;
	uint16_t itemCount = Mode == OBD2Mode::Info ? OBD2AvailablePIDs.size() : CANEvents.size();
	uint16_t pageCount = std::ceil((float)itemCount / CANPAGEITEMS);

	if (pendingCmd == "f") {										// Freeze Display
		freeze = !freeze;
	} else if (pendingCmd[0] == 'o') {								// OBD2 Query mode
		Mode = (Mode != OBD2Mode::Query || pendingCmd.length() > 1) ? OBD2Mode::Query : OBD2Mode::Off;
		if (Mode == OBD2Mode::Query) {
			OBD2QueryMode(pendingCmd);
		}
	} else if (pendingCmd == "info") {								// Switch to SAE standard diagnostic info mode
		pageNo = 0;
		if (Mode == OBD2Mode::Info) {
			Mode = OBD2Mode::Off;
		} else {
			OBD2InfoState = OBD2State::Start;
			Mode = OBD2Mode::Info;
			OBD2Info();			// initiate info mode state machine
		}
	} else if (pendingCmd == "dump") {								// Dump data to serial
		for (auto ce : CANEvents) {
			uartSendString("ID: 0x" + CANIdToHex(ce.id) + " L:" + CANWordToBytes(ce.dataLow) + " H:" + CANWordToBytes(ce.dataHigh) + " Hits:" + IntToString(ce.hits) + '\n');
		}
	} else if (pendingCmd == "raw") {								// Dump data to serial in raw format - eg for generating test scripts
		for (auto ce : CANEvents) {
			uartSendString("0x" + CANIdToHex(ce.id) + ", 0x" + HexToString(ce.dataLow) + ", 0x" + HexToString(ce.dataHigh) + '\n');
		}
	} else if (viewIDMode) {
		if (pendingCmd == "q") {									// Exit view ID mode
			viewIDMode = false;
		} else {
			cmdValid = false;
		}
	} else {
		if (pendingCmd == "p") {									// Page Down
			pageNo = (pageNo == pageCount - 1) ? 0 : pageNo + 1;
		} else if (pendingCmd == "u") {								// Page Up
			pageNo = (pageNo == 0) ? pageCount - 1 : pageNo - 1;
		} else if (pendingCmd == "c") {								// Clear Results
			CANEvents.clear();
		} else if (pendingCmd == "s") {		// Sort by ID
			std::stable_sort(CANEvents.begin(), CANEvents.end(), [&] (CANEvent c1, CANEvent c2) { return c1.id < c2.id; });
		} else if (pendingCmd == "sn") {							// Sort by newly updated
			std::stable_sort(CANEvents.begin(), CANEvents.end(), [&] (CANEvent c1, CANEvent c2) { return c1.updated > c2.updated; });
		} else if (std::isdigit(pendingCmd[0])) {					// View ID
			// view id mode - search to check we have event with matching ID and store iterator if so
			uint16_t id = StringToOBD2(pendingCmd);

			if (Mode == OBD2Mode::Info) {
				auto pid = std::find_if(OBD2AvailablePIDs.begin(), OBD2AvailablePIDs.end(), [&] (OBD2Pid p)	{ return (p.service << 8) + p.pid == id; } );
				if (pid != OBD2AvailablePIDs.end()) {
					viewIDMode = true;
					viewPid = pid;
				} else {
					cmdValid = false;
				}

			} else {
				// If in sniffing mode display ID by CAN id; if in info mode use the PID code which
				auto ce = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)
					{ return Mode == OBD2Mode::Query ? ce.id == 0x7E8 && ce.PID() == id : ce.id == id; } );

				if (ce != CANEvents.end()) {
					viewIDMode = true;
					viewEvent = ce;
				} else {
					cmdValid = false;
				}
			}
		} else {
			cmdValid = false;
		}
	}

	if (freeze && pendingCmd != "f")		// Clear freeze status when another valid command is received
		freeze = false;

	return cmdValid;
}


void CANHandler::DrawUI() {
	// Draw standard UI elements

	bool cmdValid = ProcessCmd();
	uint16_t itemCount = Mode == OBD2Mode::Info ? OBD2AvailablePIDs.size() : CANEvents.size();
	uint16_t pageCount = std::ceil((float)itemCount / CANPAGEITEMS);

	if (cmdValid and !freeze)
		lcd.ColourFill(0, 0, lcd.width - 1, CANDrawHeight - 1, LCD_BLACK);

	if (!viewIDMode) {
		lcd.DrawString(280, CANDrawHeight, IntToString(itemCount), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(180, CANDrawHeight, "p. " + IntToString(pageNo + 1) + "/" + IntToString(pageCount), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	}

	if (pendingCmd != "") {
		lcd.ColourFill(0, CANDrawHeight, lcd.width - 1, lcd.height - 1, LCD_BLACK);
		if (Mode == OBD2Mode::Query && cmdValid) {
			lcd.DrawString(10, CANDrawHeight, "OBD2: " + HexToString(OBD2Cmd), &lcd.Font_Large, LCD_GREEN, LCD_BLACK);
		} else {
			lcd.DrawString(10, CANDrawHeight, (viewIDMode ? "ID detail: " : "Cmd: ") + pendingCmd, &lcd.Font_Large, cmdValid ? LCD_GREEN : LCD_RED, LCD_BLACK);
		}
	}

	pendingCmd.clear();
}

