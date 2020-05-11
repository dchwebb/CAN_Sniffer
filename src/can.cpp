#include <can.h>


std::string CANHandler::IntToString(const uint32_t& v) {
	std::stringstream ss;
	ss << v;
	return ss.str();
}

std::string CANHandler::HexToString(const uint32_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << v;
	return ss.str();
}

std::string CANHandler::HexByte(const uint16_t& v) {
	std::stringstream ss;
	ss.width(2);
	ss.fill('0');
	ss << std::hex << v;
	return ss.str();
}

std::string CANHandler::CANIdToHex(const uint16_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(3) << std::hex << v;
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
			DrawId();
			DrawUI();
		} else {
			if (CANPos >= CANPAGEITEMS || static_cast<uint16_t>(CANPos + (pageNo * CANPAGEITEMS)) >= (uint16_t)CANEvents.size()) {
				CANPos = 0;
				DrawUI();
			} else {
				DrawList(CANEvents[CANPos + (pageNo * CANPAGEITEMS)]);
				CANPos++;
			}
		}
	} else {
		DrawUI();
	}

}



void CANHandler::DrawList(const CANEvent& event) {
	if (freeze) return;

	uint8_t top = (CANDRAWHEIGHT * CANPos) + 5;

	lcd.DrawString(10, top, CANIdToHex(event.id), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Draw bytes as hex values in alternating colours
	for (uint8_t c = 0; c < 8; ++c) {
		lcd.DrawString(60 + (c * 29), top, HexByte(((c < 4 ? event.dataLow : event.dataHigh) >> (8 * (c % 4))) & 0xFF), &lcd.Font_Large, (c % 2 ? LCD_YELLOW : LCD_ORANGE), LCD_BLACK);
	}
}



void CANHandler::DrawId() {
	if (freeze) return;

	lcd.DrawString(10, 5, "ID: 0x" + CANIdToHex(viewID->id) + " (" + IntToString(viewID->id) + ")", &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Print out high low bytes in hex

	lcd.DrawString(10, 30, "L:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	lcd.DrawString(40, 30, CANWordToBytes(viewID->dataLow), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 30, "(" + IntToString(viewID->dataLow) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	lcd.DrawString(10, 50, "H:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	lcd.DrawString(40, 50, CANWordToBytes(viewID->dataHigh), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 50, "(" + IntToString(viewID->dataHigh) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);


	// print out last update time and number of hits
	lcd.DrawString(10, 75, "Updated: " + IntToString(std::round((float)(SysTickVal - viewID->updated) / 10)) + "ms  ", &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
	lcd.DrawString(10, 95, "Hits: " + IntToString(viewID->hits), &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);


}

uint32_t StringToOBD2(const std::string s) {
	uint32_t id;
	std::stringstream ss;
	ss << std::hex << s;
	ss >> id;
	return id;
}

void CANHandler::OBD2QueryMode(const std::string& s){
	//	CANEvents.clear();
	//	CANUpdateFilters(OBD2Mode ? 0x700 : 0x0, OBD2Mode ? 0x700 : 0x0);

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

	//SendCAN(0x7DF, 0xCC0C0102, 0xCCCCCCCC, false);			// Engine RPM
}

void CANHandler::OBD2Info(){
	//CANEvents.clear();
	//	CANUpdateFilters(OBD2Mode ? 0x700 : 0x0, OBD2Mode ? 0x700 : 0x0);


	//enum class OBD2State { Start, PIDQuery00, PIDQuery20, PIDQuery40, PIDQuery60, Update };

	switch (OBD2InfoState) {
		case OBD2State::Start :
			CANEvents.clear();
			OBD2AvailablePIDs.clear();
			OBD2Cmd = 0xCC000102;
			OBD2InfoState = OBD2State::PIDQuery00;
			break;
		case OBD2State::PIDQuery00: {
			// FIXME - dummy code to simulate back PIDs
			CANEvents.push_back({0x7E8, 0x98004106, 0x0013C03B, SysTickVal, 0});
			auto event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)			// check if data returned yet
					{ return ce.id == 0x7E8 && (ce.dataLow & 0xFFFF00) == 0x004100; } );
			if (event != CANEvents.end()) {
				uint32_t idMask = (event->dataLow & 0xFF000000) + ((event->dataHigh & 0xFF) << 16) + (event->dataHigh & 0xFF00) + ((event->dataHigh & 0xFF0000) >> 16);
				for (uint8_t id = 0; id < 32; ++id) {
					if (0x80000000 >> id & idMask)
						OBD2AvailablePIDs.push_back({ id + 1 });
				}
				OBD2InfoState = OBD2State::PIDQuery20;
			}

		}
		break;

		case OBD2State::PIDQuery20: {
			std::stringstream ss;
			ss << "Available PIDs: ";
			for (auto p : OBD2AvailablePIDs) {
				ss << HexByte(p.pid) << ", ";
			}
			uartSendString(ss.str());
			OBD2InfoState = OBD2State::PIDQuery40;
		}
		break;

		default :
			// do nothing
			break;
	}


}


bool CANHandler::ProcessCmd() {
	if (pendingCmd.empty())
		return false;

	bool cmdValid = true;
	uint16_t pageCount = std::ceil((float)CANEvents.size() / CANPAGEITEMS);

	if (pendingCmd == "f") {								// Freeze Display
		freeze = !freeze;
	} else if (pendingCmd[0] == 'o') {							// OBD2 Query mode
		Mode = (Mode != OBD2Mode::Query || pendingCmd.length() > 1) ? OBD2Mode::Query : OBD2Mode::Off;
		if (Mode == OBD2Mode::Query) {
			OBD2QueryMode(pendingCmd);
		}
	} else if (pendingCmd == "info") {							// Dump data to serial
		OBD2InfoState = OBD2State::Start;
		Mode = OBD2Mode::Info;
		OBD2Info();
	} else if (pendingCmd == "dump") {							// Dump data to serial
		for (auto ce : CANEvents) {
			uartSendString("0x" + CANIdToHex(ce.id) + ' ' + CANWordToBytes(ce.dataLow) + ' ' + CANWordToBytes(ce.dataHigh) + '\n');
		}
	} else if (viewIDMode) {
		if (pendingCmd == "q") {									// Exit view ID mode
			viewIDMode = false;
		} else {
			cmdValid = false;
		}
	} else {
		if (pendingCmd == "p") {								// Page Down
			pageNo = (pageNo == pageCount - 1) ? 0 : pageNo + 1;
		} else if (pendingCmd == "u") {								// Page Up
			pageNo = (pageNo == 0) ? pageCount - 1 : pageNo - 1;
		} else if (pendingCmd == "c") {								// Clear Results
			CANEvents.clear();
		} else if (pendingCmd == "si") {							// Sort by ID
			std::stable_sort(CANEvents.begin(), CANEvents.end(), [&] (CANEvent c1, CANEvent c2) { return c1.id < c2.id; });
		} else if (pendingCmd == "sn") {							// Sort by newly updated
			std::stable_sort(CANEvents.begin(), CANEvents.end(), [&] (CANEvent c1, CANEvent c2) { return c1.updated > c2.updated; });
		} else if (pendingCmd == "test") {							// Test Mode on/off
			sendTestData = !sendTestData;
		} else if (std::isdigit(pendingCmd[0])) {					// View ID
			// view id mode - search to check we have event with matching ID and store iterator if so
			uint16_t id;
			std::stringstream ss;
			ss << std::hex << pendingCmd;
			ss >> id;

			auto ce = CANEvents.begin();
			ce = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce) { return ce.id == id; } );
			if (ce != CANEvents.end()) {
				viewIDMode = true;
				viewID = ce;
			} else {
				cmdValid = false;
			}
		} else {
			cmdValid = false;
		}
	}

	if (freeze && pendingCmd != "f")		// Clear freeze status when another valid command is received
		freeze = false;

	return cmdValid;
}



// Draw standard UI elements
void CANHandler::DrawUI() {

	bool cmdValid = ProcessCmd();
	uint16_t pageCount = std::ceil((float)CANEvents.size() / CANPAGEITEMS);

	if (cmdValid and !freeze)
		lcd.ColourFill(0, 0, lcd.width - 1, CANDrawHeight - 1, LCD_BLACK);

	if (!viewIDMode) {
		lcd.DrawString(280, CANDrawHeight, IntToString(CANEvents.size()), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
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

