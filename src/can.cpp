#include <can.h>

std::string floatToString(float f, bool smartFormat) {
	std::string s;
	std::stringstream ss;

	if (smartFormat && f > 10000) {
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


std::string intToString(uint32_t v) {
	std::stringstream ss;
	ss << v;
	return ss.str();
}

std::string hexByte(uint16_t v) {
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

void CANHandler::ProcessCAN() {

	while (QueueSize > 0) {
		bool edited = false;
		rawCANEvent nextEvent = Queue[QueueRead];
		QueueInc();


		//	Overwrite last event if the id is the same
		if (!CANEvents.empty()) {

			auto event = CANEvents.begin();

			event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce) { return ce.id == nextEvent.id; } );
			if (event->id == nextEvent.id) {
				event->dataLow = nextEvent.dataLow;
				event->dataHigh = nextEvent.dataHigh;
				event->updated = SysTickVal;
				event->hits++;
				edited = true;
			}
		}

		if (!edited)
			CANEvents.push_front({nextEvent.id, nextEvent.dataLow, nextEvent.dataHigh, SysTickVal, 0});

		// erase first item if list greater than maximum size
		if (CANEvents.size() > 50)
			CANEvents.erase(CANEvents.end());

	}


	// Draw CAN events one at a time
	if (CANEvents.size() > 0) {
		if (viewIDMode) {
			DrawId();
			DrawUI();
		} else {
			if (CANPos >= CANPAGEITEMS || CANPos + (pageNo * CANPAGEITEMS) >= CANEvents.size()) {
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


inline void CANHandler::QueueInc() {
	QueueSize--;
	QueueRead = (QueueRead + 1) % CANQUEUESIZE;
}

void CANHandler::DrawList(const CANEvent& event) {

	uint8_t top = (CANDRAWHEIGHT * CANPos) + 5;

	lcd.DrawString(10, top, CANIdToHex(event.id), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Draw bytes as hex values in alternating colours
	for (uint8_t c = 0; c < 8; ++c) {
		lcd.DrawString(60 + (c * 29), top, hexByte(((c < 4 ? event.dataLow : event.dataHigh) >> (8 * (c % 4))) & 0xFF), &lcd.Font_Large, (c % 2 ? LCD_YELLOW : LCD_ORANGE), LCD_BLACK);
	}
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





void CANHandler::DrawId() {
	lcd.DrawString(10, 5, "ID: 0x" + CANIdToHex(viewID->id) + " (" + intToString(viewID->id) + ")", &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Print out high low labels and decimal values
	lcd.DrawString(10, 30, "H:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	lcd.DrawString(10, 50, "L:", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	// Print out high low bytes in hex
	lcd.DrawString(40, 30, CANWordToBytes(viewID->dataHigh), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 30, "(" + intToString(viewID->dataHigh) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	lcd.DrawString(40, 50, CANWordToBytes(viewID->dataLow), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
	lcd.DrawString(163, 50, "(" + intToString(viewID->dataLow) + ")", &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

	// print out last update time and number of hits
	lcd.DrawString(10, 75, "Updated: " + intToString(std::round((float)(SysTickVal - viewID->updated) / 10)) + "ms", &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
	lcd.DrawString(10, 95, "Hits: " + intToString(viewID->hits), &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);


}

void CANHandler::DrawUI() {
	// Draw standard UI elements
	uint16_t pageCount = std::ceil((float)CANEvents.size() / CANPAGEITEMS);
	bool cmdValid = true;

	if (pendingCmd == "p") {									// Page Down
		pageNo = (pageNo == pageCount - 1) ? 0 : pageNo + 1;
	} else if (pendingCmd == "u") {								// Page Up
		pageNo = (pageNo == 0) ? pageCount - 1 : pageNo - 1;
	} else if (pendingCmd == "test") {							// Test Mode on/off
		sendTestData = !sendTestData;
	} else if (pendingCmd == "dump") {							// Dump data to serial
		for (auto ce : CANEvents) {
			uartSendString("0x" + CANIdToHex(ce.id) + ' ' + CANWordToBytes(ce.dataLow) + ' ' + CANWordToBytes(ce.dataHigh) + '\n');
		}
	} else if (pendingCmd == "q" && viewIDMode) {				// Exit view ID mode
		viewIDMode = false;
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

	if (cmdValid)
		lcd.ColourFill(0, 0, lcd.width - 1, CANDrawHeight - 1, LCD_BLACK);

	if (!viewIDMode) {
		lcd.DrawString(180, CANDrawHeight, intToString(CANEvents.size()), &lcd.Font_Large, LCD_CYAN, LCD_BLACK);
		lcd.DrawString(230, CANDrawHeight, "p. " + intToString(pageNo + 1) + "/" + intToString(pageCount), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	}

	if (pendingCmd != "") {
		lcd.ColourFill(0, CANDrawHeight, lcd.width - 1, lcd.height - 1, LCD_BLACK);
		lcd.DrawString(10, CANDrawHeight, (viewIDMode ? "ID detail: " : "Cmd: ") + pendingCmd, &lcd.Font_Large, cmdValid ? LCD_GREEN : LCD_RED, LCD_BLACK);
	}

	pendingCmd.clear();
}

