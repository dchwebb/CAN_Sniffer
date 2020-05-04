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

std::string intToHexString(uint32_t v) {
	std::stringstream ss;
	ss << "0x";
	ss << std::hex << v;
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
				edited = true;
			}
		}

		if (!edited)
			CANEvents.push_front({nextEvent.id, nextEvent.dataLow, nextEvent.dataHigh});

		// erase first item if list greater than maximum size
		if (CANEvents.size() > 12)
			CANEvents.erase(CANEvents.end());

	}


	if (QueueSize < 10) {
		// Draw CAN events one at a time
		if (CANEvents.size() > 0) {
			if (CANPos >= CANEvents.size()) {
				CANPos = 0;
			}
			DrawEvent(CANEvents[CANPos]);
			CANPos++;
		}
	}
}


inline void CANHandler::QueueInc() {
	QueueSize--;
	QueueRead = (QueueRead + 1) % CANQUEUESIZE;
}

void CANHandler::DrawEvent(const CANEvent& event) {

	uint8_t top = CANDRAWHEIGHT * CANPos;

	lcd.DrawString(10, top, "ID:" + intToHexString(event.id) + " H:" + intToHexString(event.dataHigh), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	//lcd.DrawString(10, 30, "Low:" + intToHexString(canDataLow), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	//lcd.DrawString(10, 50, "High:" + intToHexString(canDataHigh), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
}

