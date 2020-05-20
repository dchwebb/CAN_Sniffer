#include <can.h>

// Process incoming CAN messages
void CANHandler::ProcessQueue() {

	while (QueueSize > 0) {
		bool edited = false;
		rawCANEvent nextEvent = Queue[QueueRead];
		QueueSize--;
		QueueRead = (QueueRead + 1) % CANQUEUESIZE;

		//	Overwrite last event if the id is the same
		if (!CANEvents.empty()) {

			auto event = CANEvents.begin();
			if (Mode == OBDMode::Info) {
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
		if (Mode != OBDMode::Info) {
			if (viewIDMode) {
				DrawEvent();
				DrawUI();
			} else {
				if (CANPos >= RowCount || (uint16_t)(CANPos + (pageNo * RowCount)) >= (uint16_t)CANEvents.size()) {
					CANPos = 0;
					DrawUI();
				} else {
					uint16_t i = CANPos + (pageNo * RowCount);
					DrawEvents(CANEvents[i]);
					CANPos++;
				}
			}
		}
	} else {
		DrawUI();
	}
}


void CANHandler::ProcessOBD(){
	//	State machine to query which PIDs are available and build a list; then enter update state which cycles through available PIDs querying latest data
	switch (OBDInfoState) {
		case OBDState::Start :
			CANUpdateFilters(0x700, 0x700);				// FIXME - should be 0x7E0 ?
			OBDAvPIDs.clear();
			OBDAvPIDs.push_back({ 3, 0, GetPIDLookup(3, 0) });		// Add item for DTC errors
			QueueSize = 0;
			CANEvents.clear();
			OBDCmd = 0xCC000102;
			SendCAN(0x7DF, OBDCmd, 0xCCCCCCCC);
			PIDCounter = 0;
			ServiceCounter = 1;
			PIDQueryErrors = 0;
			OBDInfoState = OBDState::PIDQuery;
#ifdef TESTMODE
			// FIXME - dummy code to simulate actual PIDs
			InjectTestData();
#endif
			break;

		case OBDState::PIDQuery: {
			ProcessQueue();

			auto event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)			// check if data returned yet
					{ return ce.id == 0x7E8 && (ce.dataLow & 0xFFFF00) == static_cast<uint32_t>((PIDCounter << 16) + (ServiceCounter << 8) + 0x004000); } );

			if (event != CANEvents.end()) {
				// Create bit mask showing which PIDs car supports and add to available PIDs (ignoring the last PID which is used where another page of PID commands is available)
				uint32_t idMask = event->ABCD();
				for (uint8_t id = 0; id < 31; ++id) {
					if (0x80000000 >> id & idMask) {
						uint16_t pidId = id + 1 + PIDCounter;
						OBDAvPIDs.push_back({ ServiceCounter, pidId, GetPIDLookup(ServiceCounter, pidId) });
					}
				}

				// check if there are no further PID commands available
				if (ServiceCounter == 9) {
					OBDInfoState = OBDState::List;
				} else if ((idMask & 1) == 0) {
					ServiceCounter = 9;
					PIDCounter = 0;
					OBDCmd = 0xCC000902 + (PIDCounter << 16);
				} else {
					PIDCounter += 0x20;
					OBDCmd = 0xCC000102 + (PIDCounter << 16);
				}
			} else {
				PIDQueryErrors++;

				if (PIDQueryErrors > 100)
					OBDInfoState = OBDState::List;			// Timeout
				else
					SendCAN(0x7DF, OBDCmd, 0xCCCCCCCC);		// resend command
			}
		}
		break;

		case OBDState::List: {
			std::stringstream ss;
			ss << "Available PIDs: ";
			for (auto p : OBDAvPIDs) {
				ss << IntToString(p.service) + HexByte(p.pid) << ", ";
			}
			uartSendString(ss.str() + '\n');
			PIDCounter = 0;
			OBDInfoState = OBDState::Update;
		}
		break;


		case OBDState::Update: {
			// Cycle through available PIDs generating queries, then waiting for response
			OBDPid& pid = OBDAvPIDs[PIDCounter];

			OBDCmd = 0xCC000002 + (pid.pid << 16) + (pid.service << 8);		// 02 = number of data bytes
			uint8_t queueSize = QueueSize;
			SendCAN(0x7DF, OBDCmd, 0xCCCCCCCC);
			PIDQueryErrors = 0;
			RandTestData(pid); 		// Randomise some data for display update testing

			// Wait until a response is available or time-outs
			uint32_t waitCount = 0;
			while (waitCount < CANTIMEOUT && queueSize == QueueSize) {
				waitCount++;
			}
#ifdef TESTMODE
			waitCount = 0;
#endif
			// Response received
			if (waitCount < CANTIMEOUT) {
				ProcessQueue();
				auto event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)
						{ return (ce.id & 0xF00) == 0x700 && ce.Service() == pid.service && (pid.service == 3 || ce.PID() == pid.pid); } );

				// If data returned store in appropriate format
				if (event != CANEvents.end()) {
					pid.dataLow = event->dataLow;
					pid.dataHigh = event->dataHigh;
					pid.updated = event->updated;

					// Check if single or multi frame
					if (event->SingleFrame()) {
						// Check if we have lookup info and store calculated value to generate calculations (usually the A or AB bytes)
						if (pid.info != PIDLookup.end()) {
							if		(pid.info->calc == PIDCalc::A)		pid.calcVal = event->A();
							else if (pid.info->calc == PIDCalc::AB)		pid.calcVal = event->AB();
							else 										pid.calcVal = pid.ABCD();

							// capture minimum and maximum values
							if (pid.calcVal > pid.valMax)	pid.valMax = pid.calcVal;
							if (pid.calcVal < pid.valMin)	pid.valMin = pid.calcVal;
						}
					} else {
						/*	Multiframe data packets
						https://en.wikipedia.org/wiki/ISO_15765-2			NB - flow control packets must be sent to 0x7E0 rather than 0x7DF
						Request next frames - | 3 = flow control 0 = Continue To Send | 00 = remaining "frames" to be sent without flow control or delay | 01= <= 127, separation time in milliseconds
						*/
						OBDCmd = 0xCC010030;
						SendCAN(0x7E0, OBDCmd, 0xCCCCCCCC);

						// Check how many additional frames to expect - there will be 6 in the first packet and up to 7 in remaining packets
						uint8_t frameCount = std::ceil(((float)event->ByteCount() - 6.0f) / 7.0f);

						waitCount = 0;
						pid.multiFrameData.clear();

						// Add bytes from first frame to multiFrameData vector
						pid.AddToMultiFrame(event->dataHigh, 0); 			// FIXME - testing indicates first byte is just 01 so maybe part of header

#ifdef TESTMODE
						if (pid.pid == 2) {
							TestInsert(0x7e8, 0x58584521, 0x45424247);		// VIN packet 2
							TestInsert(0x7e8, 0x34454322, 0x33303431);		// VIN Packet 3
						} else {
							TestInsert(0x7E8, 0x312d3121, 0x32364334);		// Calibration id p2
							TestInsert(0x7e8, 0x462d3522, 0x0000444b);		// Calibration id p3
						}
#endif
						// wait for remaining frames and add information to multiFrameData vector
						while (frameCount > 0 && waitCount < CANTIMEOUT) {
							if (QueueSize > 0) {
								rawCANEvent nextEvent = Queue[QueueRead];
								QueueSize--;
								QueueRead = (QueueRead + 1) % CANQUEUESIZE;

								// Check if continuation frame
								if ((nextEvent.dataLow & 0xF0) == 0x20) {
									if (pid.hits == 0) {
										uartSendString("MultiByte:" + IntToString(pid.service) + HexByte(pid.pid) + " " +
												CANIdToHex(nextEvent.id) + ", 0x" + HexToString(nextEvent.dataLow) + ", 0x" + HexToString(nextEvent.dataHigh) + '\n');
									}
									pid.AddToMultiFrame(nextEvent.dataLow, 1); 			// Skip byte 0 as this is continuation info header
									pid.AddToMultiFrame(nextEvent.dataHigh, 0);
									frameCount--;
								}
								waitCount = 0;
							} else {
								waitCount++;
							}
						}
					}
				}
			}

			if (waitCount == CANTIMEOUT) {
				uartSendString("Query Timeout:" + HexToString(OBDCmd, true) + '\n');
			} else {
				pid.hits++;
			}

			// PID processed so increment counter, skipping immutable items that have already been updated
			PIDCounter = (1 + PIDCounter) % OBDAvPIDs.size();
			while (OBDAvPIDs[PIDCounter].hits > 0 && OBDAvPIDs[PIDCounter].info != PIDLookup.end() && OBDAvPIDs[PIDCounter].info->noUpdate) {
				PIDCounter = (1 + PIDCounter) % OBDAvPIDs.size();
			}
		}
		break;

	}

	// Draw CAN events one at a time
	if (viewIDMode) {
		DrawPid();
		DrawUI();
	} else {
		if (CANPos >= RowCount || (uint16_t)(CANPos + (pageNo * RowCount)) >= (uint16_t)OBDAvPIDs.size()) {
			CANPos = 0;
			DrawUI();
		} else {
			uint16_t i = CANPos + (pageNo * RowCount);
			DrawPids(OBDAvPIDs[i]);
			CANPos++;
		}
	}
}


std::vector<PIDItem>::const_iterator CANHandler::GetPIDLookup(const uint8_t& service, const uint16_t& id) {
	return std::find_if(PIDLookup.cbegin(), PIDLookup.cend(), [&] (PIDItem pi) { return pi.id == (service << 8) + id; } );
}


void CANHandler::DrawPids(OBDPid& obdItem) {
	// Draw list of SAE OBD2 standard diagnostics (also updates the available PIDs vector with current, min, max and raw values)
	if (freeze) return;

	uint8_t top = (RowHeight * CANPos) + 5;

	// Find latest event data and update values in Available PIDs vector
	lcd.DrawString(10, top, CANIdToHex((obdItem.service << 8) + obdItem.pid), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	// Check if we have lookup info for the OBD2 item
	if (!viewRaw && obdItem.info != PIDLookup.end()) {
		// If the found PID is in the lookup use the friendly name and calculation lambda
		lcd.DrawString(50, top, obdItem.info->name, &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(190, top, (obdItem.hits > 0 ? obdItem.info->calcn(obdItem, obdItem.calcVal).substr(0, 11) : ""), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);

	} else if (obdItem.hits > 0) {
		// We have data but no lookup information - show bytes
		lcd.DrawString(50, top, HexToString(obdItem.ABCD(), true), &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
	}
}


void CANHandler::DrawPid() {
	// Draw detail for a single PID item
	if (freeze) return;

	bool infoAv = (viewPid->info != PIDLookup.end());

	lcd.DrawString(10, 5, "Service: " + IntToString(viewPid->service) + " PID: " + HexByte(viewPid->pid), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);
	lcd.DrawString(10, 30, "Diagnostic", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(130, 30, (infoAv ? viewPid->info->name : "Unknown"), &lcd.Font_Large, LCD_LIGHTBLUE, LCD_BLACK);

	if (infoAv) {
		lcd.DrawString(10, 55, "Current", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(130, 55, viewPid->info->calcn(*viewPid, viewPid->calcVal), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);

		if (viewPid->multiFrameData.size() == 0 && !viewPid->info->noUpdate) {
			lcd.DrawString(10, 75, "Maximum", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
			lcd.DrawString(10, 95, "Minimum", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);

			lcd.DrawString(130, 95, viewPid->info->calcn(*viewPid, viewPid->valMin), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
			lcd.DrawString(130, 75, viewPid->info->calcn(*viewPid, viewPid->valMax), &lcd.Font_Large, LCD_YELLOW, LCD_BLACK);
		}
	}

	if (viewPid->multiFrameData.size() > 0) {
		lcd.DrawString(10, 80, "Raw Data", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		uint16_t xpos = 10;
		uint8_t ypos = 105;
		for (auto d : viewPid->multiFrameData) {
			lcd.DrawString(xpos, ypos, HexByte(d), &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);
			xpos += 25;
			if (xpos > 290) {
				xpos = 10;
				ypos += 25;
			}
		}

	} else {
		lcd.DrawString(10, 120, "Raw ABCD", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(130, 120, HexToString(viewPid->ABCD(), true), &lcd.Font_Large, LCD_ORANGE, LCD_BLACK);

		// print out last update time and number of hits
		lcd.DrawString(10, 145, "Updated", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(130, 145, FloatToString((float)(SysTickVal - viewPid->updated) / 100, true) + "s  ", &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
	}
	lcd.DrawString(10, 168, "Updates", &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	lcd.DrawString(130, 168, IntToString(viewPid->hits), &lcd.Font_Large, LCD_MAGENTA, LCD_BLACK);
}


void CANHandler::DrawEvents(const CANEvent& event) {
	// Draw list of CAN packets passively sniffed
	if (freeze) return;

	uint8_t top = (RowHeight * CANPos) + 5;

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


void CANHandler::OBDQueryMode(const std::string& s){
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
		uint8_t b1 = HexStringToOBD(pendingCmd.substr(1, 2));
		uint8_t b2 = s.length() == 5 ? HexStringToOBD(pendingCmd.substr(3, 2)) : 0xCC;
		OBDCmd = b0 + (b1 << 8) + (b2 << 16) + (0xCC << 24);
	} else {
		OBDCmd = 0xCC0C0102;
	}
}



void CANHandler::SendCAN(const uint16_t& canID, const uint32_t& dataLow, const uint32_t& dataHigh) {
#ifndef TESTMODE
	CANCmd(canID, dataLow, dataHigh);
//	uartSendString("sent: 0x" + CANIdToHex(canID) + ", 0x" + HexToString(dataLow) + ", 0x" + HexToString(dataHigh) + '\n');
#else
//	uartSendString("0x" + CANIdToHex(canID) + ", 0x" + HexToString(dataLow) + ", 0x" + HexToString(dataHigh) + '\n');
#endif
}

void CANHandler::LogMsg(const uint16_t& canID, const uint32_t& dataLow, const uint32_t& dataHigh) {
	uartSendString("rec: 0x" + CANIdToHex(canID) + ", 0x" + HexToString(dataLow) + ", 0x" + HexToString(dataHigh) + '\n');
}

bool CANHandler::ProcessCmd() {
	// Checks if a valid command has been sent via UART and processes accordingly

	if (pendingCmd.empty())
		return false;

	bool cmdValid = true;
	uint16_t itemCount = Mode == OBDMode::Info ? OBDAvPIDs.size() : CANEvents.size();
	uint16_t pageCount = std::ceil((float)itemCount / RowCount);

	if (pendingCmd == "f") {										// Freeze Display
		freeze = !freeze;
	} else if (pendingCmd[0] == 'o') {								// OBD Query mode
		Mode = (Mode != OBDMode::Query || pendingCmd.length() > 1) ? OBDMode::Query : OBDMode::Off;
		if (Mode == OBDMode::Query) {
			OBDQueryMode(pendingCmd);
		}
	} else if (pendingCmd == "info") {								// Switch to SAE standard diagnostic info mode
		pageNo = 0;
		if (Mode == OBDMode::Info) {
			Mode = OBDMode::Off;
		} else {
			OBDInfoState = OBDState::Start;
			Mode = OBDMode::Info;
			ProcessOBD();			// initiate info mode state machine
		}
	} else if (Mode == OBDMode::Info && pendingCmd == "raw") {
		viewRaw = !viewRaw;
	} else if (pendingCmd == "dump") {								// Dump data to serial
		for (auto ce : CANEvents) {
			uartSendString("ID: 0x" + CANIdToHex(ce.id) + " L:" + CANWordToBytes(ce.dataLow) + " H:" + CANWordToBytes(ce.dataHigh) + " Hits:" + IntToString(ce.hits) + '\n');
		}
	} else if (pendingCmd == "dumpraw") {								// Dump data to serial in raw format - eg for generating test scripts
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
			uint16_t id = HexStringToOBD(pendingCmd);

			if (Mode == OBDMode::Info) {
				auto pid = std::find_if(OBDAvPIDs.begin(), OBDAvPIDs.end(), [&] (OBDPid p)	{ return (p.service << 8) + p.pid == id; } );
				if (pid != OBDAvPIDs.end()) {
					viewIDMode = true;
					viewPid = pid;
				} else {
					cmdValid = false;
				}

			} else {
				// If in sniffing mode display ID by CAN id; if in info mode use the PID code which
				auto ce = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)
					{ return Mode == OBDMode::Query ? ce.id == 0x7E8 && ce.PID() == id : ce.id == id; } );

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
	uint16_t itemCount = Mode == OBDMode::Info ? OBDAvPIDs.size() : CANEvents.size();
	uint16_t pageCount = std::ceil((float)itemCount / RowCount);

	if (cmdValid and !freeze)
		lcd.ColourFill(0, 0, lcd.width - 1, DrawHeight - 1, LCD_BLACK);

	if (!viewIDMode) {
		lcd.DrawString(280, DrawHeight, IntToString(itemCount), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
		lcd.DrawString(180, DrawHeight, "p. " + IntToString(pageNo + 1) + "/" + IntToString(pageCount), &lcd.Font_Large, LCD_WHITE, LCD_BLACK);
	}

	if (pendingCmd != "") {
		lcd.ColourFill(0, DrawHeight, lcd.width - 1, lcd.height - 1, LCD_BLACK);
		if (Mode == OBDMode::Query && cmdValid) {
			lcd.DrawString(10, DrawHeight, "OBD: " + HexToString(OBDCmd), &lcd.Font_Large, LCD_GREEN, LCD_BLACK);
		} else {
			lcd.DrawString(10, DrawHeight, (viewIDMode ? "ID detail: " : "Cmd: ") + pendingCmd, &lcd.Font_Large, cmdValid ? LCD_GREEN : LCD_RED, LCD_BLACK);
		}
	}

	pendingCmd.clear();
}


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


std::string CANHandler::IntToString(const int32_t& v) {
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


uint32_t CANHandler::HexStringToOBD(const std::string& s) {
	uint32_t id;
	std::stringstream ss;
	ss << std::hex << s;
	ss >> id;
	return id;
}


void CANHandler::TestInsert(const uint16_t& id, const uint32_t& dataLow, const uint32_t& dataHigh) {
	Queue[QueueWrite] = {id, dataLow, dataHigh};
	QueueSize++;
	QueueWrite = (QueueWrite + 1) % CANQUEUESIZE;
}

void CANHandler::InjectTestData() {
		TestInsert(0x7E8, 0x98004106, 0x0013C03B);
		TestInsert(0x7E8, 0xA0204106, 0x00010000);
		TestInsert(0x7E8, 0x00404106, 0x00000002);
		TestInsert(0x7E8, 0x54004906, 0x00000000);
		TestInsert(0x7E8, 0x02014106, 0x0000E80E);
		TestInsert(0x7E8, 0x6D044103, 0x00000000);
		TestInsert(0x7E8, 0x41054103, 0x00000000);
		TestInsert(0x7E8, 0x480B4103, 0x00000000);
		TestInsert(0x7E8, 0x0C0C4104, 0x000000BE);
		TestInsert(0x7E8, 0x000D4103, 0x00000000);
		TestInsert(0x7E8, 0x370F4103, 0x00000000);
		TestInsert(0x7E8, 0x03104104, 0x00000080);
		TestInsert(0x7E8, 0x00114103, 0x00000000);
		TestInsert(0x7E8, 0x04124103, 0x00000000);
		TestInsert(0x7E8, 0x061C4103, 0x00000000);
		TestInsert(0x7E8, 0x001F4104, 0x00000031);
		TestInsert(0x7E8, 0x00214104, 0x00000000);
		TestInsert(0x7E8, 0x0B234104, 0x0000009A);
		TestInsert(0x7E8, 0x004F4106, 0x00230000);
		TestInsert(0x7E8, 0xC4024306, 0x0001C015);		// DTC
		TestInsert(0x7E8, 0x02491410, 0x30465701);		// o0902 VIN
		TestInsert(0x7e8, 0x58584521, 0x45424247);		// VIN packet 2
		TestInsert(0x7e8, 0x34454322, 0x33303431);		// VIN Packet 3
		TestInsert(0x7E8, 0x04491310, 0x39474201);		// o0904 Calibration ID
		TestInsert(0x7E8, 0x01064907, 0xCD8EFFF3);		// o0906 Calibration Verification Numbers (CVN)

}

void CANHandler::RandTestData(const OBDPid& pid) {
#ifdef TESTMODE
	// Randomise some data
	auto event = std::find_if(CANEvents.begin(), CANEvents.end(), [&] (CANEvent ce)
			{ return (ce.id & 0xF00) == 0x700 && ce.Service() == pid.service && (pid.service == 3 || ce.PID() == pid.pid); } );
	if (event != CANEvents.end()) {
		uint8_t a = (event->dataLow & 0xFF000000) >> 24;
		if (a > 4 && a < 0xFE) {
			a += (std::rand() % 3) - 1;
			event->dataLow = (event->dataLow & 0x00FFFFFF) + (a << 24);
		}
	}
	event->hits++;
#endif
}


