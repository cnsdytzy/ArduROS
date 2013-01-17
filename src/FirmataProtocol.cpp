/*
 * FirmataProtocol.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: ams
 *
 *  Implements v2.3 of the Firmata Protocol, ideally with a minimum of other code around it.
 */

#include "FirmataProtocol.hpp"

using namespace std;

void FirmataProtocol::init(char* serPort) {
	/**
	 * Set up a serial port
	 */
	// Firmata speaks 57.6k serial
	try {
		device.open(serPort, 57600);
	} catch (cereal::Exception& e) {
		ROS_FATAL("Failed to open the serial port!!!");
		ROS_BREAK();
	}
	ROS_INFO("The serial port is opened.");
}

void FirmataProtocol::deinit() {
	device.close();
}

/* This protocol uses the MIDI message format, but does not use the whole
 * protocol.  Most of the command mappings here will not be directly usable in
 * terms of MIDI controllers and synths.  It should co-exist with MIDI without
 * trouble and can be parsed by standard MIDI interpreters.  Just some of the
 * message data is used differently.
 *
 * MIDI format: http://www.harmony-central.com/MIDI/Doc/table1.html
 *
 *                              MIDI
 * type                command  channel    first byte            second byte
 *----------------------------------------------------------------------------
 * analog I/O message    0xE0   pin #      LSB(bits 0-6)         MSB(bits 7-13)
 * digital I/O message   0x90   port       LSB(bits 0-6)         MSB(bits 7-13)
 * report analog pin     0xC0   pin #      disable/enable(0/1)   - n/a -
 * report digital port   0xD0   port       disable/enable(0/1)   - n/a -
 *
 * sysex start           0xF0
 * set pin mode(I/O)     0xF4              pin # (0-127)         pin state(0=in)
 * sysex end             0xF7
 * protocol version      0xF9              major version         minor version
 * system reset          0xFF
 *
 */

/* SysEx-based commands (0x00-0x7F) are used for an extended command set.
 *
 * type                command  first byte       second byte      ...
 *-----------------------------------------------------------------------------
 * string                0x71   char *string ...
 * firmware name/version 0x79   major version   minor version     char *name...
 */

//----------------------- Data Message Expansion ---------------------------------
/* two byte digital data format
 * 0  digital data, 0x90-0x9F, (MIDI NoteOn, but different data format)
 * 1  digital pins 0-6 bitmask
 * 2  digital pins 7-13 bitmask
 */

/* analog 14-bit data format
 * 0  analog pin, 0xE0-0xEF, (MIDI Pitch Wheel)
 * 1  analog least significant 7 bits
 * 2  analog most significant 7 bits
 */

/* version report format
 * -------------------------------------------------
 * 0  version report header (0xF9) (MIDI Undefined)
 * 1  major version (0-127)
 * 2  minor version (0-127)
 */

//--------------------- Control Message Expansion -----------------------------
/* set pin mode
 * 1  set digital pin mode (0xF4) (MIDI Undefined)
 * 2  pin number (0-127)
 * 3  state (INPUT/OUTPUT/ANALOG/PWM/SERVO, 0/1/2/3/4)
 */

/* toggle analogIn reporting by pin
 * 0  toggle digitalIn reporting (0xC0-0xCF) (MIDI Program Change)
 * 1  disable(0)/enable(non-zero)
 */

/* toggle digital port reporting by port
 * 0  toggle digital port reporting (0xD0-0xDF) (MIDI Aftertouch)
 * 1  disable(0)/enable(non-zero)
 */

/* request version report
 * 0  request version report (0xF9) (MIDI Undefined)
 */

//--------------------- Sysex message expansion -------------------------------
/* Generic Sysex Message
 * 0     START_SYSEX (0xF0)
 * 1     sysex command (0x00-0x7F)
 * x     between 0 and MAX_DATA_BYTES 7-bit bytes of arbitrary data
 * last  END_SYSEX (0xF7)
 */

// Following are sysex commands used in this version of the protocol:
#define RESERVED_COMMAND        0x00 // 2nd SysEx data byte is a chip-specific command (AVR, PIC, TI, etc).
#define ANALOG_MAPPING_QUERY    0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE 0x6A // reply with mapping info
#define CAPABILITY_QUERY        0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE     0x6C // reply with supported modes and resolution
#define PIN_STATE_QUERY         0x6D // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE      0x6E // reply with a pin's current mode and value
#define EXTENDED_ANALOG         0x6F // analog write (PWM, Servo, etc) to any pin
#define SERVO_CONFIG            0x70 // set max angle, minPulse, maxPulse, freq
#define STRING_DATA             0x71 // a string message with 14-bits per char
#define SHIFT_DATA              0x75 // shiftOut config/data message (34 bits)
#define I2C_REQUEST             0x76 // I2C request messages from a host to an I/O board
#define I2C_REPLY               0x77 // I2C reply messages from an I/O board to a host
#define I2C_CONFIG              0x78 // Configure special I2C settings such as power pins and delay times
#define REPORT_FIRMWARE         0x79 // report name and version of the firmware
#define SAMPLING_INTERVAL       0x7A // sampling interval
#define SYSEX_NON_REALTIME      0x7E // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME          0x7F // MIDI Reserved for realtime messages
//--------------------- Query Firmware and Version ----------------------------
/* Query Firmware Name and Version
 * 0  START_SYSEX (0xF0)
 * 1  queryFirmware (0x79)
 * 2  END_SYSEX (0xF7)
 */

/* Receive Firmware Name and Version (after query)
 * 0  START_SYSEX (0xF0)
 * 1  queryFirmware (0x79)
 * 2  major version (0-127)
 * 3  minor version (0-127)
 * 4  first 7-bits of firmware name
 * 5  second 7-bits of firmware name
 * x  ...for as many bytes as it needs)
 * 6  END_SYSEX (0xF7)
 */

#define START_SYSEX 0xF0
#define END_SYSEX 0xF7

void FirmataProtocol::queryFirmware(char* reply) {
	//Set up and send the query
	char* msg = new char[3];
	msg[0] = 0xF0;
	msg[1] = 0x79;
	msg[2] = 0xF7;
	//TODO memory leak around new
	device.write(msg, 3);

	//Try for a reply, parameters are buffer, length, and timeout
	try {
		device.read(reply, 1024, 1000);
	} catch (cereal::TimeoutException& e) {
		ROS_ERROR("Timeout!");
	}

	//Parse the response
	int ii = 0;
	if (reply[ii] == char(START_SYSEX)) {
		while (reply[ii] != char(END_SYSEX)) {
			printf("%02X", ((unsigned char *) reply)[ii] );
			ii++;
		}
	}
//
//	//Return the protocol string
//	return "Parse not implemented yet";
}

//--------------------- Extended Analog ----------------------------------------
/* Query Firmware Name and Version
 * 0  START_SYSEX (0xF0)
 * 1  queryFirmware (0x79)
 * 2  END_SYSEX (0xF7)
 */

/* Receive Firmware Name and Version (after query)
 * 0  START_SYSEX (0xF0)
 * 1  queryFirmware (0x79)
 * 2  major version (0-127)
 * 3  minor version (0-127)
 * 4  first 7-bits of firmware name
 * 5  second 7-bits of firmware name
 * x  ...for as many bytes as it needs)
 * 6  END_SYSEX (0xF7)
 */

//-------------------- Query Capability -----------------------------------------
/* capabilities query
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  capabilities query (0x6B)
 * 2  END_SYSEX (0xF7) (MIDI End of SysEx - EOX)
 */

/* capabilities response
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  capabilities response (0x6C)
 * 2  1st mode supported of pin 0
 * 3  1st mode's resolution of pin 0
 * 4  2nd mode supported of pin 0
 * 5  2nd mode's resolution of pin 0
 ...   additional modes/resolutions, followed by a single 127 to mark the
 end of the first pin's modes.  Each pin follows with its mode and
 127, until all pins implemented.
 * N  END_SYSEX (0xF7)
 */

//--------------------- Query Analog Map ----------------------------------------
/* analog mapping query
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  analog mapping query (0x69)
 * 2  END_SYSEX (0xF7) (MIDI End of SysEx - EOX)
 */

/* analog mapping response
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  analog mapping response (0x6A)
 * 2  analog channel corresponding to pin 0, or 127 if pin 0 does not support analog
 * 3  analog channel corresponding to pin 1, or 127 if pin 1 does not support analog
 * 4  analog channel corresponding to pin 2, or 127 if pin 2 does not support analog
 ...   etc, one byte for each pin
 * N  END_SYSEX (0xF7)
 */

//---------------------- Query Pin State ------------------------------------------
/* pin state query
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  pin state query (0x6D)
 * 2  pin (0 to 127)
 * 3  END_SYSEX (0xF7) (MIDI End of SysEx - EOX)
 */

/* pin state response
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  pin state response (0x6E)
 * 2  pin (0 to 127)
 * 3  pin mode (the currently configured mode)
 * 4  pin state, bits 0-6
 * 5  (optional) pin state, bits 7-13
 * 6  (optional) pin state, bits 14-20
 ...  additional optional bytes, as many as needed
 * N  END_SYSEX (0xF7)
 */

//-------------------------------- I2C ---------------------------------------------
/* I2C read/write request
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  I2C_REQUEST (0x76)
 * 2  slave address (LSB)
 * 3  slave address (MSB) + read/write and address mode bits
 {7: always 0} + {6: reserved} + {5: address mode, 1 means 10-bit mode} +
 {4-3: read/write, 00 => write, 01 => read once, 10 => read continuously, 11 => stop reading} +
 {2-0: slave address MSB in 10-bit mode, not used in 7-bit mode}
 * 4  data 0 (LSB)
 * 5  data 0 (MSB)
 * 6  data 1 (LSB)
 * 7  data 1 (MSB)
 * ...
 * n  END_SYSEX (0xF7)
 */

/* I2C reply
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  I2C_REPLY (0x77)
 * 2  slave address (LSB)
 * 3  slave address (MSB)
 * 4  register (LSB)
 * 5  register (MSB)
 * 6  data 0 LSB
 * 7  data 0 MSB
 * ...
 * n  END_SYSEX (0xF7)
 */

/* I2C config
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  I2C_CONFIG (0x78)
 * 2  Delay in microseconds (LSB)
 * 3  Delay in microseconds (MSB)
 * ... user defined for special cases, etc
 * n  END_SYSEX (0xF7)
 */

//--------------------------------- Sampling Interval -----------------------------------
/* Set sampling interval
 * -------------------------------
 * 0  START_SYSEX (0xF0) (MIDI System Exclusive)
 * 1  SAMPLING_INTERVAL (0x7A)
 * 2  sampling interval on the millisecond time scale (LSB)
 * 3  sampling interval on the millisecond time scale (MSB)
 * 4  END_SYSEX (0xF7)
 */

//-------------------------------- Servos ------------------------------------------------
/* servo config
 * --------------------
 * 0  START_SYSEX (0xF0)
 * 1  SERVO_CONFIG (0x70)
 * 2  pin number (0-127)
 * 3  minPulse LSB (0-6)
 * 4  minPulse MSB (7-13)
 * 5  maxPulse LSB (0-6)
 * 6  maxPulse MSB (7-13)
 * 7  END_SYSEX (0xF7)
 */

/* set digital pin mode
 * --------------------
 * 1  set digital pin mode (0xF4) (MIDI Undefined)
 * 2  pin number (0-127)
 * 3  state (INPUT/OUTPUT/ANALOG/PWM/SERVO, 0/1/2/3/4)
 */

/* write to servo, servo write is performed if the pins mode is SERVO
 * ------------------------------
 * 0  ANALOG_MESSAGE (0xE0-0xEF)
 * 1  value lsb
 * 2  value msb
 */
