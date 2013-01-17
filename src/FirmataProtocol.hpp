/*
 * FirmataProtocol.hpp
 *
 *  Created on: Jan 16, 2013
 *      Author: ams
 */

#ifndef FIRMATAPROTOCOL_HPP_
#define FIRMATAPROTOCOL_HPP_

#include "cereal_port/CerealPort.h"
#include "ros/ros.h"

class FirmataProtocol{
public:
	void init(char* serPort);
	void deinit();
	void queryFirmware(char* reply);
private:
	cereal::CerealPort device;
};


#endif /* FIRMATAPROTOCOL_HPP_ */
