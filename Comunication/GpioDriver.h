/*
 * GpioDriver.h
 *
 *  Created on: 6 kwi 2017
 *      Author: Admin
 */

#ifndef GPIODRIVER_H_
#define GPIODRIVER_H_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

class GpioDriver {
public:
	GpioDriver(std::string p_pinNumber, const char* p_direction);
	~GpioDriver();

	void setPin();
	void resetPin();
	void writePinState(bool p_state);
	void togglePin();

private:
	int m_pinValuefd;
	std::string m_pinNumber;
};

#endif /* GPIODRIVER_H_ */
