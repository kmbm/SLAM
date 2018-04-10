/*
 * GpioDriver.cpp
 *
 *  Created on: 6 kwi 2017
 *      Author: Admin
 */

#include <GpioDriver.h>
#include <RuntimeError.h>
#include "iostream"

GpioDriver::GpioDriver(std::string p_pinNumber, const char* p_direction)
{
	int l_exportfd, l_directionfd;
	l_exportfd = open("/sys/class/gpio/export",O_WRONLY);
	if(l_exportfd<0)
	{
		DeviceInitializationFailureException("Cannot open gpio export file");
	}
	m_pinNumber = p_pinNumber;
	write(l_exportfd,p_pinNumber.c_str(),4);
	close(l_exportfd);

	std::string l_directionPath = "/sys/devices/soc0/amba/e000a000.gpio/gpio/gpio" + p_pinNumber + "/direction";
	l_directionfd = open (l_directionPath.c_str(),O_RDWR);
	if(l_directionfd<0)
	{
		DeviceInitializationFailureException("Cannot open gpio direction file");
	}

	write(l_directionfd,p_direction,4);
	close(l_directionfd);

	std::string l_valuePath = "/sys/devices/soc0/amba/e000a000.gpio/gpio/gpio" + p_pinNumber + "/value";

	m_pinValuefd = open(l_valuePath.c_str(),O_RDWR);
	if(m_pinValuefd < 0)
	{
		DeviceInitializationFailureException("Cannot open gpio value file");
	}
}

GpioDriver::~GpioDriver()
{
	int l_unexportfd = open("/sys/class/gpio/export",O_WRONLY);

	if(l_unexportfd<0)
	{
		DeviceInitializationFailureException("Cannot open gpio unexport file");
	}

	write(l_unexportfd,m_pinNumber.c_str(),4);
	close(l_unexportfd);
}

void GpioDriver::setPin()
{
	write(m_pinValuefd,"1",1);
}

void GpioDriver::resetPin()
{
	write(m_pinValuefd,"0",1);
}

void GpioDriver::writePinState(bool p_state)
{
	if (p_state == true)
	{
		setPin();
	}
	else
	{
		resetPin();
	}
}
void GpioDriver::togglePin()
{
	char* p_pinState;
	read(m_pinValuefd,p_pinState,1);

	if (p_pinState)
	{
		write(m_pinValuefd,"0",1);
	}
	else
	{
		write(m_pinValuefd,"1",1);
	}
}
