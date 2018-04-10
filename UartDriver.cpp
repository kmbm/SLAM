/*
 * UartDriver.cpp
 *
 *  Created on: 26 lut 2017
 *      Author: Admin
 */

#include <UartDriver.h>

UartDriver::UartDriver(std::string p_uartName) {
	struct termios l_serial;

	m_uartHandler = open(p_uartName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (m_uartHandler < 0) {
		throw DeviceInitializationFailureException("Cannot open " + static_cast<std::string>(p_uartName));
	}

	if (tcgetattr(m_uartHandler, &l_serial) < 0) {
		throw DeviceInitializationFailureException("Cannot get uart configuration");
	}

	l_serial.c_iflag = 0;
	l_serial.c_oflag = 0;
	l_serial.c_lflag = 0;
	l_serial.c_cflag = CS8|CREAD|CLOCAL;;
	l_serial.c_cc[VMIN] = 5;
	l_serial.c_cc[VTIME] = 10;

	tcsetattr(m_uartHandler, TCSANOW, &l_serial);
}

UartDriver::~UartDriver() {
	close(m_uartHandler);
}

void UartDriver::sendData(const char* p_data, int p_size){
	write(m_uartHandler,p_data, p_size);
}

void UartDriver::readData(char* p_buffer, int p_size){
	read(m_uartHandler, p_buffer, p_size);
}

