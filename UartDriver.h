/*
 * UartDriver.h
 *
 *  Created on: 26 lut 2017
 *      Author: Admin
 */

#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <memory>

#include "RuntimeError.h"

class UartDriver {
public:
	UartDriver(std::string p_uartName);
	~UartDriver();

	void sendData(const char* p_data, int p_size);
	void readData(char* p_buffer, int p_size);

private:
	int m_uartHandler;
};

#endif /* UARTDRIVER_H_ */
