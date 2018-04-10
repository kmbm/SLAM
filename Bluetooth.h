/*
 * Bluetooth.h
 *
 *  Created on: 23 kwi 2017
 *      Author: Admin
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "UartDriver.h"
#include <memory>

class Bluetooth {
public:
	Bluetooth();
	//~Bluetooth();

	void send(const char* p_data, int p_size);
	void receive(char* p_buffer, int p_size);
private:
	std::unique_ptr<UartDriver> m_uartHost;
};

#endif /* BLUETOOTH_H_ */
