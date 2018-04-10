/*
 * Bluetooth.cpp
 *
 *  Created on: 23 kwi 2017
 *      Author: Admin
 */

#include <Bluetooth.h>

Bluetooth::Bluetooth() :
	m_uartHost(std::make_unique<UartDriver>("/dev/ttyUL1"))
{}

void Bluetooth::send(const char* p_data, int p_size)
{
	m_uartHost->sendData(p_data, p_size);
}

void Bluetooth::receive(char* p_buffer, int p_size)
{
	m_uartHost->readData(p_buffer, p_size);
}
