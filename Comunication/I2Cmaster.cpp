/*
 * I2Cmaster.cpp
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#include "I2Cmaster.h"

I2Cmaster::I2Cmaster(const char* p_device) {
	m_i2cHandler = open(p_device, O_RDWR);
	if (m_i2cHandler < 0)
	{
		throw DeviceInitializationFailureException("Cannot open " + static_cast<std::string>(p_device));
	}
}

void I2Cmaster::setSlaveAddress(int p_address){
	if( ioctl(m_i2cHandler, I2C_SLAVE, p_address) < 0){
		throw DeviceInitializationFailureException("Cannot set i2c slave address for" + std::to_string(p_address));
	}
}

void I2Cmaster::writeRegister(uint8_t p_address, uint8_t p_data){
	uint8_t l_writeBuffer[2];
	l_writeBuffer[0] = p_address;
	l_writeBuffer[1] = p_data;

	if (write(m_i2cHandler, l_writeBuffer, 2) < 2) {
		throw CommunicationFailureException("Cannot send i2c data");
	}
	//std::cout<<write(m_i2cHandler, l_writeBuffer, 2)<<std::endl;
}

int8_t I2Cmaster::readRegister(uint8_t p_address){
	int8_t l_buffer[1];
	l_buffer[0] = p_address;
	write(m_i2cHandler, l_buffer, 1);

	if (read(m_i2cHandler, l_buffer, 1) != 1) {
		throw CommunicationFailureException("Cannot read i2c data");
	}

	return l_buffer[0];
}

I2Cmaster::~I2Cmaster() {
	close(m_i2cHandler);
}

