/*
 * I2Cmaster.h
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#ifndef I2CMASTER_H_
#define I2CMASTER_H_

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <string>

#include "RuntimeError.h"

class I2Cmaster {
public:
	I2Cmaster(const char* p_device);
	~I2Cmaster();

	void setSlaveAddress(int p_address);
	void writeRegister(uint8_t p_address, uint8_t p_data);
	int8_t readRegister(uint8_t p_address);
private:
	int m_i2cHandler;
};

#endif /* I2CMASTER_H_ */
