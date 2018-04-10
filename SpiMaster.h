/*
 * SpiMaster.h
 *
 *  Created on: 6 kwi 2017
 *      Author: Admin
 */

#ifndef SPIMASTER_H_
#define SPIMASTER_H_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

class SpiMaster {
public:
	SpiMaster(const std::string& p_device);
	~SpiMaster();

	uint8_t spiTransfer(const unsigned int& dataLength, char *buffer_tx, char *buffer_rx);
private:
	int m_spiHandler;

	static const uint8_t m_mode = 3;
};

#endif /* SPIMASTER_H_ */
