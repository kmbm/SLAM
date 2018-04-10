/*
 * SpiMaster.cpp
 *
 *  Created on: 6 kwi 2017
 *      Author: Admin
 */

#include <SpiMaster.h>
#include "RuntimeError.h"

SpiMaster::SpiMaster(const std::string& p_device)
{
	m_spiHandler = open(p_device.c_str(),O_RDWR);
	if (m_spiHandler < 0)
	{
		DeviceInitializationFailureException("Cannot open SPI file");
	}

	if (ioctl(m_spiHandler, SPI_IOC_WR_MODE, m_mode) < 0)
	{
		DeviceInitializationFailureException("Cannot set SPI mode");
	}

}

SpiMaster::~SpiMaster()
{
	close(m_spiHandler);
}

uint8_t SpiMaster::spiTransfer(const unsigned int& dataLength, char *buffer_tx, char *buffer_rx)
{
	struct spi_ioc_transfer l_transfer {};

	l_transfer.tx_buf = (unsigned long)buffer_tx;
	l_transfer.rx_buf = (unsigned long)buffer_rx;
	l_transfer.len = dataLength;
	l_transfer.delay_usecs = 1000;
	l_transfer.speed_hz = 2000000;

	int l_transferStatus = ioctl(m_spiHandler, SPI_IOC_MESSAGE(1), &l_transfer);
	if (l_transferStatus < dataLength)
	{
		throw CommunicationFailureException("Cannot transfer SPI data");
	}
	return l_transferStatus;
}
