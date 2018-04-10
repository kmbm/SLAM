/*
 * AxiBram.cpp
 *
 *  Created on: 14 gru 2017
 *      Author: Admin
 */

#include <AxiBram.h>

AxiBram::AxiBram()
{
	m_uioFileHandler = open("/dev/uio0", O_RDWR);

	configureUioSize();
	configureUioAddress();
	mapUioMemory();
}

void AxiBram::writeData(int p_data, int p_address)
{
	m_memoryPtr[p_address] = p_data;
}

char AxiBram::readData(int p_address)
{
	return m_memoryPtr[p_address];
}

void AxiBram::configureUioSize()
{
	FILE * l_sizeFileHandler = fopen("/sys/class/uio/uio0/maps/map0/size", "r");
	fscanf(l_sizeFileHandler, "0x%08X", &m_uioSize);
	fclose(l_sizeFileHandler);
}

void AxiBram::configureUioAddress()
{
	FILE * l_addressFileHandler = fopen("/sys/class/uio/uio0/maps/map0/addr", "r");
	fscanf(l_addressFileHandler, "0x%08X", &m_uioAddress);
	fclose(l_addressFileHandler);
}

void AxiBram::mapUioMemory()
{
	size_t l_offset = 0;
	m_memoryPtr = (char*)mmap(NULL, 1, PROT_READ|PROT_WRITE, MAP_SHARED, m_uioFileHandler, l_offset);
}

AxiBram::~AxiBram()
{
	close(m_uioFileHandler);
}
