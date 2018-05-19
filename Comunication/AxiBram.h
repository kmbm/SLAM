/*
 * AxiBram.h
 *
 *  Created on: 14 gru 2017
 *      Author: Admin
 */

#ifndef AXIBRAM_H_
#define AXIBRAM_H_

#include "stdio.h"
#include "iostream"
#include "fcntl.h"
#include "sys/mman.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

class AxiBram {
public:
	AxiBram();
	~AxiBram();

	void writeData(int p_data, int p_address);
	char readData(int p_address);

private:
	void configureUioSize();
	void configureUioAddress();
	void mapUioMemory();

	int m_uioFileHandler;
	size_t m_uioSize;
	int m_uioAddress;
	char* m_memoryPtr;
};

#endif /* AXIBRAM_H_ */
