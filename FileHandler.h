/*
 * FileHandler.h
 *
 *  Created on: 3 sty 2018
 *      Author: Admin
 */

#ifndef FILEHANDLER_H_
#define FILEHANDLER_H_

#include <iostream>
#include <fstream>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <vector>

class FileHandler {
public:
	FileHandler(const char* p_fileName);
	~FileHandler();

	void writeData(const std::vector<double>&);
	void writeData(double);
	void newLine();

private:
	std::ofstream  m_fileHandler;
};

#endif /* FILEHANDLER_H_ */
