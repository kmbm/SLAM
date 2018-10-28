/*
 * FileHandler.cpp
 *
 *  Created on: 3 sty 2018
 *      Author: Admin
 */

#include <FileHandler.h>

FileHandler::FileHandler(const char* p_fileName)
{
	m_fileHandler.open(p_fileName);
}

FileHandler::~FileHandler()
{
	m_fileHandler.close();
}

void FileHandler::writeData(const std::vector<double>& p_data)
{
	m_fileHandler << "dupa dupa dupa";
}

void FileHandler::writeData(double p_data)
{
	m_fileHandler << p_data;
}

void FileHandler::newLine()
{
	m_fileHandler << std::endl;
}
