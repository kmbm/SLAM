/*
 * ManualMotorController.cpp
 *
 *  Created on: 30 kwi 2017
 *      Author: Admin
 */

#include <ManualMotorController.h>

ManualMotorController::ManualMotorController()
{
	int idx = 960;
	for(int i = 0; i<5; i++)
	{
		m_motorControlGpio.push_back(GpioDriver(std::to_string(idx), "out"));
		idx++;
	}
}

void ManualMotorController::sendCommandToPL(const char* p_buffer)
{/*
	if (*p_buffer | 0xFE == 0xFF)
	{
		if (m_mode == 0)
		{
			m_mode = 1;
			startManualMode();
		}
	}
	else
	{
		if (m_mode == 1)
		{
			m_mode = 0;
			stopManualMode();
		}
	}
*/
	if(*p_buffer) startManualMode();
	switch(*p_buffer)
	{
		case '2':
			writeMotorControl(false, true, true, true);
			break;
		case '3':
			writeMotorControl(false, false, true, false);
			break;
		case '4':
			writeMotorControl(false, false, true, true);
			break;
		case '5':
			writeMotorControl(false, true, false, false);
			break;
		case '6':
			writeMotorControl(false, true, false, true);
			break;
		case '7':
			writeMotorControl(false, true, true, true);
			break;
		case '8':
			writeMotorControl(true, false, false, false);
			break;
		case '9':
			writeMotorControl(true, false, false, true);
			break;
		case '1':
			writeMotorControl(false, false, false, false);
			break;
		default:
			writeMotorControl(false, false, false, false);
	}
}

void ManualMotorController::startManualMode()
{
	m_motorControlGpio[0].setPin();
}

void ManualMotorController::stopManualMode()
{
	m_motorControlGpio[0].resetPin();
}

void ManualMotorController::writeMotorControl(bool p_bit4, bool p_bit3, bool p_bit2, bool p_bit1)
{
	m_motorControlGpio[1].writePinState(p_bit1);
	m_motorControlGpio[2].writePinState(p_bit2);
	m_motorControlGpio[3].writePinState(p_bit3);
	m_motorControlGpio[4].writePinState(p_bit4);
	std::cerr << p_bit1 << p_bit2 << p_bit3 << p_bit4 << std::endl;
}
