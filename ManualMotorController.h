/*
 * ManualMotorController.h
 *
 *  Created on: 30 kwi 2017
 *      Author: Admin
 */

#ifndef MANUALMOTORCONTROLLER_H_
#define MANUALMOTORCONTROLLER_H_

#include "Bluetooth.h"
#include "GpioDriver.h"
#include "vector"

class ManualMotorController {
public:
	ManualMotorController();

	void startManualMode();
	void stopManualMode();
	void sendCommandToPL(const char* p_buffer);
	void writeMotorControl(bool p_bit4, bool p_bit3, bool p_bit2, bool p_bit1);


private:
	std::vector<GpioDriver> m_motorControlGpio;
	int m_mode;
};

#endif /* MANUALMOTORCONTROLLER_H_ */
