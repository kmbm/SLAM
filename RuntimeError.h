/*
 * RuntimeError.h
 *
 *  Created on: 19 mar 2017
 *      Author: Admin
 */

#ifndef RUNTIMEERROR_H_
#define RUNTIMEERROR_H_

#include <stdexcept>
#include <string>

class DeviceInitializationFailureException : public std::runtime_error {
public:
	using runtime_error::runtime_error;
};

class CommunicationFailureException : public std::runtime_error {
public:
	using runtime_error::runtime_error;
};

#endif /* RUNTIMEERROR_H_ */
