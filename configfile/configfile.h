/*
 * configfile.h
 *
 *  Created on: Sep 17, 2018
 *      Author: Krzysiek
 */

#ifndef CONFIGFILE_CONFIGFILE_H_
#define CONFIGFILE_CONFIGFILE_H_

#include <iostream>
#include <string>
#include <map>

namespace GMapping{

class AutoVal {
public:
  AutoVal() {};
  explicit AutoVal(const std::string&);
  explicit AutoVal(double);
  explicit AutoVal(int);
  explicit AutoVal(unsigned int);
  explicit AutoVal(bool);
  explicit AutoVal(const char*);

  AutoVal(const AutoVal&);
  AutoVal& operator=(const AutoVal&);

  AutoVal& operator=(double);
  AutoVal& operator=(int);
  AutoVal& operator=(unsigned int);
  AutoVal& operator=(bool);
  AutoVal& operator=(const std::string&);

public:
  operator std::string() const;
  operator double() const;
  operator int() const;
  operator unsigned int() const;
  operator bool() const;

protected:
  std::string toLower(const std::string& source) const;

private:
  std::string m_value;
};

class ConfigFile {
  std::map<std::string,AutoVal> m_content;

public:
  ConfigFile();
  ConfigFile(const std::string& configFile);
  ConfigFile(const char* configFile);

  bool read(const std::string& configFile);
  bool read(const char* configFile);


  const AutoVal& value(const std::string& section,
		       const std::string& entry) const;

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       double def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       const char* def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       bool def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       int def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       unsigned int def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       const std::string& def);

  void dumpValues(std::ostream& out);


 protected:
  std::string trim(const std::string& source, char const* delims = " \t\r\n") const;
  std::string truncate(const std::string& source, const char* atChar) const;
  std::string toLower(const std::string& source) const;
  void insertValue(const std::string& section, const std::string& entry, const std::string& thevalue );
};

};

#endif /* CONFIGFILE_CONFIGFILE_H_ */
