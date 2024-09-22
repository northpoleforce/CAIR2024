#ifndef ReadConfigFile_H
#define ReadConfigFile_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

std::map<std::string, int> readConfigFile(const std::string& filename);

#endif // ReadConfigFile_H