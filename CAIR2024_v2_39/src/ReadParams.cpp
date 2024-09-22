#include "ReadParams.h"

std::map<std::string, int> readConfigFile(const std::string& filename) {
    std::map<std::string, int> configParams;
    std::ifstream configFile(filename);

    if (configFile.is_open()) {
        std::string line;
        while (std::getline(configFile, line)) {
            std::istringstream iss(line);
            std::string key, value_str;
            int value;

            if (std::getline(std::getline(iss, key, '='), value_str)) {
                value = std::stoi(value_str);
                configParams[key] = value;
            }
        }
        configFile.close();
    }

    return configParams;
}