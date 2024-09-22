#include "ReadParams.h"

int main()
{
    std::map<std::string, int> configParams = readConfigFile("../config/HSVconfig.txt");

    for (const auto& [key, value] : configParams) {
        std::cout << key << " = " << value << std::endl;
    }

    return 0;
}