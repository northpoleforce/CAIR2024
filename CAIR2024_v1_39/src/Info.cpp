#include "Info.h"

void greenshow(std::string str)
{
    std::cout << "\033[0;32m" << str << "\033[0m" << std::endl;
}
void greenINFO(std::string str)
{
    std::cout << "\033[1;32m" << str << "\033[0m" << std::endl;
}
void blueshow(std::string str)
{
    std::cout << "\033[0;34m" << str << "\033[0m" << std::endl;
}
void blueINFO(std::string str)
{
    std::cout << "\033[1;34m" << str << "\033[0m" << std::endl;
}
void redshow(std::string str)
{
    std::cout << "\033[0;33m" << str << "\033[0m" << std::endl;
}
void redINFO(std::string str)
{
    std::cout << "\033[1;33m" << str << "\033[0m" << std::endl;
}