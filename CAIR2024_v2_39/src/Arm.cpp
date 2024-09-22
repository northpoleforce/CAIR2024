#include "Arm.h"

void runPythonScript(const char *filePath)
{
    blueshow(filePath);
    
    // 打开Python脚本文件
    FILE *file = fopen(filePath, "r");
    if (file != NULL)
    {
        // 执行Python脚本
        PyRun_SimpleFile(file, filePath);
        fclose(file);
    }
    
}

void Equip()
{
    greenINFO("Equiping...");
    runPythonScript("../py_v1/Equip.py");
}
void Output_1()
{
    greenINFO("Output_1...");
    runPythonScript("../py_v1/Output_1.py");
}
void Output_2_front()
{
    greenINFO("Output_2_front...");
    runPythonScript("../py_v1/Output_2_front.py");
}
void Output_2_back()
{
    greenINFO("Output_2_back...");
    runPythonScript("../py_v1/Output_2_back.py");
}
void Output_3_front()
{
    greenINFO("Output_3_front...");
    runPythonScript("../py_v1/Output_3_front.py");
}
void Output_3_back()
{
    greenINFO("Output_3_back...");
    runPythonScript("../py_v1/Output_3_back.py");
}
void Output_4()
{
    greenINFO("Output_4...");
    runPythonScript("../py_v1/Output_4.py");
}

void putDown(int task1, int task2, int taskNow)
{
    std::stringstream ss;
    ss << "task1: " << task1 << " task2: " << task2 << " | taskNow: " << taskNow;
    blueINFO(ss.str());
    if (task1 == 1 && task2 == 2)
    {
        if (taskNow == 1)
            Output_1();
        else if (taskNow == 2)
            Output_2_back();
    }
    if (task1 == 1 && task2 == 3)
    {
        if (taskNow == 1)
            Output_1();
        else if (taskNow == 3)
            Output_3_back();
    }
    if (task1 == 1 && task2 == 4)
    {
        if (taskNow == 1)
            Output_1();
        else if (taskNow == 4)
            Output_4();
    }
    if (task1 == 2 && task2 == 3)
    {
        if (taskNow == 2)
            Output_2_front();
        else if (taskNow == 3)
            Output_3_back();
    }
    if (task1 == 2 && task2 == 4)
    {
        if (taskNow == 2)
            Output_2_front();
        else if (taskNow == 4)
            Output_4();
    }
    if (task1 == 3 && task2 == 4)
    {
        if (taskNow == 3)
            Output_3_front();
        else if (taskNow == 4)
            Output_4();
    }
}