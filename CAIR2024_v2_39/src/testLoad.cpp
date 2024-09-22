# include "Arm.h"

int main()
{
    Py_Initialize(); // 初始化Python环境
    // 装载物资
    Equip();
    Py_Finalize(); // 清理并关闭Python环境
}