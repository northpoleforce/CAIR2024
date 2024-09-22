# include "Arm.h"

int main() {
    Py_Initialize(); // 初始化Python环境
    // Equip();
    putDown(1, 3, 1);
    putDown(1, 3, 3);

    Py_Finalize(); // 清理并关闭Python环境

    return 0; 
}