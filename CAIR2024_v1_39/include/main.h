#ifndef MAIN_H
#define MAIN_H

#include "move.h"
#include "Arm.h"

#include "materialDetector.h"
#include "Camera.h"
#include "visionCPU.h"
#include "taskDetector.h"

extern Task task[6];

class Timer
{
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    int seconds;

public:
    void reset(int s)
    {
        seconds = s;
        start = std::chrono::high_resolution_clock::now();
    }
    bool isTimeUp()
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        return duration.count() >= seconds;
    }
};

void keepGoing(PIDController &pidYaw, Custom &moveController, cv::Point target_point, cv::Point midMidPoint)
{
    // Yaw(通过两点距离差来控制Yaw) 前进
    float vYaw = -pidYaw.PT(target_point.x, midMidPoint.x, 10);
    // std::cout << "vYaw (calc): " << vYaw << std::endl;
    moveController.setVelocity(0.2, 0, vYaw);
}

void adjust(PIDController &pidXTaskPre, PIDController &pidYTaskPre, PIDController &pidYawTaskPre, Custom &moveController,
            float xp, float yp, float xr, bool &flag, float minus)
{
    float vX = -pidXTaskPre.PT(-0.2, xp, 0.05);
    float vY = -pidYTaskPre.PT(-0.1, yp, 0.05);
    float vYaw = pidYawTaskPre.PT(0, xr, 0.2) * minus;
    moveController.setVelocity(vX, vY, vYaw);
    if (vX == 0 && vY == 0 && vYaw == 0)
        flag = true;
}

void getTasks(Task task[], int &task1st, int &task2nd)
{
    task[0].have = true;
    MaterialDetector materialDetector;
    materialDetector.detect(task);
    for (int i = 1; i <= 4; ++i)
    {
        if (task[i].have)
        {
            if (task1st == -1)
                task1st = i;
            else if (task2nd == -1)
                task2nd = i;
        }
    }
}
int getTaskNow(Task task[])
{
    for (int i = 1; i <= 4; ++i)
        if (task[i].have && !task[i].done)
            return i;
    if (task[0].have && !task[0].done)
        return 0;
    return -1;
}
void workTask1or2(int taskNow, int taskDetected, Task task[], int task1st, int task2nd,
                  PIDController &pidYaw, Custom &moveController, cv::Point &target_point, cv::Point &midMidPoint,
                  PIDController &pidXTaskPre, PIDController &pidYTaskPre, PIDController &pidYawTaskPre,
                  float xp, float yp, float xr, Timer &timer, Camera &cam)
{
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
        }
        else
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].ready)
    { // 任务开始前：对齐任务标签
        if (taskDetected == taskNow)
        {
            adjust(pidXTaskPre, pidYTaskPre, pidYawTaskPre, moveController,
                   xp, yp, xr, task[taskNow].ready, 1);
            if (task[taskNow].ready) // 对齐后
            {
                moveController.still();                  // 停下来歇会
                moveController.rotateLeft(30);           // 左转30度
                moveController.forwardWalkNew(0.6, 0.3); // 前进0.6米，速度0.3m/s
                moveController.still();                  // 停下来歇会
                timer.reset(10);                         // 10秒倒计时：等待放下物资
                cam.reopen();
            }
        }
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：放下物资
        if (!timer.isTimeUp())
        { // 未到时间：继续前进
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        }
        else
        { // 到时间：放下物资
            moveController.sitDown();
            putDown(task1st, task2nd, taskNow);
            moveController.standUp();
            task[taskNow].putDown = true;
            cam.reopen();
        }
        return;
    }
    if (!task[taskNow].done)
    { // 任务结束：离开任务区域
        if (taskDetected == taskNow)
        {
            adjust(pidXTaskPre, pidYTaskPre, pidYawTaskPre, moveController,
                   xp, yp, xr, task[taskNow].done, -1);
            if (task[taskNow].done)
            {
                moveController.forwardWalkNew(0.8, 0.3);
                cam.reopen();
            }
        }
        else
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        return;
    }
}
void workTask3or4(int taskNow, int taskDetected, Task task[], int task1st, int task2nd,
                  PIDController &pidYaw, Custom &moveController, cv::Point target_point, cv::Point midMidPoint, Timer &timer, Camera &cam)
{
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
            task[taskNow].ready = true;
            timer.reset(10);
        }
        else
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：放下物资
        if (!timer.isTimeUp())
        { // 未到时间：继续前进
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        }
        else
        { // 到时间：放下物资
            moveController.sitDown();
            putDown(task1st, task2nd, taskNow);
            moveController.standUp();
            task[taskNow].putDown = true;
            task[taskNow].done = true;
            cam.reopen();
        }
        return;
    }
}
void workTask0(int taskNow, int taskDetected, Task task[],
               PIDController &pidYaw, Custom &moveController, cv::Point target_point, cv::Point midMidPoint)
{
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
            task[taskNow].ready = true;
        }
        else
            keepGoing(pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：回到启停区
        moveController.moveRight(0.2, 3);
        task[taskNow].putDown = true;
        task[taskNow].done = true;
        return;
    }
}

#endif