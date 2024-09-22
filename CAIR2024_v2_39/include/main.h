#ifndef MAIN_H
#define MAIN_H

#include "move.h"
#include "Arm.h"

#include "materialDetector.h"
#include "ExtraCamera.h"
#include "Camera.h"
#include "visionCPU.h"
#include "taskDetector.h"
#include "ReadParams.h"

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

void keepGoing(float xSpeed, PIDController &pidYaw, Custom &moveController, cv::Point target_point, cv::Point midMidPoint)
{
    // Yaw(通过两点距离差来控制Yaw) 前进
    float vYaw = pidYaw.PT(target_point.x, midMidPoint.x, 10);
    // std::cout << "vYaw (calc): " << vYaw << std::endl;
    moveController.setVelocity(xSpeed, 0, vYaw);
}

void getTasks(Task task[], int &task1st, int &task2nd, ExtraCamera &cam)
{
    task[0].have = task[5].have = true;
    MaterialDetector materialDetector;
    materialDetector.detect(task, cam);
    for (int i = 1; i <= 4; ++i)
    {
        if (task[i].have)
        {
            if (task1st == -1)
                task1st = i;
            else
                task2nd = i;
        }
    }
}
int getTaskNow(Task task[])
{
    // for (int i = 0; i <= 5; ++i)
    // {
    //     std::cout << i << " have:" << task[i].have << " " << "done: " << task[i].done << "\n";
    // }
    if ((!task[1].have || task[1].done) && task[5].have && !task[5].done)
        return 5;
    for (int i = 1; i <= 4; ++i)
        if (task[i].have && !task[i].done)
            return i;
    if (task[0].have && !task[0].done)
        return 0;
    return -1;
}
void workTask1(int taskNow, int taskDetected, Task task[], int task1st, int task2nd, RoadDetector &roadDetector,
               PIDController &pidYaw, Custom &moveController, cv::Point &target_point, cv::Point &midMidPoint,
               Timer &timer, ExtraCamera &cam, bool testMove)
{
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
        }
        else
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].ready)
    { // 任务开始前：对齐任务标签
        if (taskDetected == taskNow)
        {
            task[taskNow].ready = true;
            if (task[taskNow].ready) // 对齐后
            {
                moveController.still();                  // 停下来歇会
                moveController.rotateLeft(45);           // 左转30度
                moveController.forwardWalkNew(0.6, 0.3); // 前进0.6米，速度0.3m/s
                moveController.still();                  // 停下来歇会
                timer.reset(4);                          // 10秒倒计时：等待放下物资
                // cam.reopen();
            }
        }
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：放下物资
        if (!timer.isTimeUp())
        { // 未到时间：继续前进
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        }
        else
        {                           // 到时间：放下物资
            moveController.still(); // 停下来歇会
            moveController.sitDown();
            if (!testMove)
            {
                putDown(task1st, task2nd, taskNow);
            }
            moveController.standUp();
            moveController.still(); // 停下来歇会
            task[taskNow].putDown = true;
        }
        return;
    }
    if (!task[taskNow].done)
    { // 任务结束：离开任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].done = true;
            moveController.forwardWalkNew(0.4, 0.3); // 前进0.6米，速度0.3m/s
            roadDetector.point_rate = 0.45;
        }
        else
        {
            roadDetector.point_rate = 0.55;
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        }

        return;
    }
}
void workTask2(int taskNow, int taskDetected, Task task[], int task1st, int task2nd, RoadDetector &roadDetector,
               PIDController &pidYaw, Custom &moveController, cv::Point &target_point, cv::Point &midMidPoint,
               Timer &timer, ExtraCamera &cam, bool testMove)
{
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
        }
        else
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].ready)
    { // 任务开始前：对齐任务标签
        if (taskDetected == taskNow)
        {
            task[taskNow].ready = true;
            if (task[taskNow].ready) // 对齐后
            {
                moveController.still();                  // 停下来歇会
                moveController.rotateLeft(45);           // 左转30度
                moveController.forwardWalkNew(0.6, 0.3); // 前进0.6米，速度0.3m/s
                moveController.still();                  // 停下来歇会
                timer.reset(6);                          // 10秒倒计时：等待放下物资
                // cam.reopen();
            }
        }
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：放下物资
        if (!timer.isTimeUp())
        { // 未到时间：继续前进
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        }
        else
        {                           // 到时间：放下物资
            moveController.still(); // 停下来歇会
            moveController.sitDown();
            if (!testMove)
            {
                putDown(task1st, task2nd, taskNow);
            }
            moveController.standUp();
            moveController.still(); // 停下来歇会
            task[taskNow].putDown = true;
        }
        return;
    }
    if (!task[taskNow].done)
    { // 任务结束：离开任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].done = true;
            // moveController.forwardWalkNew(0.4, 0.3); // 前进0.6米，速度0.3m/s
            moveController.rotateLeft(-30);
            roadDetector.point_rate = 0.45;
        }
        else
        {
            roadDetector.point_rate = 0.55;
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        }

        return;
    }
}
void workTask3or4(int taskNow, int taskDetected, Task task[], int task1st, int task2nd, RoadDetector &roadDetector,
                  PIDController &pidYaw, Custom &moveController, cv::Point target_point, cv::Point midMidPoint, Timer &timer, ExtraCamera &cam, bool testMove)
{
    roadDetector.point_rate = 0.5;
    if (!task[taskNow].arrived)
    { // 未到达任务区域：前进寻找任务区域
        if (taskDetected == taskNow)
        {
            task[taskNow].arrived = true;
            task[taskNow].ready = true;
            timer.reset(7);
        }
        else
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：放下物资
        if (!timer.isTimeUp())
        { // 未到时间：继续前进
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        }
        else
        { // 到时间：放下物资
            moveController.sitDown();
            if (!testMove)
            {
                putDown(task1st, task2nd, taskNow);
            }
            moveController.standUp();
            task[taskNow].putDown = true;
            task[taskNow].done = true;
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
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：回到启停区
        moveController.moveRight(0.25, 3);
        task[taskNow].putDown = true;
        task[taskNow].done = true;
        return;
    }
}

void workTask5(int taskNow, int taskDetected, Task task[],
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
            keepGoing(0.2, pidYaw, moveController, target_point, midMidPoint);
        return;
    }
    if (!task[taskNow].putDown)
    { // 任务进行中：左移直走
        moveController.moveRight(-0.3, 5);
        moveController.forwardWalkNew(0.8, 0.3);
        moveController.rotateLeft(45);
        task[taskNow].putDown = true;
        task[taskNow].done = true;
        return;
    }
}

#endif