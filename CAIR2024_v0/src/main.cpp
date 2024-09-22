#include "main.h"

int main(int argc, char **argv)
{
    task[0].have = task[1].have = task[2].have = task[3].have = task[4].have = 1;
    // // 识别物资领取对应任务
    // getTasks(task);

    // 载入 运动 组件
    Custom moveController(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &moveController);

    // 移出启停区
    moveController.moveRight(-0.2, 4);

    // 启动视觉导航
    taskDetector taskDetector; // 载入 任务区域识别 组件
    int taskDetected = -1;
    float xp, yp, xr;

    // PIDs
    PIDController pidYaw(0.01, 0, 0); // Yaw轴PID
    // 任务区域重定位PID
    PIDController pidXTaskPre(1.5, 0, 0);
    PIDController pidYTaskPre(2, 0, 0);
    PIDController pidYawTaskPre(1, 0, 0);

    Timer timer;

    Camera cam2(2); // 开启摄像头
    // cv::Mat frame = cam2.getSharedFrame();
    cv::Mat frame = cam2.getFrame();
    RoadDetector roadDetector(frame, 1 / 2.0, 3 / 4.0, 9 / 10.0); // 载入 道路识别 组件

    while (true)
    {
        double t = (double)cv::getTickCount(); // 帧率计时开始

        // frame = cam2.getSharedFrame(); // 获取视频流图像
        frame = cam2.getFrame();
        // 获取道路导航数据
        cv::Point target_point, midMidPoint;
        double angle;
        cv::Mat result;
        roadDetector.getPointAndAngel(frame, midMidPoint, target_point, angle, result);
        taskDetector.detect(frame, result, taskDetected, xp, yp, xr); // 检测任务标签
        int taskNow = getTaskNow(task);                              // 获取当前任务
        std::cout << "taskNow: " << taskNow << "\n";
        std::cout << "taskDetected: " << taskDetected << "\n";
        // 任务流转执行
        if (taskNow == 1 || taskNow == 2)
            workTask1or2(taskNow, taskDetected, task,
                         pidYaw, moveController, target_point, midMidPoint,
                         pidXTaskPre, pidYTaskPre, pidYawTaskPre, xp, yp, xr, timer, cam2);
        else if (taskNow == 3 || taskNow == 4)
            workTask3or4(taskNow, taskDetected, task,
                         pidYaw, moveController, target_point, midMidPoint, timer, cam2);
        else if (taskNow == 0)
            workTask0(taskNow, taskDetected, task, pidYaw, moveController, target_point, midMidPoint);

        // 计算帧率显示在图像上
        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        cv::putText(result, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("result", result);
        cv::waitKey(1);
    }

    return 0;
}