#include "main.h"

int main(int argc, char **argv)
{
    Py_Initialize(); // 初始化Python环境

    // 识别物资领取对应任务
    int task1st = -1, task2nd = -1;
    // getTasks(task, task1st, task2nd);

    // DEBUG: 任务区域识别
    // task[0].have = task[1].have = task[2].have = task[3].have = task[4].have = 1;
    task[0].have = task[3].have = task[4].have = true;
    task1st = 3, task2nd = 4;
    // task[0].have = task[task1st].have = task[task2nd].have = true;

    // 装载物资
    // Equip();

    // 载入 运动 组件
    Custom moveController(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &moveController);

    // // 移出启停区
    // moveController.standUp();
    // moveController.moveRight(-0.3, 4);

    bool testMove = true;

    // 启动视觉导航
    PIDController pidYaw(0.02, 0, 0); // Yaw轴PID
    std::map<std::string, int> configParams = readConfigFile("../config/HSVconfig.txt"); // 加载颜色阈值参数
    taskDetector taskDetector;                                                           // 载入 任务区域识别 组件
    int taskDetected = -1;
    Timer timer;

    // 拓展的下巴相机
    bool camOk;
    ExtraCamera cam0(camOk);
    if (!camOk)
    {
        std::cout << "Failed to open the camera" << std::endl;
        return -1;
    }
    // 面部相机
    Camera cam1(1);
    cv::Mat frame = cam0.getFrame();
    RoadDetector roadDetector(frame, 0.45, 0, 0.25, configParams); // 载入 道路识别 组件
    bool loadFrontCamera = false;
    while (true)
    {
        double t = (double)cv::getTickCount(); // 帧率计时开始
        frame = cam0.getFrame();
        // 获取道路导航数据
        cv::Point target_point, midMidPoint;
        double angle;
        cv::Mat result;
        roadDetector.getPointAndAngel(frame, midMidPoint, target_point, angle, result);

        int taskNow = getTaskNow(task); // 获取当前任务
        std::cout << "taskNow: " << taskNow << "\n";

        if (taskNow == 5 && !loadFrontCamera)
        {
            cam1.reopen();
            loadFrontCamera = true;
        }
        // 检测任务标签
        taskDetected = -1;
        if (loadFrontCamera && !task[5].arrived)
        {
            cv::Mat frontFrame = cam1.getFrame();
            cv::Mat frontResult = frontFrame.clone();
            taskDetector.detect(frontFrame, frontResult, taskDetected); // 检测任务标签
        }
        else
        {
            taskDetector.detect(frame, result, taskDetected); // 检测任务标签
        }
        std::cout << "taskDetected: " << taskDetected << "\n";

        // 任务流转执行
        if (taskNow == 1)
            workTask1(taskNow, taskDetected, task, task1st, task2nd, roadDetector,
                      pidYaw, moveController, target_point, midMidPoint, timer, cam0, testMove);
        else if (taskNow == 2)
            workTask2(taskNow, taskDetected, task, task1st, task2nd, roadDetector,
                      pidYaw, moveController, target_point, midMidPoint, timer, cam0, testMove);
        else if (taskNow == 3 || taskNow == 4)
            workTask3or4(taskNow, taskDetected, task, task1st, task2nd, roadDetector,
                         pidYaw, moveController, target_point, midMidPoint, timer, cam0, testMove);
        else if (taskNow == 0)
            workTask0(taskNow, taskDetected, task, pidYaw, moveController, target_point, midMidPoint);
        else if (taskNow == 5)
            workTask5(taskNow, taskDetected, task, pidYaw, moveController, target_point, midMidPoint);

        // 计算帧率显示在图像上
        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        cv::putText(result, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("result", result);
        cv::waitKey(1);
    }

    Py_Finalize(); // 清理并关闭Python环境

    return 0;
}