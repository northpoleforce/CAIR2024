#include "Camera.h"
#include "ReadParams.h"
#include "taskDetector.h"

int main(int argc, char **argv)
{
    Camera cam1(1);
    std::map<std::string, int> configParams = readConfigFile("../config/HSVconfig.txt");

    cv::Mat frame = cam1.getFrame();

    int HLower = configParams["HLower"];
    int HUpper = configParams["HUpper"];
    int SLower = configParams["SLower"];
    int SUpper = configParams["SUpper"];
    int VLower = configParams["VLower"];
    int VUpper = configParams["VUpper"];

    taskDetector taskDetector; // 载入 任务区域识别 组件
    while (true)
    {
        double t = (double)cv::getTickCount(); // 帧率计时开始

        frame = cam1.getFrame().clone();
        cv::Mat result = frame.clone();
        int taskDetected = -1;
        taskDetector.detect(frame, result, taskDetected); 
        std::cout << "Task Detected: " << taskDetected << std::endl;

        // 计算帧率显示在图像上
        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        cv::putText(result, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Front Camera(result)", result);
        cv::waitKey(1);
    }

    return 0;
}