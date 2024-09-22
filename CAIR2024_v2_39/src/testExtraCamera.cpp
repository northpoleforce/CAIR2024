#include "ExtraCamera.h"
#include "ReadParams.h"
#include "taskDetector.h"

int main(int argc, char **argv)
{
    bool ok;
    ExtraCamera extraCamera(ok);
    if (!ok)
    {
        std::cout << "Failed to open the camera" << std::endl;
        return -1;
    }

    std::map<std::string, int> configParams = readConfigFile("../config/HSVconfig.txt");

    cv::Mat frame = extraCamera.getFrame();
    RoadDetector roadDetector(frame, 0.5, 0.25, 0.5, configParams);

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

        frame = extraCamera.getFrame().clone();
        cv::Mat mask = roadDetector.getMask(frame.clone(), HLower, HUpper, SLower, SUpper, VLower, VUpper);
        cv::imshow("Mask", mask);
        int taskDetected = -1;
        cv::Mat result = frame.clone();
        taskDetector.detect(frame, result, taskDetected); 
        std::cout << "Task Detected: " << taskDetected << std::endl;

        // 计算帧率显示在图像上
        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        cv::putText(result, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Extra Camera(result)", result);
        cv::waitKey(1);
    }

    return 0;
}
