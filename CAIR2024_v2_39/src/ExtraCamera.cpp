#include "ExtraCamera.h"

ExtraCamera::ExtraCamera(bool &ok)
{
    cap = new cv::VideoCapture(1); // 打开加装的摄像头
    if (!cap->isOpened())           // 检查是否成功打开摄像头
    {
        std::cout << "Could not open the camera" << std::endl;
        ok = false;
        return;
    }
    ok = true;
}
cv::Mat ExtraCamera::getFrame()
{
    std::cout << "Frame capturing..." << std::endl;
    cv::Mat frame;
    while (true)
    {
        bool bSuccess = cap->read(frame); // 读取一帧
        if (bSuccess)
        {
            std::cout << "getFrame() success" << std::endl;
            cv::flip(frame, frame, -1);
            return frame;
        }
    }
}
