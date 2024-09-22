#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480);
    ~Camera();
    void reopen();

    void waiting_for_good_frame(int t);
    cv::Mat getSharedFrame();
    void flashFrame();
    cv::Mat getFrame();

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
    std::string udp_send_integrated_pipe_0;
    cv::Mat sharedFrame; // 共享Frame
    std::mutex mtx;
    bool stop_thread;
};

#endif // CAMERA_H