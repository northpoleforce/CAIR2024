#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "Camera.h"

void waiting_for_good_frame(Camera &cam, cv::Mat &frame, int t)
{
    std::cout << "waiting for good frame!\n";
    auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (duration.count() >= t)
        {
            break;
        }
        frame = cam.getFrame();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 休眠一段时间以防止过度使用CPU
    }
    std::cout << "good frame comes!\n";
}

int main(int argc, char **argv)
{
    Camera cam2(2);
    // Camera cam5(5);
    cv::Mat cam2_frame, cam5_frame;
    // waiting_for_good_frame(cam2, cam2_frame, 3);
    while (true)
    {
        cam2_frame = cam2.getFrame();
        // cam5_frame = cam5.getFrame();
        cv::imshow("Camera 2", cam2_frame);
        // cv::imshow("Camera 5", cam5_frame);
        cv::waitKey(1);
        // if (cv::waitKey(1) == 27)
        //     break;
    }

    return 0;
}