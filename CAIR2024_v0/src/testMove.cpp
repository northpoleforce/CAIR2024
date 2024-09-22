#include "Camera.h"
#include "visionCPU.h"
#include "move.h"

int main(int argc, char **argv)
{
    // 运动
    Custom moveController(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &moveController);
    PIDController pidY(-0.002, 0, 0); // Y轴PID
    PIDController pidYaw(0.01, 0, 0); // Yaw轴PID
    // 视觉
    Camera cam(2);
    cv::Mat frame = cam.getFrame();
    RoadDetector roadDetector(frame, 1 / 2.0, 3 / 4.0, 9 / 10.0);

    while (true)
    {
        double t = (double)cv::getTickCount(); // 帧率数据
        frame = cam.getFrame();

        // 获取道路修正数据
        cv::Mat result;
        cv::Point target_point, midMidPoint;
        double angle;
        roadDetector.getPointAndAngel(frame, midMidPoint, target_point, angle, result);
        // 计算帧率显示在图像上
        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        cv::putText(result, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("result", result);
        cv::waitKey(1);

        // 控制
        {
            // // 调试Y
            // float vy = pidY.P(target_point.x, midMidPoint.x);
            // std::cout << "vy (calc): " << vy << std::endl;
            // moveController.setVelocity(0, vy, 0);
            // // 调试Yaw
            // float vYaw = pidYaw.P(90, angle);
            // std::cout << "vYaw (calc): " << vYaw << std::endl;
            // moveController.setVelocity(0, 0, vYaw);
            // // 调试Y和Yaw
            // float vy = pidY.PT(target_point.x, midMidPoint.x, 10);
            // float vYaw = pidYaw.PT(90, angle, 2);
            // std::cout << "vy (calc): " << vy << std::endl;
            // std::cout << "vYaw (calc): " << vYaw << std::endl;
            // moveController.setVelocity(0, vy, vYaw);
            // // 组合前进
            // float vy = pidY.PT(target_point.x, midMidPoint.x, 10);
            // float vYaw = pidYaw.PT(90, angle, 2);
            // std::cout << "vy (calc): " << vy << std::endl;
            // std::cout << "vYaw (calc): " << vYaw << std::endl;
            // moveController.setVelocity(0.1, vy, vYaw);
        }

        // // Yaw(通过两点距离来控制Yaw) 前进
        // float vYaw = -pidYaw.PT(target_point.x, midMidPoint.x, 10);
        // std::cout << "vYaw (calc): " << vYaw << std::endl;
        // moveController.setVelocity(0.1, 0, vYaw);
    }

    return 0;
}