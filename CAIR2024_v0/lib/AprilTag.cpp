#include <iostream>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>

int main() {
    // 打开默认摄像头
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        std::cerr << "Could not open the camera." << std::endl;
        return -1;
    }

    // 设置AprilTag检测器
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if(frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 将OpenCV图像转换为AprilTag图像格式
        image_u8_t im = {.width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data};

        // 检测标记
        zarray_t *detections = apriltag_detector_detect(td, &im);

        // 遍历检测到的标记并打印ID
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            std::cout << "Detected AprilTag with ID: " << det->id << std::endl;
            cv::rectangle(frame, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
        }

        // 清理并释放资源
        apriltag_detections_destroy(detections);

        // 显示图像
        cv::imshow("AprilTag Detections from Camera", frame);
        char key = (char)cv::waitKey(10);
        if (key == 27) break; // 按下ESC键退出
    }

    // 清理并释放资源
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}