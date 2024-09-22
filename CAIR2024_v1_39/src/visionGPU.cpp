#include <opencv2/opencv.hpp>

#include "Camera.h"

cv::Mat getMask(cv::Mat src,
                int yellowHLower = 10, int yellowHUpper = 40,
                int yellowSLower = 50, int yellowSUpper = 255,
                int yellowVLower = 0, int yellowVUpper = 255)
{
    // 转hsv，中值滤波去噪
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    cv::medianBlur(src, src, 5);
    // 黄色阈值掩膜
    cv::Mat mask;
    const cv::Scalar yellow_lower = cv::Scalar(yellowHLower, yellowSLower, yellowVLower);
    const cv::Scalar yellow_upper = cv::Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    cv::inRange(src, yellow_lower, yellow_upper, mask);
    return mask;
}

int main(int argc, char **argv)
{
    Camera cam2(2);
    cv::Mat cam2_frame;
    cv::Mat mask;

    double fps;
    double t = (double)cv::getTickCount();

    while (true)
    {
        cam2_frame = cam2.getFrame();

        // 起始位置比例
        float rectangle_top = 1 / 2.0;
        // 结束位置比例
        float rectangle_bottom = 6 / 10.0;
        // 绘制图形段
        cv::line(cam2_frame,
                 cv::Point(0, cam2_frame.rows * rectangle_top),
                 cv::Point(cam2_frame.cols, cam2_frame.rows * rectangle_top),
                 cv::Scalar(0, 255, 0), 2);
        cv::line(cam2_frame,
                 cv::Point(0, cam2_frame.rows * rectangle_bottom),
                 cv::Point(cam2_frame.cols, cam2_frame.rows * rectangle_bottom),
                 cv::Scalar(0, 0, 255), 2);
        // 取图像段
        cv::Rect roi(0, cam2_frame.rows * rectangle_top,
                     cam2_frame.cols, cam2_frame.rows * (rectangle_bottom - rectangle_top));
        cv::Mat roi_frame = cam2_frame(roi);
        cv::imshow("ROI", roi_frame);

        // mask = getMask(cam2_frame.clone(), 10, 40, 50, 255, 0, 255);
        mask = getMask(roi_frame.clone(), 10, 40, 50, 255, 0, 255);
        cv::imshow("Mask", mask);

        // 计算帧率
        fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
        t = (double)cv::getTickCount();
        // 将帧率显示在图像上

        cv::putText(cam2_frame, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Camera 2", cam2_frame);

        cv::waitKey(1);
    }

    return 0;
}
