#include "taskDetector.h"

void taskDetector::detect(const cv::Mat &frame,
                          cv::Mat &result, int &flag)
{
    // 检测标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    flag = -1;
    if (!ids.empty())
    {
        flag = ids[0];
        // 如果找到了标记，绘制它们
        cv::aruco::drawDetectedMarkers(result, corners, ids);
    }
}