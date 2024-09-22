#ifndef TASK_DETECTOR_H
#define TASK_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class taskDetector
{
private:
    // 创建ArUco字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

public:
    void detect(const cv::Mat &frame,
                cv::Mat &result, int &flag);
};

#endif // TASK_DETECTOR_H