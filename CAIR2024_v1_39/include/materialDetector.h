#ifndef MATERIALDETECTOR_H
#define MATERIALDETECTOR_H

#include <thread>
#include <chrono>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "tasks.h"
#include "Camera.h"

extern Task task[6];

void greenshow(std::string str);
void greenINFO(std::string str);
void blueshow(std::string str);
void blueINFO(std::string str);
void redshow(std::string str);
void redINFO(std::string str);

class MaterialDetector
{
private:
    // 创建ArUco字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // 检测标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // Camera *camera;

public:
    // MaterialDetector();
    int getTaskID();
    void detect(Task task[]);
};

#endif