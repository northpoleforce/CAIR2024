#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

#include "ReadParams.h"

class RoadDetector
{
public:
    float point_rate;
    float rectangle_top_rate, rectangle_bottom_rate;
    int rectangle_top, rectangle_bottom;
    int    HLower, HUpper, SLower, SUpper, VLower, VUpper;
    RoadDetector(
        const cv::Mat &frame,
        float point_rate,            // 目标点位置比例
        float rectangle_top_rate,     // 起始位置比例
        float rectangle_bottom_rate, // 结束位置比例
        std::map<std::string, int> configParams);
    // 获取导航数据
    void getPointAndAngel(const cv::Mat &frame,
                          cv::Point &midMidPoint, cv::Point &target_point, double &angle, cv::Mat &result);
    // 获取导航段和目标点
    void getRectAndTargetPoint(const cv::Mat &frame,
                               cv::Mat &roi_frame, cv::Point &target_point);
    // 获取掩膜：黄色阈值
    cv::Mat getMask(cv::Mat src,
                    int yellowHLower, int yellowHUpper,
                    int yellowSLower, int yellowSUpper,
                    int yellowVLower, int yellowVUpper);
    // 获取中点们：获取每行白色像素的中点
    std::vector<cv::Point> getMidPoints(const cv::Mat &mask);
    // 取中点们的中点：求和取均值得中点
    cv::Point getMidMidPoint(const std::vector<cv::Point> &midPoints);
    // 求中点们的拟合直线
    void getFittedLine(const std::vector<cv::Point> &midPoints,
                       cv::Vec4f &line, double &angle);
    // 绘制导航段
    cv::Mat draw(const cv::Mat &frame,
                 const cv::Point &target_point,
                 const std::vector<cv::Point> &midPoints,
                 const cv::Point &midMidPoint,
                 const cv::Vec4f &line,
                 const double &angle);
};

#endif // VISION_H