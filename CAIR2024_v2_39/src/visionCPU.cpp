#include "visionCPU.h"

RoadDetector::RoadDetector(
    const cv::Mat &frame,
    float point_rate,                        // 目标点位置比例
    float rectangle_top_rate,                // 起始位置比例
    float rectangle_bottom_rate,             // 结束位置比例
    std::map<std::string, int> configParams) // 颜色阈值
    : point_rate(point_rate),
      rectangle_top_rate(rectangle_top_rate),
      rectangle_bottom_rate(rectangle_bottom_rate)
{
    rectangle_top = frame.rows * rectangle_top_rate;
    rectangle_bottom = frame.rows * rectangle_bottom_rate;
    HLower = configParams["HLower"];
    HUpper = configParams["HUpper"];
    SLower = configParams["SLower"];
    SUpper = configParams["SUpper"];
    VLower = configParams["VLower"];
    VUpper = configParams["VUpper"];
}
// 获取导航数据
void RoadDetector::getPointAndAngel(const cv::Mat &frame,
                                    cv::Point &midMidPoint, cv::Point &target_point, double &angle, cv::Mat &result)
{
    cv::Mat rect;
    getRectAndTargetPoint(frame, rect, target_point);                                     // 获取导航段和目标点
    cv::Mat mask = getMask(rect.clone(), HLower, HUpper, SLower, SUpper, VLower, VUpper); // 获取掩膜
    // cv::imshow("mask", mask);
    // cv::waitKey(1);
    std::vector<cv::Point> midPoints = getMidPoints(mask); // 获取中点们
    midMidPoint = getMidMidPoint(midPoints);               // 取中点们的中点
    cv::Vec4f line;
    getFittedLine(midPoints, line, angle); // 求中点们的拟合直线
    result = draw(frame, target_point, midPoints, midMidPoint, line, angle);
}
// 获取导航段和目标点
void RoadDetector::getRectAndTargetPoint(const cv::Mat &frame,
                                         cv::Mat &roi_frame, cv::Point &target_point)
{
    cv::Rect roi(0, rectangle_top,
                 frame.cols, (rectangle_bottom - rectangle_top));
    roi_frame = frame(roi);
    target_point = cv::Point(roi_frame.cols * point_rate, rectangle_top + roi_frame.rows / 2);
}
// 获取掩膜：黄色阈值
cv::Mat RoadDetector::getMask(cv::Mat src,
                              int yellowHLower, int yellowHUpper,
                              int yellowSLower, int yellowSUpper,
                              int yellowVLower, int yellowVUpper)
{
    // 转hsv，中值滤波去噪
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    cv::medianBlur(src, src, 5);
    cv::Mat mask;
    const cv::Scalar yellow_lower = cv::Scalar(yellowHLower, yellowSLower, yellowVLower);
    const cv::Scalar yellow_upper = cv::Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    cv::inRange(src, yellow_lower, yellow_upper, mask);
    return mask;
}
// 获取中点们：获取每行白色像素的中点
std::vector<cv::Point> RoadDetector::getMidPoints(const cv::Mat &mask)
{
    std::vector<cv::Point> midPoints;
    for (int y = 0; y < mask.rows; ++y)
    {
        cv::Mat row = mask.row(y);
        // 求和取均值得中点
        std::vector<int> whitePixelPositions;
        for (int x = 0; x < row.cols; ++x)
        {
            if (row.at<uchar>(x) == 255)
            {
                whitePixelPositions.push_back(x);
            }
        }
        if (!whitePixelPositions.empty())
        {
            int mid = std::accumulate(whitePixelPositions.begin(), whitePixelPositions.end(), 0) / whitePixelPositions.size();
            midPoints.push_back(cv::Point(mid, y + rectangle_top));
        }
    }
    return midPoints;
}
// 取中点们的中点：求和取均值得中点
cv::Point RoadDetector::getMidMidPoint(const std::vector<cv::Point> &midPoints)
{
    cv::Point midMidPoint;
    if (!midPoints.empty())
    {
        int midX = std::accumulate(midPoints.begin(), midPoints.end(), 0, [](int sum, const cv::Point &point)
                                   { return sum + point.x; }) /
                   midPoints.size();
        int midY = std::accumulate(midPoints.begin(), midPoints.end(), 0, [](int sum, const cv::Point &point)
                                   { return sum + point.y; }) /
                   midPoints.size();
        midMidPoint = cv::Point(midX, midY);
    }
    return midMidPoint;
}
// 求中点们的拟合直线
void RoadDetector::getFittedLine(const std::vector<cv::Point> &midPoints,
                                 cv::Vec4f &line, double &angle)
{
    // Check if there are enough points for line fitting
    if (midPoints.size() >= 2)
    {
        cv::fitLine(midPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line[0], vy = line[1];
        float x = line[2], y = line[3];
        angle = atan2(vy, vx) * 180.0 / CV_PI; // Convert radian to degree
        if (angle < 0)
            angle += 180;
    }
}
// 绘制导航段
cv::Mat RoadDetector::draw(const cv::Mat &frame,
                           const cv::Point &target_point,
                           const std::vector<cv::Point> &midPoints,
                           const cv::Point &midMidPoint,
                           const cv::Vec4f &line,
                           const double &angle)
{
    cv::Mat result = frame.clone();
    // 绘制导航段
    cv::line(result,
             cv::Point(0, rectangle_top),
             cv::Point(frame.cols, rectangle_top),
             cv::Scalar(0, 255, 0), 2);
    cv::line(result,
             cv::Point(0, rectangle_bottom),
             cv::Point(frame.cols, rectangle_bottom),
             cv::Scalar(0, 255, 0), 2);
    // 在导航段的目标点处绘制圆
    cv::circle(result, target_point, 5, cv::Scalar(0, 255, 0), -1);
    // 绘制导航段的目标点所在的竖直线
    cv::line(result,
             cv::Point(target_point.x, rectangle_top),
             cv::Point(target_point.x, rectangle_bottom),
             cv::Scalar(0, 255, 0), 2);
    // 绘制中点们
    for (const auto &midPoint : midPoints)
    {
        cv::circle(result, midPoint, 1, cv::Scalar(255, 0, 0), -1);
    }
    // 绘制中点们的中点
    cv::circle(result, midMidPoint, 5, cv::Scalar(0, 0, 255), -1);
    // 绘制中点们的拟合直线
    // Calculate the x coordinates at rectangle_top and rectangle_bottom
    float vx = line[0], vy = line[1];
    float x = line[2], y = line[3];
    int x1 = x + (rectangle_top - y) * (vx / vy);
    int x2 = x + (rectangle_bottom - y) * (vx / vy);
    cv::line(result, cv::Point(x1, rectangle_top), cv::Point(x2, rectangle_bottom), cv::Scalar(0, 0, 255), 2);
    // 可视化角度
    cv::putText(result, "Angle: " + std::to_string(angle), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    return result;
}