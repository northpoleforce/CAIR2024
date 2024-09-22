#include "taskDetector.h"

void taskDetector::detect(const cv::Mat &frame,
                          cv::Mat &result, int &flag, float &xp, float &yp, float &xr)
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
        // 估计姿态
        cv::Mat rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        xr = rvecs.at<cv::Vec3d>(0)[0];
        // xr = filter.filter(rvecs.at<cv::Vec3d>(0)[0]);
        xp = tvecs.at<cv::Vec3d>(0)[1], yp = tvecs.at<cv::Vec3d>(0)[0];
        std::cout << "x y:" << xp << " " << yp << std::endl;
        std::cout << "xr:" << xr << std::endl;
    }
}