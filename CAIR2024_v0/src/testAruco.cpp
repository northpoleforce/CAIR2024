#include <opencv2/aruco.hpp>

#include "Camera.h"

int main()
{
    Camera camera(2);

    // 相机的内参
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 6.9280202257690246e+02, 2.4139837284139807e+00, 4.1529485401277941e+02,
                            0., 7.1310835266467575e+02, 4.3888779500356623e+02,
                            0., 0., 1.);
    // 相机的畸变系数
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << 6.5608461313067001e-01, -1.5134503590225326e+00,
                          6.2380235102929881e-03, 3.2964624860536132e-03);
    // ArUco标记的实际大小（单位：米）
    float markerLength = 0.06;

    while (true)
    {
        cv::Mat frame = camera.getFrame();
        cv::Mat inputImage = frame;

        // 创建ArUco字典
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

        // 检测标记
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);

        cv::Mat rvecs, tvecs;
        // 估计姿态
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        if (!ids.empty())
        {
            // 如果找到了标记，绘制它们
            cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
            // 打印标记的ID和位置
            // 打印相机相对于每个标记的位置和旋转角度
            // for (unsigned int i = 0; i < ids.size(); i++)
            // {
            //     std::cout << "ID = " << ids[i] << ", Position = " << corners[i] << std::endl;
            //     std::cout << "ID = " << ids[i] << ", Rotation = " << rvecs[i] << ", Translation = " << tvecs[i] << std::endl;
            // }
            for (unsigned int i = 0; i < ids.size(); i++)
            {
                std::cout << "ID = " << ids[i]
                          << ", Rotation = " << rvecs.at<cv::Vec3d>(i)
                          << ", Translation = " << tvecs.at<cv::Vec3d>(i)
                          << std::endl;
                std::cout << ", Rotation(x) = " << rvecs.at<cv::Vec3d>(i)[0] << std::endl;
            }
        }
        cv::imshow("image.jpg", inputImage);
        cv::waitKey(1);
    }

    return 0;
}