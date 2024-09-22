#include <opencv2/aruco.hpp>

#include "Camera.h"
#include "move.h"

class MovingAverageFilter
{
public:
    MovingAverageFilter(size_t window_size) : window_size_(window_size) {}

    double filter(const double &rvec)
    {
        buffer_.push_back(rvec);
        if (buffer_.size() > window_size_)
        {
            buffer_.pop_front();
        }

        double sum = 0;
        std::cout << "buffer: ";
        for (const auto &r : buffer_)
        {
            std::cout << r << " ";
            sum += r;
        }
        std::cout << "\n";
        std::cout << "--------\n";
        // std::cout << static_cast<double>(buffer_.size());
        std::cout << sum / static_cast<double>(buffer_.size()) << "\n";
        std::cout << "============\n";

        return sum / static_cast<double>(buffer_.size());
    }

private:
    std::deque<double> buffer_;
    size_t window_size_;
};

void save2record(cv::Mat &inputImage)
{
    // Get the current time stamp
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    // Convert the time stamp to string
    std::ostringstream os;
    os << millis;
    // Create the file name
    std::string filename = "./record/" + os.str() + ".png";
    // Save the image
    cv::imwrite(filename, inputImage);
}

int main()
{
    Custom moveController(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &moveController);

    PIDController pidX(1.5, 0.01, 0);  // X的PID
    PIDController pidY(2, 0.01, 0);    // Y的PID
    PIDController pidYaw(1, 0.005, 0); // Yaw的PID

    // ArUco标记的实际大小（单位：米）
    float markerLength = 0.06;
    // 相机的内参
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 6.9280202257690246e+02, 2.4139837284139807e+00, 4.1529485401277941e+02,
                            0., 7.1310835266467575e+02, 4.3888779500356623e+02,
                            0., 0., 1.);
    // 相机的畸变系数
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << 6.5608461313067001e-01, -1.5134503590225326e+00,
                          6.2380235102929881e-03, 3.2964624860536132e-03);
    Camera camera(2);

    MovingAverageFilter filter(10);

    while (true)
    {
        cv::Mat frame = camera.getFrame();
        cv::Mat inputImage = frame.clone();
        save2record(inputImage);

        // 创建ArUco字典
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

        // 检测标记
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);

        if (!ids.empty())
        {
            // 如果找到了标记，绘制它们
            cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
            // 估计姿态
            cv::Mat rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            // 打印相机相对于每个标记的位置和旋转角度
            for (unsigned int i = 0; i < ids.size(); i++)
            {
                std::cout << "ID = " << ids[i]
                          << ", Rotation = " << rvecs.at<cv::Vec3d>(i)
                          << ", Translation = " << tvecs.at<cv::Vec3d>(i)
                          << std::endl;
                // float xr = rvecs.at<cv::Vec3d>(i)[0];
                float xr = filter.filter(rvecs.at<cv::Vec3d>(i)[0]);
                float xp = tvecs.at<cv::Vec3d>(i)[1], yp = tvecs.at<cv::Vec3d>(i)[0];
                std::cout << ", Rotation(x) = " << xr << std::endl;
                std::cout << "x y:" << xp << " " << yp << std::endl;
                float vX = -pidX.PT(-0.2, xp, 0.05);
                float vY = -pidY.PT(-0.1, yp, 0.05);
                float vYaw = -pidYaw.PT(0, xr, 0.2);
                std::cout << "vYaw = " << vYaw << std::endl;
                std::cout << "vX = " << vX << std::endl;
                std::cout << "vY = " << vY << std::endl;

                // moveController.setVelocity(vX, 0, 0);
                // moveController.setVelocity(0, vY, 0);
                // moveController.setVelocity(0, 0, vYaw);

                // moveController.setVelocity(vX, vY, 0);
                moveController.setVelocity(vX, vY, vYaw);

                if (vX == 0 && vY == 0 && vYaw == 0)
                    break;
            }
        }
        else
        {
            moveController.setVelocity(0, 0, 0);
        }
        std::cout << "check1\n";
        cv::imshow("image.jpg", inputImage);
        cv::waitKey(1);
        std::cout << "check2\n";
    }

    return 0;
}