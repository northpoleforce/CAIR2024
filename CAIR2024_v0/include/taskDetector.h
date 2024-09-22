#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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
        // std::cout << "buffer: ";
        for (const auto &r : buffer_)
        {
            // std::cout << r << " ";
            sum += r;
        }
        // std::cout << "\n";
        // std::cout << "--------\n";
        // std::cout << static_cast<double>(buffer_.size());
        // std::cout << sum / static_cast<double>(buffer_.size()) << "\n";
        // std::cout << "============\n";
        return sum / static_cast<double>(buffer_.size());
    }

private:
    std::deque<double> buffer_;
    size_t window_size_;
};

class taskDetector
{
private:
    // ArUco标记的实际大小（单位：米）
    float markerLength = 0.06;
    // 相机的内参
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 6.9280202257690246e+02, 2.4139837284139807e+00, 4.1529485401277941e+02,
                            0., 7.1310835266467575e+02, 4.3888779500356623e+02,
                            0., 0., 1.);
    // 相机的畸变系数
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << 6.5608461313067001e-01, -1.5134503590225326e+00,
                          6.2380235102929881e-03, 3.2964624860536132e-03);
    // 创建ArUco字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // 平滑滤波器
    MovingAverageFilter filter;

public:
    taskDetector() : filter(10) {}
    void detect(const cv::Mat &frame,
                cv::Mat &result, int &flag, float &xp, float &yp, float &xr);
};