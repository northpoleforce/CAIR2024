#include <string>
#include <thread>
#include <unistd.h>
#include <numeric>
#include <opencv2/opencv.hpp>

// #include <apriltag.h>
// #include <tag25h9.h>
// #include <Python.h>

#include "move.h"

#define DEBUG_AUTO_CRUISE
#define VISION

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480)
        : cam_id(cam_id), width(width), height(height)
    {
        std::string ip_last_segment = "15";
        std::string udpstr_prev_data = "udpsrc address=192.168.123." + ip_last_segment + " port=";
        std::vector<int> udp_port = {9201, 9202, 9203, 9204, 9205};
        std::string udpstr_behind_data = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
        udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
        std::cout << udp_send_integrated_pipe_0 << std::endl;
        cap.open(udp_send_integrated_pipe_0);
    }
    ~Camera()
    {
        cv::destroyAllWindows();
        cap.release();
    }
    void reopen()
    {
        cap.release();
        cap.open(udp_send_integrated_pipe_0);
    }
    cv::Mat getFrame()
    {
        cv::Mat frame;
        while (!cap.read(frame))
            ;
        cv::resize(frame, frame, cv::Size(width, height));
        if (cam_id == 1)
        {
            cv::flip(frame, frame, -1);
        }
        return frame;
    }
    std::string udp_send_integrated_pipe_0;

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
};

class ControlParams
{
public:
    ControlParams(int atc, float vX, float ox,
                  float Y_Kp, float Y_Ki, float Y_Kd,
                  float R_Kp, float R_Ki, float R_Kd) : apriltagCOUNT_DOWN(atc),
                                                        vX(vX),
                                                        offsetX(ox),
                                                        Y_Kp(Y_Kp), Y_Ki(Y_Ki), Y_Kd(Y_Kd),
                                                        R_Kp(R_Kp), R_Ki(R_Ki), R_Kd(R_Kd)
    {
    }
    int apriltagCOUNT_DOWN = 10;
    float offsetX = 0.0;
    float vX = 0.0;
    float Y_Kp = 0.0, Y_Ki = 0.0, Y_Kd = 0.0;
    float R_Kp = 0.0, R_Ki = 0.0, R_Kd = 0.0;
    int task1ID = -1, task2ID = -1;
};

struct Tasks
{
    bool task_have[5], task_put[5], task_done[5];
    Tasks()
    {
        memset(task_have, false, sizeof(task_have));
        memset(task_put, false, sizeof(task_put));
        memset(task_done, false, sizeof(task_done));
        task_have[0] = true;
    }
} tasks;
bool taskI_check_notDone(int i)
{
    return tasks.task_have[i] && !tasks.task_done[i];
}
bool taskI_check_notPut(int i)
{
    return tasks.task_have[i] && !tasks.task_put[i];
}

class PIDController
{
    double Kp, Ki, Kd;
    double integral = 0.0;
    double previous_error = 0.0;

public:
    PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
    double control(double setpoint, double measured_value)
    {
        double error = setpoint - measured_value;
        return Kp * error;
    }
};

class ControlRect
{
public:
    ControlRect() {}
    int width = 0, height = 0; // 图像的大小
    int left = 0, top = 0;     // 控制矩形的左上角
    int right = 0, bottom = 0; // 控制矩形的右下角
};

class CheckPoints
{
public:
    CheckPoints()
    {
        memset(time_marker, -1, sizeof(time_marker));
    }
    std::time_t time_marker[5];
};

ControlRect controlRect;
CheckPoints cp;

std::array<std::array<std::array<std::string, 2>, 4>, 4> py_files = {{{{
                                                                          {"", ""},
                                                                          {"../test/1-2(left).py", "../test/1-2(right).py"},
                                                                          {"../test/1-3(left).py", "../test/1-3(right).py"},
                                                                          {"../test/1-4(left).py", "../test/1-4(right).py"},
                                                                      }},
                                                                      {{
                                                                          {"", ""},
                                                                          {"", ""},
                                                                          {"../test/2-3(left).py", "../test/2-3(right).py"},
                                                                          {"../test/2-4(left).py", "../test/2-4(right).py"},
                                                                      }},
                                                                      {{
                                                                          {"", ""},
                                                                          {"", ""},
                                                                          {"", ""},
                                                                          {"../test/3-4(left).py", "../test/3-4(right).py"},
                                                                      }},
                                                                      {{{"", ""},
                                                                        {"", ""},
                                                                        {"", ""},
                                                                        {"", ""}}}}};

cv::Mat getMask(cv::Mat &src,
                int yellowHLower = 16, int yellowHUpper = 29,
                int yellowSLower = 80, int yellowSUpper = 200,
                int yellowVLower = 110, int yellowVUpper = 220)
{
    // 转hsv，中值滤波去噪
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
    cv::medianBlur(src, src, 5);
    // 黄色阈值掩膜
    cv::Mat mask;
    const cv::Scalar yellow_lower = cv::Scalar(yellowHLower, yellowSLower, yellowVLower);
    const cv::Scalar yellow_upper = cv::Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    cv::inRange(src, yellow_lower, yellow_upper, mask);
    // 获取控制关键（下巴）
    const float top_start = 2 / 3.0;
    const float height_ratio = 1 / 20.0;
    controlRect.top = src.rows * top_start;
    controlRect.left = 0;
    controlRect.bottom = src.rows;
    controlRect.bottom = controlRect.top + src.rows * height_ratio;
    controlRect.right = src.cols;
    return mask;
}

#ifdef DEBUG_AUTO_CRUISE

bool cp_markerI_out_s(int i, int s)
{
    if (cp.time_marker[i] == -1)
        return false;
    return std::difftime(std::time(nullptr), cp.time_marker[i]) >= s;
}

float calErr(const cv::Mat &img)
{
    const cv::Rect rect(controlRect.left, controlRect.top, controlRect.right - controlRect.left, controlRect.bottom - controlRect.top);
    const cv::Mat &subImg = img(rect);
    std::vector<cv::Point2f> points;
    // 遍历每一行
    for (int y = 0; y < subImg.rows; y++)
    {
        int count = 0;
        int sum = 0;
        // 遍历每一列
        for (int x = 0; x < subImg.cols; x++)
        {
            // 如果这个像素是白色的
            if (subImg.at<uchar>(y, x) > 0)
            {
                sum += x;
                count++;
            }
        }
        // 计算这一行的像素中点
        if (count > 0)
        {
            points.push_back(cv::Point2f(static_cast<float>(sum) / count, y));
        }
    }
    // 计算x坐标的中心
    float sumX = std::accumulate(points.begin(), points.end(), 0.0f, [](float sum, const cv::Point2f &point)
                                 { return sum + point.x; });
    float centerX = sumX / points.size();
    // 计算图像的中心
    const int imgCenterX = img.cols / 2;
    std::cout << "deltaX: " << centerX - imgCenterX << std::endl;
    return centerX - imgCenterX;
}


// 不错的参数
ControlParams params(
    10,            // apriltagCOUNT_DOWN，计数器，每x帧检测一次
    0.15,          // vX，前进速度
    30,            // offsetX，偏移量
    0.00002, 0, 0, // Y_Kp, Y_Ki, Y_Kd：Y轴PID参数
    -0.015, 0, 0); // R_Kp, R_Ki, R_Kd：R轴PID参数

PIDController pidY(params.Y_Kp, params.Y_Ki, params.Y_Kd);
PIDController pidR(params.R_Kp, params.R_Ki, params.R_Kd);

void point_walking(Custom &custom, const cv::Mat &mask, float y_offset)
{
    auto err = calErr(mask); // 计算误差
    double offsetX = params.offsetX;
    if (err < 0)
        err += offsetX;
    else
        err -= offsetX;
    float Youtput = y_offset;
    float Routput = pidR.control(0, err);
    std::cout << "Error: " << err << std::endl;
    std::cout << "Youtput: " << Youtput << std::endl;
    std::cout << "Routput: " << Routput << std::endl;
    custom.setVelocity(params.vX, Youtput, Routput);
}
void out_area(Camera &cam, Custom &custom, int taskID)
{ // 离开任务区
    std::cout << "\033[1;31mOut of the area" << "\033[0m" << std::endl;
    // 先停下来稳定
    custom.setVelocity(0, 0, 0);
    sleep(1);
    // 前进
    if (taskID == 1)
    {
        custom.setVelocity(0.2, 0, 0);
        sleep(2);
    }
    else
    {
        custom.setVelocity(0, 0, 0.25);
        sleep(2);
        custom.setVelocity(0.2, 0, 0);
        sleep(3);
    }
    custom.setVelocity(0, 0, 0);
    sleep(1);
    int marker_id = -1;
    // cv::Mat img, mask;
    // cam.reopen();
    // while (1)
    // {
    //     img = cam.getFrame();
    //     marker_id = apriltagDetector(img);
    //     if (marker_id == -1)
    //     {
    //         break;
    //     }
    //     img = cam.getFrame();
    //     mask = getMask(img, 10, 40, 50, 255, 0, 255);
    //     point_walking(custom, mask, taskID == 2 ? -0.2 : -0.35);
    // }
}
bool now_time_out(std::time_t t, int s)
{
    return std::difftime(std::time(nullptr), t) >= s;
}

int main()
{
    // tasks.task_have[1] = true;
    // tasks.task_have[2] = true;
    // tasks.task_have[3] = true;
    // tasks.task_have[4] = true;

    std::cout << "\033[1;32mProgram is READY\033[0m" << std::endl;
    std::cout << "\033[1;34mReading tasks...\033[0m" << std::endl;
    // 识别标签，读取任务
    std::cout << "\033[1;32mTasks are ready\033[0m" << std::endl;
    std::cout << "\033[1;31mPlease input ENTER to start!!\033[0m" << std::endl;
    std::cin.get();
    std::cout << "\033[1;32mStart transporting...\033[0m" << std::endl;

    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);

    // custom.standDown();
    // sleep(5);
    // custom.standUp();

    // 先出启动区
    custom.setVelocity(0.0, 0.45, 0.0);
    sleep(2);
    custom.setVelocity(0.0, 0.0, 0.0);
    sleep(1);

    // // debug
    // params.task1ID = 1;
    // params.task2ID = 2;
    // tasks.task_have[params.task1ID] = tasks.task_have[params.task2ID] = true;

    // 视觉处理控制运动
    cv::Mat img, mask;
    Camera cam(2);
    std::time_t time_start = std::time(nullptr);
    int tasksFinished = 0;
    while (1)
    {
        img = cam.getFrame();

        mask = getMask(img, 10, 40, 50, 255, 0, 255);
        point_walking(custom, mask, 0);
        for (int i = 1; i <= 4; ++i)
            if (taskI_check_notPut(i) && cp_markerI_out_s(i, 20))
            {
                // custom.setVelocity(0, 0, -0.4);
                // sleep(1);
                // custom.setVelocity(0, 0.2, 0);
                // sleep(1);
                std::cout << "\033[1;31mTask " << i << " is put\033[0m" << std::endl;
                custom.standDown();
                sleep(1);
                custom.standUp();
                sleep(1);
                cam.reopen();
            }
    }
    return 0;
}
#endif