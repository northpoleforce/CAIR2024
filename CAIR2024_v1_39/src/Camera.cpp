#include "Camera.h"

Camera::Camera(int cam_id, int width, int height)
    : cam_id(cam_id), width(width), height(height)
{
    std::string ip_last_segment = "15";
    std::string udpstr_prev_data = "udpsrc address=192.168.123." + ip_last_segment + " port=";
    std::vector<int> udp_port = {9201, 9202, 9203, 9204, 9205};
    std::string udpstr_behind_data = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
    std::cout << udp_send_integrated_pipe_0 << std::endl;
    cap.open(udp_send_integrated_pipe_0);

    // stop_thread = false;
    // std::thread threadA(&Camera::flashFrame, this);
    // threadA.detach(); // 让线程在后台运行

    // waiting_for_good_frame(3);
}

Camera::~Camera()
{
    stop_thread = true;
    cv::destroyAllWindows();
    cap.release();
}

void Camera::reopen()
{
    cap.release();
    cap.open(udp_send_integrated_pipe_0);
    // waiting_for_good_frame(3);
}

void Camera::waiting_for_good_frame(int t)
{
    std::stringstream tip;
    tip << "waiting for good frame " << t << "s!\n";
    std::cout << tip.str();
    cv::Mat frame;
    auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (duration.count() >= t)
            break;
        // frame = getSharedFrame();
        frame = getFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 休眠一段时间以防止过度使用CPU
    }
    std::cout << "good frame comes!\n";
}

// cv::Mat Camera::getSharedFrame()
// {
//     std::lock_guard<std::mutex> lock(mtx);
//     return sharedFrame.clone();
// }
// void Camera::flashFrame()
// {
//     while (!stop_thread)
//     {
//         std::lock_guard<std::mutex> lock(mtx);
//         sharedFrame = getFrame();
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
// }
cv::Mat Camera::getFrame()
{
    cv::Mat frame;
    while (!cap.read(frame))
    {
        if (frame.empty())
        {
            std::cout << "Camera no img!!!" << std::endl;
            continue;
        }
    }
    cv::resize(frame, frame, cv::Size(width, height));
    if (cam_id == 1)
    {
        cv::flip(frame, frame, -1);
    }
    return frame;
}