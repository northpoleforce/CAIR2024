#include <iostream>  // 导入输入输出流库，用于打印信息
#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#define USE_CAMERA

#ifdef USE_CAMERA
int main(int argc, char *argv[]) {
    int deviceNode = 0; ///< default 0 -> /dev/video0
    cv::Size frameSize(1856, 800); ///< default frame size 1856x800
    int fps = 30; ///< default camera fps: 30
    UnitreeCamera cam("/home/unitree/Documents/test/rc_dog_full/config/stereo_camera_config.yaml"); ///< init camera by device node number
    if (!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);

    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size 相当于/2 /2 = /4
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing

    usleep(500000);
    // 设置AprilTag检测器
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // 添加一个用于记录已检测到的标签ID的变量

// Variables to keep track of detected IDs
std::vector<int> detected_ids;
    while (cam.isOpened()) {
        cv::Mat left, right, film;
        if (!cam.getRectStereoFrame(left, right, film)) { ///< get rectify left,right frame
            usleep(1000);
            continue;
        }
        cv::Mat gray;
        cv::cvtColor(film, gray, cv::COLOR_BGR2GRAY);

        // 将OpenCV图像转换为AprilTag图像格式
        image_u8_t im = {.width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data};

        // 检测标记
        zarray_t *detections = apriltag_detector_detect(td, &im);
        

        // 对于每一个检测到的标签
        // 检查此ID是否已经被检测过
        // 如果这个ID是新检测到的，就将其添加到detected_ids中
        // 如果检测到两个不同的ID，就跳出循环

        // 遍历检测到的标记并打印ID
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            std::cout << "Detected AprilTag with ID: " << det->id << std::endl;
            cv::rectangle(film, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);

            // Check if this ID is already detected
            if (std::find(detected_ids.begin(), detected_ids.end(), det->id) == detected_ids.end()) {
                detected_ids.push_back(det->id);
            }

            // If two different IDs are detected, break the loop
            if (detected_ids.size() == 2) {
                break;
            }
        }
        if (detected_ids.size() == 2) {
            break;
        }
        }

        // 清理并释放资源
        apriltag_detections_destroy(detections);

        // 显示图像
        cv::imshow("AprilTag Detections from Camera", film);
        char key = (char)cv::waitKey(10);
        if (key == 27) break; // 按下ESC键退出

    }
    // 清理并释放资源
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    cam.stopCapture();  ///< stop camera capturing
    
    // 使用scp命令上传drop.txt到指定的机器上


    
    // 根据检测到的ID生成drop.txt文件内容
    // 如果检测到某个ID，相应的dropNode值为1，否则为0

    // Generate and write to drop.txt
    std::ofstream drop_file("drop.txt");
    drop_file << "dropNode1 = " << (std::find(detected_ids.begin(), detected_ids.end(), 1) != detected_ids.end() ? "1" : "0") << std::endl;
    drop_file << "dropNode2 = " << (std::find(detected_ids.begin(), detected_ids.end(), 2) != detected_ids.end() ? "1" : "0") << std::endl;
    drop_file << "dropNode3 = " << (std::find(detected_ids.begin(), detected_ids.end(), 3) != detected_ids.end() ? "1" : "0") << std::endl;
    drop_file << "dropNode4 = " << (std::find(detected_ids.begin(), detected_ids.end(), 4) != detected_ids.end() ? "1" : "0") << std::endl;
    drop_file.close();

    // 使用scp命令上传drop.txt到指定的机器上

    // Use scp command to upload drop.txt
    system("scp drop.txt unitree@192.168.15:/home/unitree");
    return 0;  // 程序正常结束
}
#endif

