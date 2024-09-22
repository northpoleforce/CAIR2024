#ifndef EXTRA_CAMERA_H
#define EXTRA_CAMERA_H

#include <opencv2/opencv.hpp>

#include "visionCPU.h"

class ExtraCamera
{
private:
    cv::VideoCapture *cap;
public:
    ExtraCamera(bool &ok);
    cv::Mat getFrame();
};

#endif // EXTRA_CAMERA_H