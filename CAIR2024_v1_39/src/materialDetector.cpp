#include "Info.h"
#include "materialDetector.h"

// MaterialDetector::MaterialDetector()
// {
//     camera = new Camera(1);
// }
int MaterialDetector::getTaskID()
{
    Camera camera(1);
    cv::Mat frame;
    while (true)
    {
        // frame = camera->getSharedFrame();
        frame = camera.getFrame();
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        if (!ids.empty())
            return ids[0];
    }
}
void MaterialDetector::detect(Task task[])
{
    blueINFO("Start detecting tasks...");

    redshow("Please press ENTER to detect the 1st task...");
    std::cin.get();
    int id1 = getTaskID();
    std::stringstream ss;
    ss << "The 1st task is: " << id1;
    greenshow(ss.str());
    redshow("Please press ENTER to detect the 2nd task...");
    std::cin.get();
    int id2 = getTaskID();
    ss.str(""), ss.clear();
    ss << "The 2nd task is: " << id2;
    greenshow(ss.str());

    if (id1 > id2)
        std::swap(id1, id2);
    ss.str(""), ss.clear();
    ss << "The tasks is: " << id1 << " " << id2;
    greenINFO(ss.str());

    task[id1].have = task[id2].have = true;
    blueINFO("Please press ENTER to start tasks!");
    std::cin.get();
}