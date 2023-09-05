//
// Created by dinhnambkhn on 23/08/2023.
//
#include <opencv2/opencv.hpp>

#define DEG2RAD(v)  (v * CV_PI / 180)
#define Rx(rx)      (cv::Matx33d(1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx)))
#define Ry(ry)      (cv::Matx33d(cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry)))
#define Rz(rz)      (cv::Matx33d(cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1))

//create a mouse handler, which will be used to drag the mouse and draw a line
class MouseDrag {
public:
    MouseDrag() : dragged(false) {}

    bool dragged;
    cv::Point start, end;   //start and end points of the line
};
//mouse event handler
void MouseEventHandler(int event, int x, int y, int flags, void *param) {
    if (param == nullptr) return;
    auto *drag = (MouseDrag *) param;
    if (event == cv::EVENT_LBUTTONDOWN) {
        drag->dragged = true;
        drag->start = cv::Point(x, y);
        drag->end = cv::Point(0, 0);
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (drag->dragged) drag->end = cv::Point(x, y);
    } else if (event == cv::EVENT_LBUTTONUP) {
        if (drag->dragged) {
            drag->dragged = false;
            drag->end = cv::Point(x, y);
        }
    }
}
int main()
{
    //load the image
    constexpr auto input = "/home/dinhnambkhn/3dv_tutorial/bin/data/daejeon_station.png";
    constexpr double f = 810.5, cx = 480, cy = 270, L = 3.31; // intrinsic parameters, L is the length of the camera
    // rotation
    cv::Point3d cam_ori(DEG2RAD(-18.7), DEG2RAD(-8.2), DEG2RAD(2.0)); //rotation of the camera: x, y, z


    return 0;
}
