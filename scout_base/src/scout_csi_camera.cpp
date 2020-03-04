// csi_cam2.cpp
// Using a CSI camera module ES1019 connected to a
// NVIDIA Jetson Xavier Developer Kit using OpenCV
//$ g++ -o simple_opencv -Wall -std=c++11 simple_opencv_v4l2.cpp -I/usr/local/include/opencv4/ -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio

/* 
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "v4l2src device=/dev/video0 ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)UYVY, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
*/

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/types_c.h>

using namespace cv;

using namespace std;

int main(int argc, char **argv)
{
    VideoCapture cap2("v4l2src device=/dev/video2 ! video/x-raw,width=1280,height=720,format=UYVY,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink");

    if (!cap2.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }

    if (cap2.isOpened())
    {
        for (;;)
        {
            Mat frame2;

            cap2 >> frame2;

            imshow("original", frame2);

            waitKey(1);
        }
    }
}
