//
// Created by minzhao on 18-12-9.
//

#ifndef LASER_SCAN_CLASSFICATION_CAMERA_H
#define LASER_SCAN_CLASSFICATION_CAMERA_H

#include "opencv2/opencv.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace cv;
using namespace std;
namespace ranging {
    class Camera {
    public:
        bool captureImg(string& path);

        void testImg();

        int openCam(VideoCapture&, int index = 1);

        bool closeCam();

    private:
        Mat frame;
        VideoCapture cap;
        ofstream outfile;
    };
}
#endif //LASER_SCAN_CLASSFICATION_CAMERA_H