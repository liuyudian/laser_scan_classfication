#include <iostream>
#include <calculate.h>
#include "opencv2/opencv.hpp"
#include "camera.h"
#include "classify.h"
#include "common.h"
#include "cross_center.h"

//#define CAPTURE
int main(int argc, char **argv) {
    ranging::Camera camera;
    ranging::Classify classify;
    ranging::Calculate calculate(ECLIPSEWIDTH, ECLIPSEHEIGHT, STANDARDLENGTH);
    ranging::Cross_center cross_center(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
    VideoCapture capture;
    Mat frame;

#ifndef CAPTURE
    int ret = camera.openCam(capture);
    if (ret != 0) {
        std::cout << "open cam failed" << std::endl;
        return -1;
    }
//    ret = classify.init_svm();
//    if (ret != 0) {
//        std::cout << "init svm failed" << std::endl;
//        return -1;
//    }
    std::cout << "开始识别..." << std::endl;
    for (;;) {
        capture >> frame;
        if (frame.empty()) break; // end of video stream
        ranging::Cross_Property cross_property = cross_center.Crossline_Reg(frame);
        cout << "angle laser: "
             << atan((cross_property.horizontal[1] - cross_property.horizontal[3]) / (cross_property.horizontal[0] - cross_property.horizontal[2])) << endl;
        cv::Rect cross_center1(cross_property.center.x, cross_property.center.y, 2, 2);
        rectangle(frame, cross_center1, cv::Scalar(255, 0, 0), 4);
        ranging::eclipse<float> eclipse = calculate.detect_eclipse(frame);


        cout << "angle: " << eclipse.rotate << endl;         //这是用矩形旋转算出来的角度
        cout << "angle2: " << eclipse.rotate_slope << endl;  //这是用斜率算出来的角度


        cv::Rect rect_point(static_cast<int>(eclipse.x), static_cast<int>(eclipse.y), 2, 2);
        rectangle(frame, rect_point, cv::Scalar(0, 255, 0), 3);
        cv::Rect rect_center(RESOLUTION_CENTER_WIDTH, RESOLUTION_CENTER_HEIGHT, 2, 2);
        rectangle(frame, rect_center, cv::Scalar(0, 0, 255), 3);
        calculate.calculate(cross_property.center, eclipse);
        imshow("this is you, smile! :)", frame);
//    waitKey(0);
//    classify.start_classfication(frame);

//        int c = waitKey(1);
//        switch (c) {
//            case 'q' : {
//                cout << "stop capture" << endl;
//                // release the camera
//                camera.closeCam();
//                return 0;
////                break;
//            }
//            /*print current succeed ratio*/
//            case 'p': {
//                classify.print_ratio(total_count);
//                break;
//            }
//            default: {
//                break;
//            }
//        }

    }
#else
    /*to capture img and produce dataset*/
    camera.captureImg(10);
#endif
    return 0;
}