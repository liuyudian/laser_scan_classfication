#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "camera.h"
#include "classify.h"
#include "cross_center.h"

using namespace cv;
using namespace std;

//#define  RESOLUTION_WIDTH 1280
//#define  RESOLUTION_HEIGHT 720
//#define  RESOLUTION_WIDTH 640
//#define  RESOLUTION_HEIGHT 480
#define  RESOLUTION_WIDTH 1920
#define  RESOLUTION_HEIGHT 1080

int main(int argc, char **argv)
{
//    ifstream f in("../imglist.txt");
//    ifstream fin("../imglist_logic.txt");
    ifstream fin("/home/minzhao/Document/laser_scan_classfication/imglist_rang106.txt");
    string filename;

//    string imgname = "../dataset_ranging_0106/4.jpg";
//    Mat img = imread(imgname);
//    ranging::Cross_center test_cross(RESOLUTION_WIDTH,RESOLUTION_HEIGHT);
//    test_cross.Crossline_Reg(img);

    while(getline(fin,filename))
    {
        cout << filename << endl;
//        string imgname = "../dataset_logic/" + filename; //720p data
//        string imgname = "../dataset/" + filename; //480p data
        string imgname = "../dataset_ranging_0106/" + filename; //1080p data
        Mat img = imread(imgname);

        ranging::Cross_center test_cross(RESOLUTION_WIDTH,RESOLUTION_HEIGHT);
        ranging::Cross_Property crossp;
        crossp=test_cross.Crossline_Reg(img);

        line( img, Point(crossp.vertical[0], crossp.vertical[1]), Point(crossp.vertical[2], crossp.vertical[3]), Scalar(0,255,0), 1, CV_AA);
        line( img, Point(crossp.horizontal[0], crossp.horizontal[1]), Point(crossp.horizontal[2], crossp.horizontal[3]), Scalar(255,0,0), 1, CV_AA);
        vector<int> center_pos = test_cross.Get_center(img);
        Rect rect_center(crossp.center.x-2,crossp.center.y-2,5,5);
        rectangle(img, rect_center, Scalar(255,255,0),2);
        imshow("img_with_center",img);
        waitKey(0);
    };

//    ranging::Camera camera;
//    ranging::Classify classify;
//    VideoCapture capture;
//    Mat frame;
//    unsigned long total_count = 0;
//    int ret = camera.openCam(capture);
//    if (ret != 0) {
//        std::cout << "open cam failed" << std::endl;
//        return -1;
//    }
//    ret = classify.init_svm();
//    if (ret != 0) {
//        std::cout << "init svm failed" << std::endl;
//        return -1;
//    }
//    std::cout << "开始识别..." << std::endl;
//    for (;;) {
//        total_count++;
//        capture >> frame;
//        if (frame.empty()) break; // end of video stream
//        imshow("this is you, smile! :)", frame);
//        classify.start_classfication(frame);
//        int c = waitKey(1);
//        switch (c) {
//            case 'q' : {
//                cout << "stop capture" << endl;
//                // release the camera
//                camera.closeCam();
//                break;
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
//
//    }

//    /*to capture img and produce dataset*/
//    camera.captureImg(10);
    return 0;
}
