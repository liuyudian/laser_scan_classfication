//
// Created by liukai on 18-12-31.
//

#ifndef LASER_SCAN_CLASSFICATION_CROSS_CENTER_H
#define LASER_SCAN_CLASSFICATION_CROSS_CENTER_H

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

namespace ranging {
    struct Cross_Property
    {
        Vec4i vertical;
        Vec4i horizontal;
        Point center;
    };

    class Cross_center {
    public:
        Cross_center(int, int);
        ~Cross_center();
        Mat Crop_img(Mat& img);
        vector<int> Mat_sum_dim(Mat&);
        vector<int > Get_center(Mat&);
        vector<int > Temp_iter();
        vector<int > Out_dim(int, int);
        Cross_Property Crossline_Reg(Mat&);
        Point Cal_intersection(Vec4i, Vec4i);
    private:
        int W;
        int H;
        vector<int> Crop_params;
        vector<int> WCrop;
        vector<int> HCrop;
        int NTemp;
        int WCross;
        int stride;
        float Thresh;
    };
}

#endif //LASER_SCAN_CLASSFICATION_CROSS_CENTER_H