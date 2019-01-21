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

namespace ranging{
    struct Cross_Property
    {
        Vec4i vertical;
        Vec4i horizontal;
        Point center;
    };

    class Cross_center{
    public:
        Cross_center(int, int);
        ~Cross_center();
        void test();
        Mat Crop_img(Mat& img);
        vector<int> Mat_sum_dim(Mat& );
        vector<int > Get_center(Mat& );
        vector<uint8_t > Temp_iter();
        vector<uint16_t > Out_dim(int, int);
        Cross_Property Crossline_Reg(Mat& );
//        CrossLines Cal_cross_lines(vector<Vec4i> );
        Point Cal_intersection(Vec4i,Vec4i);
    private:
        //Mat img;
        uint16_t W;
        uint16_t H;
        vector<uint16_t> WCrop;
        vector<uint16_t> HCrop;
        uint8_t NTemp;
        uint8_t WCross;
        uint8_t stride;
        //vector<vector<uint16_t >> cross_simi;
        vector<uint16_t > center;
    };
}

#endif //LASER_SCAN_CLASSFICATION_CROSS_CENTER_H
