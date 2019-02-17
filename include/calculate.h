#pragma once
//
// Created by minzhao on 18-12-31.
//

#ifndef LASER_SCAN_CLASSFICATION_CALCULATE_H
#define LASER_SCAN_CLASSFICATION_CALCULATE_H

#include <iostream>
#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/opencv.h>
#include <dlib/server.h>
#include <vector>
#include "common.h"
#include "camera.h"

using namespace std;
namespace ranging {
    class Calculate {
    public:
        Calculate(double EclipseWidth, double EclipseHeight, double standard_length) : realEclipseWidth_(EclipseWidth), realEclipseHeight_(EclipseHeight),
                                                                                       realStandard_(standard_length), ratio_(0) {}

        eclipse<float > detectEclipse(cv::Mat);

        eclipse<float> detectEclipseUseDlib(cv::Mat&);

        eclipse<float> detectEclipseAndCalculateUseDlib(cv::Mat&);

        point<double> calculateUseCrossLaser(Point &laser_marker, eclipse<float > eclipse);

        point<double> CalculateUseRatio();

        float calDist(float, float, float, float);

        double ratio_; //��ʵ�߶������صı���
        double realEclipseWidth_;
        double realEclipseHeight_;
        double realStandard_;
        int center_x_;
        int center_y_;
    };
}
#endif //LASER_SCAN_CLASSFICATION_CALCULATE_H