//
// Created by minzhao on 18-12-31.
//

#ifndef LASER_SCAN_CLASSFICATION_CALCULATE_H
#define LASER_SCAN_CLASSFICATION_CALCULATE_H

#include <iostream>
#include <vector>
#include "common.h"
#include "camera.h"

using namespace std;
namespace ranging {
    class Calculate {
    public:
        Calculate(double EclipseWidth, double EclipseHeight, double standard_length) : realEclipseWidth_(EclipseWidth), realEclipseHeight_(EclipseHeight),
                                                                                       realStandard_(standard_length), ratio_(0) {}

        eclipse<float > detect_eclipse(cv::Mat);

        point<double> calculate(Point &laser_marker, eclipse<float > eclipse);

        float calDist(float ,float ,float , float);

        double ratio_; //真实尺度与像素的比例
        double realEclipseWidth_;
        double realEclipseHeight_;
        double realStandard_;
        int center_x_;
        int center_y_;
    };
}
#endif //LASER_SCAN_CLASSFICATION_CALCULATE_H
