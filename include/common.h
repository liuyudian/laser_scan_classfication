//
// Created by minzhao on 18-12-31.
//

#ifndef LASER_SCAN_CLASSFICATION_COMMON_H
#define LASER_SCAN_CLASSFICATION_COMMON_H
#include "opencv2/opencv.hpp"
namespace ranging {

#define ECLIPSEWIDTH 51.1
#define ECLIPSEHEIGHT 80.5
#define STANDARDLENGTH 346
#define RESOLUTION_WIDTH 1920
#define RESOLUTION_HEIGHT 1080
#define RESOLUTION_CENTER_WIDTH (RESOLUTION_WIDTH / 2)
#define RESOLUTION_CENTER_HEIGHT (RESOLUTION_HEIGHT / 2)

#define ORIGINWIDTH 640
#define ORIGINHEIGHT 360

#define M_PI 3.14159265358979323846

    template<class T>
    struct point {
        inline point() : x(0), y(0), z(0) {}

        inline point(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

        inline ~point() {}

        T x, y, z;

        std::string DebugString() const {
            return "[ " + std::to_string(x) + " , " + std::to_string(y) + " , " + std::to_string(z) + " ]";
        }
    };

    template<class T>
    struct eclipse : public point<T> {
        inline eclipse() : point<T>(), width(0), height(0) {}

        inline eclipse(T _x, T _y, T _z, T _width, T _height) : point<T>(_x, _y, _z), width(_width), height(_width) {}

        std::string DebugEclipse() const {
            return "[ " + std::to_string(point<T>::x) + " , " + std::to_string(point<T>::y) + " , " + std::to_string(point<T>::z) + " , " + std::to_string(width) + " , " +
                   std::to_string(height) + " ]";
        }

        T width, height; //��Բ���س���
        T rotate;
        T rotate_slope;
        T pixel_x, pixel_y;
        cv::Point2f points[4];
    };

}
#endif //LASER_SCAN_CLASSFICATION_COMMON_H