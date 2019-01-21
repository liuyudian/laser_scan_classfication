//
// Created by minzhao on 18-12-31.
//

#include <cmath>
#include "calculate.h"
#include "opencv2/opencv.hpp"

using namespace cv;
namespace ranging {
    point<double> Calculate::calculate(Point &laser_marker, eclipse<float > eclipse) {

        ratio_ = (eclipse.height / realEclipseHeight_ + eclipse.width / realEclipseWidth_) / 2.0;
        if (ratio_ == 0)
            std::cerr << "ratio is zero!!!" << std::endl;
        point<double> output; //in mm

        output.x = (RESOLUTION_CENTER_WIDTH - eclipse.x - eclipse.width / 2) / ratio_;
        output.y = (RESOLUTION_CENTER_HEIGHT - eclipse.y - eclipse.height / 2) / ratio_;
        output.z = (laser_marker.x - RESOLUTION_CENTER_WIDTH) * tan(M_PI / 3.0) / ratio_ + realStandard_;
        cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << endl;
        return output;

    }

    eclipse<float> Calculate::detect_eclipse(cv::Mat srcImg) {
        eclipse<float> eclipse;
//        Mat srcImg = imread("/home/minzhao/Document/laser_scan_classfication/dataset/15.jpg");
//        cout << srcImg.rows << " " << srcImg.cols << endl;
        vector<vector<Point>> contours;    //储存轮廓
        vector<Vec4i> hierarchy;
        double FACTOR = 2.5;
        double cannyThr = 200;
        cv::Mat currImg, cannyImg;
        cvtColor(srcImg, currImg, COLOR_BGR2GRAY);
//        imshow("GRAY", currImg);
        medianBlur(currImg, currImg, 7);
//        imshow("media", currImg);
        threshold(currImg, currImg, 80, 255, THRESH_BINARY);
//        imshow("binary", currImg);
        Canny(currImg, cannyImg, cannyThr, cannyThr * FACTOR);
//        imshow("canny", cannyImg);
        findContours(cannyImg, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0)
            cout << "find no contours" << endl;
        Mat lineSrc = Mat::zeros(cannyImg.rows, cannyImg.cols, CV_8UC3);
        for (int index = 0; index < contours.size(); index++) {
            drawContours(lineSrc, contours, index, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8/*, hierarchy*/);
        }
//        imshow("line", lineSrc);

        //find the max contours
        double areaMax = 0;
        int areaMaxIndex = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > areaMax) {
                areaMax = area;
                areaMaxIndex = i;//记录最大轮廓的索引号
            }
        }
        vector<cv::Point> contourMax = contours[areaMaxIndex];
        vector<vector<cv::Point>> contours_poly(1);
        approxPolyDP(cv::Mat(contourMax), contours_poly[0], 3, true);
        cv::RotatedRect boundRect = cv::minAreaRect(cv::Mat(contours_poly[0]));  //获得斜矩形
        cv::Mat drawing = cv::Mat::zeros(srcImg.size(), CV_8UC3);
        drawContours(drawing, contours_poly, 0, cv::Scalar(0, 0, 255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
//        rectangle(drawing, boundRect.)
//        rectangle(drawing, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 1, 8, 0);

        Point2f vertices[4];
        boundRect.points(vertices);//获取矩形的四个点


        eclipse.x = boundRect.center.x; //椭圆左上的坐标x
        eclipse.y = boundRect.center.y; //椭圆左上的坐标y
        eclipse.rotate = boundRect.angle;
        eclipse.height = boundRect.size.height;
        eclipse.width = boundRect.size.width;


        eclipse.points[0] = vertices[0];
        eclipse.points[1] = vertices[1];
        eclipse.points[2] = vertices[2];
        eclipse.points[3] = vertices[3];

        if(calDist(vertices[0].x,vertices[1].x,vertices[0].y,vertices[1].y) < calDist(vertices[2].x,vertices[1].x,vertices[2].y,vertices[1].y)) {
            eclipse.rotate_slope = atan((vertices[0].y - vertices[1].y) / (vertices[0].x - vertices[1].x));
        } else {
            eclipse.rotate_slope = atan((vertices[2].y - vertices[1].y) / (vertices[2].x - vertices[1].x));
        }


//        cout << boundRect.x << " " << boundRect.y << " " << boundRect.width << " " << boundRect.height << endl;
//        imshow("pointImg", srcImg);
//        imshow("contour", drawing);
//        waitKey();
        return eclipse;
    }


    float Calculate::calDist(float x0, float x1, float y0, float y1) {
        return fabs((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
    }
}