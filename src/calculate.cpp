//
// Created by minzhao on 18-12-31.
//

#include <cmath>
#include "calculate.h"
#include "opencv2/opencv.hpp"

using namespace cv;
namespace ranging {
    point<double> Calculate::calculateUseCrossLaser(Point &laser_marker, eclipse<float > eclipse) {
        ratio_ = (eclipse.height / realEclipseHeight_ + eclipse.width / realEclipseWidth_) / 2.0;
        if (ratio_ == 0)
            std::cerr << "ratio is zero!!!" << std::endl;
        point<double> output; //in mm

        output.x = (RESOLUTION_CENTER_WIDTH - eclipse.pixel_x - eclipse.width / 2) / ratio_;
        output.y = (RESOLUTION_CENTER_HEIGHT - eclipse.pixel_y - eclipse.height / 2) / ratio_;
        output.z = (laser_marker.x - RESOLUTION_CENTER_WIDTH) * tan(M_PI / 3.0) / ratio_ + realStandard_;
        cout << "x: " << output.x << " y: " << output.y << " z: " << output.z << endl;
        return output;
    }

    point<double> Calculate::CalculateUseRatio() {
        point<double> output;
        cv::Rect final_result;
        cv::Point center, center_offset;
        center.x = final_result.x + final_result.width / 2;
        center.y = final_result.y + final_result.height / 2;
        center_offset.x = center.x - RESOLUTION_WIDTH / 2;
        center_offset.y = center.y - RESOLUTION_HEIGHT / 2;
        float real_x = (float)center_offset.x / final_result.width *ECLIPSEWIDTH / 10.0;
        float real_y = (float)center_offset.y / final_result.height*ECLIPSEHEIGHT / 10.0;

        float eclipsedist = 0;
        float ratio = (float)RESOLUTION_HEIGHT / (float)ORIGINHEIGHT;
        float origin_result_height = final_result.height / ratio;
        if (origin_result_height >= 298.1614764)
            eclipsedist = 638.6916038;
        else if (origin_result_height<298.1614764 && origin_result_height >= 232.0460545)
            eclipsedist = 651.1582857;
        else if (origin_result_height<232.0460545 && origin_result_height >= 192.0289138)
            eclipsedist = 654.8709981;
        else if (origin_result_height<192.0289138 && origin_result_height >= 164.4588792)
            eclipsedist = 660.4201803;
        else if (origin_result_height<164.4588792 && origin_result_height >= 143.9595777)
            eclipsedist = 663.7013099;
        else if (origin_result_height<143.9595777 && origin_result_height >= 127.452425)
            eclipsedist = 675.3753149;
        else if (origin_result_height<127.452425 && origin_result_height >= 112.4986336)
            eclipsedist = 666.614796;
        else if (origin_result_height<112.4986336 && origin_result_height >= 102.0157998)
            eclipsedist = 668.3346267;
        else if (origin_result_height<102.0157998 && origin_result_height >= 93.97657921)
            eclipsedist = 669.7011807;
        else if (origin_result_height<93.97657921&& origin_result_height >= 86.47428686)
            eclipsedist = 671.985775;
        else if (origin_result_height<86.47428686 && origin_result_height >= 79.99550006)
            eclipsedist = 670.31;
        else if (origin_result_height<79.99550006 && origin_result_height >= 74.4611457)
            eclipsedist = 669.8318902;
        else if (origin_result_height<74.4611457 && origin_result_height >= 69.97453522)
            eclipsedist = 670.7568733;
        else if (origin_result_height<69.97453522 && origin_result_height >= 65.5014975)
            eclipsedist = 675.8008822;
        else
            eclipsedist = 666.0056159;

        float real_z = float(eclipsedist*ratio)*ECLIPSEHEIGHT / 10.0 / final_result.height;

        output.x = real_x;
        output.y = real_y;
        output.z = real_z;
        return output;

    }

    eclipse<float> Calculate::detectEclipse(cv::Mat srcImg) {
        eclipse<float> eclipse;
        vector<vector<Point>> contours;    //��������
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
                areaMaxIndex = i;//��¼���������������
            }
        }
        vector<cv::Point> contourMax = contours[areaMaxIndex];
        vector<vector<cv::Point>> contours_poly(1);
        approxPolyDP(cv::Mat(contourMax), contours_poly[0], 3, true);
        cv::RotatedRect boundRect = cv::minAreaRect(cv::Mat(contours_poly[0]));  //���б����
        cv::Mat drawing = cv::Mat::zeros(srcImg.size(), CV_8UC3);
        drawContours(drawing, contours_poly, 0, cv::Scalar(0, 0, 255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
        //        rectangle(drawing, boundRect.)
        //        rectangle(drawing, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 1, 8, 0);

        Point2f vertices[4];
        boundRect.points(vertices);//��ȡ���ε��ĸ���


        eclipse.x = boundRect.center.x; //��Բ���ϵ�����x
        eclipse.y = boundRect.center.y; //��Բ���ϵ�����y
        eclipse.rotate = boundRect.angle;
        eclipse.height = boundRect.size.height;
        eclipse.width = boundRect.size.width;


        eclipse.points[0] = vertices[0];
        eclipse.points[1] = vertices[1];
        eclipse.points[2] = vertices[2];
        eclipse.points[3] = vertices[3];

        if (calDist(vertices[0].x, vertices[1].x, vertices[0].y, vertices[1].y) < calDist(vertices[2].x, vertices[1].x, vertices[2].y, vertices[1].y)) {
            eclipse.rotate_slope = atan((vertices[0].y - vertices[1].y) / (vertices[0].x - vertices[1].x)) * 180 / M_PI;
        }
        else {
            eclipse.rotate_slope = atan((vertices[2].y - vertices[1].y) / (vertices[2].x - vertices[1].x)) * 180 / M_PI;
        }


        //        cout << boundRect.x << " " << boundRect.y << " " << boundRect.width << " " << boundRect.height << endl;
        //        imshow("pointImg", srcImg);
        //        imshow("contour", drawing);
        //        waitKey();
        return eclipse;
    }

    eclipse<float> Calculate::detectEclipseUseDlib(cv::Mat& frame) {
        typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<8> > image_scanner_type;
        eclipse<float> output;
        image_scanner_type scanner;
        //scanner.set_detection_window_size(32, 51);
        scanner.set_detection_window_size(16, 25);
        dlib::object_detector<image_scanner_type> detector;
        dlib::deserialize("detector_home.svm") >> detector;
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame); // only stores pointer, no deep copy
        std::vector<dlib::rectangle> dets = detector(dlib_img);
        std::vector<cv::Rect> dets_cv;
        int lowThreshold = 30;
        int ratio = 3;
        int kernel_size = 3;
        if (dets.size() == 0) {
            output.height = -1;
            return output;
        }
        for (unsigned long i = 0; i < dets.size(); i++) {
            dlib::rectangle temp_dlib = dets[i];
            cv::Rect temp_cv(temp_dlib.left(), temp_dlib.top(), temp_dlib.width(), temp_dlib.height());
            dets_cv.push_back(temp_cv);
            //get bouding box using opencv
            cv::Mat RectImg, dst;
            float marginX = 0.1* temp_dlib.width();
            float marginY = 0.1* temp_dlib.height();
            cv::Rect larger_roi(temp_dlib.left() - marginX, temp_dlib.top() - marginY, temp_dlib.width() + 2 * marginX, temp_dlib.height() + 2 * marginY);
            larger_roi.x = min(max(larger_roi.x, 0), frame.cols - larger_roi.width);
            larger_roi.y = min(max(larger_roi.y, 0), frame.rows - larger_roi.height);
            RectImg = frame(larger_roi);
            cv::cvtColor(RectImg, RectImg, CV_RGB2GRAY);
            cv::blur(RectImg, RectImg, cv::Size(3, 3));
            Canny(RectImg, dst, lowThreshold, lowThreshold*ratio, kernel_size);
            //cv::rectangle(frame, dets_cv[i], cv::Scalar(0, 0, 255));
            //imshow(window_name, dst);
            std::vector< std::vector< cv::Point> > contours;
            cv::findContours(dst, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (contours.size() == 0)
                break;
            //find the max contours
            double areaMax = 0;
            int areaMaxIndex = 0;
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if (area > areaMax)
                {
                    areaMax = area;
                    areaMaxIndex = i;//��¼���������������
                }
            }
            vector<cv::Point>contourMax = contours[areaMaxIndex];

            //vector<vector<cv::Point> > contours_thresh;
            //double minArea = 2000;//400 is a good value
            //double maxArea = 10000;//400 is a good value
            //for (size_t i = 0; i < contours.size(); i++)
            //{
            //	double area = cv::contourArea(contours[i]);
            //	cout << area << endl;
            //	if (area > minArea &&area < maxArea)
            //	{
            //		cout << area << endl;
            //		contours_thresh.push_back(contours[i]);
            //	}
            //}

            vector<vector<cv::Point>>contours_poly(1);
            cv::Rect boundRect;
            approxPolyDP(cv::Mat(contourMax), contours_poly[0], 3, true);
            boundRect = cv::boundingRect(cv::Mat(contours_poly[0]));

            cv::RotatedRect boundRectRotate = cv::minAreaRect(cv::Mat(contours_poly[0]));  //���б����
            //			cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);
            //			drawContours(drawing, contours_poly, i, cv::Scalar(0, 0, 255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
            //			rectangle(drawing, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 2, 8, 0);

            //			cv::Mat result(dst.size(), CV_8U, cv::Scalar(255));
            //			drawContours(result, contours, -1, cv::Scalar(0), 2);
            //the final result
            cv::Rect final_result(temp_dlib.left() + boundRect.x - marginX, temp_dlib.top() + boundRect.y - marginY, boundRect.width, boundRect.height);
            cv::rectangle(frame, final_result, cv::Scalar(0, 255, 255));  //����Բ�ϻ���������

            cv::Point center, center_offset;
            center.x = final_result.x + final_result.width / 2;
            center.y = final_result.y + final_result.height / 2;


            output.x = 0;
            output.y = 0;
            output.z = 0;
            output.height = final_result.height;
            output.width = final_result.width;
            output.pixel_x = center.x;
            output.pixel_y = center.y;
            output.rotate = boundRectRotate.angle;

            cv::Rect rect_center(center.x, center.y, 2, 2);
            rectangle(frame, rect_center, cv::Scalar(0, 0, 255), 3);

            return output;
        }
    }

    eclipse<float> Calculate::detectEclipseAndCalculateUseDlib(cv::Mat& frame) {
        eclipse<float> output;
        typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<8> > image_scanner_type;
        image_scanner_type scanner;
        //scanner.set_detection_window_size(32, 51);
        scanner.set_detection_window_size(16, 25);
        dlib::object_detector<image_scanner_type> detector;
        dlib::deserialize("detector_home.svm") >> detector;
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame); // only stores pointer, no deep copy
        std::vector<dlib::rectangle> dets = detector(dlib_img);
        std::vector<cv::Rect> dets_cv;
        int lowThreshold = 30;
        int ratio = 3;
        int kernel_size = 3;
        if (dets.size() == 0) {
            output.height = -1;
            return output;
        }
        for (unsigned long i = 0; i < dets.size(); i++) {
            dlib::rectangle temp_dlib = dets[i];
            cv::Rect temp_cv(temp_dlib.left(), temp_dlib.top(), temp_dlib.width(), temp_dlib.height());
            dets_cv.push_back(temp_cv);
            //get bouding box using opencv
            cv::Mat RectImg, dst;
            float marginX = 0.1* temp_dlib.width();
            float marginY = 0.1* temp_dlib.height();
            cv::Rect larger_roi(temp_dlib.left() - marginX, temp_dlib.top() - marginY, temp_dlib.width() + 2 * marginX, temp_dlib.height() + 2 * marginY);
            larger_roi.x = min(max(larger_roi.x, 0), frame.cols - larger_roi.width);
            larger_roi.y = min(max(larger_roi.y, 0), frame.rows - larger_roi.height);
            RectImg = frame(larger_roi);
            cv::cvtColor(RectImg, RectImg, CV_RGB2GRAY);
            cv::blur(RectImg, RectImg, cv::Size(3, 3));
            Canny(RectImg, dst, lowThreshold, lowThreshold*ratio, kernel_size);
            //cv::rectangle(frame, dets_cv[i], cv::Scalar(0, 0, 255));
            //imshow(window_name, dst);
            std::vector< std::vector< cv::Point> > contours;
            cv::findContours(dst, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (contours.size() == 0)
                break;
            //find the max contours
            double areaMax = 0;
            int areaMaxIndex = 0;
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if (area > areaMax)
                {
                    areaMax = area;
                    areaMaxIndex = i;//��¼���������������
                }
            }
            vector<cv::Point>contourMax = contours[areaMaxIndex];

            //vector<vector<cv::Point> > contours_thresh;
            //double minArea = 2000;//400 is a good value
            //double maxArea = 10000;//400 is a good value
            //for (size_t i = 0; i < contours.size(); i++)
            //{
            //	double area = cv::contourArea(contours[i]);
            //	cout << area << endl;
            //	if (area > minArea &&area < maxArea)
            //	{
            //		cout << area << endl;
            //		contours_thresh.push_back(contours[i]);
            //	}
            //}

            vector<vector<cv::Point>>contours_poly(1);
            cv::Rect boundRect;
            approxPolyDP(cv::Mat(contourMax), contours_poly[0], 3, true);
            boundRect = cv::boundingRect(cv::Mat(contours_poly[0]));
            cv::RotatedRect boundRectRotate = cv::minAreaRect(cv::Mat(contours_poly[0]));  //���б����
            //			cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);
            //			drawContours(drawing, contours_poly, i, cv::Scalar(0, 0, 255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
            //			rectangle(drawing, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 2, 8, 0);

            //			cv::Mat result(dst.size(), CV_8U, cv::Scalar(255));
            //			drawContours(result, contours, -1, cv::Scalar(0), 2);
            //the final result
            cv::Rect final_result(temp_dlib.left() + boundRect.x - marginX, temp_dlib.top() + boundRect.y - marginY, boundRect.width, boundRect.height);
            cv::rectangle(frame, final_result, cv::Scalar(0, 255, 255));  //����Բ�ϻ���������

            cv::Point center, center_offset;
            center.x = final_result.x + final_result.width / 2;
            center.y = final_result.y + final_result.height / 2;


            center_offset.x = RESOLUTION_WIDTH / 2 - center.x;
            center_offset.y = RESOLUTION_HEIGHT / 2 - center.y;
            float real_x = (float)center_offset.x / final_result.width * ECLIPSEWIDTH;
            float real_y = (float)center_offset.y / final_result.height* ECLIPSEHEIGHT;
            //��EclipseDist��һ����һ��ֵ��Ϊ������
            //float real_z = float(EclipseDist)*EclipseHeight / final_result.height;

            float eclipsedist = 0;
            float ratio = (float)RESOLUTION_HEIGHT / (float)ORIGINHEIGHT;
            float origin_result_height = final_result.height / ratio;
            if (origin_result_height >= 298.1614764)
                eclipsedist = 638.6916038;
            else if (origin_result_height<298.1614764 && origin_result_height >= 232.0460545)
                eclipsedist = 651.1582857;
            else if (origin_result_height<232.0460545 && origin_result_height >= 192.0289138)
                eclipsedist = 654.8709981;
            else if (origin_result_height<192.0289138 && origin_result_height >= 164.4588792)
                eclipsedist = 660.4201803;
            else if (origin_result_height<164.4588792 && origin_result_height >= 143.9595777)
                eclipsedist = 663.7013099;
            else if (origin_result_height<143.9595777 && origin_result_height >= 127.452425)
                eclipsedist = 675.3753149;
            else if (origin_result_height<127.452425 && origin_result_height >= 112.4986336)
                eclipsedist = 666.614796;
            else if (origin_result_height<112.4986336 && origin_result_height >= 102.0157998)
                eclipsedist = 668.3346267;
            else if (origin_result_height<102.0157998 && origin_result_height >= 93.97657921)
                eclipsedist = 669.7011807;
            else if (origin_result_height<93.97657921&& origin_result_height >= 86.47428686)
                eclipsedist = 671.985775;
            else if (origin_result_height<86.47428686 && origin_result_height >= 79.99550006)
                eclipsedist = 670.31;
            else if (origin_result_height<79.99550006 && origin_result_height >= 74.4611457)
                eclipsedist = 669.8318902;
            else if (origin_result_height<74.4611457 && origin_result_height >= 69.97453522)
                eclipsedist = 670.7568733;
            else if (origin_result_height<69.97453522 && origin_result_height >= 65.5014975)
                eclipsedist = 675.8008822;
            else
                eclipsedist = 666.0056159;

            float real_z = float(eclipsedist*ratio) * ECLIPSEHEIGHT / final_result.height;

            output.x = real_x;
            output.y = real_y;
            output.z = real_z;
            output.height = final_result.height;
            output.width = final_result.width;
            output.pixel_x = center.x;
            output.pixel_y = center.y;
            output.rotate = boundRectRotate.angle;
            cout << output.x << " " << output.y << " " << output.z << endl;
            cv::Rect rect_center(center.x, center.y, 2, 2);   //��ʾͼ������
            rectangle(frame, rect_center, cv::Scalar(0, 0, 255), 3);

            return output;
        }
    }


    float Calculate::calDist(float x0, float x1, float y0, float y1) {
        return fabs((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1));
    }
}