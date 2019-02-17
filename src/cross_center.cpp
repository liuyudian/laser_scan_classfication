//
// Created by liukai on 18-12-31.
//
#include <iostream>
#include "cross_center.h"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace cv;
using namespace std;

namespace ranging {
    Cross_center::Cross_center(int width, int height) : H(height), W(width) {
        if (height == 720)
        {
            WCrop.push_back(360);
            WCrop.push_back(960);
            HCrop.push_back(180);
            HCrop.push_back(480);
            NTemp = 100;
            WCross = 4;
            stride = 3;
            Crop_params.push_back(150);
            Crop_params.push_back(50);
            Crop_params.push_back(400);
            Crop_params.push_back(200);
        }
        else if (height == 480)
        {
            WCrop.push_back(120);
            WCrop.push_back(540);
            HCrop.push_back(120);
            HCrop.push_back(320);
            NTemp = 80;
            WCross = 3;
            stride = 2;
            Crop_params.push_back(120);
            Crop_params.push_back(50);
            Crop_params.push_back(320);
            Crop_params.push_back(200);
        }
        else if (height == 1080)
        {
            //            WCrop.push_back(640);
            //            WCrop.push_back(1280);
            //            HCrop.push_back(240);
            //            HCrop.push_back(600);
            WCrop.push_back(640);
            WCrop.push_back(1440);
            HCrop.push_back(240);
            HCrop.push_back(720);
            NTemp = 120;
            WCross = 6;
            stride = 4;
            Crop_params.push_back(150);
            Crop_params.push_back(100);
            Crop_params.push_back(400);
            Crop_params.push_back(320);
        }

        Thresh = 0.8;
    };

    Cross_center::~Cross_center() = default;;

    Point Cross_center::Cal_intersection(Vec4i line1, Vec4i line2)
    {
        Point center;
        int a1 = line1[3] - line1[1];
        int b1 = line1[2] - line1[0];
        int c1 = line1[2] * line1[1] - line1[0] * line1[3];

        int a2 = line2[3] - line2[1];
        int b2 = line2[2] - line2[0];
        int c2 = line2[2] * line2[1] - line2[0] * line2[3];

        int tmp = a1*b2 - a2*b1;

        if (tmp != 0)
        {
            center.x = (b1*c2 - b2*c1) / (tmp);
            center.y = (c2*a1 - c1*a2) / (tmp);
        }

        return center;
    }
    Cross_Property Cross_center::Crossline_Reg(Mat& img)
    {
        Mat image;
        Mat mv[3];
        split(img, mv);
        image = Crop_img(mv[2]);
        Mat midimg, dstimg, edgeimg;
        //        Canny(image, midimg, 50, 200, 3);
        auto th = static_cast<int>(floor(Thresh * 255));
        threshold(image, midimg, th, 255, CV_THRESH_BINARY);
        //        Canny(midimg, edgeimg, 50, 200, 3);
        //        cvtColor(midimg, dstimg, CV_GRAY2BGR);
        //        imshow("midimg",midimg);
        //        waitKey(0);

        //计算模板
        int Ow, Oh;
        vector<int > out_dim = Out_dim(midimg.cols, midimg.rows);
        vector<int > temp_iter = Temp_iter();
        Ow = out_dim[0];
        Oh = out_dim[1];
        //        cout<<midimg.cols<<"\t"<<midimg.rows<<endl;
        //        cout<<"Ow: "<<Ow<<endl;
        //        cout<<"Oh: "<<Oh<<endl;
        //        cout<<temp_iter[0]<<"\t"<<temp_iter[1]<<endl;
        int istart = temp_iter[0];
        // 用十字模板匹配原图像，截取激光十字图像
        Rect wrect(0, istart, NTemp, WCross);
        Rect hrect(istart, 0, WCross, NTemp);
        int outlenght;
        outlenght = (Ow*Oh);
        vector<double> tmp_out(outlenght);
        for (int ih = 0; ih<Oh; ih++)
        {
            for (int iw = 0; iw<Ow; iw++)
            {
                Rect tmprect(iw*stride, ih*stride, NTemp, NTemp);
                Mat tmpmat = Mat(midimg, tmprect);
                Mat tmpw = Mat(tmpmat, wrect);
                Mat tmph = Mat(tmpmat, hrect);
                double sumtemp = 0;
                Scalar tmpsum = sum(tmpw);
                sumtemp += tmpsum[0];
                tmpsum = sum(tmph);
                sumtemp += tmpsum[0];
                tmp_out[ih*Ow + iw] = sumtemp;
            }
        }
        // 计算十字图像的左上坐标
        auto maxvalue = max_element(tmp_out.begin(), tmp_out.end());
        int maxpos = static_cast<int>(maxvalue - tmp_out.begin());
        int hidx = maxpos / Ow;
        int widx = maxpos%Ow;
        int left = widx*stride;
        int top = hidx*stride;
        //        cout<<maxpos<<"\t"<<hidx<<"\t"<<widx<<"\t"<<left<<"\t"<<top<<endl;
        int left2 = max(0, left - Crop_params[0]);
        int top2 = max(0, top - Crop_params[1]);
        int right2 = min(left2 + Crop_params[2], midimg.cols);
        int bottom2 = min(top2 + Crop_params[3], midimg.rows);
        Rect rect_select(left2, top2, right2 - left2, bottom2 - top2);
        Mat mat_select(midimg, rect_select);
        cvtColor(mat_select, dstimg, CV_GRAY2BGR);
        //        imshow("imgselect",mat_select);
        //        waitKey(0);

        //十字线段拟合
        vector<Vec4i> lines_ori;//定义一个矢量结构lines用于存放得到的线段矢量集合
        //        HoughLinesP(midimg, lines_ori, 1, CV_PI/180, 100, 150, 10 );
        HoughLinesP(mat_select, lines_ori, 1, CV_PI / 180, 100, 150, 10);
        vector<Vec4i> vlines, hlines;
        vector<double> vweight, hweight;
        double hweight_sum = 0, vweight_sum = 0;
        double vx1 = 0, vx2 = 0, vy1 = 0, vy2 = 0;
        double hy1 = 0, hy2 = 0, hx1 = 0, hx2 = 0;
        for (auto l : lines_ori) {
            double weight = sqrt(pow(l[0] - l[2], 2) + pow(l[1] - l[3], 2));
            if (abs(l[0] - l[2])<10 && abs(l[1] - l[3])>100)
            {
                vweight.push_back(weight);
                vweight_sum += weight;
                vlines.push_back(l);

                if (l[1]<l[3])
                {
                    vx1 = vx1 + l[0] * weight;
                    vx2 = vx2 + l[2] * weight;
                    vy1 += l[1] * weight;
                    vy2 += l[3] * weight;
                }
                else
                {
                    vx1 = vx1 + l[2] * weight;
                    vx2 = vx2 + l[0] * weight;
                    vy1 += l[3] * weight;
                    vy2 += l[1] * weight;
                }
            }
            else if (abs(l[0] - l[2])>100 && abs(l[1] - l[3])<10)
            {
                hweight.push_back(weight);
                hweight_sum += weight;
                hlines.push_back(l);
                if (l[0]<l[2])
                {
                    hy1 = hy1 + l[1] * weight;
                    hy2 = hy2 + l[3] * weight;
                    hx1 += l[0] * weight;
                    hx2 += l[2] * weight;
                }
                else
                {
                    hy1 = hy1 + l[3] * weight;
                    hy2 = hy2 + l[1] * weight;
                    hx1 += l[2] * weight;
                    hx2 += l[0] * weight;
                }
                //                line( dstimg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
                //                cout<<"hline: "<<l<<"weight: "<<weight<<endl;
            }

        }
        int ivx1 = static_cast<int>(round(vx1 / (vweight_sum)));
        int ivx2 = static_cast<int>(round(vx2 / (vweight_sum)));
        int ivy1 = static_cast<int>(round(vy1 / vweight_sum));
        int ivy2 = static_cast<int>(round(vy2 / vweight_sum));
        int ihy1 = static_cast<int>(round(hy1 / (hweight_sum)));
        int ihy2 = static_cast<int>(round(hy2 / (hweight_sum)));
        int ihx1 = static_cast<int>(round(hx1 / hweight_sum));
        int ihx2 = static_cast<int>(round(hx2 / hweight_sum));

        Vec4i vline = { ivx1,ivy1,ivx2,ivy2 };
        Vec4i hline = { ihx1,ihy1,ihx2,ihy2 };
        Point center = Cal_intersection(vline, hline);
        Cross_Property output;
        //        output.center.x=center.x+WCrop[0];
        //        output.center.y=center.y+HCrop[0];
        output.center.x = center.x + WCrop[0] + left2;
        output.center.y = center.y + HCrop[0] + top2;
        for (int i = 0; i<2; i++)
        {
            output.horizontal[2 * i] = hline[2 * i] + WCrop[0] + left2;
            output.vertical[2 * i] = vline[2 * i] + WCrop[0] + left2;
            output.horizontal[2 * i + 1] = hline[2 * i + 1] + HCrop[0] + top2;
            output.vertical[2 * i + 1] = vline[2 * i + 1] + HCrop[0] + top2;
        }

        //        cout<<center.x<<"\t"<<center.y<<endl;
        //        Rect rect_center(center.x-2,center.y-2,5,5);
        //        rectangle(dstimg, rect_center, Scalar(0,0,255),3);

        //        line( dstimg, Point(ivx1, ivy1), Point(ivx2, ivy2), Scalar(0,255,0), 1, CV_AA);
        //        line( dstimg, Point(ihx1, ihy1), Point(ihx2, ihy2), Scalar(255,0,0), 1, CV_AA);
        //        imshow("line", dstimg);
        //        waitKey(0);
        return output;
    };

    vector<int > Cross_center::Get_center(Mat& img)
    {
        // 提取红色通道, BGR, mv[2]为红色通道
        Mat mv[3];
        split(img, mv);
        Mat imgbw;
        threshold(mv[2], imgbw, 190, 255, CV_THRESH_BINARY);
        //        imshow("imbw",imgbw);
        // 根据WCrop和HCrop进行裁剪
        Mat image;
        image = Crop_img(imgbw);
        //        imshow("imcrop",image);
        //        waitKey(0);
        // 计算输出的模板维度
        int Ow, Oh;
        vector<int > out_dim = Out_dim(image.cols, image.rows);
        vector<int > temp_iter = Temp_iter();
        Ow = out_dim[0];
        Oh = out_dim[1];
        //        cout<<"Ow: "<<Ow<<endl;
        //        cout<<"Oh: "<<Oh<<endl;
        int istart;
        istart = temp_iter[0];
        // 用十字模板匹配原图像，截取激光十字图像
        Rect wrect(0, istart, NTemp, WCross);
        Rect hrect(istart, 0, WCross, NTemp);
        vector<double> tmp_out(Ow*Oh);
        for (int ih = 0; ih<Oh; ih++)
        {
            for (int iw = 0; iw<Ow; iw++)
            {
                Rect tmprect(iw*stride, ih*stride, NTemp, NTemp);
                Mat tmpmat = Mat(image, tmprect);
                Mat tmpw = Mat(tmpmat, wrect);
                Mat tmph = Mat(tmpmat, hrect);
                double sumtemp = 0;
                Scalar tmpsum = sum(tmpw);
                sumtemp += tmpsum[0];
                tmpsum = sum(tmph);
                sumtemp += tmpsum[0];
                tmp_out[ih*Ow + iw] = sumtemp;
            }
        }
        // 计算十字图像的左上坐标
        auto maxvalue = max_element(tmp_out.begin(), tmp_out.end());
        int maxpos = static_cast<int>(maxvalue - tmp_out.begin());
        int hidx = maxpos / Ow;
        int widx = maxpos%Ow;
        int left = widx*stride;
        int top = hidx*stride;
        //        cout<<maxpos<<endl;
        //        cout<<hidx<<endl<<widx<<endl<<left<<endl<<top<<endl;
        // 计算十字图像在水平和垂直方向的投影，估计中心坐标
        Rect rect_select(left, top, NTemp, NTemp);
        Mat mat_select(image, rect_select);
        //        for(int idxh=0;idxh<mat_select.rows;idxh++)
        //        {
        //            for(int idxw=0;idxw<mat_select.cols;idxw++)
        //            {
        //                cout<<mat_select.at<uchar>(idxw,idxh)<<" ";
        //            }
        //            cout<<endl;
        //        }
        Mat mat_select_trans;
        transpose(mat_select, mat_select_trans);
        vector<int> hhist = Mat_sum_dim(mat_select);
        vector<int> whist = Mat_sum_dim(mat_select_trans);
        auto maxvalue2 = max_element(begin(whist), end(whist));
        int wcenter = static_cast<int>(distance(begin(whist), maxvalue2));
        auto maxvalue3 = max_element(begin(hhist), end(hhist));
        int hcenter = static_cast<int>(distance(begin(hhist), maxvalue3));
        vector<int > output;
        //        cout<<"wcenter: "<<wcenter<<endl;
        //        for(int i=0;i<whist.size();i++)
        //            cout<<whist[i]<<endl;
        //        cout<<"hcenter: "<<hcenter<<endl;
        //        for(int i=0;i<hhist.size();i++)
        //            cout<<hhist[i]<<endl;
        output.push_back(wcenter + left + WCrop[0]);
        output.push_back(hcenter + top + HCrop[0]);
        //        cout<<output[0]<<endl;
        //        cout<<output[1]<<endl;

        return output;
    };

    vector<int> Cross_center::Mat_sum_dim(Mat& image)
    {
        vector<int> sumtemp;
        for (int ih = 0; ih<image.rows; ih++)
        {
            int tmp = 0;
            for (int iw = 0; iw<image.cols; iw++)
            {
                tmp += image.at<uchar>(iw, ih);
            }
            sumtemp.push_back(tmp);
        }
        return sumtemp;
    };

    vector<int > Cross_center::Temp_iter()
    {
        vector<int > iter;
        int tmp = NTemp / 2;
        int tmp2 = WCross / 2;
        int out1 = tmp - tmp2;
        int out2 = tmp + tmp2;
        iter.push_back(out1);
        iter.push_back(out2);
        return iter;
    };

    vector<int > Cross_center::Out_dim(int W, int H) {
        vector<int > tmp;
        int tmpw, tmph;
        tmpw = (W - NTemp) / stride + 1;
        tmph = (H - NTemp) / stride + 1;
        tmp.push_back(tmpw);
        tmp.push_back(tmph);
        return tmp;
    };

    Mat Cross_center::Crop_img(Mat& img)
    {
        int wstart, w, hstart, h;
        wstart = WCrop[0];
        w = WCrop[1] - wstart;
        hstart = HCrop[0];
        h = HCrop[1] - hstart;
        Rect rect(wstart, hstart, w, h);
        Mat outimg = Mat(img, rect);
        return outimg;
    };
}
