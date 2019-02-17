#include <iostream>
#include "calculate.h"
#include "opencv2/opencv.hpp"
#include "camera.h"
//#include "classify.h"
#include "common.h"
#include "cross_center.h"
#include <unistd.h>

char resultbuff[50];
typedef struct resultovertcp {
    char flag1;
    char flag2;
    short x;
    short y;
    short z;
    short angle;
    short open_laser;  //表示通知机器是否到达500mm附近

} ResultOverTcp;
ResultOverTcp result_over_tcp;

class serv : public dlib::server_iostream {

    void on_connect(
            std::istream &in,
            std::ostream &out,
            const std::string &foreign_ip,
            const std::string &local_ip,
            unsigned short foreign_port,
            unsigned short local_port,
            uint64 connection_id
    ) {
        // The details of the connection are contained in the last few arguments to
        // on_connect().  For more information, see the documentation for the
        // server_iostream.  However, the main arguments of interest are the two streams.
        // Here we also print the IP address of the remote machine.
        //cout << "Got a connection from " << foreign_ip << endl;

        // Loop until we hit the end of the stream.  This happens when the connection
        // terminates.
#if 0
        for (int i = 0;; i++) {

            if (resultbuff[i] == '\0')
                break;
            else
                out << resultbuff[i];

        }
        memset(resultbuff, 0, sizeof(resultbuff));
#endif
        char *ptr = (char *) &result_over_tcp;
        for (int i = 0; i < 10; i++) {
            out << ptr[i];

        }
        memset(ptr, 0, sizeof(result_over_tcp));

    }

};

//#define CAPTURE
int main(int argc, char **argv) {

    ranging::Camera camera;
    //ranging::Classify classify;
    ranging::Calculate calculate(ECLIPSEWIDTH, ECLIPSEHEIGHT, STANDARDLENGTH);
    ranging::Cross_center cross_center(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
    VideoCapture capture;
    Mat frame;
    ranging::eclipse<float> eclipse;
    ranging::Cross_Property cross_property;
    ranging::point<double> range;
    char txtBuff[50]; //文字
    short open_laser = 0;  //控制是否打开十字光心, 0表示关闭，1表示打开

    std::cout << "启动程序方法: laser_range.exe id" << endl;
    std::cout << "0:使用0号设备摄像头" << std::endl;
    std::cout << "1:使用1号设备摄像头" << std::endl;
    std::cout << "2:使用2号设备摄像头" << std::endl;
    std::cout << "如果程序不能打开摄像头，尝试使用id为0,1,2，都试一遍" << std::endl;

    cout << "当前使用摄像头ID: " << argv[1] << endl;
    int ret = camera.openCam(capture, atoi(argv[1]));
    if (ret != 0) {
        std::cout << "open cam failed" << std::endl;
        return -1;
    }

    if (argc==4 && atoi(argv[2]) == 1) {
        string path = argv[3];
        camera.captureImg(path);
    } else {
        std::cout << "开始测距..." << std::endl;
        for (;;) {
            capture >> frame;
            if (frame.empty()) {
                printf("--(!) No captured frame -- Break!");
            }

            if (open_laser == 0) {
                eclipse = calculate.detectEclipseAndCalculateUseDlib(frame);  //使用dlib识别椭圆，并计算距离
                if (eclipse.height == -1) {
                    continue;
                }
//            sprintf(txtBuff, "1--real_x=%f, real_y=%f, real_z=%f\n", eclipse.x, eclipse.y, eclipse.z);
                //sprintf(txtBuff, "z_angle=%f, y_angle=0.0, x_angle=0.0", eclipse.rotate + 90);
//            cv::putText(frame, txtBuff, cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
                //ranging::eclipse<float> eclipse = calculate.detectEclipse(frame);  //使用opencv识别椭圆，并给出椭圆光心像素坐标
                result_over_tcp.flag1 = 0xed;
                result_over_tcp.flag2 = 0xff;
                result_over_tcp.x = (eclipse.x);
                result_over_tcp.y = (eclipse.y);
                result_over_tcp.z = (eclipse.z);
                result_over_tcp.angle = eclipse.rotate + 90;
                if (eclipse.z <= 500) {
                    open_laser = 1;
                    result_over_tcp.open_laser = open_laser;
                    continue;
                }
                cv::imshow("检测结果", frame); //显示当前帧
                if (cvWaitKey(2) == 27)
                    break;
            } else {
                cross_property = cross_center.Crossline_Reg(frame); //识别十字光心
                cv::Rect cross_center(cross_property.center.x, cross_property.center.y, 2, 2);  //显示十字光心
                rectangle(frame, cross_center, cv::Scalar(255, 0, 0), 4);
                cv::Rect rect_center(RESOLUTION_CENTER_WIDTH, RESOLUTION_CENTER_HEIGHT, 2, 2); //显示图像中心
                //rectangle(frame, rect_center, cv::Scalar(0, 0, 255), 3);
                eclipse = calculate.detectEclipseUseDlib(frame); //识别椭圆
                if (eclipse.height == -1) {
                    continue;
                }

                //eclipse = calculate.detectEclipse(frame);
                range = calculate.calculateUseCrossLaser(cross_property.center, eclipse); //带入计算x y z
                //sprintf(txtBuff, "z_angle=%f, y_angle=0.0, x_angle=0.0\n", eclipse.rotate + 90);

                cv::putText(frame, txtBuff, cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
#if 0
                sprintf(txtBuff, "2--real_x=%f, real_y=%f, real_z=%f\n", range.x, range.y, range.z);
            result_over_tcp.flag1 = 0xed;
            result_over_tcp.flag2 = 0xff;
            result_over_tcp.x = (range.x);
            result_over_tcp.y = (range.y);
            result_over_tcp.z = (range.z);
            result_over_tcp.angle = eclipse.rotate + 90;
            result_over_tcp.open_laser = open_laser;
#endif
                cv::imshow("检测结果", frame); //显示当前帧
                if (cvWaitKey(2) == 27)
                    break;
            }
        }
    }


    return 0;
}