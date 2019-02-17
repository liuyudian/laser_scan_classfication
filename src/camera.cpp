//
// Created by minzhao on 18-12-9.
//
#include "camera.h"
#include "common.h"
namespace ranging {
    bool Camera::captureImg(string& path) {
        cout << "path: " << path << endl;
        string data = path + "/data.txt";
        outfile.open(data.c_str(), ios::binary | ios::in | ios::out | ios::trunc);
        // open the default camera, use something different from 0 otherwise;
        // Check VideoCapture documentation.
//        if (!cap.open(1))
//            return false;
        //cap.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION_WIDTH);
        //cap.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION_HEIGHT);
        double frame_rate = cap.get(CV_CAP_PROP_FPS);
        int frame_idx = 0;
        string jpg_name;
        for (;;) {
            cap >> frame;
            if (frame.empty()) break; // end of vidQeo stream
            imshow("this is you, smile! :)", frame);
            int c = waitKey(1);
            switch (c) {
                case 27: {
                    cout << "stop capture" << endl;
                    // release the camera
                    cap.release();
                    break;
                }
                case 'q': {
                    frame_idx++;
                    jpg_name = path + "/" +  to_string(frame_idx) + ".jpg";
                    imwrite(jpg_name, frame);
                    outfile << jpg_name << endl;
                    outfile << "1" << endl;
                    cout << "p: " << frame_idx << endl;
                    break;
                }
                case 'w': {
                    frame_idx++;
                    jpg_name = path + "/" + to_string(frame_idx) + ".jpg";
                    imwrite(jpg_name, frame);
                    outfile << jpg_name << endl;
                    outfile << "-1" << endl;
                    cout << "n: " << frame_idx << endl;
                    break;
                }
                default: {
                    break;
                }
            }

        }

        outfile.close();
        return true;
    }
    bool Camera::closeCam() {
        cap.release();
        return true;
    }
    int Camera::openCam(VideoCapture& capture, int index) {
        // open the default camera, use something different from 0 otherwise;
        // Check VideoCapture documentation.
        if (!cap.open(index)) {
            cout << "open cam failed" << endl;
            return -1;
        }
        cap.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION_WIDTH);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION_HEIGHT);
        capture = cap;
        return 0;
    }
    void Camera::testImg() {

    }
}