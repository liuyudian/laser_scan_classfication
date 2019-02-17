//
// Created by minzhao on 18-12-18.
//
#include "classify.h"

namespace ranging {
    Classify::Classify() : failed_count_(0), succeed_count_(0) {

    }
    int Classify::start_classfication(const cv::Mat &img) {
        cv::Mat testimg;
        testimg = img;
        //针对测试集进行识别
        cv::resize(testimg, testimg, cv::Size(120, 160));
        std::vector<float> descriptors;
        hog_.compute(testimg, descriptors);
        cv::Mat testDescriptor = cv::Mat::zeros(1, descriptors.size(), CV_32FC1);
        for (size_t i = 0; i < descriptors.size(); i++) {
            testDescriptor.at<float>(0, i) = descriptors[i];
        }
        float label = svm_->predict(testDescriptor);
        if (label < 0)
            failed_count_++;
        else
            succeed_count_++;
        //        std::cout << "label test： " << label << std::endl;
    }

    int Classify::init_svm() {
        //HOG检测器，用来计算HOG描述子的
        //检测窗口(48,48),块尺寸(16,16),块步长(8,8),cell尺寸(8,8),直方图bin个数9
        cv::HOGDescriptor hog_r(cv::Size(120, 160), cv::Size(20, 20), cv::Size(10, 10), cv::Size(10, 10), 9);
        hog_ = hog_r;
        //HOG描述子的维数，由图片大小、检测窗口大小、块大小、细胞单元中直方图bin个数决定
        int DescriptorDim;
        //从XML文件读取训练好的SVM模型
        cv::Ptr<cv::ml::SVM> svm_r = cv::ml::SVM::load("/home/minzhao/Document/laser_scan_classfication/config/SVM_HOG.xml");
        if (svm_r->empty()) {
            std::cout << "load svm detector failed!!!" << std::endl;
            return -1;
        }
        svm_ = svm_r;
        return 0;
    }

    void Classify::print_ratio(unsigned long count) {
        std::cout << "succeed count: " << succeed_count_ << " fail count: " << failed_count_ << std::endl;
        std::cout << "succeed ratio: " << succeed_count_ / static_cast<double >(count) << " failed ratio: " << failed_count_ / static_cast<double >(count)
                  << std::endl;
    }

    void Classify::Train_svm_hog() {

    }
}
