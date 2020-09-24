
#ifndef TRSLAM_FEATURE_H
#define TRSLAM_FEATURE_H

#include <opencv2/opencv.hpp>


namespace trslam {

class Feature {
public:
    float calculateLK(cv::Mat & img1, std::vector<cv::Point2f> & pt1,
                cv::Mat & img2, std::vector<cv::Point2f> & pt2,
                std::vector<uchar> & status) {
        if (img1.empty() || img2.empty()) return -1.;

        std::vector<cv::KeyPoint> kp1, kp2;
        std::vector<cv::Point2f> pt2_;
        if (pt1.empty()) {
            cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
            detector->detect(img1, kp1); // 从img1中提取特征点
            for (auto &kp: kp1) pt1.push_back(kp.pt);
        }
        if (pt2.empty()) {
            cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
            detector->detect(img2, kp2); // 从img1中提取特征点
            for (auto &kp: kp2) pt2_.push_back(kp.pt);
        }

        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, 
                        cv::Size(21, 21), 3);

        int n = 0;
        for (int i = 0; i < status.size(); i++) {
            if (status[i]) { // 如果存在对应光流点
                n++;
                cv::line(img2, pt1[i], pt2[i], cv::Scalar(0, 255, 0));
            }
            cv::circle(img2, pt2[i], 2, cv::Scalar(0, 255, 0), 2);
        }
        for (int i = 0; i < pt2_.size(); i++) {
            cv::circle(img2, pt2_[i], 2, cv::Scalar(0, 100, 200), 1);
        }
        //没有匹配的特征点重新查找
        return (float)n/(float)status.size();
    }

private:

};


}


#endif