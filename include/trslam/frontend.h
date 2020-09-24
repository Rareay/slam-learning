
#ifndef TRSLAM_FRONTEND_H
#define TRSLAM_FRONTEND_H

#include <opencv2/opencv.hpp>

namespace trslam {

class Frontend {
public:
    int CacheFrameNum; // 缓存帧数
    std::vector<std::vector<uchar>> FrameStatus;
    int FeaturePointNum; // 每帧特征点最大个数
    std::vector<uchar> frontStatus;

    Frontend(int frame_num, int point_num) {
        CacheFrameNum = frame_num;
        FeaturePointNum = point_num;
        {
            std::vector<uchar> temp(point_num, 0);
            temp.swap(frontStatus);
        }
    };

    // 跟踪特征点
    float trackFeaturePoints(cv::Mat & img1, cv::Mat & img2, 
                             std::vector<cv::Point2f> & pt1,
                             std::vector<cv::Point2f> & pt2,
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
        return (float)n/(float)status.size();       if (img1.empty() || img2.empty()) return -1.;

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
    };


    // 添加跟踪点 

    // 根据缓存帧过滤出有效跟踪点
    void flterTrackPoints() {
        if (FrameStatus.size() != CacheFrameNum) return;
        std::vector<uchar> status(FeaturePointNum);
        for (int i = 0; i < FeaturePointNum; i++) {
            if (frontStatus[i] == 1) {
                if (FrameStatus[0][i] == 0)
                    frontStatus[i] = 0;
            } else {
                bool track_it = true;
                for (int j = 0; j < CacheFrameNum; j++) {
                    if (FrameStatus[j][i] == 0) {
                        track_it = false;
                        break;
                    }
                }
                if (track_it) frontStatus[i] = 1;
            }
        }
    }

private:

};

}


#endif