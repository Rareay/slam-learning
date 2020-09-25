
#ifndef TRSLAM_FRONTEND_H
#define TRSLAM_FRONTEND_H

#include "trslam/frame.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <unistd.h>

namespace trslam {

class Frontend {
public:
    unsigned int id = 0;
    uint CacheFrameNum; // 缓存帧数
    std::vector<std::vector<uchar>> FrameStatus;
    uint FeaturePointNum; // 每帧特征点最大个数
    std::vector<uchar> frontStatus;
    std::vector<trslam::Frame> CacheFrames;
    trslam::Frame frontFrame_; // 缓存的上一帧,也是输出的最新帧
    trslam::Frame frontFrame__;// 缓存的上上帧

    Frontend(int frame_num, int point_num) {
        CacheFrameNum = frame_num;
        FeaturePointNum = point_num;
        {
            std::vector<uchar> temp(point_num, 0);
            temp.swap(frontStatus);
        }
    };

    /** @brief 前端计算
     * @param 
     */
    void FrontendCalculate(cv::Mat img) {
        trslam::Frame frame;
        frame.image = img;
        frame.id = id;
        if (CacheFrames.size() < 1) { // 如果缓存帧为空，追加当前帧后就返回
            std::vector<uchar> status(FeaturePointNum, 0);
            addFrameStatus(status);
            CacheFrames.push_back(frame);
            return ;
        }
        // 根据缓存的最后一帧和当前帧计算当前帧的特征点
        trslam::Frame last_frame = CacheFrames[CacheFrames.size() - 1];
        float f = trackFeaturePoints(last_frame.image, frame.image,
                                     last_frame.feature, frame.feature,
                                     frame.feature_match);

        std::vector<uchar> status(frame.feature_match);
        addFrameStatus(status);

        if (CacheFrames.size() < CacheFrameNum) { // 如果缓存帧未满，追加当前帧后就返回
            CacheFrames.push_back(frame);
            return ;
        }

        // 队列追加当前帧，在缓存帧的前两帧用来计算位姿
        frontFrame__.image.release();
        frontFrame__ = frontFrame_;
        frontFrame_ = CacheFrames[0];
        for (uint i = 0; i < CacheFrames.size() - 1; i++) {
            CacheFrames[i] = CacheFrames[i+1];
        }
        CacheFrames[CacheFrames.size() - 1] = frame;
        // 过滤出特征点
        flterTrackedPoints();
        std::vector<uchar> flter_status(frontStatus);
        frontFrame_.feature_match = flter_status;

        if (frontFrame__.id == -1) {
            return;
        }
        // 计算位姿
        estimatePose_2d2d(frontFrame__.feature,
                          frontFrame_.feature,
                          frontFrame_.feature_match,
                          frontFrame_.pose);

    }
    
    /** @brief 位姿计算
     * @param pt1 图像1的特征点
     * @param pt2 图像2的特征点
     * @param status 特征匹配信息
     * @param pt1 预测出的位姿
     */
    void estimatePose_2d2d(std::vector<cv::Point2f> & pt1,
                           std::vector<cv::Point2f> & pt2,
                           std::vector<uchar> & status,
                           Sophus::SE3d & pose) {
        cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

        std::vector<cv::Point2f> keypoints1, keypoints2;
        for (uint i = 0; i < status.size(); i++) {
            if (status[i]) {
                keypoints1.push_back(pt1[i]);
                keypoints2.push_back(pt2[i]);
            }
        }

        //-- 计算本质矩阵
        cv::Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
        double focal_length = 521;      //相机焦距, TUM dataset标定值
        cv::Mat essential_matrix;
        essential_matrix = findEssentialMat(keypoints1, 
                                            keypoints2, 
                                            focal_length, 
                                            principal_point);

        cv::Mat R, t;
        cv::recoverPose(essential_matrix, keypoints1, keypoints2, 
                        R, t, focal_length, principal_point);
        Eigen::Matrix3d R_eigen;
        Eigen::Vector3d t_eigen;
        cv::cv2eigen(R, R_eigen);
        cv::cv2eigen(t, t_eigen);
        Sophus::SE3d SE3_Rt(R_eigen, t_eigen);
        pose = SE3_Rt;
    }
    // 跟踪特征点
    float trackFeaturePoints(cv::Mat & img1, cv::Mat & img2, 
                             std::vector<cv::Point2f> & pt1,
                             std::vector<cv::Point2f> & pt2,
                             std::vector<uchar> & status) {
        if (img1.empty() || img2.empty()) return -1.;

        std::vector<cv::KeyPoint> kp1;
        if (pt1.empty()) {
            cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
            detector->detect(img1, kp1); // 从img1中提取特征点
            for (auto &kp: kp1) pt1.push_back(kp.pt);
        }

        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, 
                        cv::Size(21, 21), 3);

        float f = flterMatchingPoints(pt1, pt2, status);

        addTrackedPoints(img2, pt2, status);

        addFrameStatus(status);

        return f;
    };

    /** @brief 更新缓存帧的匹配状态
     * @param status 最新一帧的匹配状态
     */
    void addFrameStatus(std::vector<uchar> & status) {
        std::vector<uchar> s(status);
        if (FrameStatus.size() < CacheFrameNum)
            FrameStatus.push_back(s);
        else {
            for (uint i = 0; i < FrameStatus.size() - 1; i++) {
                FrameStatus[i].swap(FrameStatus[i+1]);
            }
            FrameStatus[FrameStatus.size() - 1] = s;
        }
    }

    /**
     * @brief 过滤距离远的匹配点
     * @param pt1 特征点1
     * @param pt2 特征点2
     * @param status 已匹配的状态，1：匹配， 0：未匹配
     * @param threshold （可选）匹配点距离的平方大于 threshold 则将对应 status 设为 0
     * @return 匹配百分比
     */
    float flterMatchingPoints(std::vector<cv::Point2f> & pt1,
                           std::vector<cv::Point2f> & pt2,
                           std::vector<uchar> & status,
                           int threshold = 400) {
        int n = 0;
        for (uint i = 0; i < pt1.size(); i++) {
            if (status[i]) {
                int dx = pt1[i].x - pt2[i].x;
                int dy = pt1[i].y - pt2[i].y;
                dx *= dx;
                dy *= dy;
                if (dx + dy > threshold)
                    status[i] = 0;
                else
                    n++;
            }
        }
        return (float)n / (float)pt1.size();
    }

    /** @brief 添加跟踪点 
     * @param img 需要添加跟踪点的图像
     * @param pt 已有的特征点
     * @param status 各个特征点的状态（有效还是无效）
     * @param min_distance (可选)新点的最小距离
     */
    void addTrackedPoints(cv::Mat & img, std::vector<cv::Point2f> & pt,
                          std::vector<uchar> & status, int min_distance = 5) {
        // 找出一批可选特征点
        std::vector<cv::KeyPoint> kp;
        std::vector<cv::Point2f> alternative_pt; // 可选特征点
        cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
        detector->detect(img, kp);
        for (auto &k: kp) alternative_pt.push_back(k.pt);

        // 把有效点打点在蒙板上
        cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
        for (uint i = 0; i < pt.size(); i++) {
            uchar * ptr = mask.ptr<uchar>((int)pt[i].y);
            ptr[(int)pt[i].x] = 1;
        }

        // 查找出和有效点距离远的点
        std::vector<cv::Point2f> new_pt;
        for (uint i = 0; i < alternative_pt.size(); i++) {
            int x_min, x_max;
            int y_min, y_max;

            x_min = alternative_pt[i].x - min_distance;
            if (x_min < 0)
                x_min = 0;

            x_max = alternative_pt[i].x + min_distance;
            if (x_max > img.cols - 1) 
                x_max = img.cols - 1;

            y_min = alternative_pt[i].y - min_distance;
            if (y_min < 0)
                y_min = 0;

            y_max = alternative_pt[i].y + min_distance;
            if (y_max > img.rows) 
                y_max = img.rows;
            
            bool suit_point = true;
            for (int y = y_min; y < y_max; y++) {
                uchar * ptr = mask.ptr<uchar>(y);
                for (int x = x_min; x < x_max; x++) {
                    if (ptr[x] == 1) {
                        suit_point = false;
                        break;
                    }
                }
                if (!suit_point) break;
            }

            if (suit_point) {
                new_pt.push_back(alternative_pt[i]);
            }
        }

        // 用新点覆盖无效点
        uint new_point_id = 0;
        for (uint i = 0; i < pt.size(); i++) {
            if (status[i] == 0) {
                pt[i] = new_pt[new_point_id];
                new_point_id ++;
                if (new_point_id >= new_pt.size())
                    break;
            }
        }
    }

    /** @brief 根据缓存帧过滤出有效跟踪点
     */
    void flterTrackedPoints() {
        if (FrameStatus.size() != CacheFrameNum) return;
        std::vector<uchar> status(FeaturePointNum);
        for (uint i = 0; i < FeaturePointNum; i++) {
            if (frontStatus[i] == 1) {
                if (FrameStatus[0][i] == 0)
                    frontStatus[i] = 0;
            } else {
                bool track_it = true;
                for (uint j = 0; j < CacheFrameNum; j++) {
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