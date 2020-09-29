/** @file frontend.cpp
 * @brief 前端
 * 
 * @author Tamray
 * @date 2020.09.24
 */


#include "trslam/frontend.h"

namespace trslam {

Frontend::Frontend(int frame_num, int point_num) {
    CacheFrameNum = frame_num;
    FeaturePointNum = point_num;
    {
        std::vector<uchar> temp(point_num, 0);
        //temp.swap(frontStatus);
        temp.swap(frontFrame_.feature_match);
    }
    {
        std::vector<uint> temp(point_num, 0);
        temp.swap(mRoasid);
    }
}

void Frontend::FrontendCalculate(cv::Mat img) {
    trslam::Frame frame;
    frame.image = img;
    frame.id = id;
    id ++;
    if (CacheFrames.size() < 1) { // 如果缓存帧为空，追加当前帧后就返回
        std::vector<uchar> status(FeaturePointNum, 0);
        frame.feature_match = status;
        CacheFrames.push_back(frame);
        return ;
    }
    // 根据缓存的最后一帧和当前帧计算当前帧的特征点
    uint l = CacheFrames.size();
    float f = trackFeaturePoints(CacheFrames[l-1].image, frame.image,
                                    CacheFrames[l-1].feature, frame.feature,
                                    frame.feature_match);

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

    if (frontFrame__.id == -1) {
        return;
    }
    // 计算位姿
    estimatePose_2d2d(frontFrame__.feature,
                        frontFrame_.feature,
                        frontFrame_.feature_match,
                        frontFrame_.pose);

    // 刷新地图路标
    refreshRoasid(frontFrame_.image,
                  frontFrame__.feature,
                  frontFrame_.feature,
                  frontFrame__.feature_match,
                  frontFrame_.feature_match,
                  frontFrame_.pose);
                  
    // 刷新地图位姿
    refreshPosture(id, frontFrame_.pose);

    // 刷新地图帧特征
    refreshFramefeature(id, frontFrame_.feature, frontFrame_.feature_match);
}
 

void Frontend::refreshRoasid(cv::Mat & img,
                       std::vector<cv::Point2f> & ptr1,
                       std::vector<cv::Point2f> & ptr2,
                       std::vector<uchar> & statue1,
                       std::vector<uchar> & statue2,
                       Sophus::SE3d pose)
{
    for (int i = 0; i < statue1.size(); i++) {
        if (statue2[i] == 1) {
            if (statue1[i] == 0) {
                // 更新路标id
                mRoasidMax++;
                mRoasid[i] = mRoasidMax;
                // 计算路标
                Roadsign roas;
                roas.id = mRoasidMax;
                roas.rods = calculateRoadsign(img, ptr1[i], ptr2[i], pose);
                mMap.pushRoadsign(roas);
            }
        }
    }

}


Eigen::Vector3d Frontend::calculateRoadsign(cv::Mat & img,
                                            cv::Point2f pt1,
                                            cv::Point2f pt2,
                                            Sophus::SE3d pose)
{

}                                    

void Frontend::refreshPosture(uint id, Sophus::SE3d pose)
{
    Postrue p;
    p.id = id;
    p.pose = pose;
    mMap.pushPosture(p);
}

void Frontend::refreshFramefeature(uint id,
                             std::vector<cv::Point2f> & ptr,
                             std::vector<uchar> & statue)
{
    std::vector<cv::Point2f> p;
    std::vector<uint> p_rods;
    for (int i = 0; i < statue.size(); i++) {
        if (statue[i] == 1) {
            p.push_back(ptr[i]);
            p_rods.push_back(mRoasid[i]);
        }
    }
    Framefeature framefeature;
    framefeature.id = id;
    framefeature.ptr = p;
    framefeature.ptr_rods = p_rods;
    mMap.pushFramefeature(framefeature);
}





void Frontend::estimatePose_2d2d(std::vector<cv::Point2f> & pt1,
                        std::vector<cv::Point2f> & pt2,
                        std::vector<uchar> & status,
                        Sophus::SE3d & pose1,
                        Sophus::SE3d & pose2) {
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> keypoints1, keypoints2;
    for (uint i = 0; i < status.size(); i++) {
        if (status[i]) {
            keypoints1.push_back(pt1[i]);
            keypoints2.push_back(pt2[i]);
        }
    }

    //计算本质矩阵
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
    //std::cout << "R：\n" << R_eigen.matrix() << std::endl;
    //std::cout << "t：\n" << t_eigen.matrix() << std::endl;
    Sophus::SE3d SE3_Rt(R_eigen, t_eigen);
    pose2 = pose1 * SE3_Rt;
}

float Frontend::trackFeaturePoints(cv::Mat & img1, cv::Mat & img2, 
                            std::vector<cv::Point2f> & pt1,
                            std::vector<cv::Point2f> & pt2,
                            std::vector<uchar> & status) {
    if (img1.empty() || img2.empty()) return -1.;
    std::vector<cv::KeyPoint> kp1;
    if (pt1.empty()) {
        std::vector<cv::Point2f> pt(FeaturePointNum, cv::Point2f(0,0));
        pt1 = pt;
        std::vector<uchar> s(FeaturePointNum, 0);
        addTrackedPoints(img1, pt1, s, 8);
    }

    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, 
                    cv::Size(21, 21), 3);
    // 忽略特征点（0，0）的匹配
    for (uint i = 0; i < FeaturePointNum; i++) {
        if (pt1[i].x == 0 && pt1[i].y == 0) {
            pt2[i].x = 0; pt2[i].y = 0;
            status[i] = 0;
        }
    }

    float f = flterMatchingPoints(pt1, pt2, status, 10000);

    addTrackedPoints(img2, pt2, status);

    return f;
};

float Frontend::flterMatchingPoints(std::vector<cv::Point2f> & pt1,
                        std::vector<cv::Point2f> & pt2,
                        std::vector<uchar> & status,
                        int threshold) {
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

void Frontend::addTrackedPoints(cv::Mat & img, std::vector<cv::Point2f> & pt,
                        std::vector<uchar> & status, int min_distance ) {
    // 找出一批可选特征点
    std::vector<cv::KeyPoint> kp;
    std::vector<cv::Point2f> alternative_pt; // 可选特征点
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img, kp);
    for (auto &k: kp) alternative_pt.push_back(k.pt);

    // 把有效点打点在蒙板上
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    for (uint i = 0; i < pt.size(); i++) {
        int x = pt[i].x;
        int y = pt[i].y;
        if (x <= 0 || x >= mask.cols || 
            y <= 0 || y >= mask.rows) continue;
        uchar * ptr = mask.ptr<uchar>(y);
        ptr[x] = 1;
    }

    // 查找出和有效点距离远的点
    std::vector<cv::Point2f> new_pt;
    int new_pt_id = 0;
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
            cv::Point2f p(alternative_pt[i]);
            new_pt.push_back(p);
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

void Frontend::flterTrackedPoints() {
    if (CacheFrames.size() != CacheFrameNum) return;
    std::vector<uchar> status(FeaturePointNum);
    for (uint i = 0; i < FeaturePointNum; i++) {
        for (uint j = 0; j < CacheFrameNum; j++) {
            if (CacheFrames[j].feature_match[i] != 0) continue;
            if (frontFrame__.feature_match[i] == 0) {
                frontFrame_.feature_match[i] = 0;
            }
            break;
        }
    }
}

}