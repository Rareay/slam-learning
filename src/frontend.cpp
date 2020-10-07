/** @file frontend.cpp
 * @brief 前端
 * 
 * @author Tamray
 * @date 2020.09.24
 */


#include "trslam/frontend.h"

namespace trslam {

Frontend::Frontend(int frame_num, int point_num, bool show) {
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
    mShow = show;
}

void Frontend::FrontendCalculate(cv::Mat img) {
    trslam::Frame frame;
    frame.image = img;
    frame.id = id;
    std::cout << "id: " << id << std::endl;
    id ++;
    if (CacheFrames.size() < 1) { // 如果缓存帧为空，追加当前帧后就返回
        std::vector<uchar> status(FeaturePointNum, 0);
        frame.feature_match = status;
        CacheFrames.push_back(frame);
        return ;
    }
    // 根据缓存的最后一帧和当前帧计算当前帧的特征点
    uint l = CacheFrames.size();
    trackFeaturePoints(CacheFrames[l-1].image, frame.image,
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
                        frontFrame__.pose,
                        frontFrame_.pose);

    // 刷新地图路标
    refreshRoasid(frontFrame__.feature,
                  frontFrame_.feature,
                  frontFrame__.feature_match,
                  frontFrame_.feature_match,
                  frontFrame__.pose,
                  frontFrame_.pose);
                  
    // 刷新地图位姿
    refreshPosture(id, frontFrame_.pose);

    // 刷新地图帧特征
    refreshFramefeature(id, frontFrame_.feature, frontFrame_.feature_match);

    showPicture();
}
 

void Frontend::refreshRoasid(std::vector<cv::Point2f> & ptr1,
                       std::vector<cv::Point2f> & ptr2,
                       std::vector<uchar> & statue1,
                       std::vector<uchar> & statue2,
                       SE3 pose1, SE3 pose2)
{
    std::vector<cv::Point2f> pts_1, pts_2;
    std::vector<uint> match_ptr_index;
    for (uint i = 0; i < statue2.size(); i++) { // 找出有效特征点
        if (statue2[i]) {
            match_ptr_index.push_back(i);
            pts_1.push_back(mCamera.pixel2camera(ptr1[i]));
            pts_2.push_back(mCamera.pixel2camera(ptr2[i]));
        }
    }
    cv::Mat T1, T2;
    cv::Mat pts_4d;
    cv::eigen2cv(pose1.matrix3x4(), T1);
    cv::eigen2cv(pose2.matrix3x4(), T2);
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d); // 对有效特征点三角测量
    std::vector<Vec3> rods_temp; // 路标点
    for (int i=0; i<pts_4d.cols; i++)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化 
        Vec3 rod;
        cv::cv2eigen(x, rod);
        rods_temp.push_back(rod);
    }

    for (uint i = 0; i < statue1.size(); i++) {
        if (statue2[i] && (statue1[i] == 0)) { // 选出新的路标点，并存储
            // 更新路标id
            mRoasidMax++;
            mRoasid[i] = mRoasidMax;

            for (uint j = 0; j < match_ptr_index.size(); j++) {
                if (match_ptr_index[j] == i){ 
                    Roadsign roadsign;
                    roadsign.id = mRoasidMax;
                    roadsign.rods = rods_temp[j];
                    mMappoint.pushRoadsign(roadsign);
                }
            }
        }
    }

}


void Frontend::refreshPosture(uint id, SE3 pose)
{
    Postrue p;
    p.id = id;
    p.pose = pose;
    mMappoint.pushPosture(p);
}

void Frontend::refreshFramefeature(uint id,
                             std::vector<cv::Point2f> & ptr,
                             std::vector<uchar> & statue)
{
    std::vector<cv::Point2f> p;
    std::vector<uint> p_rods;
    for (uint i = 0; i < statue.size(); i++) {
        if (statue[i] == 1) {
            p.push_back(ptr[i]);
            p_rods.push_back(mRoasid[i]);
        }
    }
    Framefeature framefeature;
    framefeature.id = id;
    framefeature.ptr = p;
    framefeature.ptr_rods = p_rods;
    mMappoint.pushFramefeature(framefeature);
}





void Frontend::estimatePose_2d2d(std::vector<cv::Point2f> & pt1,
                        std::vector<cv::Point2f> & pt2,
                        std::vector<uchar> & status,
                        SE3 & pose1,
                        SE3 & pose2) {
    cv::Mat K = cv::Mat::ones(3, 3, CV_64FC1);
    cv::eigen2cv(mCamera.getK(), K);
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
    Mat33 R_eigen;
    Vec3 t_eigen;
    cv::cv2eigen(R, R_eigen);
    cv::cv2eigen(t, t_eigen);
    SE3 SE3_Rt(R_eigen, t_eigen);
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

    // 过滤距离近的特征点
    filterFeaturePoints(img1, pt1, status, 10);

    // 忽略特征点（0，0）的匹配
    for (uint i = 0; i < FeaturePointNum; i++) {
        if (pt1[i].x == 0 && pt1[i].y == 0) {
            pt2[i].x = 0; pt2[i].y = 0;
            status[i] = 0;
        }
    }

    // 过滤距离远的匹配点
    float f = flterMatchingPoints(pt1, pt2, status, 10000);

    // 添加跟踪点
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
    //uint new_pt_id = 0;
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


void Frontend::filterFeaturePoints(cv::Mat & img, 
                         std::vector<cv::Point2f> & pt,
                         std::vector<uchar> & status,
                         int min_distance)
{
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    for (uint i = 0; i < status.size(); i++) {
        if (status[i]) {
            int x = pt[i].x;
            int y = pt[i].y;
            if (x <= 0 || x >= mask.cols || 
                y <= 0 || y >= mask.rows) {
                status[i] = 0;
                continue;
            }
            uchar * ptr = mask.ptr<uchar>(y);
            ptr[x] = 1;
        }
    }

    for (uint i = 0; i < status.size(); i++) {
        if (status[i]) {
            int x = pt[i].x;
            int y = pt[i].y;
            // 把原来位置的点置0
            uchar * ptr = mask.ptr<uchar>(y);
            ptr[x] = 0;

            int x_min, x_max;
            int y_min, y_max;

            x_min = x - min_distance;
            if (x_min < 0) x_min = 0;

            x_max = x + min_distance;
            if (x_max > img.cols - 1) x_max = img.cols - 1;

            y_min = y - min_distance;
            if (y_min < 0) y_min = 0;

            y_max = y + min_distance;
            if (y_max > img.rows) y_max = img.rows;
            
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
                uchar * ptr = mask.ptr<uchar>(y);
                ptr[x] = 1;
            } else {
                status[i] = 0;
            }
        }
    }


}


void Frontend::showPicture()
{
    cv::Mat img = frontFrame_.image.clone();
    std::vector<cv::Point2f> pt1 = frontFrame__.feature;
    std::vector<cv::Point2f> pt2 = frontFrame_.feature;
    std::vector<uchar> s = frontFrame_.feature_match;
    for (uint i = 0; i < s.size(); i++) {
        if (s[i]) {
            cv::circle(img, pt2[i], 1, cv::Scalar(50, 0, 200), 2);
            cv::line(img, pt1[i],  pt2[i], cv::Scalar(0, 255, 0));
        }
    }
    cv::imshow("dataset", img);
    if (cv::waitKey(1) >= 0) exit(0);
}




}