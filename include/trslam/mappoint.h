
#ifndef TRSLAM_MAPPOINT_H
#define TRSLAM_MAPPOINT_H

#include "trslam/common_include.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

namespace trslam {

/** @brief 路标
 */
struct Roadsign
{
    uint id;
    Vec3 rods;
};

/** @brief 普通帧
 */
struct ComFrame
{
    uint id;
    cv::Mat image;
    std::vector<cv::Point2f> ptr; // 特征点
    std::vector<uchar> ptr_status; // 相对于上一帧的特征匹配状态
};

/** @brief 关键帧
 */
struct KeyFrame
{
    uint id;
    std::vector<cv::Point2f> ptr; // 特征点
    std::vector<uchar> ptr_status; // 相对于上一关键帧的特征匹配状态
    SE3 T;
    std::vector<uint> ptr_rods; // 特征点对应的路标点id
};



/** @brief 地图点：位姿、路标、帧特征
 */
class Mappoint {
public:
    /** @brief 创建、初始化地图点
     */
    static void createMappoint();

    void pushRoadsign(Roadsign roadsign);
    void remainRoadsign(uint remain_num);
    void readRoadsign(std::vector<Roadsign> & roadsigns);
    void refreshRoadsign(std::vector<Roadsign> & roadsigns);

    void pushComFrame(ComFrame comframe);
    void remainComFrame(uint remain_num);
    void readComFrame(std::vector<ComFrame> & comframes);
    void readOneComFrame(uint id, ComFrame & comframe);
    void refreshComFrame(std::vector<ComFrame> & comframes);

    void pushKeyFrame(KeyFrame keyframe);
    void remainKeyFrame(uint remain_num);
    void readKeyFrame(std::vector<KeyFrame> & keyframes);
    void readLastKeyFrame(KeyFrame & keyframe);
    void refreshKeyFrame(std::vector<KeyFrame> & keyframes);


private:

    static std::shared_ptr<Mappoint> mMappoint;
    
    std::vector<Roadsign> mRoadsigns;
    std::vector<ComFrame> mComFrame;
    std::vector<KeyFrame> mKeyframe;

    boost::shared_mutex shr_mutex_roadsign;
    boost::shared_mutex shr_mutex_comframe;
    boost::shared_mutex shr_mutex_keyframe;
};





}


#endif