
#ifndef TRSLAM_MAPPOINT_H
#define TRSLAM_MAPPOINT_H

#include "trslam/common_include.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

namespace trslam {

/** @brief 位姿
 */
struct Postrue
{
    uint id;
    SE3 pose;
};

/** @brief 路标
 */
struct Roadsign
{
    uint id;
    Vec3 rods;
};

/** @brief 帧
 */
struct Framefeature
{
    uint id;
    std::vector<cv::Point2f> ptr; // 特征点
    std::vector<uint> ptr_rods; // 特征点对应路标id
};


/** @brief 地图点：位姿、路标、帧特征
 */
class Mappoint {
public:
    /** @brief 创建、初始化地图点
     */
    static void createMappoint();

    void pushPosture(Postrue posture);
    void erasePosture(uint remain_num);
    void readPostrue(std::vector<Postrue> & postures);
    void readOnePosture(uint id, Postrue & posturue);
    void refreshPosture(std::vector<Postrue> & postures);

    void pushRoadsign(Roadsign roadsign);
    void eraseRoadsign(uint remain_num);
    void readRoadsign(std::vector<Roadsign> & roadsigns);
    void refreshRoadsign(std::vector<Roadsign> & roadsigns);

    void pushFramefeature(Framefeature feature);
    void eraseFramefeature(uint remain_num);
    void readFramefeature(std::vector<Framefeature> & features);
    void readOneFramefeature(uint id, Framefeature & feature);
    void refreshFramefeature(std::vector<Framefeature> & features);

    void pushKeyframe(uint id);
    void eraseKeyframe(uint remain_num);
    void readKeyframe(std::vector<uint> & ids);
    void readLastKeyframe(uint & id);
    void refreshKeyframe(std::vector<uint> & ids);

private:

    static std::shared_ptr<Mappoint> mMappoint;
    
    std::vector<Postrue> mPostures;
    std::vector<Roadsign> mRoadsigns;
    std::vector<Framefeature> mFramefeatures;
    std::vector<uint> mKeyframe;

    boost::shared_mutex shr_mutex_posture;
    boost::shared_mutex shr_mutex_roadsign;
    boost::shared_mutex shr_mutex_frame;
    boost::shared_mutex shr_mutex_keyframe;
};





}


#endif