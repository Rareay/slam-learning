/** @file map.h
 * @brief 地图
 * 
 * @author Tamray
 * @date 2020.09.23
 */

#ifndef TRSLAM_MAP_H
#define TRSLAM_MAP_H

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
    unsigned int id;
    Sophus::SE3d pose;
};

/** @brief 路标
 */
struct Roadsign
{
    unsigned int id;
    Eigen::Vector3d rods;
};

/** @brief 帧
 */
struct Frameinfo
{
    unsigned int id;
    std::vector<cv::Point2f> ptr; // 特征点
    std::vector<unsigned int> ptr_rods; // 特征点对应路标id
};


class Map {
public:
    static void createMap();

    void pushPosture(Postrue posture);
    void erasePosture(int remain_num);
    void readPostrue(std::vector<Postrue> & postures);
    void refreshPosture(std::vector<Postrue> & postures);

    void pushRoadsign(Roadsign roadsign);
    void eraseRoadsign(int remain_num);
    void readRoadsign(std::vector<Roadsign> & roadsigns);
    void refreshRoadsign(std::vector<Roadsign> & roadsigns);

    void pushFrame(Frameinfo frame);
    void eraseFrame(int remain_num);
    void readFrame(std::vector<Frameinfo> & frames);
    void refreshFrame(std::vector<Frameinfo> & frames);

    void pushKeyframe(unsigned int id);
    void eraseKeyframe(int remain_num);
    void readKeyframe(std::vector<unsigned int> & ids);
    void refreshKeyframe(std::vector<unsigned int> & ids);

    Eigen::Vector4d readParam();

private:

    static std::shared_ptr<Map> mMap;
    
    std::vector<Postrue> mPostures;
    std::vector<Roadsign> mRoadsigns;
    std::vector<Frameinfo> mFrame;
    std::vector<unsigned> mKeyframe;

    Eigen::Vector4d mParam; // 相机内参

    boost::shared_mutex shr_mutex_posture;
    boost::shared_mutex shr_mutex_roadsign;
    boost::shared_mutex shr_mutex_frame;
    boost::shared_mutex shr_mutex_keyframe;
};





}


#endif