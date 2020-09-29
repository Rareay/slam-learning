/** @file frontend.h
 * @brief 前端
 * 
 * @author Tamray
 * @date 2020.09.24
 */

#ifndef TRSLAM_FRONTEND_H
#define TRSLAM_FRONTEND_H

#include "trslam/frame.h"
#include "trslam/map.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <unistd.h>

namespace trslam {

class Frontend {
public:
    Map mMap;   
    std::vector<uint> mRoasid;
    uint mRoasidMax = 0;
    uint id = 0;
    uint CacheFrameNum; // 缓存帧数
    uint FeaturePointNum; // 每帧特征点最大个数
    std::vector<trslam::Frame> CacheFrames;
    trslam::Frame frontFrame_; // 缓存的上一帧,也是输出的最新帧
    trslam::Frame frontFrame__;// 缓存的上上帧

    Frontend(int frame_num, int point_num);

    ~Frontend() {}

    /** @brief 前端计算
     * @param 
     */
    void FrontendCalculate(cv::Mat img);



    /** @brief 更新地图路标
     */
    void refreshRoasid(cv::Mat & img,
                       std::vector<cv::Point2f> & ptr1,
                       std::vector<cv::Point2f> & ptr2,
                       std::vector<uchar> & statue1,
                       std::vector<uchar> & statue2,
                       Sophus::SE3d pose);

    /** @brief 三角测量
     */
    Eigen::Vector3d calculateRoadsign(cv::Mat & img,
                                      cv::Point2f pt1,
                                      cv::Point2f pt2,
                                      Sophus::SE3d pose);

    /** @brief 更新地图帧特征
     */
    void refreshPosture(uint id, Sophus::SE3d pose);

    /** @brief 更新地图帧特征
     */
    void refreshFramefeature(uint id,
                             std::vector<cv::Point2f> & ptr1,
                             std::vector<uchar> & statue1);

    
    /** @brief 位姿计算
     * @param pt1 图像1的特征点
     * @param pt2 图像2的特征点
     * @param status 特征匹配信息
     * @param pt1 预测出的位姿
     */
    void estimatePose_2d2d(std::vector<cv::Point2f> & pt1,
                           std::vector<cv::Point2f> & pt2,
                           std::vector<uchar> & status,
                           Sophus::SE3d & pose1,
                           Sophus::SE3d & pose2); 

    /** 跟踪特征点
     */
    float trackFeaturePoints(cv::Mat & img1, cv::Mat & img2, 
                             std::vector<cv::Point2f> & pt1,
                             std::vector<cv::Point2f> & pt2,
                             std::vector<uchar> & status);


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
                           int threshold = 4000);

    /** @brief 添加跟踪点 
     * @param img 需要添加跟踪点的图像
     * @param pt 已有的特征点
     * @param status 各个特征点的状态（有效还是无效）
     * @param min_distance (可选)新点的最小距离
     */
    void addTrackedPoints(cv::Mat & img, std::vector<cv::Point2f> & pt,
                          std::vector<uchar> & status, int min_distance = 5);

    /** @brief 根据缓存帧过滤出有效跟踪点
     */
    void flterTrackedPoints();

private:

};

}


#endif