/** @file frame.h
 * @brief 帧信息
 * 
 * @author Tamray
 * @date 2020.09.26
 */

#ifndef TRSLAM_FRAME_H
#define TRSLAM_FRAME_H

#include "trslam/common_include.h"

namespace trslam {

class Frame {
public:
    int id = -1;// id
    cv::Mat image;               // 图像
    std::vector<cv::Point2f> feature; // 特征点
    std::vector<uchar> feature_match;   // 和上一帧的匹配状态
    SE3 pose;           // 位姿

private:
};

}


#endif