/** @file dataset.h
 * @brief 读取数据集
 * 
 * @author Tamray
 * @date 2020.09.27
 */

#ifndef TRSLAM_DATESET_H
#define TRSLAM_DATESET_H

#include "trslam/common_include.h"
#include "trslam/config.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/format.hpp>

#define DATA_VIDEO 0
#define DATA_PICTURE 1

namespace trslam {


/** @brief 数据集的读取类
 * 
 * 根据 yaml 文件的配置，读取视频数据或图片数据
 * 
 * @note 使用该类前，一定要先初始化 trslam::Config，见 ./include/trslam/config.h
 */
class Dataset {
public:
    /** @brief 获取一帧
     */
    cv::Mat GetFrame();
    Dataset();
    ~Dataset();

private:
    int mDateType = -1;
    cv::VideoCapture mCap;
    int mVideoW = 0;
    int mVideoH = 0;
    int mVideoRate = 0;
    std::string mPicturePath;
    int mPictureRate = 0;
    int mPictureIndexMin = 0;
    int mPictureIndexMax = 0;
    int mPictureIndex = 0;

    void GetVideoFrame(cv::Mat &img);
    void GetPictureFrame(cv::Mat &img);
};

}
#endif