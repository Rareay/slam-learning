/** @file dataset.cpp
 * @brief 读取数据集
 * 
 * @author Tamray
 * @date 2020.09.27
 */

#include "trslam/common_include.h"
#include "trslam/dataset.h"


namespace trslam {


Dataset::Dataset()
{
    std::string type = Config::Get<std::string>("data_type");
    if (type == "video") {
        mDateType = DATA_VIDEO;
        std::string path = Config::Get<std::string>("video_path");
        if (path == "") {
            mDateType = -1;
            std::cout << "配置文件缺少视频地址！" << std::endl;
            exit(0);
            return ;
        }
        mCap.open(path);
        if (!mCap.isOpened()) {
            std::cout << "视频无法打开！" << std::endl;
            exit(0);
            return ;
        }
        int w = Config::Get<int>("video_w");
        int h = Config::Get<int>("video_h");
        int rate = Config::Get<int>("video_rate");
        mVideoW = w == 0 ? 900 : w;
        mVideoH = h == 0 ? 500 : h;
        mVideoRate = rate == 0 ? 1 : rate;

    } else if (type == "picture") {
        mDateType = DATA_PICTURE;
        std::string path = Config::Get<std::string>("picture_path");
        if (path == "") {
            mDateType = -1;
            std::cout << "配置文件缺少图片地址！" << std::endl;
            exit(0);
            return ;
        }
        mPicturePath = path;
        int rate = Config::Get<int>("picture_rate");
        int index_min = Config::Get<int>("picture_index_min");
        int index_max = Config::Get<int>("picture_index_max");
        mPictureRate = rate == 0 ? 1 : rate;
        mPictureIndexMin = index_min == 0 ? 0 : index_min;
        mPictureIndexMax = index_max == 0 ? 0 : index_max;
        mPictureIndex = mPictureIndexMin;

    } else {
        std::cout << "配置文件缺少数据描述！" << std::endl;
        exit(0);
    }
}

Dataset::~Dataset()
{
    mCap.release();
}

cv::Mat Dataset::GetFrame()
{
    cv::Mat img;
    if (mDateType == DATA_VIDEO) {
        GetVideoFrame(img);
    } else if (mDateType == DATA_PICTURE) {
        GetPictureFrame(img);
    }

    return img;
}

void Dataset::GetVideoFrame(cv::Mat &img)
{
    mCap >> img;
    cv::waitKey(mVideoRate);
    if (!img.empty()) {
        cv::resize(img, img, cv::Size(mVideoW, mVideoH));
    }
}

void Dataset::GetPictureFrame(cv::Mat &img)
{
    boost::format fmt("%s/%06d.png");
    std::string path = (fmt % mPicturePath % mPictureIndex).str();
    mPictureIndex++;
    if (mPictureIndex > mPictureIndexMax) return ;
    img = cv::imread(path);
    cv::waitKey(mPictureRate);
}


}