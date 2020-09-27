/** @file config.cpp
 * @brief 读取 yaml 配置文件
 * 
 * @author Tamray
 * @date 2020.09.27
 */

#include "trslam/config.h"


namespace trslam {

bool Config::SetParameterFile(const std::string &filename) {
    if (mConfig == nullptr)
        mConfig = std::shared_ptr<Config>(new Config);
    mConfig->mFile = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (mConfig->mFile.isOpened() == false) {
        mConfig->mFile.release();
        cv::FileStorage write(filename.c_str(), cv::FileStorage::WRITE);
        write<<"param_int"<<0;
        write<<"p_float"<<1.1;
        write<<"p_str"<<"abc";
        write<<"p_Mat"<<cv::Mat::ones(3,3,CV_8U);
        write.release();
        std::cout << "文件 " << filename << " 不存在，但创建了该文件。" << std::endl;
        return false;
    }
    return true;
}

Config::~Config() {
    if (mFile.isOpened())
        mFile.release();
}

std::shared_ptr<Config> Config::mConfig = nullptr;


}