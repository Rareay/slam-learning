/** @file config.h
 * @brief 读取 yaml 配置文件
 * 
 * @author Tamray
 * @date 2020.09.27
 */


#ifndef TRSLAM_CONFIG_H
#define TRSLAM_CONFIG_H

#include "trslam/common_include.h"
#include <memory>

namespace trslam {

/**
 * @brief 配置类
 *
 * @code 
   trslam::Config::SetParameterFile("./config/default.yaml");
   int a;
   a = trslam::Config::Get<int>("param");
   @endcode
 */
class Config {
   private:
    static std::shared_ptr<Config> mConfig;
    cv::FileStorage mFile;

   public:
    Config() {} 
    ~Config();

    /** @brief 设置文件路径
     */
    static bool SetParameterFile(const std::string &filename);

    /** @brief 读取值
     */
    template <typename T>
    static T Get(const std::string &key) {
        return T(Config::mConfig->mFile[key]);
    }
};

}  // namespace trslam

#endif