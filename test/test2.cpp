#include <iostream>
#include<opencv2/opencv.hpp>
#include "trslam/config.h"
#include "trslam/dataset.h"
#include "trslam/map.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

#include <opencv2/core/eigen.hpp>
int main()
{
    cv::Mat img = (cv::Mat_<float>(4,1) << 1.,2.,3.,4.);
    Vec3 v;
    cv::cv2eigen(img, v);
    std::cout << img << std::endl;
    std::cout << v.matrix() << std::endl;
    return 0;
}