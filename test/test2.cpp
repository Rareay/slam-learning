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
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q(R);
    Eigen::Vector3d t(1, 0, 0);
    Sophus::Sim3d sim3(q, t);
    std::cout << sim3.matrix() << std::endl << std::endl;
    sim3.setScale(0.1);
    td::cout << sim3.matrix() << std::endl << std::endl;

    return 0;
}