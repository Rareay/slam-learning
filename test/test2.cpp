#include <iostream>
#include<opencv2/opencv.hpp>
#include "trslam/config.h"
#include "trslam/dataset.h"
#include "trslam/map.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

int main()
{
//    std::string p = "./config/default2.yaml";
//    trslam::Config::SetParameterFile(p);
//    trslam::Dataset data;
//    cv::Mat img;
//    for (int i = 0; i < 100; i++) {
//        img = data.GetFrame();
//        cv::imshow("1", img);
//    }
{
    trslam::Map map;
    map.createMap();

    trslam::Postrue p1;
    Sophus::SE3d pose;
    p1.id = 0; 
    p1.pose = pose;
    map.pushPosture(p1);
    p1.id = 1; 
    map.pushPosture(p1);
    p1.id = 2; 
    map.pushPosture(p1);
}
{
    trslam::Map map;

    trslam::Postrue p1;
    Sophus::SE3d pose;
    p1.id = 0; 
    p1.pose = pose;
    map.pushPosture(p1);
    p1.id = 1; 
    map.pushPosture(p1);
    p1.id = 2; 
    map.pushPosture(p1);

    std::vector<trslam::Postrue> ps;
    map.readPostrue(ps);

    for (int i = 0; i < ps.size(); i++) {
        std::cout << ps[i].id << "\n" << ps[i].pose.matrix() << std::endl;
    }
}

    return 0;
}