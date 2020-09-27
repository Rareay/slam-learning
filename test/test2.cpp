#include <iostream>
#include<opencv2/opencv.hpp>
#include "trslam/config.h"
#include "trslam/dataset.h"


int main()
{
    std::string p = "./config/default2.yaml";
    trslam::Config::SetParameterFile(p);
    trslam::Dataset data;
    cv::Mat img;
    for (int i = 0; i < 100; i++) {
        img = data.GetFrame();
        cv::imshow("1", img);
    }

    return 0;
}