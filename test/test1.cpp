#include "trslam/common_include.h"
#include "trslam/feature.h"
#include "trslam/frontend.h"
#include "trslam/viewer.h"
#include "trslam/config.h"
#include "trslam/dataset.h"

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

int main()
{
    trslam::Config::SetParameterFile("./config/default2.yaml");
    trslam::Dataset data;

    //std::vector<Sophus::SE3d> poses;
    Sophus::SE3d current_pose;
    trslam::Viewer viewer;
    viewer.ViewPositon();
    
    trslam::Frontend frontend(3, 500);

    cv::Mat img;
    while (1) {
        img = data.GetFrame();
        if (img.empty()) break;

        frontend.FrontendCalculate(img);
        //std::cout << frontend.frontFrame_.id << ":\n" 
        //        << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
        current_pose = frontend.frontFrame_.pose * current_pose;
        viewer.m_positions.push_back(current_pose);
        
        if (frontend.frontFrame__.id != -1) {
            cv::Mat img = frontend.frontFrame_.image.clone();
            std::vector<cv::Point2f> pt1 = frontend.frontFrame__.feature;
            std::vector<cv::Point2f> pt2 = frontend.frontFrame_.feature;
            std::vector<uchar> s = frontend.frontFrame_.feature_match;
            for (int i = 0; i < s.size(); i++) {
                if (s[i]) {
                    cv::circle(img,pt2[i], 1, cv::Scalar(50, 0, 200), 2);
                    cv::line(img, pt1[i],  pt2[i], cv::Scalar(0, 255, 0));
                }
            }
            cv::imshow("1", img);
            if (cv::waitKey(100) >= 0) break;
        }
    }
    std::cout << "over" << std::endl;
    //cv::imshow("1", img2);
    //cv::waitKey(0);
    return 0;
}


