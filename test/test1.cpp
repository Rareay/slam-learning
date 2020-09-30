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
    SE3 p;
    trslam::Camera::setCameraParam(520.9, 512.0, 325.1, 249.7, 0., p);
    trslam::Mappoint::createMappoint();
    trslam::Dataset data;

    trslam::Viewer viewer;
    viewer.ViewPositon();
    
    trslam::Frontend frontend(3, 100);

    cv::Mat img;
    while (1) {
        img = data.GetFrame();
        if (img.empty()) break;

        frontend.FrontendCalculate(img);
    }
    std::cout << "over" << std::endl;
    return 0;
}


