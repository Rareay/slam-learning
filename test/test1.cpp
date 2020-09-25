#include "trslam/common_include.h"
#include "trslam/feature.h"
#include "trslam/frontend.h"

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

int main()
{
    cv::Mat img1 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000000.png");
    cv::Mat img2 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000001.png");
    cv::Mat img3 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000002.png");
    cv::Mat img4 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000003.png");
    cv::Mat img5 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000004.png");
    cv::Mat img6 = cv::imread("/home/tianru/slam/KITTI/dataset/sequences/01/image_0/000005.png");

    trslam::Frontend frontend(3, 100);
    frontend.FrontendCalculate(img1);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
    frontend.FrontendCalculate(img2);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
    frontend.FrontendCalculate(img3);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
    frontend.FrontendCalculate(img4);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
    frontend.FrontendCalculate(img5);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;
    frontend.FrontendCalculate(img6);
    std::cout << frontend.frontFrame_.id << ":\n" 
              << frontend.frontFrame_.pose.matrix() << "\n" << std::endl;

    //cv::imshow("1", img2);
    //cv::waitKey(0);

    return 0;
}

#if 0
// 光流 匹配特征点
void LightFlow(cv::Mat & img1, cv::Mat & img2, 
               std::vector<cv::Point2f> & keypoints_1,
               std::vector<cv::Point2f> & keypoints_2);

// 对级约束 计算位姿
void PoseEstimation_2d2d(std::vector<cv::Point2f> match_keypoints_1,
                         std::vector<cv::Point2f> match_keypoints_2,
                         cv::Mat &R, cv::Mat &t);


int main()
{
    PlotPosition plotpositions;
    plotpositions.Plot();

    cv::Mat R, t;

    std::string video_path = "/home/tianru/slam/slam-learning/data/video/demo3.mp4";
    cv::VideoCapture cap;


    cap.open(video_path);
    if(!cap.isOpened())
        return 0;
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);  //帧宽度
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //帧高度
    cv::Mat img1, img2;

    while (1) {
        cap >> img2;

        if (img2.empty()) break;

        cv::resize(img2, img2, cv::Size(width/3, height/3));
        if (img1.empty()) {
            img1 = img2.clone();
            continue;
        }

        std::vector<cv::Point2f> keypoint1;
        std::vector<cv::Point2f> keypoint2;
        LightFlow(img1, img2, keypoint1, keypoint2);
        cv::imshow("1", img1);
        img1 = img2.clone();

        cv::Mat R_temp, t_temp;
        PoseEstimation_2d2d(keypoint1, keypoint2, R_temp, t_temp);
        if (R.empty()) R = R_temp.clone();
        else R = R * R_temp;
        t_temp /= 12.;
        if (t.empty()) t = t_temp.clone();
        else t = t + t_temp;

        Eigen::Matrix3d m_R;
        m_R << R.at<cv::Vec3d>(0,0)[0],
              R.at<cv::Vec3d>(0,0)[1],
              R.at<cv::Vec3d>(0,0)[2],
              R.at<cv::Vec3d>(0,1)[0],
              R.at<cv::Vec3d>(0,1)[1],
              R.at<cv::Vec3d>(0,1)[2],
              R.at<cv::Vec3d>(0,2)[0],
              R.at<cv::Vec3d>(0,2)[1],
              R.at<cv::Vec3d>(0,2)[2];
        std::cout.precision(16);
        //std::cout << "\nR:" << R << "\nt:" << t << std::endl;
        Eigen::Quaterniond q = Eigen::Quaterniond(m_R); // R 的四元数
        //std::cout << q.coeffs() << std::endl;

        Eigen::Vector3d v_t;
        v_t << t.at<cv::Vec3d>(0,0)[0], // t 的向量
              t.at<cv::Vec3d>(0,0)[1],
              t.at<cv::Vec3d>(0,0)[2];
        //std::cout << v_t << std::endl;
        Eigen::Isometry3d Twr(q);
        Twr.pretranslate(v_t);
        plotpositions.m_positions.push_back(Twr);
        if (cv::waitKey(33) >= 0) break;
    }
    std::cout << "close video..." << std::endl;
    cap.release();

    return 0;
}

void LightFlow(cv::Mat & img1, cv::Mat & img2, 
               std::vector<cv::Point2f> & keypoints_1,
               std::vector<cv::Point2f> & keypoints_2)
{
    keypoints_1.clear();
    keypoints_2.clear();
    if (img1.empty() || img2.empty()) return;

    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1); // 从img1中提取特征点

    for (auto &kp: kp1) keypoints_1.push_back(kp.pt); // 改变特征点格式，从 KeyPoint 到 Point2f

    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, keypoints_1, keypoints_2,
                             status, error); // 计算光流

    for (int i = 0; i < keypoints_2.size(); i++) {
        if (status[i]) { // 如果存在对应光流点，就绘制下面的点、线
            cv::circle(img1, keypoints_1[i], 2, cv::Scalar(0, 255, 0), 2);
            cv::line(img1, keypoints_1[i], keypoints_2[i], cv::Scalar(0, 255, 0));
        }
    }
}

void PoseEstimation_2d2d(std::vector<cv::Point2f> match_keypoints_1,
                         std::vector<cv::Point2f> match_keypoints_2,
                         cv::Mat &R, cv::Mat &t)
{
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    ////-- 计算基础矩阵
    //cv::Mat fundamental_matrix;
    //fundamental_matrix = cv::findFundamentalMat(match_keypoints_1, 
    //                                            match_keypoints_2,
    //                                            CV_FM_8POINT);

    //-- 计算本质矩阵
    cv::Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;      //相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = findEssentialMat(match_keypoints_1, 
                                        match_keypoints_2, 
                                        focal_length, 
                                        principal_point);

   cv::recoverPose(essential_matrix, match_keypoints_1, match_keypoints_2, 
                   R, t, focal_length, principal_point);
}

#endif

