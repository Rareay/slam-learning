
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>

// 光流 匹配特征点
void LightFlow(cv::Mat & img1, cv::Mat & img2, 
               std::vector<cv::Point2f> & keypoints_1,
               std::vector<cv::Point2f> & keypoints_2);

// 对级约束 计算位姿
void PoseEstimation_2d2d(std::vector<cv::Point2f> match_keypoints_1,
                         std::vector<cv::Point2f> match_keypoints_2,
                         cv::Mat &R, cv::Mat &t);

// 绘制路径
void DrawTrajectory(std::vector<Eigen::Isometry3d, 
                                Eigen::aligned_allocator<Eigen::Isometry3d>> poses);

int main()
{
//    std::string pic_path = "/home/tianru/slam/slambook2-master/data/picture/demo1/";
//    std::vector<std::string> pictures;
//    for (int i = 0; i < 95; i++) {
//        std::string p = pic_path + std::to_string(i) + ".png";
//        pictures.push_back(p);
//    }
//
//    for (int i = 0; i < pictures.size() - 1; i++) {
//        cv::Mat img1, img2;
//        img1 = cv::imread(pictures[i], 1);
//        img2 = cv::imread(pictures[i+1], 1);
//        cv::Mat output = LightFlow(img1, img2);
//        cv::imshow("1", output);
//        cv::waitKey(500);
//    }
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    cv::Mat R, t;

    std::string video_path = "/home/tianru/slam/slambook2-master/data/demo1_0.mp4";
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
        poses.push_back(Twr);

		if (cv::waitKey(33) >= 0) break;
	}
    std::cout << "close video..." << std::endl;
	cap.release();

    std::cout << "start plot..." << std::endl;
    DrawTrajectory(poses);

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

void DrawTrajectory(std::vector<Eigen::Isometry3d, 
                                Eigen::aligned_allocator<Eigen::Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Eigen::Vector3d Ow = poses[i].translation();
      Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
      Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
      Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size()-1; i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}