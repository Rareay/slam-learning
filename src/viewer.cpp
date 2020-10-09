/** @file viewer.cpp
 * @brief openGL 三维视图，显示位姿、路标等
 * 
 * @author Tamray
 * @date 2020.09.25
 */

#include "trslam/common_include.h"
#include "trslam/viewer.h"
#include "trslam/config.h"

namespace trslam {




void Viewer::PlotPostion()
{
    boost::try_mutex::scoped_try_lock  lock(m_iomutex);//锁定mutex
    if (lock.owns_lock())
    {
        std::cout << "Start Plot Positions..." << std::endl;
        pangolin::CreateWindowAndBind("Position Viewer", 1024, 768);
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
            usleep(50000);   // sleep 50 ms

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(2);

            std::vector<KeyFrame> keyframe;
            mappoint.readKeyFrame(keyframe);

            std::vector<Roadsign> roadsigns;
            mappoint.readRoadsign(roadsigns);

            //std::cout << roadsigns.size() << "-----------" << keyframe.size() << std::endl;
            if (roadsigns.empty() || keyframe.empty()) continue;

            // 绘制世界坐标系
            glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 0.0);
            glVertex3d(0, 0, 0);
            glVertex3d(0, 0, 10);
            glColor3f(0.0, 1.0, 1.0);
            glVertex3d(0, 0, 0);
            glVertex3d(0, 10, 0);
            glColor3f(1.0, 0.0, 1.0);
            glVertex3d(0, 0, 0);
            glVertex3d(10, 0, 0);
            glEnd();

            // 绘制点
            int showrods = Config::Get<int>("showrods");
            if (showrods != 0) {
                glPointSize(5);
                glColor3f(1.0, 0.0, 0.0);
                for (uint i = 0; i < roadsigns.size(); i++) {
                    Vec3 r = roadsigns[i].rods;
                    glBegin(GL_POINTS);
                    glVertex3d(r.matrix()[0], r.matrix()[1], r.matrix()[2]);
                    glEnd();
                    //std::cout << roadsigns.size() << " " << r.matrix() << std::endl;
                }
            }


            // 绘制位姿
            for (size_t i = 0; i < keyframe.size(); i++) {
                SE3 se3 = keyframe[i].T;
                // 画每个位姿的三个坐标轴
                //Eigen::Vector3d Ow  = m_positions[i].translation();
                Eigen::Vector3d Xw  = se3 * (0.1 * Eigen::Vector3d(-1, 0, 0));
                Eigen::Vector3d Yw_ = se3 * (0.1 * Eigen::Vector3d(0, 2, 0));
                Eigen::Vector3d Yw  = se3 * (0.1 * Eigen::Vector3d(0, -2, 0));
                Eigen::Vector3d Zw  = se3 * (0.1 * Eigen::Vector3d(0, 0, 2));
                Eigen::Vector3d Zw_ = se3 * (0.1 * Eigen::Vector3d(0, 0, -2));
                glBegin(GL_LINES);
                glColor3f(1.0, 0.0, 0.0);
                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glVertex3d(Yw[0], Yw[1], Yw[2]);

                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glVertex3d(Yw_[0], Yw_[1], Yw_[2]);

                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glVertex3d(Zw[0], Zw[1], Zw[2]);

                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glVertex3d(Zw_[0], Zw_[1], Zw_[2]);

                glColor3f(0.0, 1.0, 0.0);
                glVertex3d(Yw[0], Yw[1], Yw[2]);
                glVertex3d(Zw[0], Zw[1], Zw[2]);

                glVertex3d(Zw[0], Zw[1], Zw[2]);
                glVertex3d(Yw_[0], Yw_[1], Yw_[2]);

                glVertex3d(Yw_[0], Yw_[1], Yw_[2]);
                glVertex3d(Zw_[0], Zw_[1], Zw_[2]);

                glVertex3d(Zw_[0], Zw_[1], Zw_[2]);
                glVertex3d(Yw[0], Yw[1], Yw[2]);
                glEnd();
            }
            // 画出连线
            for (size_t i = 0; i < keyframe.size()-1; i++) {
                glColor3f(0.0, 0.0, 0.0);
                glBegin(GL_LINES);
                auto p1 = keyframe[i].T, p2 = keyframe[i + 1].T;
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
            pangolin::FinishFrame();
        }
    }
    else
    {
        std::cout << "创建绘制位姿线程失败" << std::endl;
        boost::thread::yield(); //释放控制权
    }
}


}