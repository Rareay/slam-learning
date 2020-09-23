#include "trslam/common_include.h"
#include "trslam/viewer.h"

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
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(2);

            if (m_positions.empty()) continue;

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

            for (size_t i = 0; i < m_positions.size(); i++) {
                // 画每个位姿的三个坐标轴
                Eigen::Vector3d Ow = m_positions[i].translation();
                Eigen::Vector3d Xw = m_positions[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
                Eigen::Vector3d Yw = m_positions[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
                Eigen::Vector3d Zw = m_positions[i] * (0.1 * Eigen::Vector3d(0, 0, 1));
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
            for (size_t i = 0; i < m_positions.size()-1; i++) {
                glColor3f(0.0, 0.0, 0.0);
                glBegin(GL_LINES);
                auto p1 = m_positions[i], p2 = m_positions[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
            pangolin::FinishFrame();
            usleep(5000);   // sleep 5 ms
        }
    }
    else
    {
        std::cout << "创建绘制位姿线程失败" << std::endl;
        boost::thread::yield(); //释放控制权
    }
}


}