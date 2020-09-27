/** @file viewer.h
 * @brief openGL 三维视图，显示位姿、路标等
 * 
 * @author Tamray
 * @date 2020.09.25
 */

#ifndef TRSLAM_VIEWER_H
#define TRSLAM_VIEWER_H

#include "trslam/common_include.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <Eigen/Core>

#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sophus/se3.hpp>

namespace trslam {

class Viewer {
public:
    //Viewer(std::vector<Eigen::Isometry3d> & pose) {
    Viewer() {
        //m_positions = pose;
    }
    ~Viewer() {
        if (m_thread_plot != nullptr) {
            m_thread_plot->interrupt();
            m_thread_plot->join();
        }
    }

    // 绘图
    int ViewPositon() {
        boost::function0<void> f = boost::bind(&Viewer::PlotPostion, this);
        m_thread_plot = new boost::thread(f);
        return 0;
    }
    std::vector<Sophus::SE3d> m_positions;

private:
    // 位姿数据
    ///std::vector<Eigen::Isometry3d> m_positions;
    // 路标数据

    boost::thread * m_thread_plot = nullptr;
    boost::try_mutex m_iomutex;
    // 开始绘图
    void PlotPostion();

};

}

#endif