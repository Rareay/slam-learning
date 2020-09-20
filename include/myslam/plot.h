#include <iostream>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void DrawTrajectory(std::vector<Eigen::Isometry3d, 
                                Eigen::aligned_allocator<Eigen::Isometry3d>> poses);

class PlotPosition
{
public:
    ~PlotPosition();
    std::vector<Eigen::Isometry3d,
                Eigen::aligned_allocator<Eigen::Isometry3d>> m_positions;
    void Plot();
private:
    boost::thread * m_thread_plot;
    boost::try_mutex m_iomutex;
    void StartPlot();

};
 
