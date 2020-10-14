
#ifndef TRSLAM_BACKEND_H
#define TRSLAM_BACKEND_H

#include "trslam/common_include.h"
#include "trslam/g2o_types.h"
#include "trslam/mappoint.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace trslam {


class Backend {
public:
    Backend(uint cameras_nums) { mCameras = cameras_nums; }

    int startOptimization() {
        boost::function0<void> f = boost::bind(&Backend::doOptimization, this);
        m_thread_opti = new boost::thread(f);
        return 0;
    }

private:
    void SolveBA()
    {
        unsigned int t_start = boost::get_system_time().time_of_day().total_milliseconds();
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        // use LM
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        // vertex
        std::map<uint, VertexPoseAndIntrinsics *> vertex_pose_intrinsics;
        std::map<uint, VertexPoint *> vertex_points; 

        std::vector<KeyFrame> keyframes;
        mMappoint.readKeyFrame(keyframes);
        if (keyframes.size() < mCameras) return;

        //for (uint i = 0; i < keyframes.size(); i++) {
        //    std::cout << keyframes[i].T.matrix3x4() << std::endl;
        //}
        uint index_start = keyframes.size() - mCameras + 1;
        uint index_end   = keyframes.size();
        uint keyframes_id_max = 0;
        for (uint i = 0; i < keyframes.size(); i++) {
            keyframes_id_max = keyframes_id_max > keyframes[i].id ? 
                               keyframes_id_max : keyframes[i].id + 1; 
        }

        for (uint i = index_start; i < index_end; i++) {
            for (uint j = 0; j < keyframes[i].ptr_status.size(); j++) {
                if (keyframes[i].ptr_status[j]) {
                    VertexPoint * p = nullptr;
                    vertex_points.insert( // 插入
                        std::map<uint, VertexPoint *>::value_type(
                        keyframes_id_max + keyframes[i].ptr_rods[j], p));
                }
            }
        }
        std::map<uint, VertexPoint *>::iterator iter;
        for (iter = vertex_points.begin(); iter != vertex_points.end(); iter++) {
            // 路标顶点
            VertexPoint *v_point = new VertexPoint();
            Roadsign roadsign;
            mMappoint.readOneRoadsign(iter->first - keyframes_id_max, roadsign);
            v_point->setId(iter->first);
            v_point->setEstimate(roadsign.rods);
            v_point->setMarginalized(true);
            optimizer.addVertex(v_point);
            iter->second = v_point;
        }

        for (uint i = index_start; i < index_end; i++) { // 遍历需要优化的关键帧
            // 位姿顶点
            VertexPoseAndIntrinsics *v_carmera = new VertexPoseAndIntrinsics();
            v_carmera->setId(keyframes[i].id); // 设置相机id
            v_carmera->setEstimate(PoseAndIntrinsics(keyframes[i].T));
            optimizer.addVertex(v_carmera);
            vertex_pose_intrinsics.insert(
                std::map<uint, VertexPoseAndIntrinsics *>::value_type(
                keyframes[i].id, v_carmera));

            for(uint j = 0; j < keyframes[i].ptr_status.size(); j++) {
                if (keyframes[i].ptr_status[j]) {
                    // 边，即像素坐标
                    EdgeProjection * edge = new EdgeProjection();

                    std::map<uint, VertexPoint *>::iterator it;
                    it = vertex_points.find(keyframes[i].ptr_rods[j]);
                    if (it == vertex_points.end()) continue;

                    edge->setVertex(0, v_carmera);
                    edge->setVertex(1, it->second);
                    Vec2 pixel;
                    pixel << keyframes[i].ptr[j].x, keyframes[i].ptr[j].y;
                    edge->setMeasurement(pixel);
                    edge->setInformation(Mat22::Identity());
                    edge->setRobustKernel(new g2o::RobustKernelHuber());
                    optimizer.addEdge(edge);
                }
            }
        }

        optimizer.initializeOptimization();
        optimizer.optimize(40);

        // 用优化的数据更新原数据
        std::vector<Roadsign> roadsigns;
        mMappoint.readRoadsign(roadsigns);
        std::map<uint, VertexPoint *>::iterator iter_road;
        for (iter_road = vertex_points.begin(); iter_road != vertex_points.end(); iter_road++) {
            for (uint i = 0; i < roadsigns.size(); i++) {
                if (roadsigns[i].id == iter_road->first - keyframes.size()) {
                    roadsigns[i].rods = iter_road->second->estimate();
                    //std::cout << "路标" << roadsigns[i].id << ": " << roadsigns[i].rods << std::endl;
                }
            }
        }
        mMappoint.refreshRoadsign(roadsigns);

        std::map<uint, VertexPoseAndIntrinsics *>::iterator iter_carmer;
        for (iter_carmer = vertex_pose_intrinsics.begin(); 
             iter_carmer != vertex_pose_intrinsics.end(); 
             iter_carmer++) {
            for (uint i = 0; i < keyframes.size(); i++) {
                if (keyframes[i].id == iter_carmer->first) {
                    SE3 T;
                    PoseAndIntrinsics pose;
                    pose = iter_carmer->second->estimate();
                    pose.set_to(T);
                    keyframes[i].T = T;
                    //std::cout << "位姿" << keyframes[i].id << ": \n" <<keyframes[i].T.matrix3x4() << std::endl;
                }
            }
        }
        last_keyframe_id = keyframes[keyframes.size()-1].id;
        unsigned int t_end = boost::get_system_time().time_of_day().total_milliseconds();
        std::cout << "Save BA use " << t_end - t_start << " ms." << std::endl;
        mMappoint.refreshKeyFrame(keyframes);


    }
    void doOptimization() {
        boost::try_mutex::scoped_try_lock lock(m_iomutex);
        if (lock.owns_lock()) {
            while (1) {
                KeyFrame keyframe;
                mMappoint.readLastKeyFrame(keyframe);
                if (keyframe.id != last_keyframe_id) {
                    SolveBA();
                }
                boost::thread::sleep(boost::get_system_time()
                             + boost::posix_time::milliseconds(10));
            }
        } else {
            std::cout << "创建优化线程失败" << std::endl;
            boost::thread::yield();
        }
    }

private:
    Mappoint mMappoint;
    uint mCameras;
    uint last_keyframe_id = 0;

    boost::thread * m_thread_opti = nullptr;
    boost::try_mutex m_iomutex;
};


















}


#endif