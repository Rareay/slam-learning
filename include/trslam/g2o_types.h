/** @file g2o_types.h
 * @brief g2o 的相关数据类型
 * 
 * @author Tamray
 * @date 2020.10.10
 */

#ifndef TRSLAM_G2O_TYPES_H
#define TRSLAM_G2O_TYPES_H

#include "trslam/common_include.h"
#include "trslam/mappoint.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>



namespace trslam {



/// 姿态和内参的结构
struct PoseAndIntrinsics {
    PoseAndIntrinsics() {}

    /// set from given data address
    explicit PoseAndIntrinsics(SE3 pose) {
        rotation = pose.so3();
        translation = pose.translation();
    }

    /// 将估计值放入内存
    void set_to(SE3 &pose) {
        SE3 T(rotation, translation);
        pose = T;
    }

    SO3 rotation;
    Vec3 translation = Vec3::Zero();
};







/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
class VertexPoseAndIntrinsics : public g2o::BaseVertex<6, PoseAndIntrinsics> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoseAndIntrinsics() {}

    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    }

    virtual void oplusImpl(const double *update) override {
        _estimate.rotation = SO3::exp(Vec3(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Vec3(update[3], update[4], update[5]);
    }


    /// 根据估计值投影一个点
    Vec2 project(const Vec3 &point) {
        Vec3 pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2];
        return Vec2(pc[0], pc[1]);
    }

    virtual bool read(std::istream &in) { return true; }

    virtual bool write(std::ostream &out) const { return true; }
};




class VertexPoint : public g2o::BaseVertex<3, Vec3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual void setToOriginImpl() override {
        _estimate = Vec3(0, 0, 0);
    }

    virtual void oplusImpl(const double *update) override {
        _estimate += Vec3(update[0], update[1], update[2]);
    }

    virtual bool read(std::istream &in) { return true; }

    virtual bool write(std::ostream &out) const { return true; }
};






class EdgeProjection :
    public g2o::BaseBinaryEdge<2, Vec2, VertexPoseAndIntrinsics, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
        auto v1 = (VertexPoint *) _vertices[1];
        auto proj = v0->project(v1->estimate());
        _error = proj - _measurement;
    }

    // use numeric derivatives
    virtual bool read(std::istream &in) { return true; }

    virtual bool write(std::ostream &out) const { return true; }

};






}


#endif