
#ifndef TRSLAM_CAMERA_H
#define TRSLAM_CAMERA_H

#include "trslam/common_include.h"
#include <boost/thread.hpp>



namespace trslam {




class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    static void setCameraParam(double fx, double fy, double cx, double cy, 
                               double baseline, const SE3 pose);

    void setPose(const SE3 &pose);

    SE3 getPose() const { return pose_; }

    Mat33 getK() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

    
    cv::Point2f pixel2camera( const cv::Point2d & p);

private:
    static std::shared_ptr<Camera> mCamera;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0;  // Camera intrinsics
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics

 };





}

#endif