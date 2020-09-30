
#include "trslam/camera.h"


namespace trslam {

std::shared_ptr<Camera> Camera::mCamera = nullptr;

void Camera::setCameraParam(double fx, double fy, double cx, double cy, 
                            double baseline, const SE3 pose) {
    if (mCamera == nullptr) {
        mCamera = std::shared_ptr<Camera>(new Camera);
    }
    Camera::mCamera->fx_ = fx;
    Camera::mCamera->fy_ = fy;
    Camera::mCamera->cx_ = cx;
    Camera::mCamera->cy_ = cy;
    Camera::mCamera->baseline_ = baseline;
    Camera::mCamera->pose_ = pose;
    Camera::mCamera->pose_inv_ = pose.inverse();
}

void Camera::setPose(const SE3 &pose) 
{
    Camera::mCamera->pose_ = pose;
    Camera::mCamera->pose_inv_ = pose.inverse();
}


Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    return Camera::mCamera->pose_ * T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * Camera::mCamera->pose_inv_ * p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(
            Camera::mCamera->fx_ * p_c(0, 0) / p_c(2, 0) + Camera::mCamera->cx_,
            Camera::mCamera->fy_ * p_c(1, 0) / p_c(2, 0) + Camera::mCamera->cy_
    );
}

Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    return Vec3(
            (p_p(0, 0) - Camera::mCamera->cx_) * depth / Camera::mCamera->fx_,
            (p_p(1, 0) - Camera::mCamera->cy_) * depth / Camera::mCamera->fy_,
            depth
    );
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}




cv::Point2f Camera::pixel2camera(const cv::Point2d& p )
{
    return cv::Point2f
    (
        ( p.x - Camera::mCamera->cx_ ) / Camera::mCamera->fx_, 
        ( p.y - Camera::mCamera->cy_ ) / Camera::mCamera->fy_ 
    );
}



}