//
// Created by xiang on 2021/11/11.
//

#include "static_imu_init.h"

namespace sad {

bool StaticIMUInit::AddIMU(const ORB_SLAM3::IMU::Point& imu)
{
    if (init_success_)
    {
      return true;
    }
    if (options_.use_speed_for_static_checking_ && !is_static_)
    {
        std::cout  << "等待车辆静止" << "\n";
        init_imu_deque_.clear();
        return false;
    }
    if (init_imu_deque_.empty()) 
    {
        // 记录初始静止时间
        init_start_time_ = imu.t;
    }
    // 记入初始化队列
    init_imu_deque_.push_back(imu);
    // 初始化经过时间
    double init_time = imu.t - init_start_time_;
    std::cout << "init_time= " << init_time << "Dont move !!! \n";
    if (init_time > options_.init_time_seconds_) 
    {
      // 尝试初始化逻辑
      TryInit();
      init_imu_deque_.clear();
    }

    // 维持初始化队列长度
    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_)
    {
      init_imu_deque_.pop_front();
    }

    current_time_ = imu.t;
    return false;
}

//bool StaticIMUInit::AddOdom(const Odom& odom) {
//    // 判断车辆是否静止
//    if (init_success_) {
//        return true;
//    }
//
//    if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
//        is_static_ = true;
//    } else {
//        is_static_ = false;
//    }
//
//    current_time_ = odom.timestamp_;
//    return true;
//}

// 尝试初始化
bool StaticIMUInit::TryInit() 
{
    if (init_imu_deque_.size() < 10) 
    {
      return false;
    }
    // （1）计算陀螺仪和加速度计的测量均值
    Eigen::Vector3f mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(init_imu_deque_,mean_acce,mean_gyro);
    std::cout << "mean acce: " << mean_acce.transpose() << "\n";

    // （2）计算Imu下重力向量：以acce均值为方向（静止时测得的是 -g），取 9.8长度为重力
    gravity_Imu = -mean_acce / mean_acce.norm() * options_.gravity_norm_;
    for (auto &data : init_imu_deque_)
    {
      data.a += gravity_Imu;
    }
    // （4）估计加速度计和陀螺仪测量的零偏：去掉重力的影响，重新计算加速度计的均值 即可得到bias
    ComputeMeanAndCovDiag(init_imu_deque_,mean_acce,mean_gyro);
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    // (5) 世界坐标系下重力方向的定义
    Eigen::Vector3f gravInWorldNormed(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f gravInWorld(0.0f, 0.0f, -9.81f);

    // (6) 计算重力在相机系下的表示
    Eigen::Matrix3f Tbc ;
    Tbc << -0.00328703, 0.99994907, 0.00954209,
        0.99991735, 0.00316802, 0.01245992,
        0.01242906, 0.00958226, -0.99987684;
    Eigen::Vector3f gravInCam = Tbc.transpose() * gravity_Imu;
    Eigen::Vector3f gravInCamNormed = gravInCam / gravInCam.norm();

    // (7) 通过相机坐标系下的重力方向和真实坐标系下的重力方向，估计相机在真实世界系下的姿态 Rcw
    // 叉乘求转轴模长
    Eigen::Vector3f v = gravInWorldNormed.cross(gravInCamNormed);
    const float nv = v.norm();
    // 求转角大小
    const float cosg = gravInWorldNormed.dot(gravInCamNormed);
    const float ang = acos(cosg);
    // v/nv 表示垂直于两个向量的轴  ang 表示转的角度，组成角轴
    Eigen::Vector3f vzg = v*ang/nv;
    std::cout << "gravInWorldNormed = " << gravInWorldNormed.transpose()
              << "\n nv = " << nv << ", v " << v.transpose() << "\n";
    Eigen::Matrix<float,3,3> Rcw_ = Sophus::SO3f::exp(vzg).matrix();

    std::cout << "IMU 静止初始化成功，初始化时间= " << current_time_ - init_start_time_
              << "\nbg = " << init_bg_.transpose()
              << "\nba = " << init_ba_.transpose()
              << "\ngravInImu = " << gravity_Imu.transpose()
              << "\ngravInCam = " << gravInCam.transpose()
              << "\nnorm: " << gravity_Imu.norm() << "\n"
              << "\n 验证 ：Rcw_ = \n" << Rcw_
              << "\n 验证：(Rcw_*gravInWorld ?= gravInCam) = \n"<<  Rcw_ * gravInWorld << "\n";
    init_success_ = true;
    return true;
}

void StaticIMUInit::ComputeMeanAndCovDiag(const std::deque<ORB_SLAM3::IMU::Point> &imu_data,
                                          Eigen::Vector3f &mean_acc,
                                          Eigen::Vector3f &mean_gyro)
{
  size_t len = imu_data.size();
  for (auto imu : imu_data)
  {
    mean_acc += imu.a;
    mean_gyro += imu.w;
  }
  mean_acc /= len;
  mean_gyro /= len;
}

}  // namespace sad
