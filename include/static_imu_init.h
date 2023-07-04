#ifndef STATIC_IMU_INIT_H
#define STATIC_IMU_INIT_H

#include "ImuTypes.h"
#include <deque>


class StaticIMUInit {
   public:
    struct Options {
        Options() {}
        double init_time_seconds_ = 5.0;     // 静止时间
        int init_imu_queue_max_size_ = 20000;  // 初始化IMU队列最大长度
        int static_odom_pulse_ = 5;           // 静止时轮速计输出噪声
        double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
        double max_static_acce_var = 0.05;    // 静态下加计测量方差
        double gravity_norm_ = 9.81;          // 重力大小
        bool use_speed_for_static_checking_ = true;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    };

    /// 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {}

    /// 添加IMU数据
    bool AddIMU(const ORB_SLAM3::IMU::Point & imu);
    /// 添加轮速数据
    // bool AddOdom(const Odom& odom);

    /// 判定初始化是否成功
    bool InitSuccess() const { return init_success_; }

    void ComputeMeanAndCovDiag(const std::deque<ORB_SLAM3::IMU::Point> &imu_data,
                                            Eigen::Vector3f &mean_acc,
                                            Eigen::Vector3f &mean_gyro);

    /// 获取各Cov, bias, gravity
    Eigen::Vector3f GetCovGyro() const { return cov_gyro_; }
    Eigen::Vector3f GetCovAcce() const { return cov_acce_; }
    Eigen::Vector3f GetInitBg() const { return init_bg_; }
    Eigen::Vector3f GetInitBa() const { return init_ba_; }
    Eigen::Vector3f GetGravity() const { return gravity_Imu; }

   private:
    /// 尝试对系统初始化
    bool TryInit();
    Options options_;
    bool init_success_ = false;
    Eigen::Vector3f cov_gyro_ = Eigen::Vector3f::Zero();  // 陀螺测量噪声协方差（初始化时评估）
    Eigen::Vector3f cov_acce_ = Eigen::Vector3f::Zero();  // 加计测量噪声协方差（初始化时评估）
    Eigen::Vector3f init_bg_ = Eigen::Vector3f::Zero();   // 陀螺初始零偏
    Eigen::Vector3f init_ba_ = Eigen::Vector3f::Zero();   // 加计初始零偏
    Eigen::Vector3f gravity_Imu = Eigen::Vector3f::Zero();   // 重力
    bool is_static_ = true;          // 标志车辆是否静止
    std::deque<ORB_SLAM3::IMU::Point> init_imu_deque_;  // 初始化用的数据
    double current_time_ = 0.0;       // 当前时间
    double init_start_time_ = 0.0;    // 静止的初始时间
};


#endif
