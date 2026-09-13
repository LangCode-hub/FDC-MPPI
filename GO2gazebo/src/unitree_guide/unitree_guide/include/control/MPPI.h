#ifndef MPPI_H
#define MPPI_H

#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <thread>
#include <future>
#include <random>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>  // Eigen矩阵库
#include <visualization_msgs/Marker.h>  // 包含Marker消息定义
#include <visualization_msgs/MarkerArray.h>
// 加入OpenCV和ROS图像相关头文件
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using RotMat = Eigen::Matrix3d;  // 与SDK中保持一致，避免冲突

// 状态量: x, y, yaw
struct State {
    double x=0;
    double y=0;
    double yaw=0;
    
    State() : x(0), y(0), yaw(0) {}
    State(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

// 控制量: x方向速度, 偏航角速度
struct Control {
    double vx;    // 前进速度
    double wz;    // 偏航角速度
    
    Control() : vx(0), wz(0) {}
    Control(double vx_, double wz_) : vx(vx_), wz(wz_) {}
};

template <typename T>  // 模板函数，支持所有数值类型（int/double 等）
T my_clamp(const T& value, const T& min_val, const T& max_val) {
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}

class MPPI {
public:
    MPPI(ros::NodeHandle& nh);
    ~MPPI() = default;
    
    // 设置目标点
    void setGoal(double x, double y);
    // 获取最优控制量
    Control getOptimalControl(const State& current_state, double global_x, double global_y, const RotMat& B2G_RotMat);
    
    // 代价地图回调函数
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    // 返回地图系下当前机器人全局坐标 (x,y)
    Eigen::Vector2d getCurrentGlobalPos() const {
    return Eigen::Vector2d(current_global_x_, current_global_y_);
    }
    void publishTrajectories(const std::vector<std::vector<State>>& all_trajectories, 
                             const RotMat& B2G_RotMat);

    
    
private:
    Eigen::Vector2d current_world_xy_;

    // 采样控制序列
    std::vector<std::vector<Control>> sampleControls();
    
    // 预测轨迹
    std::vector<State> rollout(const State& init_state, const std::vector<Control>& controls, const RotMat& B2G_RotMat);
    
    // 计算轨迹代价
    double calculateCost(const std::vector<State>& trajectory,int sample_idx);
    
    // 计算权重
    std::vector<double> calculateWeights(const std::vector<double>& costs);
    
    // 生成最优控制量
    // Control computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
    //                              const std::vector<double>& weights);
    std::vector<Control> computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
                                         const std::vector<double>& weights);
    
    // 样条插值生成稠密控制序列
    std::vector<Control> splineInterpolation(const std::vector<Control>& sparse_controls);
    
    // 从代价地图获取点的代价
    double getCostmapValue(double x, double y);
    
    // 状态转换函数
    State stateTransition(const State& state, const Control& control, double dt, const RotMat& B2G_RotMat);

    // ROS相关
    ros::Subscriber costmap_sub_;
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    std::mutex costmap_mutex_;

    ros::Subscriber edt_sub_;  // 订阅EDT地图
    cv::Mat edt_map_;          // 存储EDT地图
    std::mutex edt_mutex_;     // 保护EDT地图的线程安全

    ros::Publisher traj_pub_;
    ros::Timer traj_pub_timer_;
    std::vector<std::vector<State>> latest_trajectories_;
    std::mutex traj_mutex_;

    ros::Publisher optimal_traj_pub_;  // 最优轨迹发布器
    std::vector<Control> optimal_controls_;  // 存储完整的最优控制序列
    //定义在 MPPI 类的 private 区域，属于类的成员变量。变量名以 _ 结尾（通常是 C++ 中成员变量的命名规范），用于区分局部变量。
    //作用就是在类的不同成员函数之间共享数据，因此必须通过它传递 computeOptimalControl 的结果

    // 添加EDT地图回调函数
    void edtCallback(const sensor_msgs::Image::ConstPtr& msg);
    // 添加获取距离的函数
    double getDistanceFromEDT(double x, double y);

    void trajPubTimerCallback(const ros::TimerEvent& event);  

    void publishOptimalTrajectory(const State& current_state, const RotMat& B2G_RotMat);

    // 新增：存储最新的旋转矩阵（用于轨迹发布时的坐标转换）
    RotMat latest_B2G_RotMat_;  // 与轨迹对应的旋转矩阵

    // 添加用于坐标转换的临时变量
    RotMat B2G_RotMat_;  // 保存当前机体到全局的旋转矩阵
    double current_global_x_;  // 当前全局位置x
    double current_global_y_;  // 当前全局位置y
    
    // MPPI参数
    int N_;                  // 采样数量
    int T_;                  // 预测步长
    int K_;                  // 稀疏采样点数量
    double dt_;              // 控制时间间隔
    double lambda_;          // 温度参数
    // 控制量边界
    double vx_min_, vx_max_;
    double wz_min_, wz_max_;
    // 控制量噪声标准差
    double vx_std_, wz_std_;
    // 代价函数权重
    double distance_weight_;     // 目标点权重
    double obstacle_weight_; // 障碍物权重
    double control_weight_;  // 控制平滑性权重
    double yaw_weight_ ;//控制转向
    // 目标点
    double goal_x_, goal_y_;
    
    double terminal_progress_weight_;
    double terminal_residual_weight_;

    double yaw_weight_near_;  // 接近目标时的朝向权重
    double goal_near_threshold_;  // 接近目标的距离阈值（米）
    
    // 随机数生成器
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
    
    // 上一时刻最优控制序列
    std::vector<Control> prev_controls_;
};

#endif