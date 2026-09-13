#include "control/MPPI.h"
#include <algorithm>  //用于 std::clamp
#include <memory>     //确保 std::make_unique 相关头文件存在
#include <iostream>
#include <cmath>  // 提供M_PI（π）的定义

MPPI::MPPI(ros::NodeHandle& nh) : 
    N_(300),          // 并行采样轨迹
    T_(80),           // 预测步长
    K_(20),            // 采样点
    dt_(0.05),        // 50Hz控制频率
    lambda_(0.12),   
    vx_min_(-0.1),    // 最大后退速度
    vx_max_(1.0),     // 最大前进速度
    wz_min_(-0.9),    // 最大左转角速度
    wz_max_(0.9),     // 最大右转角速度
    vx_std_(0.7),     // 速度采样标准差
    wz_std_(1.3),     // 角速度采样标准差
    distance_weight_(80.0),
    obstacle_weight_(45.0),
    control_weight_(150),
    terminal_progress_weight_(80.0),   // 终端进展奖励的权重（数值越大，越鼓励离目标更近）
    terminal_residual_weight_(10.0),    // 终端残差距离的轻惩罚，保证真正收敛
    goal_x_(0), 
    goal_y_(0),
    normal_dist_(0.0, 1.0),
    yaw_weight_(1.5),  // 降低默认朝向权重
    yaw_weight_near_(8.0),  // 接近目标时的朝向权重
    goal_near_threshold_(2.5)// 1米内视为接近目标
    
{
    // 初始化随机数生成器
    std::random_device rd;
    rng_.seed(rd());
    
    // 订阅代价地图
    costmap_sub_ = nh.subscribe("/laser_to_costmap/costmap", 1, &MPPI::costmapCallback, this);
    //若新消息到来时，队列中已有 1 条未处理的消息，ROS 会自动丢弃旧消息，只保留最新的那条。
    // 初始化上一时刻控制序列
    prev_controls_.resize(T_, Control(0.0, 0));

    traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mppi_trajectories", 100);
    //advertise的第二个参数表示消息缓冲区大小（队列长度）
    optimal_traj_pub_ = nh.advertise<visualization_msgs::Marker>("mppi_optimal_trajectory", 10);
    traj_pub_timer_ = nh.createTimer(ros::Duration(0.1), &MPPI::trajPubTimerCallback, this);

    edt_sub_ = nh.subscribe("/laser_to_costmap/edt_map", 1, &MPPI::edtCallback, this);
}

// 角度归一化：将任意角度转换到[-π, π]范围
double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void MPPI::setGoal(double x, double y) {
    goal_x_ = x;
    goal_y_ = y;
    std::cout << "[MPPI] 目标点已更新 -> (" << x << ", " << y << ")" << std::endl;
}

void MPPI::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = msg;
    // ROS_INFO("Received costmap: width=%d, height=%d, resolution=%.2f", 
    //          msg->info.width, msg->info.height, msg->info.resolution);  // 能够正常打印
}

void MPPI::edtCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(edt_mutex_);
    // 将ROS图像消息转换为OpenCV矩阵
    try
    {
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.copyTo(edt_map_);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("EDT image conversion failed: %s", e.what());
    }
}

double MPPI::getDistanceFromEDT(double x, double y) // 距离查询
{
    std::lock_guard<std::mutex> lock(edt_mutex_);
    if (edt_map_.empty() || !costmap_)
    {
        return 0.0;  // 地图未准备好
    }

    // 坐标转换（与getCostmapValue保持一致）
    double origin_x = costmap_->info.origin.position.x;
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    
    int mx = static_cast<int>(std::round((x - origin_x) / resolution));
    int my = static_cast<int>(std::round((y - origin_y) / resolution));

    // 检查边界
    if (mx < 0 || mx >= edt_map_.cols || my < 0 || my >= edt_map_.rows)
    {
        return 0.0;  // 地图外视为距离为0
    }
    return edt_map_.at<float>(my, mx);// 返回距离值
}

Control MPPI::getOptimalControl(const State& current_state, double global_x, double global_y,const RotMat& B2G_RotMat) {//在状态机的trotting状态线程中被.调用State_Trotting::mppiThreadFunc()

    // 保存当前全局位置和旋转矩阵用于代价计算
    B2G_RotMat_ = B2G_RotMat;
    current_global_x_ = global_x;
    current_global_y_ = global_y;

    // 采样控制序列
    State local_init_state(0, 0, 0);  // 机体坐标系下的初始状态
    auto control_samples = sampleControls();
    std::vector<double> costs(N_, 0.0);
    std::vector<std::future<double>> futures;
    
    std::vector<std::vector<State>> all_trajectories;
    all_trajectories.reserve(N_);
    for (int i = 0; i < N_; ++i) {
        // 按值构造轨迹，避免传引用指向临时对象
    // 存储所有轨迹
        auto traj = rollout(local_init_state, control_samples[i], B2G_RotMat);
        all_trajectories.push_back(traj);  // 先存入容器（复制一份）
        futures.emplace_back(std::async(std::launch::async,
            //&MPPI::calculateCost, this, std::move(traj), i));  // 传 i 进去
            &MPPI::calculateCost, this, std::cref(all_trajectories.back()), i));  // 传 i 进去
            //若用 std::move(all_trajectories[i])，会将这条轨迹的所有权从容器中 “转移” 到异步任务的参数中。
    }       //转移后，all_trajectories[i] 会变成 空的无效状态。
    //后续代码中 publishTrajectories(all_trajectories, ...) 需要访问 all_trajectories[i] 来发布轨迹，但此时它已经是空的了，导致发布失败（没有轨迹数据）。
    //std::cref(all_trajectories[i]) 会传递一个 “常量引用” 给 calculateCost，表示 “临时借用这条轨迹来读数据”。
    //轨迹的所有权仍然属于 all_trajectories 容器，后续 publishTrajectories 函数可以正常访问并发布这条轨迹。
    for (int i = 0; i < N_; ++i) {  
        costs[i] = futures[i].get();
    }
    
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        latest_trajectories_ = all_trajectories;
        latest_B2G_RotMat_ = B2G_RotMat;  // 同时保存当前旋转矩阵
    }
    // 计算权重
    auto weights = calculateWeights(costs);
    
    // 计算最优控制量
    //auto optimal_control = computeOptimalControl(control_samples, weights);
    //std::vector<Control> optimal_controls = computeOptimalControl(control_samples, weights);

    // 修改后：将结果保存到成员变量
    optimal_controls_ = computeOptimalControl(control_samples, weights);  // 赋值给成员变量
    std::vector<Control> optimal_controls = optimal_controls_;//optimal_controls局部变量

    Control first_control = optimal_controls[0];  // 仅返回第一个控制量用于执行
    // 更新上一时刻控制序列 (滚动窗口)
    prev_controls_.erase(prev_controls_.begin());
    //prev_controls_.push_back(optimal_control);
    prev_controls_.push_back(first_control);
    publishOptimalTrajectory(current_state, B2G_RotMat);

    return first_control;
}

std::vector<std::vector<Control>> MPPI::sampleControls() {
    std::vector<std::vector<Control>> samples(N_);
    
    // 生成稀疏控制点
    for (int i = 0; i < N_; ++i) {
        std::vector<Control> sparse_controls(K_);
        
        // 基于上一时刻最优控制加噪声
        for (int k = 0; k < K_; ++k) {
            int t = (T_ / (K_ - 1)) * k;  // 稀疏点时间索引  (T_ / (K_ - 1)) * k,zheng ti qu zheng shu
            sparse_controls[k].vx = prev_controls_[t].vx + vx_std_ * normal_dist_(rng_);
            sparse_controls[k].wz = prev_controls_[t].wz + wz_std_ * normal_dist_(rng_);
            
        }
        // 样条插值生成稠密控制序列
        samples[i] = splineInterpolation(sparse_controls);
    }
    
    return samples;//返回值已经是稠密点了
}

std::vector<Control> MPPI::splineInterpolation(const std::vector<Control>& sparse_controls) {
    std::vector<Control> dense_controls(T_);
    
    // 简单线性插值 (实际应用中可使用三次样条)
    for (int t = 0; t < T_; ++t) {
        double ratio = static_cast<double>(t) / (T_ - 1) * (K_ - 1);
        int k = static_cast<int>(ratio);
        double alpha = ratio - k;
        
        if (k >= K_ - 1) {
            dense_controls[t] = sparse_controls.back();
        } else {
            dense_controls[t].vx = (1 - alpha) * sparse_controls[k].vx + alpha * sparse_controls[k+1].vx;
            dense_controls[t].wz = (1 - alpha) * sparse_controls[k].wz + alpha * sparse_controls[k+1].wz;
        }
    }
    
    return dense_controls;
}

std::vector<State> MPPI::rollout(const State& init_state, const std::vector<Control>& controls, const RotMat& B2G_RotMat) {
    std::vector<State> trajectory;
    trajectory.reserve(T_ + 1);
    trajectory.push_back(init_state);
    
    State current = init_state;
    for (const auto& ctrl : controls) {
        current = stateTransition(current, ctrl, dt_, B2G_RotMat);
        trajectory.push_back(current);
        //进入循环后，对每个控制量 ctrl 进行状态转换计算，得到下一个状态 current，并通过push_back(current); 将新状态追加到轨迹中。
    }
    return trajectory;
}

State MPPI::stateTransition(const State& state, const Control& control, double dt, const RotMat& B2G_RotMat) {
    State next;
    // 机体坐标系下，控制量vx产生的位移在x、y方向的分解
    next.x = state.x + control.vx * cos(state.yaw) * dt;  // 沿当前朝向的x分量
    next.y = state.y + control.vx * sin(state.yaw) * dt;  // 沿当前朝向的y分量（关键补充）
    next.yaw = state.yaw + control.wz * dt;               // 偏航角更新
    next.yaw = wrapAngle(next.yaw);                       // 归一化角度
    
    return next;
}
double MPPI::calculateCost(const std::vector<State>& trajectory,int sample_idx) {
    if (trajectory.empty()) 
    return 0.0;

    double total_cost = 0.0;
    double process_cost = 0.0;  // 过程代价（所有中间点）
    double terminal_cost = 0.0; // 终端代价（仅最后一个点）
    double obs_cost =0.0;
    double control_smooth_cost = 0.0;
    double yaw_cost =0.0;
    double dist_cost =0.0;

    double sum_dist_cost = 0.0;
    double sum_yaw_cost  = 0.0;
    double sum_obs_cost  = 0.0;
    double sum_ctrl_cost = 0.0;
    // 获取轨迹长度和终端状态
    size_t traj_len = trajectory.size();
    const auto& final_state = trajectory.back();

    // 1. 过程代价计算（除最后一个点外的所有点）
    for (size_t i = 0; i < traj_len - 1; ++i) {  // 遍历到倒数第二个点
        const auto& state = trajectory[i];
        
        // 1.1 过程目标代价（跟踪路径中间点）
        Eigen::Vector2d local_pos(state.x, state.y);
        Eigen::Vector2d global_pos = B2G_RotMat_.block<2,2>(0,0) * local_pos;
        global_pos.x() += current_global_x_;
        global_pos.y() += current_global_y_;

        double dx = global_pos.x() - goal_x_;
        double dy = global_pos.y() - goal_y_;
        // dist_cost = distance_weight_ * 0.5 * (dx*dx + dy*dy);  // 过程目标权重降低
        // 当前点到目标的距离
        double di = std::hypot(goal_x_ - global_pos.x(), goal_y_ - global_pos.y());

        // 预取下一步全局位置（用 trajectory[i+1]）
        Eigen::Vector2d next_local(trajectory[i+1].x, trajectory[i+1].y);
        Eigen::Vector2d next_global = B2G_RotMat_.block<2,2>(0,0) * next_local;
        next_global.x() += current_global_x_;
        next_global.y() += current_global_y_;

        double di_next = std::hypot(goal_x_ - next_global.x(), goal_y_ - next_global.y());

        double progress = di_next - di;          // <0 表示靠近目标
        double progress_reward = std::min(0.0, progress);//纯奖励
        dist_cost = distance_weight_  * progress_reward;  // 靠近目标 => 负值，降低总成本

        sum_dist_cost += dist_cost;
        // 把 progress_cost 加入 process_cost

        // 1.2 过程朝向代价
        double target_yaw = atan2(goal_y_ - global_pos.y(), goal_x_ - global_pos.x() + 1e-6);
        double current_global_yaw = atan2(B2G_RotMat_(1,0), B2G_RotMat_(0,0));
        double target_yaw_body = wrapAngle(target_yaw - current_global_yaw);
        double yaw_error = wrapAngle(target_yaw_body - state.yaw);

        double dist_to_goal = std::hypot(goal_x_ - global_pos.x(), goal_y_ - global_pos.y());
        // 根据距离动态调整朝向权重
        double current_yaw_weight = (dist_to_goal < goal_near_threshold_) ? 
                           yaw_weight_near_ : yaw_weight_;
        yaw_cost = current_yaw_weight * (yaw_error * yaw_error);                   
        //yaw_cost = yaw_weight_ * (yaw_error * yaw_error);  // 过程朝向权重降低
        sum_yaw_cost  += yaw_cost;
        // 1.3 障碍物代价（过程中需持续避障）
        // {
        //     std::lock_guard<std::mutex> lock(costmap_mutex_);
        //     if (costmap_) {

        //         // // 1. 构建机体朝向的旋转矩阵（绕z轴旋转state.yaw）
        //         // double cos_yaw = cos(state.yaw);
        //         // double sin_yaw = sin(state.yaw);
                
        //         // // 2. 将轨迹点局部坐标（x,y）转换为机体坐标系下的实际坐标
        //         // double x_body = state.x * cos_yaw - state.y * sin_yaw;  // 旋转后的x坐标
        //         // double y_body = state.x * sin_yaw + state.y * cos_yaw;  // 旋转后的y坐标
                
        //         // // 3. 使用转换后的坐标查询代价地图
        //         // obs_cost = obstacle_weight_ * 0.5 * getCostmapValue(x_body, y_body);
        //         Eigen::Matrix2d R_g2b = B2G_RotMat_.block<2,2>(0,0).transpose();
        //         Eigen::Vector2d p_in_costmap = R_g2b * (global_pos - Eigen::Vector2d(current_global_x_, current_global_y_));

        //         double cell_cost = getCostmapValue(p_in_costmap.x(), p_in_costmap.y());
        //         //double cell_cost = getCostmapValue(state.x, state.y);
        //         obs_cost = obstacle_weight_ * cell_cost;

        //         // if (sample_idx < 10 && (i==1 || i==10 || i==20 || i==30 || i==40 || i==50)) {
        //         // ROS_INFO("[rollout debug] sample=%d, step=%zu, local(%.2f, %.2f), yaw=%.2f",
        //         // sample_idx, i, p_in_costmap.x(), p_in_costmap.y(), state.yaw);
        //         // }
        //     }
        // }
        // 1.3 障碍物代价（过程中需持续避障）
        // 在calculateCost函数中修改障碍物代价计算部分
        // 替换原有的obs_cost计算
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            if (costmap_) {
                Eigen::Matrix2d R_g2b = B2G_RotMat_.block<2,2>(0,0).transpose();
                Eigen::Vector2d p_in_costmap = R_g2b * (global_pos - Eigen::Vector2d(current_global_x_, current_global_y_));
                // 获取到最近障碍物的距离（米）
                double distance = getDistanceFromEDT(p_in_costmap.x(), p_in_costmap.y());
                // 改为（直接用地图/世界系坐标去查）：
                //double distance = getDistanceFromEDT(global_pos.x(), global_pos.y());
                
                // 定义安全距离和代价函数参数
                const double safe_distance = 0.3;  // 安全距离
                const double influence_radius = 0.4;  // 影响半径(这两个数再小0.05就过不去了)
                const double repulsion_strength = 1.5;
                // 障碍物代价函数：距离越近代价越高，超过影响半径代价为0
                if (distance < safe_distance) {
                    // 安全距离内，代价急剧增加
                    //obs_cost = obstacle_weight_ * (1.0 - std::pow(distance / safe_distance, 2));
                    obs_cost = obstacle_weight_ * repulsion_strength * 
                    std::pow(1.0 / (distance + 1e-6 ), 2) ;
                } 
                else if (distance < influence_radius) {
                    // 安全距离到影响半径之间，代价逐渐减小
                    // double ratio = (distance - safe_distance) / (influence_radius - safe_distance);
                    // obs_cost = obstacle_weight_ * 0.1 * (1.0 - ratio);
                    obs_cost = obstacle_weight_ * 
                    std::pow(1.0 / (distance + 1e-6 ), 2) ;
                } 
                else {
                    // 超出影响半径，无障碍物代价
                    obs_cost = 0.0;
                }
                sum_obs_cost  += obs_cost;
            }
        }

        // 1.4 控制平滑性代价（过程中控制量变化）
        if (i > 1) {  // 从第三个点开始计算二阶差分
            double dvx = trajectory[i].x - 2*trajectory[i-1].x + trajectory[i-2].x;
            double dwz = trajectory[i].yaw - 2*trajectory[i-1].yaw + trajectory[i-2].yaw;
            control_smooth_cost = control_weight_ * (dvx*dvx + dwz*dwz);

            sum_ctrl_cost += control_smooth_cost;
        }

        // 累加过程代价
        process_cost += dist_cost + yaw_cost + obs_cost + control_smooth_cost;
    }

    // --------------------------
    // 2. 终端代价计算（仅最后一个点）
    // --------------------------
    // 2.1 终端目标代价（到达最终目标）
    Eigen::Vector2d final_local(final_state.x, final_state.y);
    Eigen::Vector2d final_global = B2G_RotMat_.block<2,2>(0,0) * final_local;
    final_global.x() += current_global_x_;
    final_global.y() += current_global_y_;

    // double term_dx = final_global.x() - goal_x_;
    // double term_dy = final_global.y() - goal_y_;
    // terminal_cost += distance_weight_ * 2.0 * (term_dx*term_dx + term_dy*term_dy); 

    // 2.1 终端“进展奖励” + 弱“剩余距离”惩罚
    // 计算 d0（轨迹第一点的全局位姿 -> 到目标距离）
    Eigen::Vector2d init_local(trajectory.front().x, trajectory.front().y);
    Eigen::Vector2d init_global = B2G_RotMat_.block<2,2>(0,0) * init_local;
    init_global.x() += current_global_x_;
    init_global.y() += current_global_y_;

    double d0 = std::hypot(goal_x_ - init_global.x(),  goal_y_ - init_global.y());
    double dT = std::hypot(goal_x_ - final_global.x(), goal_y_ - final_global.y());

    // 进展 = d0 - dT（越大越好）；在代价里用负号变成“奖励”
    double terminal_progress = std::max(0.0, d0 - dT);
    terminal_cost += - terminal_progress_weight_ * terminal_progress;

    // 弱“剩余距离”惩罚：线性/Huber都可；这里用线性更稳（避免靠近时平方爆炸）
    terminal_cost += terminal_residual_weight_ * dT;

    // 2.2 终端朝向代价（最终朝向精确性）
    double term_target_yaw = atan2(goal_y_ - final_global.y(), goal_x_ - final_global.x() + 1e-6);
    double term_current_yaw = atan2(B2G_RotMat_(1,0), B2G_RotMat_(0,0));
    double term_target_yaw_body = wrapAngle(term_target_yaw - term_current_yaw);
    double term_yaw_error = wrapAngle(term_target_yaw_body - final_state.yaw);

    double term_yaw_weight = (dT < goal_near_threshold_) ? yaw_weight_near_ : yaw_weight_;
    terminal_cost += term_yaw_weight * 8.0 * (term_yaw_error * term_yaw_error);
    //terminal_cost += yaw_weight_ * 8.0 * (term_yaw_error * term_yaw_error);  

    // 总代价 = 过程代价 + 终端代价
    total_cost = process_cost + terminal_cost;
    // 代价分项打印
    ROS_INFO_THROTTLE(1,
        "cost analysis: totalcost=%.3f | process=%.3f | terminal=%.3f | process distance cost=%.3f | process yaw cost=%.3f | process obstacle cost=%.3f,process control cost=%.3f" , 
        total_cost, process_cost, terminal_cost ,sum_dist_cost ,sum_yaw_cost ,sum_obs_cost ,sum_ctrl_cost
    );
    return total_cost;
}

double MPPI::getCostmapValue(double x, double y) {//x,y->local position
    if (!costmap_) {
        std::cout << "[MPPI] the map is empty！" << std::endl;
        return 0.0;
    }
    // 将世界坐标转换为栅格坐标
    int mx, my;
    if (!costmap_->info.width || !costmap_->info.height) {
        std::cout << "[MPPI] map size is 00000000！" << std::endl;
        return 0.0;
    }
    
    double origin_x = costmap_->info.origin.position.x;//是(-5)
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    
    mx = static_cast<unsigned int>(std::round((x - origin_x) / resolution));//std::round 是四舍五入
    my = static_cast<unsigned int>(std::round((y - origin_y) / resolution));
    // mx = static_cast<unsigned int>((x - origin_x) / resolution);
    // my = static_cast<unsigned int>((y - origin_y) / resolution);
    
    //std::cout << "[MPPI] 查询坐标: world(" << x << ", " << y << ") -> map(" << mx << ", " << my << ")" << std::endl;
    // 检查是否在地图范围内
    if (mx >= costmap_->info.width || my >= costmap_->info.height|| mx<0 || my<0) {
        std::cout << "[MPPI] is out of the map，location：(" << x << ", " << y << ")" << std::endl;
        return 1.0;  // 地图外视为障碍物
    }
    
    // 获取栅格代价 (0-100)
    int idx = my * costmap_->info.width + mx;//前my行共有my × width个栅格，加上当前行的第mx个栅格，总索引就是my × width + mx
    int cost = costmap_->data[idx];
    
    // //打印查询的local坐标和对应的栅格坐标、代价
    // ROS_INFO_THROTTLE(0.1, "check the obstacle: local location(%.2f, %.2f) -> grid cost=%.2f", 
    //                  x, y, static_cast<double>(cost) / 100.0);//限制 0.5 秒内最多打印 1 次
    // 归一化到[0,1]
    return static_cast<double>(cost) / 100.0;
}

// std::vector<double> MPPI::calculateWeights(const std::vector<double>& costs) {
//     std::vector<double> weights(N_);
//     double min_cost = *std::min_element(costs.begin(), costs.end());
    
//     // 计算指数权重
//     double sum = 0.0;
//     for (int i = 0; i < N_; ++i) {
//         weights[i] = exp(-(costs[i] - min_cost) / lambda_);
//         sum += weights[i];
//     }
    
//     // 归一化
//     for (int i = 0; i < N_; ++i) {
//         weights[i] /= sum;
//     }
    
//     return weights;
// }
    std::vector<double> MPPI::calculateWeights(const std::vector<double>& costs) {
        std::vector<double> weights(N_);
        if (costs.empty()) return weights;

        // 1) 先做 shift，防止指数下溢
        double min_cost = *std::min_element(costs.begin(), costs.end());
        std::vector<double> s(costs.size());//创建一个名为 s 的 std::vector<double> 类型的向量，其长度为 costs.size()---和 costs 向量一样大(700)
        for (size_t i = 0; i < costs.size(); ++i) {
            s[i] = -(costs[i] - min_cost) / lambda_;
        }
        double max_s = *std::max_element(s.begin(), s.end());

        // 2) 计算归一化的 softmax
        double sum = 0.0;
        for (auto &v : s) sum += std::exp(v - max_s);

        if (sum < 1e-12) {
            // 极端兜底：均匀权重
            for (auto &w : weights) w = 1.0 / double(N_);
            ROS_WARN("[MPPI] weight sum underflow, fallback to uniform.");
            return weights;
        }
        for (size_t i = 0; i < costs.size(); ++i) {
            weights[i] = std::exp(s[i] - max_s) / sum;
        }

        // 可选：打印一下权重展开情况
        // ROS_INFO_THROTTLE(0.5, "[MPPI] w_min=%.3e, w_max=%.3e", 
        //                   *std::min_element(weights.begin(),weights.end()),
        //                   *std::max_element(weights.begin(),weights.end()));
        return weights;
    }

// Control MPPI::computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
//                                   const std::vector<double>& weights) {
//     Control optimal(0, 0);
    
//     // 加权求和得到最优控制量
//     for (int i = 0; i < N_; ++i) {
//         optimal.vx += weights[i] * control_samples[i][0].vx;
//         optimal.wz += weights[i] * control_samples[i][0].wz;
//     }
    
//     // 控制量限幅
//     optimal.vx = my_clamp(optimal.vx, vx_min_, vx_max_);
//     optimal.wz = my_clamp(optimal.wz, wz_min_, wz_max_);
    
//     return optimal;
// }
// 修改返回类型为std::vector<Control>，生成完整最优控制序列
std::vector<Control> MPPI::computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
                                               const std::vector<double>& weights) {
    // 初始化最优控制序列（长度为预测步长T_）
    std::vector<Control> optimal_controls(T_);
    
    // 对每个时间步的控制量进行加权求和
    for (int t = 0; t < T_; ++t) {  // 遍历每个预测时间步
        optimal_controls[t].vx = 0.0;
        optimal_controls[t].wz = 0.0;
        
        for (int i = 0; i < N_; ++i) {  // 遍历所有采样轨迹
            optimal_controls[t].vx += weights[i] * control_samples[i][t].vx;
            optimal_controls[t].wz += weights[i] * control_samples[i][t].wz;
        }
        
        // 控制量限幅
        optimal_controls[t].vx = my_clamp(optimal_controls[t].vx, vx_min_, vx_max_);
        optimal_controls[t].wz = my_clamp(optimal_controls[t].wz, wz_min_, wz_max_);
    }
    
    return optimal_controls;  // 返回完整序列
}
// 添加定时器回调函数
void MPPI::trajPubTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!latest_trajectories_.empty()) {
        publishTrajectories(latest_trajectories_,latest_B2G_RotMat_);
    }
}

void MPPI::publishTrajectories(const std::vector<std::vector<State>>& all_trajectories, const RotMat& B2G_RotMat) {
    // 1. 先发布一个“删除上一帧所有轨迹”的Marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "trunk";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "mppi_trajectories";  // 与轨迹用同一个命名空间

    // delete_marker.action = visualization_msgs::Marker::DELETEALL;  // 删除该ns下所有Marker
    // traj_pub_.publish(delete_marker);

    visualization_msgs::MarkerArray marker_array;

    int id = 0;
    for (const auto& traj : all_trajectories) {
        // 过滤掉点数不足的轨迹（至少需要2个点才能绘制LINE_STRIP）
        if (traj.size() < 2) {
            continue;  // 跳过无效轨迹
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "trunk";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mppi_trajectories";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;  // 使用LINE_STRIP更高效
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;  // 关键：设置为单位四元数

        marker.scale.x = 0.01;
        marker.color.r = 0.1f;
        marker.color.g = 0.9f;
        marker.color.b = 0.1f;
        marker.color.a = 1.0f;     // 关键：不为 0**

        for (const auto& state : traj) {
            Eigen::Vector2d local_pos(state.x, state.y);
            geometry_msgs::Point p;

            p.x = state.x;
            p.y = state.y;
            p.z = 0.1;

            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }
    // 一次性发布所有轨迹
    traj_pub_.publish(marker_array);  // 注意：需要将traj_pub_改为发布MarkerArray类型
    
}
void MPPI::publishOptimalTrajectory(const State& current_state, const RotMat& B2G_RotMat) {
    // 1. 使用最优控制序列推演轨迹
    State local_init_state(0, 0, 0);  // 机体坐标系下的初始状态
    std::vector<State> optimal_trajectory = rollout(local_init_state, optimal_controls_, B2G_RotMat);

    // 至少需要2个点才能绘制LINE_STRIP
    if (optimal_trajectory.size() < 2) {
        ROS_WARN("optimal_controls_ is < 2, cannot generate trajectory.");
        return;
    }
    
    // 2. 构造Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "trunk";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mppi_optimal_trajectory";
    marker.id = 0;  // 固定ID，每次覆盖
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;  // 关键：设置为单位四元数

    marker.scale.x = 0.02;  // 线宽加粗，突出显示
    marker.color.r = 1.0f;  // 红色
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;  // 不透明

    // 3. 转换轨迹点到全局坐标系并填充
    for (const auto& state : optimal_trajectory) {
        Eigen::Vector2d local_pos(state.x, state.y);
        geometry_msgs::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.25;  // 高于其他轨迹，避免遮挡
        marker.points.push_back(p);
    }

    // 4. 发布最优轨迹
    optimal_traj_pub_.publish(marker);
}