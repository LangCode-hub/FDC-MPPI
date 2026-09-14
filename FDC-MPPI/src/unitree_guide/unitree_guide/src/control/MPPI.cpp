#include "control/MPPI.h"
#include <algorithm>  // Provides std::clamp.
#include <memory>     // Provides std::make_unique.
#include <iostream>
#include <cmath>  // Provides the M_PI constant.

MPPI::MPPI(ros::NodeHandle& nh) : 
    N_(250),          // Number of trajectories sampled in parallel.
    T_(195),           // Prediction horizon.
    K_(15),            // Number of sampled control points.
    dt_(0.05),        // 50 Hz control rate.
    lambda_(0.12),   
    vx_min_(0.2),    // Maximum reverse speed.
    vx_max_(0.8),     // Maximum forward speed.
    wz_min_(-0.4),    // Minimum yaw rate.
    wz_max_(0.4),     // Maximum yaw rate.
    vx_std_(0.06),     // Linear-velocity sampling standard deviation.
    wz_std_(0.12),     // Angular-velocity sampling standard deviation.
    distance_weight_(80.0),
    obstacle_weight_(45.0),
    control_weight_(500),
    terminal_progress_weight_(80.0),   // Terminal progress-reward weight.
    terminal_residual_weight_(10.0),    // Light terminal residual-distance penalty.
    goal_x_(0), 
    goal_y_(0),
    normal_dist_(0.0, 1.0),
    yaw_weight_(1.5),  // Reduced default heading weight.
    yaw_weight_near_(8.0),  // Heading weight near the target.
    goal_near_threshold_(2.5),// Distance threshold for the near-target region.
    mu1_(0.0), mu2_(0.0), mu3_(0.0),
    current_yaw_(0.0), k1_(0.0), k2_(0.0)
    
{
    // Initialize the random-number generator.
    std::random_device rd;
    rng_.seed(rd());
    
    // Subscribe to the costmap.
    costmap_sub_ = nh.subscribe("/laser_to_costmap/costmap", 1, &MPPI::costmapCallback, this);
    // With a queue size of one, ROS discards the old message when a newer message arrives.
    // Initialize the previous optimal control sequence.
    prev_controls_.resize(T_, Control(0.0, 0));

    traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mppi_trajectories", 100);
    // The second advertise argument is the outgoing message-queue size.
    optimal_traj_pub_ = nh.advertise<visualization_msgs::Marker>("mppi_optimal_trajectory", 10);
    traj_pub_timer_ = nh.createTimer(ros::Duration(0.1), &MPPI::trajPubTimerCallback, this);

    edt_sub_ = nh.subscribe("/laser_to_costmap/edt_map", 1, &MPPI::edtCallback, this);

    mu_sub_ = nh.subscribe("/robot/mu_estimation", 1, &MPPI::muCallback, this);
    gazebo_model_sub_ = nh.subscribe("/gazebo/model_states", 10, &MPPI::gazeboModelCallback, this);
    // Initialize the filters.
    initializeButterworthFilter(filter_order_, cutoff_freq_, dt_, vx_b, vx_a);
    initializeButterworthFilter(filter_order_, cutoff_freq_, dt_, wz_b, wz_a);
    // Initialize the filter histories.
    vx_prev.resize(filter_order_, 0.0);
    wz_prev.resize(filter_order_, 0.0);
}

// Normalize an angle to the range [-pi, pi].
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
    //          msg->info.width, msg->info.height, msg->info.resolution);  // Enable for map diagnostics.
}

void MPPI::edtCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(edt_mutex_);
    // Convert the ROS image message to an OpenCV matrix.
    try
    {
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.copyTo(edt_map_);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("EDT image conversion failed: %s", e.what());
    }
}

double MPPI::getDistanceFromEDT(double x, double y) // Query the obstacle distance.
{
    std::lock_guard<std::mutex> lock(edt_mutex_);
    if (edt_map_.empty() || !costmap_)
    {
        return 0.0;  // The map is not ready.
    }

    // Convert coordinates consistently with getCostmapValue().
    double origin_x = costmap_->info.origin.position.x;
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    
    int mx = static_cast<int>(std::round((x - origin_x) / resolution));
    int my = static_cast<int>(std::round((y - origin_y) / resolution));

    // Check map bounds.
    if (mx < 0 || mx >= edt_map_.cols || my < 0 || my >= edt_map_.rows)
    {
        return 0.0;  // Treat points outside the map as zero distance.
    }
    return edt_map_.at<float>(my, mx);// Return the distance value.
}

Control MPPI::getOptimalControl(const State& current_state, double global_x, double global_y,const RotMat& B2G_RotMat) {// Called by State_Trotting::mppiThreadFunc().

    // Save the current global position and rotation matrix for cost evaluation.
    B2G_RotMat_ = B2G_RotMat;
    current_global_x_ = global_x;// These two arguments are not otherwise used in this function.
    current_global_y_ = global_y;

    // Sample control sequences.
    State local_init_state(0, 0, 0);  // Initial state in the body frame.
    auto control_samples = sampleControls();
    std::vector<double> costs(N_, 0.0);
    std::vector<std::future<double>> futures;
    
    std::vector<std::vector<State>> all_trajectories;
    all_trajectories.reserve(N_);
    for (int i = 0; i < N_; ++i) {
        // Construct the trajectory by value so no reference points to a temporary.
    // Store every trajectory.
        auto traj = rollout(local_init_state, control_samples[i], B2G_RotMat);
        all_trajectories.push_back(traj);  // Store a copy before launching the task.
        futures.emplace_back(std::async(std::launch::async,
            //&MPPI::calculateCost, this, std::move(traj), i));  // Moving would empty the stored trajectory.
            &MPPI::calculateCost, this,
             std::cref(all_trajectories.back()), 
             std::cref(control_samples[i]),
             i));  // Pass a constant reference.
            // std::move would transfer ownership of the trajectory to the asynchronous task.
    }       // The corresponding element in all_trajectories would then be empty.
    // publishTrajectories() still needs the stored trajectories after cost evaluation.
    // std::cref passes a read-only borrowed reference to calculateCost().
    // Ownership therefore remains with all_trajectories so the trajectories can still be published.
    for (int i = 0; i < N_; ++i) {  
        costs[i] = futures[i].get();
    }
    
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        latest_trajectories_ = all_trajectories;
        latest_B2G_RotMat_ = B2G_RotMat;  // Store the corresponding rotation matrix.
    }
    // Compute weights.
    auto weights = calculateWeights(costs);

    // // Store the result in the member variable.
    // optimal_controls_ = computeOptimalControl(control_samples, weights);  // Assign to the member variable.
    // std::vector<Control> optimal_controls = optimal_controls_;// Local optimal_controls variable.

    // Control first_control = optimal_controls[0];  // Return only the first control for execution.
    // // Update the previous control sequence using a receding window.
    // prev_controls_.erase(prev_controls_.begin());// Remove the oldest control from prev_controls_.
    // //prev_controls_.push_back(optimal_control);
    // prev_controls_.push_back(first_control);// Append the latest control to prev_controls_.
    // publishOptimalTrajectory(current_state, B2G_RotMat);
    // return first_control;

    // Store the result in a member for use by the trajectory publisher.
    optimal_controls_ = computeOptimalControl(control_samples, weights);  // Update the member sequence.
    std::vector<Control> optimal_controls = optimal_controls_;  // Local copy for subsequent use.

    // ========= 1) Update prev_controls_ using this cycle's optimal sequence. =========
    // Shift the previous step 1 to the center of the current step 0,
    // the previous step 2 to the center of the current step 1, and so on.
    if (optimal_controls.size() >= static_cast<size_t>(T_)) {
        for (int t = 0; t < T_ - 1; ++t) {
            prev_controls_[t] = optimal_controls[t + 1];   // Shift forward by one time step.
        }
        // Fill the last entry with the final step of the current sequence.
        prev_controls_[T_ - 1] = optimal_controls.back();
    } else {
        // Fallback to the former receding-window update if the length is invalid.
        prev_controls_.erase(prev_controls_.begin());
        prev_controls_.push_back(optimal_controls.front());
    }

    // ========= 2) Send the first control of the current sequence to the robot. =========
    //Control first_control = optimal_controls[0];

    Control raw = optimal_controls[0];  // Raw MPPI output.

    double alpha = 0.4;  // In [0, 1]; smaller values are smoother but respond more slowly.
    Control filtered;
    double dvx_max = 0.04;  // Maximum velocity change per control cycle.
    double dwz_max = 0.01;  // Maximum angular-velocity change per control cycle.

    if (!has_last_cmd_) {
        filtered = raw;
        has_last_cmd_ = true;
    } else {
        filtered.vx = (1.0 - alpha) * last_cmd_.vx + alpha * raw.vx;
        filtered.wz = (1.0 - alpha) * last_cmd_.wz + alpha * raw.wz;
    }
    double dvx = filtered.vx - last_cmd_.vx;
    double dwz = filtered.wz - last_cmd_.wz;

    dvx = std::max(-dvx_max, std::min(dvx, dvx_max));
    dwz = std::max(-dwz_max, std::min(dwz, dwz_max));

    filtered.vx = last_cmd_.vx + dvx;
    filtered.wz = last_cmd_.wz + dwz;
    last_cmd_ = filtered;
    // Use filtered instead of raw.
    Control first_control = filtered;
    // Publish the optimal trajectory using the existing logic.
    publishOptimalTrajectory(current_state, B2G_RotMat);
    return first_control;

}

std::vector<std::vector<Control>> MPPI::sampleControls() {
    std::vector<std::vector<Control>> samples(N_);
    
    // // Generate sparse control points.
    // for (int i = 0; i < N_; ++i) {
    //     std::vector<Control> sparse_controls(K_);
        
    //     // Add noise to the previous optimal control.
    //     for (int k = 0; k < K_; ++k) {
    //         int t = (T_ / (K_ - 1)) * k;  // Sparse-point time index, rounded down to an integer.
    //         sparse_controls[k].vx = prev_controls_[t].vx + vx_std_ * normal_dist_(rng_);
    //         sparse_controls[k].wz = prev_controls_[t].wz + wz_std_ * normal_dist_(rng_);
            
    //     }
    //     // Generate a dense control sequence by spline interpolation.
    //     samples[i] = splineInterpolation(sparse_controls);
    // }
    // Generate sparse control points.
    for (int i = 0; i < N_; ++i) {
        std::vector<Control> sparse_controls(K_);
        std::vector<double> vx_noises(K_);
        std::vector<double> wz_noises(K_);
        
        // Generate the noise sequences first.
        for (int k = 0; k < K_; ++k) {
            int t = (T_ / (K_ - 1)) * k;
            vx_noises[k] = vx_std_ * normal_dist_(rng_);
            wz_noises[k] = wz_std_ * normal_dist_(rng_);
        }
        
        // Apply a low-pass filter to the noise sequences.
        std::vector<double> filtered_vx_noises(K_);
        std::vector<double> filtered_wz_noises(K_);
        // Reset the filter histories.
        std::fill(vx_prev.begin(), vx_prev.end(), 0.0);
        std::fill(wz_prev.begin(), wz_prev.end(), 0.0);
        
        for (int k = 0; k < K_; ++k) {
            filtered_vx_noises[k] = applyButterworthFilter(vx_noises[k], vx_b, vx_a, vx_prev);
            filtered_wz_noises[k] = applyButterworthFilter(wz_noises[k], wz_b, wz_a, wz_prev);
        }
        double step = static_cast<double>(T_ - 1) / (K_ - 1);  // For example, 69/14 is approximately 4.93.
        // Apply the filtered noise to the controls.
        for (int k = 0; k < K_; ++k) {
            //int t = (T_ / (K_ - 1)) * k;
            int t = static_cast<int>(std::round(step * k));    // t ∈ [0, T_-1]
            t = std::max(0, std::min(t, T_ - 1));             // Additional bounds guard.
            sparse_controls[k].vx = prev_controls_[t].vx + filtered_vx_noises[k];
            sparse_controls[k].wz = prev_controls_[t].wz + filtered_wz_noises[k];  
            // Keep the controls within safe bounds.
            sparse_controls[k].vx = std::max(vx_min_, std::min(sparse_controls[k].vx, vx_max_));
            sparse_controls[k].wz = std::max(wz_min_, std::min(sparse_controls[k].wz, wz_max_));
        }
        // Generate a dense control sequence by spline interpolation.
        samples[i] = splineInterpolation(sparse_controls);
    }
    
    
    return samples;// The returned samples are already dense sequences.
}

std::vector<Control> MPPI::splineInterpolation(const std::vector<Control>& sparse_controls) {
    std::vector<Control> dense_controls(T_);
    
    // // Simple linear interpolation; cubic splines can be used in practice.
    // for (int t = 0; t < T_; ++t) {
    //     double ratio = static_cast<double>(t) / (T_ - 1) * (K_ - 1);
    //     int k = static_cast<int>(ratio);
    //     double alpha = ratio - k;
        
    //     if (k >= K_ - 1) {
    //         dense_controls[t] = sparse_controls.back();
    //     } else {
    //         dense_controls[t].vx = (1 - alpha) * sparse_controls[k].vx + alpha * sparse_controls[k+1].vx;
    //         dense_controls[t].wz = (1 - alpha) * sparse_controls[k].wz + alpha * sparse_controls[k+1].wz;
    //     }
    // }
    
    // return dense_controls;

    int K = sparse_controls.size();
    
    if (K <= 1) {
        // Fall back to linear interpolation when too few points are available.
        for (int t = 0; t < T_; ++t) {
            dense_controls[t] = sparse_controls.front();
        }
        return dense_controls;
    }

    // Extract sparse vx and wz control points.
    std::vector<double> vx_sparse, wz_sparse;
    for (const auto& ctrl : sparse_controls) {
        vx_sparse.push_back(ctrl.vx);
        wz_sparse.push_back(ctrl.wz);
    }

    // Compute second derivatives using natural boundary conditions.
    std::vector<double> vx_y2 = computeSecondDerivatives(vx_sparse);
    std::vector<double> wz_y2 = computeSecondDerivatives(wz_sparse);

    // Generate the dense control sequence.
    for (int t = 0; t < T_; ++t) {
        // Compute the normalized position in [0, K-1].
        double ratio = static_cast<double>(t) / (T_ - 1) * (K - 1);
        int k = static_cast<int>(ratio);
        double alpha = ratio - k;

        // Handle the boundary.
        if (k >= K - 1) {
            dense_controls[t] = sparse_controls.back();
        } else {
            // Interpolate vx and wz separately using cubic splines.
            dense_controls[t].vx = cubicSplineInterp(vx_sparse, vx_y2, k, alpha);
            dense_controls[t].wz = cubicSplineInterp(wz_sparse, wz_y2, k, alpha);
            // Keep the controls within safe bounds.
            dense_controls[t].vx = std::max(vx_min_, std::min(dense_controls[t].vx, vx_max_));
            dense_controls[t].wz = std::max(wz_min_, std::min(dense_controls[t].wz, wz_max_));
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
        // Propagate each control input and append the resulting state to the trajectory.
    }
    return trajectory;
}

State MPPI::stateTransition(const State& state, const Control& control, double dt, const RotMat& B2G_RotMat) {
State next;
    double theta_T = state.yaw;  // Current body-frame yaw, corresponding to theta_T in the model.
    
    // 1. Construct the body-frame rotation matrix R_T(theta) = [cos theta, -sin theta; sin theta, cos theta].
    Eigen::Matrix2d R_T;
    R_T << cos(theta_T), -sin(theta_T),
           sin(theta_T),  cos(theta_T);
    // 2. Construct the input-velocity vector [MPPI_V_{T,b}, 0]^T from the model.
    // MPPI_V_{T,b} = control.vx, with zero lateral velocity along the body-forward direction.
    Eigen::Vector2d v_input(control.vx, 0.0);
    // 3. Resolve velocity by rotation-matrix multiplication as in model equation 3.
    // [V_bx,T; V_by,T] = R_T(θ) * [MPPI_V_{T,b}; 0]
    Eigen::Vector2d V_b_T = R_T * k1_* v_input;
    double V_bx_T = V_b_T(0);  // Body-frame X-axis velocity component.
    double V_by_T = V_b_T(1);  // Body-frame Y-axis velocity component.
    // 4. Update position using model equation 1: P_b,T+1 = P_b,T + V_b,T * dt.
    next.x = state.x + V_bx_T * dt;  // X_b,T+1 = X_b,T + V_bx,T * dt
    next.y = state.y + V_by_T * dt;  // Y_b,T+1 = Y_b,T + V_by,T * dt
    // 5. Update yaw using model equation 2: theta_T+1 = theta_T + MPPI_W_T * dt.
    next.yaw = state.yaw + k2_* control.wz * dt;  // MPPI_W_T = control.wz
    next.yaw = wrapAngle(next.yaw);  // Normalize the angle using the existing logic.
    
    return next;
}
double MPPI::calculateCost(const std::vector<State>& trajectory,
                            const std::vector<Control>& controls,
                            int sample_idx) {
    if (trajectory.empty()) 
    return 0.0;

    double total_cost = 0.0;
    double process_cost = 0.0;  // Running cost over all intermediate states.
    double terminal_cost = 0.0; // Terminal cost at the final state only.
    double obs_cost =0.0;
    double control_smooth_cost = 0.0;
    double yaw_cost =0.0;
    double dist_cost =0.0;

    double sum_dist_cost = 0.0;
    double sum_yaw_cost  = 0.0;
    double sum_obs_cost  = 0.0;
    double sum_ctrl_cost = 0.0;
    // Read the trajectory length and terminal state.
    size_t traj_len = trajectory.size();
    const auto& final_state = trajectory.back();

    // 1. Evaluate the running cost over all states except the terminal state.
    for (size_t i = 0; i < traj_len - 1; ++i) {  // Stop at the penultimate state.
        const auto& state = trajectory[i];
        
        // 1.1 Target-progress cost at intermediate states.
        Eigen::Vector2d local_pos(state.x, state.y);
        Eigen::Vector2d global_pos = B2G_RotMat_.block<2,2>(0,0) * local_pos;
        global_pos.x() += current_global_x_;
        global_pos.y() += current_global_y_;

        double dx = global_pos.x() - goal_x_;
        double dy = global_pos.y() - goal_y_;
        // dist_cost = distance_weight_ * 0.5 * (dx*dx + dy*dy);  // Alternative quadratic target cost.
        // Distance from the current point to the target.
        double di = std::hypot(goal_x_ - global_pos.x(), goal_y_ - global_pos.y());

        // Compute the next global position from trajectory[i+1].
        Eigen::Vector2d next_local(trajectory[i+1].x, trajectory[i+1].y);
        Eigen::Vector2d next_global = B2G_RotMat_.block<2,2>(0,0) * next_local;
        next_global.x() += current_global_x_;
        next_global.y() += current_global_y_;

        double di_next = std::hypot(goal_x_ - next_global.x(), goal_y_ - next_global.y());

        double progress = di_next - di;          // A negative value means progress toward the target.
        double progress_reward = std::min(0.0, progress);// Reward progress only.
        dist_cost = distance_weight_  * progress_reward;  // Progress lowers the total cost.

        sum_dist_cost += dist_cost;
        // Add the progress term to the running cost below.

        // 1.2 Running heading cost.
        double target_yaw = atan2(goal_y_ - global_pos.y(), goal_x_ - global_pos.x() + 1e-6);
        double current_global_yaw = atan2(B2G_RotMat_(1,0), B2G_RotMat_(0,0));
        double target_yaw_body = wrapAngle(target_yaw - current_global_yaw);
        double yaw_error = wrapAngle(target_yaw_body - state.yaw);

        double dist_to_goal = std::hypot(goal_x_ - global_pos.x(), goal_y_ - global_pos.y());
        // Adjust the heading weight according to distance from the target.
        double current_yaw_weight = (dist_to_goal < goal_near_threshold_) ? 
                           yaw_weight_near_ : yaw_weight_;
        yaw_cost = current_yaw_weight * (yaw_error * yaw_error);                   
        //yaw_cost = yaw_weight_ * (yaw_error * yaw_error);  // Alternative fixed running heading weight.
        sum_yaw_cost  += yaw_cost;
        // 1.3 Obstacle cost for continuous avoidance along the rollout.
        // {
        //     std::lock_guard<std::mutex> lock(costmap_mutex_);
        //     if (costmap_) {

        //         // // 1. Build the body-heading rotation matrix about the z axis.
        //         // double cos_yaw = cos(state.yaw);
        //         // double sin_yaw = sin(state.yaw);
                
        //         // // 2. Transform the local trajectory point into body-frame coordinates.
        //         // double x_body = state.x * cos_yaw - state.y * sin_yaw;  // Rotated x coordinate.
        //         // double y_body = state.x * sin_yaw + state.y * cos_yaw;  // Rotated y coordinate.
                
        //         // // 3. Query the costmap using the transformed coordinates.
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
        // 1.3 Obstacle cost replacing the previous obs_cost calculation.
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            if (costmap_) {
                Eigen::Matrix2d R_g2b = B2G_RotMat_.block<2,2>(0,0).transpose();
                Eigen::Vector2d p_in_costmap = R_g2b * (global_pos - Eigen::Vector2d(current_global_x_, current_global_y_));
                // Get the distance to the nearest obstacle in metres.
                double distance = getDistanceFromEDT(p_in_costmap.x(), p_in_costmap.y());
                // Alternative: query directly using map/world coordinates.
                //double distance = getDistanceFromEDT(global_pos.x(), global_pos.y());
                
                // Define the safety distance and obstacle-cost parameters.
                const double safe_distance = 0.3;  // Safety distance.
                const double influence_radius = 0.4;  // Influence radius; reducing either threshold by 0.05 m blocks passage.
                const double repulsion_strength = 1.5;
                // Increase cost near obstacles and set it to zero beyond the influence radius.
                if (distance < safe_distance) {
                    // Increase the cost sharply inside the safety distance.
                    //obs_cost = obstacle_weight_ * (1.0 - std::pow(distance / safe_distance, 2));
                    obs_cost = obstacle_weight_ * repulsion_strength * 
                    std::pow(1.0 / (distance + 1e-6 ), 2) ;
                } 
                else if (distance < influence_radius) {
                    // Reduce the cost between the safety distance and influence radius.
                    // double ratio = (distance - safe_distance) / (influence_radius - safe_distance);
                    // obs_cost = obstacle_weight_ * 0.1 * (1.0 - ratio);
                    obs_cost = obstacle_weight_ * 
                    std::pow(1.0 / (distance + 1e-6 ), 2) ;
                } 
                else {
                    // Apply no obstacle cost beyond the influence radius.
                    obs_cost = 0.0;
                }
                sum_obs_cost  += obs_cost;
            }
        }

        // 1.4 Control-smoothness cost over the rollout.
        // if (i > 1) {  // Compute second differences beginning with the third point.
        //     double dvx = trajectory[i].x - 2*trajectory[i-1].x + trajectory[i-2].x;
        //     double dwz = trajectory[i].yaw - 2*trajectory[i-1].yaw + trajectory[i-2].yaw;
        //     control_smooth_cost = control_weight_ * (dvx*dvx + dwz*dwz);

        //     sum_ctrl_cost += control_smooth_cost;
        // }

        // Replacement for the previous control-smoothness calculation in calculateCost():

        // 1.4 Control-smoothness cost penalizing command jumps.
        if (i > 0) {  // Compare each control with its predecessor starting at the second control.
            // Get the current and previous controls; trajectory index i corresponds to control index i.
            // controls[i] produces the intermediate trajectory state trajectory[i+1].
            const auto& current_ctrl = controls[i];    // Current control.
            const auto& prev_ctrl = controls[i-1];     // Previous control.
            
            // Compute the control-command changes.
            double dvx = current_ctrl.vx - prev_ctrl.vx;  // Linear-velocity change.
            double dwz = current_ctrl.wz - prev_ctrl.wz;  // Angular-velocity change.
            
            // Apply a squared penalty to control-command jumps.
            control_smooth_cost = control_weight_ * (dvx*dvx + dwz*dwz);
            sum_ctrl_cost += control_smooth_cost;
        }

        // Accumulate the running cost.
        process_cost += dist_cost + yaw_cost + obs_cost + control_smooth_cost;
    }

    // --------------------------
    // 2. Evaluate the terminal cost at the final state only.
    // --------------------------
    // 2.1 Terminal target cost.
    Eigen::Vector2d final_local(final_state.x, final_state.y);
    Eigen::Vector2d final_global = B2G_RotMat_.block<2,2>(0,0) * final_local;
    final_global.x() += current_global_x_;
    final_global.y() += current_global_y_;

    // double term_dx = final_global.x() - goal_x_;
    // double term_dy = final_global.y() - goal_y_;
    // terminal_cost += distance_weight_ * 2.0 * (term_dx*term_dx + term_dy*term_dy); 

    // 2.1 Terminal progress reward plus a light residual-distance penalty.
    // Compute d0 from the first global trajectory point to the target.
    Eigen::Vector2d init_local(trajectory.front().x, trajectory.front().y);
    Eigen::Vector2d init_global = B2G_RotMat_.block<2,2>(0,0) * init_local;
    init_global.x() += current_global_x_;
    init_global.y() += current_global_y_;

    double d0 = std::hypot(goal_x_ - init_global.x(),  goal_y_ - init_global.y());
    double dT = std::hypot(goal_x_ - final_global.x(), goal_y_ - final_global.y());

    // Progress is d0 - dT; negate it so greater progress acts as a reward.
    double terminal_progress = std::max(0.0, d0 - dT);
    terminal_cost += - terminal_progress_weight_ * terminal_progress;

    // Apply a light linear residual-distance penalty for stable convergence near the target.
    terminal_cost += terminal_residual_weight_ * dT;

    // 2.2 Terminal heading cost.
    double term_target_yaw = atan2(goal_y_ - final_global.y(), goal_x_ - final_global.x() + 1e-6);
    double term_current_yaw = atan2(B2G_RotMat_(1,0), B2G_RotMat_(0,0));
    double term_target_yaw_body = wrapAngle(term_target_yaw - term_current_yaw);
    double term_yaw_error = wrapAngle(term_target_yaw_body - final_state.yaw);

    double term_yaw_weight = (dT < goal_near_threshold_) ? yaw_weight_near_ : yaw_weight_;
    terminal_cost += term_yaw_weight * 8.0 * (term_yaw_error * term_yaw_error);
    //terminal_cost += yaw_weight_ * 8.0 * (term_yaw_error * term_yaw_error);  

    // Total cost is the running cost plus the terminal cost.
    total_cost = process_cost + terminal_cost;
    // Print the individual cost terms.
    // ROS_INFO_THROTTLE(1,
    //     "cost analysis: totalcost=%.3f | process=%.3f | terminal=%.3f | process distance cost=%.3f | process yaw cost=%.3f | process obstacle cost=%.3f,process control cost=%.3f" , 
    //     total_cost, process_cost, terminal_cost ,sum_dist_cost ,sum_yaw_cost ,sum_obs_cost ,sum_ctrl_cost
    // );
    return total_cost;
}

double MPPI::getCostmapValue(double x, double y) {//x,y->local position
    if (!costmap_) {
        std::cout << "[MPPI] the map is empty！" << std::endl;
        return 0.0;
    }
    // Convert world coordinates to grid coordinates.
    int mx, my;
    if (!costmap_->info.width || !costmap_->info.height) {
        std::cout << "[MPPI] map size is 00000000！" << std::endl;
        return 0.0;
    }
    
    double origin_x = costmap_->info.origin.position.x;// Expected value: -5.
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    
    mx = static_cast<unsigned int>(std::round((x - origin_x) / resolution));// std::round rounds to the nearest integer.
    my = static_cast<unsigned int>(std::round((y - origin_y) / resolution));

    // Check whether the point lies inside the map.
    if (mx >= costmap_->info.width || my >= costmap_->info.height|| mx<0 || my<0) {
        std::cout << "[MPPI] is out of the map，location：(" << x << ", " << y << ")" << std::endl;
        return 1.0;  // Treat points outside the map as obstacles.
    }
    
    // Read the grid cost in the range [0, 100].
    int idx = my * costmap_->info.width + mx;// Row-major index: my * width + mx.
    int cost = costmap_->data[idx];
    
    // // Print the queried local coordinates, grid index, and cost.
    // ROS_INFO_THROTTLE(0.1, "check the obstacle: local location(%.2f, %.2f) -> grid cost=%.2f", 
    //                  x, y, static_cast<double>(cost) / 100.0);// Throttle output to once every 0.5 seconds.
    // Normalize to [0, 1].
    return static_cast<double>(cost) / 100.0;
}

    std::vector<double> MPPI::calculateWeights(const std::vector<double>& costs) {
        std::vector<double> weights(N_);
        if (costs.empty()) return weights;

        // 1) Shift the values first to prevent exponential underflow.
        double min_cost = *std::min_element(costs.begin(), costs.end());
        std::vector<double> s(costs.size());// Allocate one shifted value per cost sample.
        for (size_t i = 0; i < costs.size(); ++i) {
            s[i] = -(costs[i] - min_cost) / lambda_;
        }
        double max_s = *std::max_element(s.begin(), s.end());

        // 2) Compute a normalized softmax.
        double sum = 0.0;
        for (auto &v : s) sum += std::exp(v - max_s);

        if (sum < 1e-12) {
            // Fall back to uniform weights in the degenerate case.
            for (auto &w : weights) w = 1.0 / double(N_);
            ROS_WARN("[MPPI] weight sum underflow, fallback to uniform.");
            return weights;
        }
        for (size_t i = 0; i < costs.size(); ++i) {
            weights[i] = std::exp(s[i] - max_s) / sum;
        }

        // Optional diagnostic output for the weight distribution.
        // ROS_INFO_THROTTLE(0.5, "[MPPI] w_min=%.3e, w_max=%.3e", 
        //                   *std::min_element(weights.begin(),weights.end()),
        //                   *std::max_element(weights.begin(),weights.end()));
        return weights;
    }

// Control MPPI::computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
//                                   const std::vector<double>& weights) {
//     Control optimal(0, 0);
    
//     // Compute the optimal control as a weighted sum.
//     for (int i = 0; i < N_; ++i) {
//         optimal.vx += weights[i] * control_samples[i][0].vx;
//         optimal.wz += weights[i] * control_samples[i][0].wz;
//     }
    
//     // Clamp the control input.
//     optimal.vx = my_clamp(optimal.vx, vx_min_, vx_max_);
//     optimal.wz = my_clamp(optimal.wz, wz_min_, wz_max_);
    
//     return optimal;
// }
// Return a vector to provide the complete optimal control sequence.
std::vector<Control> MPPI::computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
                                               const std::vector<double>& weights) {
    // Initialize one optimal control for each prediction step.
    std::vector<Control> optimal_controls(T_);
    
    // Compute the weighted control at every time step.
    for (int t = 0; t < T_; ++t) {  // Iterate over the prediction horizon.
        optimal_controls[t].vx = 0.0;
        optimal_controls[t].wz = 0.0;
        
        for (int i = 0; i < N_; ++i) {  // Iterate over all sampled trajectories.
            optimal_controls[t].vx += weights[i] * control_samples[i][t].vx;
            optimal_controls[t].wz += weights[i] * control_samples[i][t].wz;
        }
        
        // Clamp the control input.
        //optimal_controls[t].vx = my_clamp(optimal_controls[t].vx, vx_min_, vx_max_);
        //optimal_controls[t].wz = my_clamp(optimal_controls[t].wz, wz_min_, wz_max_);
    }
    
    return optimal_controls;  // Return the complete sequence.
}
// Timer callback for trajectory publication.
void MPPI::trajPubTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!latest_trajectories_.empty()) {
        publishTrajectories(latest_trajectories_,latest_B2G_RotMat_);
    }
}

void MPPI::publishTrajectories(const std::vector<std::vector<State>>& all_trajectories, const RotMat& B2G_RotMat) {
    // 1. Prepare a marker that can delete all trajectories from the previous frame.
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "trunk";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "mppi_trajectories";  // Use the same namespace as the trajectory markers.

    // delete_marker.action = visualization_msgs::Marker::DELETEALL;  // Delete every marker in this namespace.
    // traj_pub_.publish(delete_marker);

    visualization_msgs::MarkerArray marker_array;

    int id = 0;
    for (const auto& traj : all_trajectories) {
        // A LINE_STRIP requires at least two trajectory points.
        if (traj.size() < 2) {
            continue;  // Skip invalid trajectories.
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "trunk";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mppi_trajectories";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;  // LINE_STRIP is efficient for connected trajectories.
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;  // Set a unit quaternion.

        marker.scale.x = 0.01;
        marker.color.r = 0.1f;
        marker.color.g = 0.9f;
        marker.color.b = 0.1f;
        marker.color.a = 1.0f;     // Use a nonzero alpha value.

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
    // Publish all trajectories in one message.
    traj_pub_.publish(marker_array);  // traj_pub_ publishes MarkerArray messages.
    
}
void MPPI::publishOptimalTrajectory(const State& current_state, const RotMat& B2G_RotMat) {
    // 1. Roll out the optimal control sequence.
    State local_init_state(0, 0, 0);  // Initial state in the body frame.
    std::vector<State> optimal_trajectory = rollout(local_init_state, optimal_controls_, B2G_RotMat);

    // A LINE_STRIP requires at least two points.
    if (optimal_trajectory.size() < 2) {
        ROS_WARN("optimal_controls_ is < 2, cannot generate trajectory.");
        return;
    }
    
    // 2. Construct the marker message.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "trunk";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mppi_optimal_trajectory";
    marker.id = 0;  // Reuse a fixed ID so each update replaces the previous marker.
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;  // Set a unit quaternion.

    marker.scale.x = 0.02;  // Use a thicker line for emphasis.
    marker.color.r = 1.0f;  // Red.
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;  // Opaque.

    // 3. Convert and append trajectory points for publication.
    for (const auto& state : optimal_trajectory) {
        Eigen::Vector2d local_pos(state.x, state.y);
        geometry_msgs::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.25;  // Draw above the other trajectories to avoid occlusion.
        marker.points.push_back(p);
    }

    // 4. Publish the optimal trajectory.
    optimal_traj_pub_.publish(marker);
}
// Initialize a Butterworth filter.
void MPPI::initializeButterworthFilter(int order, double cutoff_freq, double dt, 
                                      std::vector<double>& b, std::vector<double>& a) {
    // Compute digital-filter coefficients using a simplified implementation.
    double fs = 1.0 / dt;              // Sampling frequency.
    double nyquist = 0.5 * fs;         // Nyquist frequency.
    double Wn = cutoff_freq / nyquist; // Normalized cutoff frequency.
    
    // Compute second-order Butterworth low-pass filter coefficients.
    if (order == 2) {
        double sqrt2 = sqrt(2.0);
        double theta = M_PI * Wn;
        double alpha = sin(theta) / (2.0 * cos(theta - M_PI/4.0));
        
        b = {1.0, 2.0, 1.0};
        for (auto& val : b) val *= alpha * alpha;
        
        a = {1.0, -2.0 * cos(theta), 2.0 * alpha * alpha - 1.0};
    }
    // This can be extended to higher-order filters.
}

// Apply the filter.
double MPPI::applyButterworthFilter(double input, std::vector<double>& b, 
                                   std::vector<double>& a, std::vector<double>& prev) {
    int order = b.size() - 1;
    double output = b[0] * input;
    
    // Apply the filter coefficients.
    for (int i = 1; i <= order; ++i) {
        if (prev.size() > i - 1) {
            output += b[i] * prev[i - 1];
        }
    }
    
    for (int i = 1; i <= order; ++i) {
        if (prev.size() > i - 1) {
            output -= a[i] * prev[i - 1];
        }
    }
    
    // Update the history.
    for (int i = order - 1; i > 0; --i) {
        if (prev.size() > i) {
            prev[i] = prev[i - 1];
        }
    }
    if (!prev.empty()) {
        prev[0] = input;
    }
    return output;
}

// Convert a quaternion to yaw.
double MPPI::quatToYaw(double x, double y, double z, double w) {
    double siny = 2.0 * (w * z + x * y);
    double cosy = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny, cosy);
}

// Callback for mu values.
void MPPI::muCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        std::lock_guard<std::mutex> lock(mu_yaw_mutex_);
        mu1_ = msg->data[0];
        mu2_ = msg->data[1];
        mu3_ = msg->data[2];
        calculateCompensationCoefficients(); // Compute coefficients after receiving mu.
    }
}

// Gazebo model-state callback.
void MPPI::gazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == robot_model_name_) {
            std::lock_guard<std::mutex> lock(mu_yaw_mutex_);
            // Extract the yaw angle.
            current_yaw_ = quatToYaw(
                msg->pose[i].orientation.x,
                msg->pose[i].orientation.y,
                msg->pose[i].orientation.z,
                msg->pose[i].orientation.w
            );
            calculateCompensationCoefficients(); // Compute coefficients after receiving yaw.
            break;
        }
    }
}

// Compute compensation coefficients k1 and k2.
void MPPI::calculateCompensationCoefficients() {
    // Ensure that the required data have been received.
    if (mu1_ != 0.0 || mu2_ != 0.0) {
        double cos_yaw = std::cos(current_yaw_);
        double sin_yaw = std::sin(current_yaw_);
        k1_ = mu1_ * cos_yaw * cos_yaw + mu2_ * sin_yaw * sin_yaw;
        k2_ = mu3_;
        // Optional diagnostic output.
        // ROS_INFO_THROTTLE(0.5, "k1: %.3f, k2: %.3f", k1_, k2_);
    }
}
