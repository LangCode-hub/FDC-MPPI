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
#include <Eigen/Dense>  // Eigen matrix library.
#include <visualization_msgs/Marker.h>  // Marker message definition.
#include <visualization_msgs/MarkerArray.h>
// OpenCV and ROS image headers.
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/ModelStates.h"

using RotMat = Eigen::Matrix3d;  // Match the SDK type and avoid conflicts.

// State variables: x, y, and yaw.
struct State {
    double x=0;
    double y=0;
    double yaw=0;
    
    State() : x(0), y(0), yaw(0) {}
    State(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

// Control variables: longitudinal velocity and yaw rate.
struct Control {
    double vx;    // Forward velocity.
    double wz;    // Yaw rate.
    
    Control() : vx(0), wz(0) {}
    Control(double vx_, double wz_) : vx(vx_), wz(wz_) {}
};

template <typename T>  // Supports numeric types such as int and double.
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
    
    // Set the target position.
    void setGoal(double x, double y);
    // Get the optimal control input.
    Control getOptimalControl(const State& current_state, double global_x, double global_y, const RotMat& B2G_RotMat);
    
    // Costmap callback.
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    // Return the current robot position (x, y) in the global map frame.
    Eigen::Vector2d getCurrentGlobalPos() const {
    return Eigen::Vector2d(current_global_x_, current_global_y_);
    }
    void publishTrajectories(const std::vector<std::vector<State>>& all_trajectories, 
                             const RotMat& B2G_RotMat);


    
    
private:
    Eigen::Vector2d current_world_xy_;

    // Sample control sequences.
    std::vector<std::vector<Control>> sampleControls();
    
    // Roll out a predicted trajectory.
    std::vector<State> rollout(const State& init_state, const std::vector<Control>& controls, const RotMat& B2G_RotMat);
    
    // Evaluate a trajectory cost.
    double calculateCost(const std::vector<State>& trajectory,const std::vector<Control>& controls,int sample_idx);
    
    // Compute weights.
    std::vector<double> calculateWeights(const std::vector<double>& costs);
    
    // Compute the optimal control sequence.
    // Control computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
    //                              const std::vector<double>& weights);
    std::vector<Control> computeOptimalControl(const std::vector<std::vector<Control>>& control_samples, 
                                         const std::vector<double>& weights);
    
    // Generate a dense control sequence by spline interpolation.
    std::vector<Control> splineInterpolation(const std::vector<Control>& sparse_controls);
    
    // Read a point cost from the costmap.
    double getCostmapValue(double x, double y);
    
    // State-transition function.
    State stateTransition(const State& state, const Control& control, double dt, const RotMat& B2G_RotMat);

    // ROS interfaces.
    ros::Subscriber costmap_sub_;
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    std::mutex costmap_mutex_;

    ros::Subscriber edt_sub_;  // EDT-map subscriber.
    cv::Mat edt_map_;          // Stored EDT map.
    std::mutex edt_mutex_;     // Protect thread-safe access to the EDT map.

    ros::Publisher traj_pub_;
    ros::Timer traj_pub_timer_;
    std::vector<std::vector<State>> latest_trajectories_;
    std::mutex traj_mutex_;

    ros::Publisher optimal_traj_pub_;  // Optimal-trajectory publisher.
    std::vector<Control> optimal_controls_;  // Complete optimal control sequence.
    // These private members use a trailing underscore to distinguish them from local variables.
    // Keeping the sequence as a member makes the computeOptimalControl result available to other methods.

    // EDT-map callback.
    void edtCallback(const sensor_msgs::Image::ConstPtr& msg);
    // Query the EDT distance.
    double getDistanceFromEDT(double x, double y);

    void trajPubTimerCallback(const ros::TimerEvent& event);  

    void publishOptimalTrajectory(const State& current_state, const RotMat& B2G_RotMat);

    // Store the latest rotation matrix for trajectory-frame conversion during publication.
    RotMat latest_B2G_RotMat_;  // Rotation matrix associated with the latest trajectories.

    // Temporary state used for frame conversion.
    RotMat B2G_RotMat_;  // Current body-to-global rotation matrix.
    double current_global_x_;  // Current global x position.
    double current_global_y_;  // Current global y position.
    
    // MPPI parameters.
    int N_;                  // Number of samples.
    int T_;                  // Prediction horizon.
    int K_;                  // Number of sparse control points.
    double dt_;              // Control time step.
    double lambda_;          // Temperature parameter.
    // Control bounds.
    double vx_min_, vx_max_;
    double wz_min_, wz_max_;
    // Control-noise standard deviations.
    double vx_std_, wz_std_;
    // Cost-function weights.
    double distance_weight_;     // Target-progress weight.
    double obstacle_weight_; // Obstacle-cost weight.
    double control_weight_;  // Control-smoothness weight.
    double yaw_weight_ ;// Heading-control weight.
    // Target position.
    double goal_x_, goal_y_;
    
    double terminal_progress_weight_;
    double terminal_residual_weight_;

    double yaw_weight_near_;  // Heading weight near the target.
    double goal_near_threshold_;  // Near-target distance threshold in metres.
    
    // Random-number generator.
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
    
    // Optimal control sequence from the previous cycle.
    std::vector<Control> prev_controls_;

    std::vector<double> computeSecondDerivatives(const std::vector<double>& x) {
        int n = x.size() - 1;  // Number of intervals = number of points - 1.
        std::vector<double> y2(n + 1, 0.0);  // Second derivatives.
        std::vector<double> u(n, 0.0);       // Temporary array.

        // Natural boundary conditions: zero second derivative at both ends.
        y2[0] = 0.0;
        u[0] = 0.0;

        // Forward elimination.
        for (int i = 1; i < n; ++i) {
            double sig = (i - 1) / (double)(i);
            double p = sig * y2[i - 1] + 2.0;
            y2[i] = (sig - 1.0) / p;
            u[i] = (x[i + 1] - 2.0 * x[i] + x[i - 1]) / (double)i;
            u[i] = (6.0 * u[i] / (2.0 * i) - sig * u[i - 1]) / p;
        }

        // Back substitution.
        for (int k = n - 1; k >= 0; --k) {
            y2[k] = y2[k] * y2[k + 1] + u[k];
        }

        return y2;
    }

    // Evaluate one point using cubic-spline interpolation.
    double cubicSplineInterp(const std::vector<double>& x, const std::vector<double>& y2, 
                            int k, double alpha) {
        int n = x.size() - 1;
        double h = 1.0;  // Normalized interval with unit spacing.
        
        double a = (1.0 + alpha) / 2.0;
        double b = (1.0 - alpha) / 2.0;
        double c = (a * a * a - a) * (h * h) / 6.0;
        double d = (b * b * b - b) * (h * h) / 6.0;
        
        return a * x[k + 1] + b * x[k] + c * y2[k + 1] + d * y2[k];
    }
    // Butterworth-filter parameters.
    std::vector<double> vx_b, vx_a;  // vx noise-filter coefficients.
    std::vector<double> wz_b, wz_a;  // wz noise-filter coefficients.
    std::vector<double> vx_prev;     // vx noise-filter history.
    std::vector<double> wz_prev;     // wz noise-filter history.
    int filter_order_ = 2;           // Filter order.
    double cutoff_freq_ = 5.0;       // Cutoff frequency (Hz).

    // Filter-related function declarations.
    void initializeButterworthFilter(int order, double cutoff_freq, double dt, 
                                    std::vector<double>& b, std::vector<double>& a);
    double applyButterworthFilter(double input, std::vector<double>& b, 
                                 std::vector<double>& a, std::vector<double>& prev);
                                 
    Control last_cmd_{0.0, 0.0};
    bool has_last_cmd_ = false;

    ros::Subscriber mu_sub_;          // Subscribes to estimated mu values.
    ros::Subscriber gazebo_model_sub_; // Subscribes to Gazebo model states.
    std::string robot_model_name_ = "go2_gazebo"; // Robot model name.
    double mu1_, mu2_, mu3_;          // Stored mu values.
    double current_yaw_;              // Current yaw angle.
    double k1_, k2_;                  // Compensation coefficients (global variables).
    std::mutex mu_yaw_mutex_;         // Protects mu and yaw data.
    

    void muCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void gazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    double quatToYaw(double x, double y, double z, double w); // Convert a quaternion to yaw.
    void calculateCompensationCoefficients(); // Compute k1 and k2.
};

#endif
