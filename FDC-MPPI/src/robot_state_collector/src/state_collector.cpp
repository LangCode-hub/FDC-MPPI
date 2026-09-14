#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "xlsxwriter.h"
#include <boost/shared_ptr.hpp>

// Custom timestamped-message wrapper.
template <typename T>
struct StampedMessage {
    boost::shared_ptr<const T> msg;
    ros::Time stamp; // Message reception timestamp.
};

// Structure that stores the robot state.
struct RobotState {
    double x;       // Global X coordinate.
    double y;       // Global Y coordinate.
    double yaw;     // Yaw angle.
    double vx;      // Velocity along the global X direction.
    double vy;      // Velocity along the global Y direction.
    double wz;      // Yaw rate.
    ros::Time time; // Synchronized timestamp.
    double ctrl_vx_global;  // Control velocity along global x.
    double ctrl_vy_global;  // Control velocity along global y.
    double ctrl_wz_global;  // Control angular velocity about global z.
};

class StateCollector {
private:
    ros::NodeHandle nh_;
    lxw_workbook* workbook_;
    lxw_worksheet* worksheet_;
    int row_ = 0;    // Excel row index.
    std::mutex state_mutex_;
    RobotState current_state_;
    std::string robot_model_name_ = "go2_gazebo"; // Robot model name.

    // Raw subscribers used to record reception times.
    ros::Subscriber model_sub_;
    ros::Subscriber control_sub_;

    // Timestamped message buffers.
    StampedMessage<gazebo_msgs::ModelStates> last_model_msg_;
    StampedMessage<geometry_msgs::Twist> last_control_msg_;
    std::mutex msg_mutex_;

    // Convert a quaternion to a yaw angle.
    double quatToYaw(double x, double y, double z, double w) {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // Model-state callback that records the reception time.
    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        last_model_msg_.msg = msg;
        last_model_msg_.stamp = ros::Time::now(); // Record the reception time.
        trySync(); // Try to match a control message.
    }

    // Control-command callback that records the reception time.
    void controlCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        last_control_msg_.msg = msg;
        last_control_msg_.stamp = ros::Time::now(); // Record the reception time.
        trySync(); // Try to match a model-state message.
    }

    // Try to synchronize the two messages by reception time.
    void trySync() {
        if (!last_model_msg_.msg || !last_control_msg_.msg) return;

        // Check the time difference (up to 50 ms is allowed).
        ros::Duration time_diff = last_model_msg_.stamp - last_control_msg_.stamp;
        if (fabs(time_diff.toSec()) < 0.03) {
            processSyncData(last_model_msg_.msg, last_control_msg_.msg, 
                           last_model_msg_.stamp); // Use reception time as the synchronization time.
            // Clear the buffers to avoid duplicate processing.
            last_model_msg_.msg.reset();
            last_control_msg_.msg.reset();
        }
    }

    // Process synchronized data.
    void processSyncData(const gazebo_msgs::ModelStates::ConstPtr& model_msg,
                        const geometry_msgs::Twist::ConstPtr& ctrl_msg,
                        const ros::Time& sync_time) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        RobotState new_state;

        // Extract robot-state data.
        for (size_t i = 0; i < model_msg->name.size(); i++) {
            if (model_msg->name[i] == robot_model_name_) {
                new_state.x = model_msg->pose[i].position.x;
                new_state.y = model_msg->pose[i].position.y;
                new_state.yaw = quatToYaw(
                    model_msg->pose[i].orientation.x,
                    model_msg->pose[i].orientation.y,
                    model_msg->pose[i].orientation.z,
                    model_msg->pose[i].orientation.w
                );
                new_state.vx = model_msg->twist[i].linear.x;
                new_state.vy = model_msg->twist[i].linear.y;
                new_state.wz = model_msg->twist[i].angular.z;
                break;
            }
        }

        // Extract control data.
        new_state.ctrl_vx_global = ctrl_msg->linear.x;
        new_state.ctrl_vy_global = ctrl_msg->linear.y;
        new_state.ctrl_wz_global = ctrl_msg->angular.z;

        // Use the manually recorded synchronization timestamp.
        new_state.time = sync_time;

        current_state_ = new_state;
    }

    // Timer callback that writes to Excel.
    void timerCallback(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // Write state data to Excel.
        worksheet_write_number(worksheet_, row_, 0, current_state_.time.toSec(), NULL);
        worksheet_write_number(worksheet_, row_, 1, current_state_.x, NULL);
        worksheet_write_number(worksheet_, row_, 2, current_state_.y, NULL);
        worksheet_write_number(worksheet_, row_, 3, current_state_.yaw, NULL);
        worksheet_write_number(worksheet_, row_, 4, current_state_.vx, NULL);
        worksheet_write_number(worksheet_, row_, 5, current_state_.vy, NULL);
        worksheet_write_number(worksheet_, row_, 6, current_state_.wz, NULL);

        // Write control data.
        worksheet_write_number(worksheet_, row_, 7, current_state_.ctrl_vx_global, NULL);
        worksheet_write_number(worksheet_, row_, 8, current_state_.ctrl_vy_global, NULL);
        worksheet_write_number(worksheet_, row_, 9, current_state_.ctrl_wz_global, NULL);
        row_++;
    }

public:
    StateCollector() {
        // Initialize subscribers with a queue size of 10.
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, &StateCollector::modelCallback, this);
        control_sub_ = nh_.subscribe("/unitree_gazebo_servo/mppi_global_control", 10, &StateCollector::controlCallback, this);

        // Create the Excel file.
        workbook_ = workbook_new("robot_state_data.xlsx");
        worksheet_ = workbook_add_worksheet(workbook_, NULL);
        
        // Write the column headers.
        const char* headers[] = {"时间戳(秒)", "X(m)", "Y(m)", "Yaw(rad)", 
                                "VX(m/s)", "VY(m/s)", "WZ(rad/s)",
                                "Ctrl_VX_global(m/s)", "Ctrl_VY_global(m/s)", "Ctrl_WZ_global(rad/s)"};
        for (int col = 0; col < 10; col++) {
            worksheet_write_string(worksheet_, row_, col, headers[col], NULL);
        }
        row_++;

        // Set a timer with a 10 ms interval.
        ros::Timer timer = nh_.createTimer(ros::Duration(0.01), 
                                         &StateCollector::timerCallback, this);

        ros::spin();
    }

    ~StateCollector() {
        workbook_close(workbook_); // Close the Excel file.
    }
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "state_collector");
    StateCollector collector;
    return 0;
}
