/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "control/MPPI.h"
#include <memory>  // Provides std::make_unique and std::unique_ptr.

#include "control/ThreadSafeQueue.h"
#include <thread>
#include <atomic>

#include "gazebo_msgs/ModelStates.h"
#include <tf2/LinearMath/Quaternion.h>
class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
    // Set the obstacle-avoidance target (added 2025-10-18).
    void setObstacleAvoidanceGoal(double x, double y);
private:
    // Gazebo state shared by the controller callbacks.
    ros::Subscriber gazebo_model_sub_;
    Eigen::Vector3d _gazebo_pos;  // Position received from Gazebo.
    std::mutex gazebo_mutex_;     // Protects the shared position data.

    ros::Time _lastGazeboTime;
    ros::Time _enterTime;
    void gazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    
    void mppiThreadFunc();  // MPPI worker-thread entry point.
    
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);

    // MPPI controller.
    std::unique_ptr<MPPI> _mppi;
    ros::NodeHandle _nh;
    bool _obstacleAvoidanceMode = false;  // Obstacle-avoidance mode switch.
    Vec2 _avoidanceGoal;                  // Obstacle-avoidance target.
    bool _MPPICompleted = false;    // Whether the autonomous task has completed.
    double _goalTolerance = 0.42;  // Goal tolerance in metres.

    // Thread-related state.
    std::thread _mppiThread;                // MPPI computation thread.
    std::atomic<bool> _mppiRunning{false};  // Worker-thread running flag.
    ThreadSafeQueue<Control> _controlQueue; // Control-command queue.
    ThreadSafeQueue<State> _stateQueue;     // State queue.
    Control _lastOptimalControl;            // Most recent control command.

    // Disturbance-force state.
    bool _applyingForce = false;          // Whether a disturbance force is active.
    ros::Time _forceStartTime;            // Disturbance-force start time.
    const double _forceDuration = 0.2;    // Disturbance-force duration in seconds.
    Eigen::Vector3d _disturbanceForce;    // Disturbance magnitude and direction (x, y, z).
    //ros::Publisher _forcePub;             // Optional publisher for applying a force in Gazebo.
    ros::ServiceClient _applyForceClient;  // Client for the Gazebo force service.
    bool _lastL2XPressed = false;  // Previous L2_X key state.

    // Force visualization.
    ros::Publisher _forceMarkerPub;  // Force-arrow publisher.
    visualization_msgs::Marker _forceMarker;  // Force-arrow marker.
    void initializeForceMarker();    // Initialize the force-arrow marker.
    void updateForceMarker();  // Update and publish the force-arrow marker.
};

#endif  // TROTTING_H
