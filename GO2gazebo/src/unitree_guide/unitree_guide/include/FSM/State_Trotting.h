/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "control/MPPI.h"
#include <memory>  // 用于 std::make_unique 和 std::unique_ptr

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
    // 设置避障目标点(2025.10.18 add)
    void setObstacleAvoidanceGoal(double x, double y);
private:
    // 在State_Trotting类的private部分添加
    ros::Subscriber gazebo_model_sub_;
    Eigen::Vector3d _gazebo_pos;  // 存储从gazebo获取的位置
    std::mutex gazebo_mutex_;     // 保护位置数据的互斥锁

    ros::Time _lastGazeboTime;
    ros::Time _enterTime;
    void gazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    
    void mppiThreadFunc();  // 关键：声明线程函数
    
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

    // MPPI控制器
    std::unique_ptr<MPPI> _mppi;
    ros::NodeHandle _nh;
    bool _obstacleAvoidanceMode = false;  // 避障模式开关
    Vec2 _avoidanceGoal;                  // 避障目标点
    bool _MPPICompleted = false;    // 自动任务是否完成
    double _goalTolerance = 0.42;  // 到达目标点的容忍范围（米）

    // 线程相关变量
    std::thread _mppiThread;                // MPPI计算线程
    std::atomic<bool> _mppiRunning{false};  // 线程运行标志
    ThreadSafeQueue<Control> _controlQueue; // 控制指令队列
    ThreadSafeQueue<State> _stateQueue;     // 状态队列
    Control _lastOptimalControl;            // 最新控制指令

    // 干扰力相关变量
    bool _applyingForce = false;          // 是否正在施加干扰力
    ros::Time _forceStartTime;            // 干扰力开始时间
    const double _forceDuration = 0.2;    // 干扰力持续时间(秒)
    Eigen::Vector3d _disturbanceForce;    // 干扰力大小和方向(x,y,z)
    //ros::Publisher _forcePub;             // 用于发布力到Gazebo的publisher
    ros::ServiceClient _applyForceClient;  // 用于调用Gazebo施加力的服务
    bool _lastL2XPressed = false;  // 记录上一时刻L2_X按键状态

    // 力可视化相关
    ros::Publisher _forceMarkerPub;  // 力箭头发布器
    visualization_msgs::Marker _forceMarker;  // 力箭头Marker
    void initializeForceMarker();    // 初始化力箭头Marker的函数
    void updateForceMarker();  // 更新并发布力箭头Marker
};

#endif  // TROTTING_H
