/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>
#include "gazebo_msgs/ModelStates.h"
#include <ros/time.h>
#include <gazebo_msgs/ApplyBodyWrench.h>  // 引入ApplyBodyWrench消息定义


State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl),_nh("~"){//xin jia le _nh("~")
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

    // 初始化MPPI控制器10.18new
    _mppi = std::make_unique<MPPI>(_nh);

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    // 初始化gazebo模型状态订阅
    gazebo_model_sub_ = _nh.subscribe("/gazebo/model_states", 10, 
                                     &State_Trotting::gazeboModelStatesCallback, this);
    _gazebo_pos.setZero();  // 初始化为零

    // 初始化最后控制指令
    _lastOptimalControl = Control(0, 0);
    
    // 启动MPPI线程
    _mppiRunning = true;
    _mppiThread = std::thread(&State_Trotting::mppiThreadFunc, this);

    _applyForceClient = _nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    // 设置默认干扰力(可自定义大小和方向)
    _disturbanceForce = Eigen::Vector3d(35, 0, 0);

    // 初始化力箭头发布器
    _forceMarkerPub = _nh.advertise<visualization_msgs::Marker>("disturbance_force_marker", 10);
    
    // 初始化力箭头Marker
    initializeForceMarker();
}

State_Trotting::~State_Trotting(){
    delete _gait;
    _mppiRunning = false;
    if (_mppiThread.joinable()) {
        _mppiThread.join();
    }
}

void State_Trotting::enter(){
    //_pcd = _est->getPosition();//设置初始目标位置 _pcd(_pcd 是期望身体位置)
    _enterTime = ros::Time::now();
    {
        std::lock_guard<std::mutex> lock(gazebo_mutex_);
        _pcd = _gazebo_pos;
    }

    // 初始化时打印一次初始位置）
    ROS_INFO("initiallllll positionnnnnnnnnnn: (%.2f, %.2f, %.2f)", _pcd.x(), _pcd.y(), _pcd.z());//初始位置赋值没问题
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);//()(2, 0)函数返回矩阵后立刻取下标
    //getFeetPosIdeal()(2,0) 给出的是“默认站立时，脚底到髋部的高度差”。
    //前面加负号就让 _pcd(2) 变成“身体应该漂在地面以上多高”。
    //_pcd(2)表示取这个向量的第 3 个元素

    _vCmdBody.setZero();//身体坐标系下的速度命令清零
    _yawCmd = _lowState->getYaw();//把当前真实朝向（ yaw 角）当成“希望保持的朝向”，后面所有偏航控制都围绕这个值
    _Rd = rotz(_yawCmd);//根据目标 yaw 生成 3×3 的旋转矩阵 _Rd，表示“希望身体最终对向哪个方向”。
                        //rotz() 是工具函数：绕世界 z 轴旋转。
    _wCmdGlobal.setZero();//世界坐标系下的角速度命令清零：我不想转圈。

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

    //setObstacleAvoidanceGoal(5, 0.0);
    setObstacleAvoidanceGoal(6.1, 2);

}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){ //dui ying jian pan shi '2'
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    {
        std::lock_guard<std::mutex> g(gazebo_mutex_);  // 使用gazebo的互斥锁
        ros::Duration lag = ros::Time::now() - _lastGazeboTime;
        if (lag.toSec() > 0.05) {
            ROS_WARN_THROTTLE(1.0, "Gazebo pose data delayed by %.3f s", lag.toSec());
        }
        _posBody = _gazebo_pos;  // 替换_est->getPosition()
    }
    
    {
        std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);
        _velBody = _est->getVelocity();  // 速度仍从估计器获取（如果需要）
    }
    // ROS_INFO_THROTTLE(0.5, "Current posBody(print in run function): (%.2f, %.2f, %.2f)", 
    //                   _posBody.x(), _posBody.y(), _posBody.z());

    //  {
    //     std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);  // 加锁保护写入
    //     _posBody = _est->getPosition();
    //     _velBody = _est->getVelocity();
    //     ROS_INFO_THROTTLE(0.5, "Current posBody(print in run function): (%.2f, %.2f, %.2f)", 
    //                  _posBody.x(), _posBody.y(), _posBody.z());
    // }  // 自动释放锁
    // _posBody = _est->getPosition();
    // _velBody = _est->getVelocity();
    // ROS_INFO_THROTTLE(0.5, "Current posBody(print in run function): (%.2f, %.2f, %.2f)", 
    //                  _posBody.x(), _posBody.y(), _posBody.z());
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    //_vCmdGlobal.segment(0,2) deng jia yv Vector2d(vx,vy),biao shi shi jie zuo biao xi xia qi wang de ji shen shui ping su du
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
}

void State_Trotting::gazeboModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(gazebo_mutex_);
    // 查找机器人模型索引（模型名为"go2_gazebo"）
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "go2_gazebo") {
            // 提取位置信息
            _gazebo_pos.x() = msg->pose[i].position.x;
            _gazebo_pos.y() = msg->pose[i].position.y;
            _gazebo_pos.z() = msg->pose[i].position.z;
            _lastGazeboTime = ros::Time::now();/////
            break;
        }
    }
}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

// 添加设置目标点函数
void State_Trotting::setObstacleAvoidanceGoal(double x, double y) {
    _avoidanceGoal = Vec2(x, y);
    _mppi->setGoal(x, y);
    _MPPICompleted = false;  // 重置完成状态（新目标开始时未完成）
    ROS_INFO("set the goal point: (%.2f, %.2f)", x, y); 
}

// 添加MPPI线程函数
void State_Trotting::mppiThreadFunc() {
    ros::Rate rate(50.0);  // 设置MPPI计算频率
    while (_mppiRunning) {
        // 等待新的状态
        State current_state; //是在getUserCmd 函数下面定义的，

        RotMat B2G_RotMat;          // 先留空
        //通过线程安全队列 _stateQueue 从主线程（State_Trotting::run 或 getUserCmd 函数）传递到 mppiThreadFunc 线程
        if(_stateQueue.wait_and_pop(current_state)){
            {
        // 2. 临界区：只在这几行里碰 _lowState
                std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);
                B2G_RotMat = _lowState->getRotMat();   // 拿旋转矩阵
                // 如果还需要 IMU、足端位置，也在这里一次性读完
            } // 3. 锁在这里自动释放，后面不再碰 _lowState
        // 获取当前旋转矩阵（需要从主线程传递或这里获取）
        //RotMat B2G_RotMat = _lowState->getRotMat();
        // 计算最优控制
        if (_obstacleAvoidanceMode) {
            //Control optimal = _mppi->getOptimalControl(current_state, B2G_RotMat);

            Control optimal = _mppi->getOptimalControl(current_state, _gazebo_pos.x(), _gazebo_pos.y(),B2G_RotMat);
            //ROS_INFO("MPPI computed: vx=%.2f m/s, wz=%.2f rad/s", optimal.vx, optimal.wz);  // 先注释掉，后面再给加回来
            //若vx始终为 0 或wz不变，说明 MPPI 未检测到障碍物（可能代价地图解析错误）。
            //若接近障碍物时wz有明显变化（如转向），说明 MPPI 决策正常，问题在指令传递。
            _controlQueue.push(optimal);
        }
    }
    rate.sleep();  // 按设定频率休眠
    }
}
// 添加初始化Marker的函数
void State_Trotting::initializeForceMarker() {
    _forceMarker.header.frame_id = "base";  // 使用世界坐标系
    _forceMarker.ns = "disturbance_force";
    _forceMarker.id = 0;
    _forceMarker.type = visualization_msgs::Marker::ARROW;
    _forceMarker.action = visualization_msgs::Marker::ADD;
    
    // 设置箭头尺寸
    _forceMarker.scale.x = 0.4;  // 箭头长度
    _forceMarker.scale.y = 0.1; // 箭头宽度
    _forceMarker.scale.z = 0.1;  // 箭头高度
    
    // 设置箭头颜色
    _forceMarker.color.r = 1.0f;
    _forceMarker.color.g = 1.0f;
    _forceMarker.color.b = 0.0f;
    _forceMarker.color.a = 1.0;  // 不透明度
}

void State_Trotting::updateForceMarker() {
    if (!_applyingForce) {
        _forceMarker.color.a = 0.01;
    } else {
        _forceMarker.color.a = 1.0;
        _forceMarker.header.stamp = ros::Time::now();
        
        // 机体坐标系下的位置：身后1米（x=-1），y=0（正中间），z=0.2（离地高度）
        _forceMarker.pose.position.x = -1.0;  // x负方向为身后
        _forceMarker.pose.position.y = 0.0;
        _forceMarker.pose.position.z = 0.2;
        
        // 力的方向：基于机体坐标系（直接使用_force的分量，无需转换旋转）
        Eigen::Vector3d forceDir = _disturbanceForce.normalized();
        // 计算从机体坐标系z轴到力方向的旋转（因力是基于机体坐标系施加的）
        tf2::Quaternion quat;
        quat.setRPY(0, 0, atan2(forceDir.y(), forceDir.x()));  // 仅考虑xy平面内的方向
        _forceMarker.pose.orientation.x = quat.x();
        _forceMarker.pose.orientation.y = quat.y();
        _forceMarker.pose.orientation.z = quat.z();
        _forceMarker.pose.orientation.w = quat.w();
    }
    _forceMarkerPub.publish(_forceMarker);
}
void State_Trotting::getUserCmd(){
// 添加避障模式开关 (使用4按键)
    if (_lowState->userCmd == UserCommand::START && !_obstacleAvoidanceMode && !_MPPICompleted) {
        _obstacleAvoidanceMode = true;
        ROS_INFO("Obstacle avoidance mode: %s", _obstacleAvoidanceMode ? "ON" : "OFF");
    }

    // 仅在避障模式开启时判断是否到达目标
    if (_obstacleAvoidanceMode && !_MPPICompleted) {
        // 获取机器人当前位置（假设从估计器获取x, y坐标）
        // Eigen::Vector3d currentPos = _est->getPosition(); 
        // Eigen::Vector2d currentXY(currentPos.x(), currentPos.y());

        // 获取机器人当前位置（加锁保护共享资源访问）
        Eigen::Vector3d currentPos;
        {
            std::lock_guard<std::mutex> g(gazebo_mutex_);  // 使用gazebo的锁
            currentPos = _gazebo_pos;

            // ROS_INFO_THROTTLE(0.2,  // 每0.2秒打印一次，避免刷屏
            // "current position: (%.2f, %.2f), goal point: (%.2f, %.2f)",
            // currentPos.x(), currentPos.y(),_avoidanceGoal.x(), _avoidanceGoal.y());
        }
        // {
        // std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);  // 加锁临界区
        // currentPos = _est->getPosition();  // 安全读取位置

        // ROS_INFO_THROTTLE(0.2,  // 每0.2秒打印一次，避免刷屏
        // "current position: (%.2f, %.2f), goal point: (%.2f, %.2f)",
        // currentPos.x(), currentPos.y(),_avoidanceGoal.x(), _avoidanceGoal.y()); 
        // }  // 自动释放锁
        
        Eigen::Vector2d currentXY(currentPos.x(), currentPos.y());

        // ROS_INFO_THROTTLE(0.2,  // 每0.2秒打印一次，避免刷屏
        // "current position: (%.2f, %.2f), goal point: (%.2f, %.2f), distance between them: %.2f",
        // currentPos.x(), currentPos.y(),_avoidanceGoal.x(), _avoidanceGoal.y(),(currentXY - _avoidanceGoal).norm()
        // );  

        double distanceToGoal = (currentXY - _avoidanceGoal).norm();
        if (distanceToGoal < _goalTolerance) {
            _MPPICompleted = true;
            ROS_INFO("already reach it!!!!distancedistancedistancedistance: %.2fm, goal position: (%.2f, %.2f)", 
         distanceToGoal, 
         _avoidanceGoal.x(), 
         _avoidanceGoal.y());
        }
    }

        //到达目标后，速度指令为0
    if (_MPPICompleted) {
        // 清零线速度和角速度指令（根据实际变量名调整）
        //ROS_INFO("2222222222Obstacle avoidance mode: %s", _obstacleAvoidanceMode ? "ON" : "OFF");
        ROS_INFO_THROTTLE(0.5, "2222222222Obstacle avoidance mode: %s",
                  _obstacleAvoidanceMode ? "ON" : "OFF");
        _vCmdBody.setZero();  // 假设_vCmdBody是身体线速度指令（x, y, z）
        _dYawCmd = 0.0;       // 假设_dYawCmd是偏航角速度指令
        _obstacleAvoidanceMode = false;

    }   
    
    // 避障模式
    // if (_obstacleAvoidanceMode) {
    //     // 获取当前机器人状态 (x, y, yaw)(理应是全局坐标系)
    //     State current_state(//run函数传进来的
    //         _posBody(0), 
    //         _posBody(1), 
    //         _yaw
    //     );
    //     // 获取机体到全局的旋转矩阵
    //     //RotMat B2G_RotMat = _lowState->getRotMat();
        
    //     _stateQueue.push(current_state);
        
    //     // 尝试获取最新控制指令
    //     Control new_control;
    //     if (_controlQueue.try_pop(new_control)) {
    //         _lastOptimalControl = new_control;
    //     }

    //     // 将MPPI计算的全局坐标系控制量转换到机体坐标系
    //     // Eigen::Vector2d vel_global(_lastOptimalControl.vx, 0);
    //     // Eigen::Vector2d vel_body = B2G_RotMat.transpose().block<2,2>(0,0) * vel_global;

    //     // 使用最新控制指令
    //     _vCmdBody(0) = _lastOptimalControl.vx;
    //     _vCmdBody(1) = 0;
    //     _dYawCmd = _lastOptimalControl.wz;
    // }
    if (_obstacleAvoidanceMode) {
    /* ---------- 1. 一次性快照（带锁） ---------- */
    State snap;
    RotMat B2G_RotMat;
    {
        std::lock_guard<std::mutex> g1(gazebo_mutex_);      // gazebo位置锁

        std::lock_guard<std::mutex> g2(_ctrlComp->lowStateMutex);   // 临界区开始
        // snap.x   = _posBody(0);
        // snap.y   = _posBody(1);

        snap.x   = _gazebo_pos.x();  // 使用gazebo位置
        snap.y   = _gazebo_pos.y();
        snap.yaw = _yaw;
        B2G_RotMat = _lowState->getRotMat();   // 拿旋转矩阵
    }                                                          // 临界区结束

    /* ---------- 2. 把快照扔进队列 ---------- */
    _stateQueue.push(snap);   // MPPI 线程会拿到完全一致的一组数据

    /* ---------- 3. 拿控制量（无锁，队列里已是拷贝） ---------- */
    Control new_control;
    if (_controlQueue.try_pop(new_control)) {
        _lastOptimalControl = new_control;
    }

    double ramp = std::min(1.0, std::max(0.0, (ros::Time::now() - _enterTime).toSec() / 1.1));
    /* ---------- 4. 应用控制量 ---------- */
    _vCmdBody(0) = ramp*_lastOptimalControl.vx;
    _vCmdBody(1) = 0.0;
    _dYawCmd     = ramp*_lastOptimalControl.wz;
    }
        // // 使用转换坐标系后的控制指令
        // _vCmdBody(0) = vel_body.x();
        // _vCmdBody(1) = 0;  
        // _dYawCmd = _lastOptimalControl.wz;
        //ROS_INFO("MPPI computed: vx=%.2f m/s, wz=%.2f rad/s", _vCmdBody(0), _dYawCmd); zhe li da yin chu lai de zhi ye hen xiao,dan shi ping hua
        
        // // 调用MPPI获取最优控制量
        // Control optimal = _mppi->getOptimalControl(current_state);
        
        // 设置控制命令
        // _vCmdBody(0) = optimal.vx;
        // _vCmdBody(1) = 0;  // 避障模式下不进行侧向移动
        // _dYawCmd = optimal.wz;
        bool currentL2XPressed = (_lowState->userCmd == UserCommand::L2_X);
        if (currentL2XPressed && !_lastL2XPressed && !_applyingForce) {
        //if (_lowState->userCmd == UserCommand::L2_X && !_applyingForce) {
        _applyingForce = true;
        _forceStartTime = ros::Time::now();  // 记录开始时间
        
        // 仅调用一次服务，设置持续时间
        gazebo_msgs::ApplyBodyWrench srv;
        srv.request.body_name = "base";
        srv.request.reference_frame = "base";  //
        srv.request.wrench.force.x = _disturbanceForce.x();  
        srv.request.wrench.force.y = _disturbanceForce.y();
        srv.request.wrench.force.z = _disturbanceForce.z();
        srv.request.wrench.torque.x = 0.0;
        srv.request.wrench.torque.y = 0.0;
        srv.request.wrench.torque.z = 0.0;
        srv.request.start_time = _forceStartTime;
        srv.request.duration = ros::Duration(2.0);  // 持续时间
        
        if (!_applyForceClient.call(srv)) {
            ROS_ERROR("Failed to apply disturbance force!");
            _applyingForce = false;  // 调用失败则重置
        } else {
            ROS_INFO("Disturbance force applied for 0.2s");
        }
        
    }
    _lastL2XPressed = currentL2XPressed;
    // 仅在力的持续时间结束后重置标志（无需重复调用服务）
    if (_applyingForce && (ros::Time::now() - _forceStartTime).toSec() >= 2.0) {
        _applyingForce = false;
        ROS_INFO("Disturbance force ended");
    }
    // 新增：更新并发布力箭头Marker
    updateForceMarker();
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;//转为全局坐标系,_vCmdBody是机体坐标系下的速度指令

    _pcd(0) = _pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt;
    //表示机器人在全局坐标系中x轴方向的目标位置坐标
    _pcd(1) = _pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt;
    //表示机器人在全局坐标系中y轴方向的目标位置坐标

//jiang su du ji fen de dao wei zhi
    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);//_yawCmd是机器人的期望偏航角,_Rd是期望的旋转矩阵,
    //通过rotz(_yawCmd)（绕 z 轴旋转的旋转矩阵），将偏航角指令转换为旋转矩阵_Rd
    //后续可通过_Rd与当前姿态矩阵（_G2B_RotMat）的偏差计算姿态控制量
    _wCmdGlobal(2) = _dYawCmd;//(2)biao shi z zhou jiao su du ming ling
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;//世界坐标系下目标位置-当前位置  _posBody = _gazebo_pos;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());
    //_wCmdGlobal：全局坐标系下的期望角速度指令（目标角速度）
    //_lowState->getGyroGlobal()：当前全局坐标系下的实际角速度
    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

