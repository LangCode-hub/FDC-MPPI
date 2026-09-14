/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>
#include "gazebo_msgs/ModelStates.h"
#include <ros/time.h>
#include <gazebo_msgs/ApplyBodyWrench.h>  // ApplyBodyWrench message definition.


State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl),_nh("~"){//xin jia le _nh("~")
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

    // Initialize the MPPI controller.
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

    // Initialize the Gazebo model-state subscriber.
    gazebo_model_sub_ = _nh.subscribe("/gazebo/model_states", 10, 
                                     &State_Trotting::gazeboModelStatesCallback, this);
    _gazebo_pos.setZero();  // Initialize the position to zero.

    // Initialize the most recent control command.
    _lastOptimalControl = Control(0, 0);
    
    // Start the MPPI worker thread.
    _mppiRunning = true;
    _mppiThread = std::thread(&State_Trotting::mppiThreadFunc, this);

    _applyForceClient = _nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    // Set the default disturbance force; its magnitude and direction are configurable.
    _disturbanceForce = Eigen::Vector3d(30, 0, 0);

    // Initialize the force-arrow publisher.
    _forceMarkerPub = _nh.advertise<visualization_msgs::Marker>("disturbance_force_marker", 10);
    
    // Initialize the force-arrow marker.
    initializeForceMarker();

    // Initialize the MPPI control-command publisher using geometry_msgs/Twist.
    mppi_control_pub_ = _nh.advertise<geometry_msgs::Twist>("mppi_global_control", 10);
}

State_Trotting::~State_Trotting(){
    delete _gait;
    _mppiRunning = false;
    if (_mppiThread.joinable()) {
        _mppiThread.join();
    }
}

void State_Trotting::enter(){
    //_pcd = _est->getPosition();// Set the initial desired body position.
    _enterTime = ros::Time::now();
    {
        std::lock_guard<std::mutex> lock(gazebo_mutex_);
        _pcd = _gazebo_pos;
    }

    // Print the initial position once when entering the state.
    ROS_INFO("initiallllll positionnnnnnnnnnn: (%.2f, %.2f, %.2f)", _pcd.x(), _pcd.y(), _pcd.z());// The initial position has been assigned correctly.
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);// Read row 2, column 0 from the returned matrix.
    // getFeetPosIdeal()(2,0) is the nominal vertical distance from a foot to its hip.
    // Negating it gives the desired body height above the ground.
    // _pcd(2) selects the third element of the desired-position vector.

    _vCmdBody.setZero();// Clear the body-frame velocity command.
    _yawCmd = _lowState->getYaw();// Use the current yaw as the heading to maintain.
    _Rd = rotz(_yawCmd);// Build the desired 3x3 orientation matrix from the target yaw.
                        // rotz() constructs a rotation about the global z axis.
    _wCmdGlobal.setZero();// Clear the global-frame angular-velocity command.

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

    //setObstacleAvoidanceGoal(5, 0.0);
    setObstacleAvoidanceGoal(5.1, 2);

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
        std::lock_guard<std::mutex> g(gazebo_mutex_);  // Lock access to the Gazebo state.
        ros::Duration lag = ros::Time::now() - _lastGazeboTime;
        if (lag.toSec() > 0.05) {
            ROS_WARN_THROTTLE(1.0, "Gazebo pose data delayed by %.3f s", lag.toSec());
        }
        _posBody = _gazebo_pos;  // Use Gazebo position instead of _est->getPosition().
    }
    
    {
        std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);
        _velBody = _est->getVelocity();  // Continue reading velocity from the estimator.
    }
    // ROS_INFO_THROTTLE(0.5, "Current posBody(print in run function): (%.2f, %.2f, %.2f)", 
    //                   _posBody.x(), _posBody.y(), _posBody.z());

    //  {
    //     std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);  // Protect the shared state.
    //     _posBody = _est->getPosition();
    //     _velBody = _est->getVelocity();
    //     ROS_INFO_THROTTLE(0.5, "Current posBody(print in run function): (%.2f, %.2f, %.2f)", 
    //                  _posBody.x(), _posBody.y(), _posBody.z());
    // }  // Release the lock automatically.
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
    // Find the robot-model index for the model named "go2_gazebo".
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "go2_gazebo") {
            // Extract the position.
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

// Set the target position.
void State_Trotting::setObstacleAvoidanceGoal(double x, double y) {
    _avoidanceGoal = Vec2(x, y);
    _mppi->setGoal(x, y);
    _MPPICompleted = false;  // Reset completion state for the new target.
    ROS_INFO("set the goal point: (%.2f, %.2f)", x, y); 
}

// MPPI thread function.
void State_Trotting::mppiThreadFunc() {
    ros::Rate rate(50.0);  // Set the MPPI computation rate.
    while (_mppiRunning) {
        // Wait for a new state.
        State current_state; // Populated from the queue in getUserCmd().

        RotMat B2G_RotMat;          // Filled after a state becomes available.
        // _stateQueue safely transfers snapshots from the main thread to this worker.
        if(_stateQueue.wait_and_pop(current_state)){
            {
        // Limit access to _lowState to this short critical section.
                std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);
                B2G_RotMat = _lowState->getRotMat();   // Read the rotation matrix.
                // Read any additional IMU or foot state here while the lock is held.
            } // Release the lock here and do not access _lowState below.
        // Obtain the current rotation matrix either here or from the main-thread snapshot.
        //RotMat B2G_RotMat = _lowState->getRotMat();
        // Compute the optimal control.
        if (_obstacleAvoidanceMode) {
            Control optimal = _mppi->getOptimalControl(current_state, _gazebo_pos.x(), _gazebo_pos.y(),B2G_RotMat);
            ROS_INFO("MPPI computed: vx=%.2f m/s, wz=%.2f rad/s", optimal.vx, optimal.wz);  // Temporarily left enabled; it can be disabled later.
            // Store the control command in the member variable.
                {
                    std::lock_guard<std::mutex> lock(control_mutex_);
                    latest_mppi_control_ = optimal;
                }
                // Transform into the global coordinate frame using a rotation matrix.
                // 1. Construct the body-to-global rotation matrix from the quaternion or Euler angles.
                Eigen::Matrix2d rot_matrix;
                //double yaw = current_state.yaw;  // Current yaw angle.
                double yaw = _lowState->getYaw(); // Get the current yaw angle in the global frame.
                rot_matrix << cos(yaw), -sin(yaw),
                              sin(yaw), cos(yaw);
                
                // 2. Control-velocity vector in the body frame.
                Eigen::Vector2d vel_body(optimal.vx, 0.0);  // Assume zero body-frame y velocity.
                // 3. Transform into the global coordinate frame.
                Eigen::Vector2d vel_global = rot_matrix * vel_body;
                // 4. Store the global control command.
                ctrl_vx_global_ = vel_global(0);
                ctrl_vy_global_ = vel_global(1);
                ctrl_wz_global_ = optimal.wz;  // Angular velocity is unchanged.
                
                // Publish the global control command.
                geometry_msgs::Twist global_ctrl_msg;
                global_ctrl_msg.linear.x = ctrl_vx_global_;
                global_ctrl_msg.linear.y = ctrl_vy_global_;
                global_ctrl_msg.angular.z = ctrl_wz_global_;
                mppi_control_pub_.publish(global_ctrl_msg);
                
                _controlQueue.push(optimal);
                
        }
    }
    rate.sleep();  // Sleep at the configured rate.
    }
}
// Initialize the force marker.
void State_Trotting::initializeForceMarker() {
    _forceMarker.header.frame_id = "base";  // Use the base frame.
    _forceMarker.ns = "disturbance_force";
    _forceMarker.id = 0;
    _forceMarker.type = visualization_msgs::Marker::ARROW;
    _forceMarker.action = visualization_msgs::Marker::ADD;
    
    // Set the arrow dimensions.
    _forceMarker.scale.x = 0.4;  // Arrow length.
    _forceMarker.scale.y = 0.1; // Arrow width.
    _forceMarker.scale.z = 0.1;  // Arrow height.
    
    // Set the arrow color.
    _forceMarker.color.r = 1.0f;
    _forceMarker.color.g = 1.0f;
    _forceMarker.color.b = 0.0f;
    _forceMarker.color.a = 1.0;  // Opacity.
}

void State_Trotting::updateForceMarker() {
    if (!_applyingForce) {
        _forceMarker.color.a = 0.01;
    } else {
        _forceMarker.color.a = 1.0;
        _forceMarker.header.stamp = ros::Time::now();
        
        // Place the marker 1 m behind the body center and 0.2 m above the ground.
        _forceMarker.pose.position.x = -1.0;  // Negative x points behind the robot.
        _forceMarker.pose.position.y = 0.0;
        _forceMarker.pose.position.z = 0.2;
        
        // Express the force direction in the body frame without an additional rotation.
        Eigen::Vector3d forceDir = _disturbanceForce.normalized();
        // Orient the marker toward the body-frame force direction.
        tf2::Quaternion quat;
        quat.setRPY(0, 0, atan2(forceDir.y(), forceDir.x()));  // Consider only the xy-plane direction.
        _forceMarker.pose.orientation.x = quat.x();
        _forceMarker.pose.orientation.y = quat.y();
        _forceMarker.pose.orientation.z = quat.z();
        _forceMarker.pose.orientation.w = quat.w();
    }
    _forceMarkerPub.publish(_forceMarker);
}
void State_Trotting::getUserCmd(){
// Enable obstacle-avoidance mode with key 4.
    if (_lowState->userCmd == UserCommand::START && !_obstacleAvoidanceMode && !_MPPICompleted) {
        _obstacleAvoidanceMode = true;
        ROS_INFO("Obstacle avoidance mode: %s", _obstacleAvoidanceMode ? "ON" : "OFF");
    }

    // Check target completion only while obstacle-avoidance mode is active.
    if (_obstacleAvoidanceMode && !_MPPICompleted) {
        // Obtain the current robot position from the estimator if needed.
        // Eigen::Vector3d currentPos = _est->getPosition(); 
        // Eigen::Vector2d currentXY(currentPos.x(), currentPos.y());

        // Read the current Gazebo position while protecting shared access.
        Eigen::Vector3d currentPos;
        {
            std::lock_guard<std::mutex> g(gazebo_mutex_);  // Lock access to the Gazebo position.
            currentPos = _gazebo_pos;

            // ROS_INFO_THROTTLE(0.2,  // Print at most once every 0.2 seconds.
            // "current position: (%.2f, %.2f), goal point: (%.2f, %.2f)",
            // currentPos.x(), currentPos.y(),_avoidanceGoal.x(), _avoidanceGoal.y());
        }
        // {
        // std::lock_guard<std::mutex> g(_ctrlComp->lowStateMutex);  // Enter the protected section.
        // currentPos = _est->getPosition();  // Read the position safely.

        // ROS_INFO_THROTTLE(0.2,  // Print at most once every 0.2 seconds.
        // "current position: (%.2f, %.2f), goal point: (%.2f, %.2f)",
        // currentPos.x(), currentPos.y(),_avoidanceGoal.x(), _avoidanceGoal.y()); 
        // }  // Release the lock automatically.
        
        Eigen::Vector2d currentXY(currentPos.x(), currentPos.y());

        // ROS_INFO_THROTTLE(0.2,  // Print at most once every 0.2 seconds.
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

        // Set velocity commands to zero after reaching the target.
    if (_MPPICompleted) {
        // Clear the linear- and angular-velocity commands.
        //ROS_INFO("2222222222Obstacle avoidance mode: %s", _obstacleAvoidanceMode ? "ON" : "OFF");
        ROS_INFO_THROTTLE(0.5, "2222222222Obstacle avoidance mode: %s",
                  _obstacleAvoidanceMode ? "ON" : "OFF");
        _vCmdBody.setZero();  // Body-frame linear-velocity command (x, y, z).
        _dYawCmd = 0.0;       // Yaw-rate command.
        _obstacleAvoidanceMode = false;

    }   
    
    // Obstacle-avoidance mode.
    // if (_obstacleAvoidanceMode) {
    //     // Obtain the current robot state (x, y, yaw) in the global frame.
    //     State current_state(// Values supplied by run().
    //         _posBody(0), 
    //         _posBody(1), 
    //         _yaw
    //     );
    //     // Obtain the body-to-global rotation matrix.
    //     //RotMat B2G_RotMat = _lowState->getRotMat();
        
    //     _stateQueue.push(current_state);
        
    //     // Try to read the latest control command.
    //     Control new_control;
    //     if (_controlQueue.try_pop(new_control)) {
    //         _lastOptimalControl = new_control;
    //     }

    //     // Convert the MPPI control from the global frame to the body frame.
    //     // Eigen::Vector2d vel_global(_lastOptimalControl.vx, 0);
    //     // Eigen::Vector2d vel_body = B2G_RotMat.transpose().block<2,2>(0,0) * vel_global;

    //     // Apply the latest control command.
    //     _vCmdBody(0) = _lastOptimalControl.vx;
    //     _vCmdBody(1) = 0;
    //     _dYawCmd = _lastOptimalControl.wz;
    // }
    if (_obstacleAvoidanceMode) {
    /* ---------- 1. Take one locked snapshot. ---------- */
    State snap;
    RotMat B2G_RotMat;
    {
        std::lock_guard<std::mutex> g1(gazebo_mutex_);      // Gazebo-position lock.

        std::lock_guard<std::mutex> g2(_ctrlComp->lowStateMutex);   // Begin the protected section.
        // snap.x   = _posBody(0);
        // snap.y   = _posBody(1);

        snap.x   = _gazebo_pos.x();  // Use the Gazebo position.
        snap.y   = _gazebo_pos.y();
        snap.yaw = _yaw;
        B2G_RotMat = _lowState->getRotMat();   // Read the rotation matrix.
    }                                                          // End the protected section.

    /* ---------- 2. Push the snapshot into the queue. ---------- */
    _stateQueue.push(snap);   // The MPPI thread receives a consistent snapshot.

    /* ---------- 3. Read a copied control command without locking. ---------- */
    Control new_control;
    if (_controlQueue.try_pop(new_control)) {
        _lastOptimalControl = new_control;
    }

    double ramp = std::min(1.0, std::max(0.0, (ros::Time::now() - _enterTime).toSec() / 1.1));
    /* ---------- 4. Apply the control command. ---------- */
    _vCmdBody(0) = ramp*_lastOptimalControl.vx;
    _vCmdBody(1) = 0.0;
    _dYawCmd     = ramp*_lastOptimalControl.wz;
    }
        // // Apply the frame-transformed control command.
        // _vCmdBody(0) = vel_body.x();
        // _vCmdBody(1) = 0;  
        // _dYawCmd = _lastOptimalControl.wz;
        //ROS_INFO("MPPI computed: vx=%.2f m/s, wz=%.2f rad/s", _vCmdBody(0), _dYawCmd); zhe li da yin chu lai de zhi ye hen xiao,dan shi ping hua
        
        // // Call MPPI to obtain the optimal control.
        // Control optimal = _mppi->getOptimalControl(current_state);
        
        // Set the control command.
        // _vCmdBody(0) = optimal.vx;
        // _vCmdBody(1) = 0;  // Disable lateral motion during obstacle avoidance.
        // _dYawCmd = optimal.wz;
        bool currentL2XPressed = (_lowState->userCmd == UserCommand::L2_X);
        if (currentL2XPressed && !_lastL2XPressed && !_applyingForce) {
        //if (_lowState->userCmd == UserCommand::L2_X && !_applyingForce) {
        _applyingForce = true;
        _forceStartTime = ros::Time::now();  // Record the start time.
        
        // Call the service once and set the requested duration.
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
        srv.request.duration = ros::Duration(2.5);  // Force duration.
        
        if (!_applyForceClient.call(srv)) {
            ROS_ERROR("Failed to apply disturbance force!");
            _applyingForce = false;  // Reset the flag if the service call fails.
        } else {
            ROS_INFO("Disturbance force applied for 0.2s");
        }
        
    }
    _lastL2XPressed = currentL2XPressed;
    // Reset the flag after the force duration without repeating the service call.
    if (_applyingForce && (ros::Time::now() - _forceStartTime).toSec() >= 2.5) {
        _applyingForce = false;
        ROS_INFO("Disturbance force ended");
    }
    // Update and publish the force-arrow marker.
    updateForceMarker();
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;// Convert the body-frame velocity command to the global frame.

    _pcd(0) = _pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt;
    // Integrate the desired global x position.
    _pcd(1) = _pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt;
    // Integrate the desired global y position.

//jiang su du ji fen de dao wei zhi
    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);// _yawCmd is the desired yaw and _Rd is the desired orientation matrix.
    // Convert the yaw command into a rotation matrix about the z axis.
    // The difference between _Rd and _G2B_RotMat is used to compute attitude control.
    _wCmdGlobal(2) = _dYawCmd;//(2)biao shi z zhou jiao su du ming ling
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;// Desired minus current position in the global frame; _posBody = _gazebo_pos.
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());
    //_wCmdGlobal: desired angular velocity in the global frame.
    //_lowState->getGyroGlobal(): measured angular velocity in the global frame.
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
