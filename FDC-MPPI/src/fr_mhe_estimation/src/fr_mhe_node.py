#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float32MultiArray, Header  
from tf.transformations import quaternion_from_euler  
# Import gazebo_msgs explicitly to avoid subscriber message-type errors.
import gazebo_msgs.msg

# Import the existing modules.
from config import ALPHA, T, WINDOW_SIZE, STATE_DIM
from function import F_kinematics
from gradient_calculator import GradientCalculator

class FRMHEEstimator:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('fr_mhe_estimator', anonymous=True)
        
        # Subscribe to the input topics with gazebo_msgs in scope.
        self.sub_measurement = rospy.Subscriber(
            '/gazebo/model_states',  
            gazebo_msgs.msg.ModelStates,  
            self.meas_callback
        )
        self.sub_input = rospy.Subscriber(
            '/unitree_gazebo_servo/mppi_global_control',  
            Twist,  
            self.input_callback
        )
        
        # Publish only mu1, mu2, and mu3 in a Float32MultiArray.
        self.pub_mu = rospy.Publisher(
            '/robot/mu_estimation', Float32MultiArray, queue_size=10
        )
        
        # Initialize the gradient calculator.
        self.grad_calculator = GradientCalculator(WINDOW_SIZE, state_dim=STATE_DIM)
        
        # Keep only the variables used by the estimator.
        self.robot_model_name = "go2_gazebo"  # Must match the robot model name in Gazebo.
        
        # State-estimation variables.
        self.state_est_start = np.zeros(STATE_DIM)
        self.first_run = True
        # Timestamped buffers storing (timestamp, data) pairs.
        self.meas_buffer_ts = []  # Measurement data: (timestamp, [x,y,theta])
        self.input_buffer_ts = []  # Input data: (timestamp, {'vx','vy','w'})
        
        # Actual MHE period (0.01 s corresponds to 100 Hz).
        self.mhe_period = T  # Keep consistent with T in config.py.
        
    def meas_callback(self, msg):
        """Process measurements (x, y, theta) from gazebo/model_states."""
        # Find the robot model index in the message.
        try:
            idx = msg.name.index(self.robot_model_name)
        except ValueError:
            rospy.logwarn_throttle(1.0, f"未找到模型 {self.robot_model_name} 在model_states中")
            return
        
        # Extract position data.
        x = msg.pose[idx].position.x
        y = msg.pose[idx].position.y
        
        # Extract yaw from the quaternion.
        q = msg.pose[idx].orientation
        theta = 2 * np.arctan2(q.z, q.w)  # Compute the yaw angle.
        timestamp = rospy.Time.now().to_sec()

        # Cache timestamped measurements and limit the buffer length.
        self.meas_buffer_ts.append( (timestamp, np.array([x, y, theta])) )
        if len(self.meas_buffer_ts) > 100:
            self.meas_buffer_ts.pop(0)

    def input_callback(self, msg):
        """Process input data (vx, vy, w) from mppi_global_control."""
        # Extract velocity data directly from the Twist message.
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        timestamp = rospy.Time.now().to_sec()
        # Cache timestamped inputs and limit the buffer length.
        self.input_buffer_ts.append( (timestamp, {'vx': vx, 'vy': vy, 'w': w}) )
        if len(self.input_buffer_ts) > 20:
            self.input_buffer_ts.pop(0)

    def resample_inputs(self, target_timestamps):
        """Upsample the 50 Hz input to the target timestamps at 100 Hz."""
        resampled = []
        # Boundary guard: return zeros when no input data are available.
        if not self.input_buffer_ts:
            return [{'vx':0, 'vy':0, 'w':0}] * len(target_timestamps)
            
        input_ts = np.array([t for t, _ in self.input_buffer_ts])
        inputs = [d for _, d in self.input_buffer_ts]
        
        for t in target_timestamps:
            # Find input data around the timestamp and guard the index bounds.
            idx = np.searchsorted(input_ts, t) - 1
            idx = max(0, min(idx, len(input_ts)-2))  # Prevent idx from going out of bounds.
            
            t0, d0 = input_ts[idx], inputs[idx]
            t1, d1 = input_ts[idx+1], inputs[idx+1]
            # Linear interpolation with division-by-zero protection.
            alpha = (t - t0) / (t1 - t0) if abs(t1 - t0) > 1e-6 else 1.0
            resampled.append({
                'vx': d0['vx'] + alpha*(d1['vx'] - d0['vx']),
                'vy': d0['vy'] + alpha*(d1['vy'] - d0['vy']),
                'w': d0['w'] + alpha*(d1['w'] - d0['w'])
            })
        return resampled

    def resample_measurements(self, target_timestamps):
        """Downsample the 500 Hz measurements to target timestamps at 100 Hz."""
        resampled = []
        # Boundary guard: return zeros when no measurements are available.
        if not self.meas_buffer_ts:
            return [np.zeros(3)] * len(target_timestamps)
            
        meas_ts = np.array([t for t, _ in self.meas_buffer_ts])
        measurements = [d for _, d in self.meas_buffer_ts]
        
        for t in target_timestamps:
            # Find the measurement nearest to the target timestamp.
            idx = np.argmin(np.abs(meas_ts - t)) if meas_ts.size > 0 else -1
            if idx >= 0:
                resampled.append(measurements[idx])
            else:
                resampled.append(np.zeros(3))  # Fill with zeros when data are insufficient.
        return resampled

    def rollout_from(self, state0, inputs):
        """Perform a multistep rollout using the existing process equation."""
        states = [state0.copy()]
        for k in range(1, WINDOW_SIZE):
            prev_state = states[k - 1]
            uk = inputs[k - 1]
            nxt = F_kinematics(prev_state, uk)
            states.append(nxt)
        return states

    def predict_one_step(self, prev_state, prev_input):
        """Perform one prediction step using the existing function."""
        return F_kinematics(prev_state, prev_input)

    def publish_mu(self, mu1, mu2, mu3, timestamp):
        """Publish only mu1, mu2, and mu3 as a Float32MultiArray message."""
        mu_msg = Float32MultiArray()
        # Populate the data with mu1, mu2, and mu3 in order.
        mu_msg.data = [mu1, mu2, mu3]
        
        self.pub_mu.publish(mu_msg)

    def run(self):
        """Run the main MHE loop at 100 Hz."""
        rate = rospy.Rate(100)  # Enforce a 100 Hz rate.
        while not rospy.is_shutdown():
            # Check whether enough raw data are available.
            if len(self.meas_buffer_ts) < 10 or len(self.input_buffer_ts) < 5:
                rospy.logwarn_throttle(1.0, "原始数据不足，等待...")
                rate.sleep()
                continue
            
            # Generate WINDOW_SIZE target timestamps in the 100 Hz window.
            current_time = rospy.Time.now().to_sec()
            window_end_time = current_time  # End the window at the current time.
            window_start_time = window_end_time - self.mhe_period * (WINDOW_SIZE - 1)
            target_timestamps = np.linspace(window_start_time, window_end_time, WINDOW_SIZE)
            
            # Resample inputs and measurements onto the target timestamps.
            inputs = self.resample_inputs(target_timestamps)
            measurements = self.resample_measurements(target_timestamps)
            
            # Ensure that the data lengths are correct.
            if len(inputs) != WINDOW_SIZE or len(measurements) != WINDOW_SIZE:
                rate.sleep()
                continue
            
            # 2. Initialize the state at the start of the window.
            if self.first_run:
                x0 = np.zeros(STATE_DIM)
                x0[0:3] = measurements[0]  # Initialize from the first measurement.
                x0[3:6] = 1.0  # Initial mu values.
                self.first_run = False
            else:
                # Predict the current starting point from the previous window start.
                x0 = self.predict_one_step(self.state_est_start, inputs[0])

            # 3. Update the window-start state by gradient descent.
            xk0 = x0.copy()
            states = self.rollout_from(xk0, inputs)
            meas = {'pose': np.array(measurements)}
            grad = self.grad_calculator.compute_gradient(states, inputs, meas)
            xk0 = xk0 - ALPHA * grad

            # 4. Update the states in the window and obtain the current estimate.
            states_updated = self.rollout_from(xk0, inputs)
            curr_state = states_updated[-1]
            self.state_est_start = xk0

            # 5. Publish the mu values.
            mu1, mu2, mu3 = curr_state[3], curr_state[4], curr_state[5]
            self.publish_mu(mu1, mu2, mu3, rospy.Time.now())
            rospy.loginfo_throttle(1.0, f"Estimated: mu1={mu1:.3f}, mu2={mu2:.3f}, mu3={mu3:.3f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        estimator = FRMHEEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("FR-MHE节点被中断，正常退出")
    except Exception as e:
        rospy.logerr(f"节点运行出错：{str(e)}")
        raise
