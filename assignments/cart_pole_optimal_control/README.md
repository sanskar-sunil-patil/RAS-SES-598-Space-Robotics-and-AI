Cart-Pole Optimal Control


1. Project Overview =

   
This project involves optimizing and analyzing an LQR (Linear Quadratic Regulator) controller for a cart-pole system under earthquake-like disturbances. The goal is to maintain the pole in the upright position while keeping the cart within its physical limits of ±2.5m, despite external disturbances from the earthquake force generator. This project allow to gain hands-on experience with control systems and disturbances, skills that are crucial for the navigation of space robots, such as lunar landers or orbital debris removal robots.

The LQR controller will be tuned to achieve the following =
Maintain the pendulum (pole) in an upright position.
Prevent the cart from exceeding its physical boundaries (±2.5m).
Stabilize the system under earthquake-like disturbances.
Ensure that control effort remains efficient.


2. Objectives =
   
The goal of this project is to analyze and tune an LQR controller for a cart-pole system subject to earthquake-like disturbances. The primary objective is to stabilize the pole in an upright position while keeping the cart within its physical constraints (±2.5 meters), even under dynamic external disturbances that simulate seismic forces.


4. System Used =
Physical Setup =
Inverted pendulum mounted on a cart.

self.M = 1.0  # Mass of cart (kg)
self.m = 1.0  # Mass of pole (kg)
self.L = 1.0  # Length of pole (m)
self.g = 9.81  # Gravity (m/s^2)

Disturbance Generator:
Generates earthquake-like forces using sine waves.
Base amplitude: 15.0 N.
Frequency range: 0.5-4.0 Hz.
Additional Gaussian noise for realistic disturbances.
ROS2 (Robot Operating System 2)=ROS2 Jazzy.
Gazebo simulation for cart-pole dynamics.
RViz for visualization and monitoring.


4. Installation and Setup =
Fork and Clone the Repository:
ROS2 Setup =
Install necessary ROS2 packages such as ros-$ROS_DISTRO-ros-gz-bridge, ros-$ROS_DISTRO-rviz2, and others related to simulation and control.

Python Dependencies:
Install necessary Python libraries: numpy, scipy, control.

Repository Setup =
Create symlink to ROS2 workspace and build the package.
Use colcon build to build the package and source the workspace.

Launch Simulation:
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py will run the Gazebo simulation with RViz visualization.


5. Controller Tuning =
LQR Controller Description:
The LQR controller is defined in lqr_controller.py, with configurable Q and R matrices.
Default Q Matrix =
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

Default R Matrix =
R = np.array([[0.1]])  # Control effort cost
Parameter Tuning:

The tuning process involves adjusting the Q matrix to penalize different states (cart position, velocity, pole angle, angular velocity) and adjusting the R matrix to control the control effort (force applied to the cart).
Objective: Minimize the pole angle deviation while ensuring the cart remains within the limits and minimizing control effort.
Tuning Steps:

Start with default Q and R: Observe the system’s behavior.
Adjust Q: Increase or decrease the values based on the state you want to prioritize (e.g., reduce the penalty on x if the cart position is less critical than stabilizing the pole).
Adjust R: Increase R if the control effort is too aggressive (leading to high forces) or too low if the system is not responsive enough.


6. Performance Metrics =
Stability Metrics:
Maximum pole angle deviation.
RMS cart position error.
Peak control force used.
Recovery time after disturbances.
Control Metrics:

The efficiency of the control effort.
Trade-off between system stability and control input efficiency.
System Constraints:

Cart position must stay within ±2.5m.
Control rate should stay at 50 Hz (set by the ROS2 control loop).


7. Pattern Optimization =
Earthquake Disturbance:
Test system behavior under various disturbance frequencies (0.5-4.0 Hz) and amplitudes (base 15.0 N).
Tunning for Optimal Coverage:
Controller Tuning: Adjust Q and R values to optimize stability under specific disturbance frequencies. Fine-tune the disturbance response for quicker recovery times and minimal overshoot.


8. Challenges and Solutions =
Challenge 1: Handling Disturbances
Solution: Carefully adjust Q and R matrices to balance stability with quick recovery. Ensure that control forces are sufficient to combat high-amplitude disturbances without overshooting.

Challenge 2: Cart position limits
Solution: Modify Q to penalize large cart position deviations more heavily, ensuring the cart stays within the physical boundaries.

Challenge 3: Control efficiency
Solution: Tune R to reduce excessive control effort while still maintaining system stability.


9. Code =

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg
import time

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
       
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
       
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
       
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
       
        # LQR cost matrices
        self.Q = np.diag([1.0, 1.0, 50.0, 50.0])  # State cost
        self.R = np.array([[0.01]])  # Control cost
       
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
       
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
       
        # Tracking statistics
        self.max_pole_angle_deviation = 0.0
        self.cart_position_errors = []
        self.peak_control_force = 0.0
       
        # Recovery time tracking
        self.disturbance_detected = False
        self.recovery_start_time = None
        self.recovery_time = None
        self.disturbance_threshold = 0.05  # Threshold for disturbance detection
       
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            10
        )
       
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )
       
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)
       
        self.get_logger().info('Cart-Pole LQR Controller initialized')
   
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
   
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            # Get indices for cart and pole joints
            cart_idx = msg.name.index('cart_to_base')  # Cart position/velocity
            pole_idx = msg.name.index('pole_joint')    # Pole angle/velocity
           
            # State vector: [x, ẋ, θ, θ̇]
            self.x = np.array([
                [msg.position[cart_idx]],     # Cart position (x)
                [msg.velocity[cart_idx]],     # Cart velocity (ẋ)
                [msg.position[pole_idx]],     # Pole angle (θ)
                [msg.velocity[pole_idx]]      # Pole angular velocity (θ̇)
            ])
           
            # Track max pole angle deviation
            self.max_pole_angle_deviation = max(self.max_pole_angle_deviation, abs(self.x[2, 0]))
           
            # Track cart position errors
            self.cart_position_errors.append(self.x[0, 0] ** 2)
           
            # Detect disturbances
            if abs(self.x[2, 0]) > self.disturbance_threshold and not self.disturbance_detected:
                self.disturbance_detected = True
                self.recovery_start_time = time.time()
                self.get_logger().info(f'Disturbance detected! Pole angle: {self.x[2, 0]:.3f} rad')
           
            # Check if system has recovered
            if self.disturbance_detected and abs(self.x[2, 0]) < self.disturbance_threshold:
                self.recovery_time = time.time() - self.recovery_start_time
                self.disturbance_detected = False
                self.get_logger().info(f'Recovery complete! Time taken: {self.recovery_time:.3f} sec')

            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
               
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')
   
    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if not self.state_initialized:
                self.get_logger().warn('State not initialized yet')
                return

            # Compute control input u = -Kx
            u = -self.K @ self.x
            force = float(u[0])
           
            # Track peak control force
            self.peak_control_force = max(self.peak_control_force, abs(force))
           
            # Compute RMS cart position error
            rms_cart_position_error = np.sqrt(np.mean(self.cart_position_errors)) if self.cart_position_errors else 0.0
           
            # Log control input and statistics periodically
            if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
                self.get_logger().info(f'State: {self.x.T}, Control force: {force:.3f}N')
                self.get_logger().info(f'Max Pole Angle Deviation: {self.max_pole_angle_deviation:.3f} rad')
                self.get_logger().info(f'RMS Cart Position Error: {rms_cart_position_error:.3f} m')
                self.get_logger().info(f'Peak Control Force Used: {self.peak_control_force:.3f} N')

                if self.recovery_time is not None:
                    self.get_logger().info(f'Last Recovery Time: {self.recovery_time:.3f} sec')
                    self.recovery_time = None  # Reset after logging
           
            # Publish control command
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
           
            self.last_control = force
            self.control_count += 1
           
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



10. Video Link =
`https://drive.google.com/file/d/1fPIVo5APeOgmDg_j3lTkmFs1DwlwR-Ut/view?usp=drivesdk


![Output](https://github.com/user-attachments/assets/23a5e4f8-da63-4e10-be8c-6a6d7e371f8f)



11. Future Considerations =
Advanced Disturbance Modeling:
Implement more complex disturbance models (e.g., varying noise levels).
Reinforcement Learning (Extra Credit):
Implement a DQN (Deep Q-Network) to train an agent to stabilize the pendulum. Compare this approach to the LQR controller in terms of performance and efficiency.

12. Conclusion =
This project provides a comprehensive hands-on approach to analyzing and tuning the LQR controller in a cart-pole system under dynamic disturbances. By adjusting the Q and R matrices, students will gain insights into system behavior, stability, and control trade-offs. The challenge of handling external disturbances while ensuring control efficiency mimics real-world conditions, such as space missions, where robots must operate under unpredictable environmental conditions.
