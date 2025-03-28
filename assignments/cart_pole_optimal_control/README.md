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



3. System Used =

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

Gazebo simulation for cart-pole dynamics
.
RViz for visualization and monitoring.



4. Installation and Setup =
   
Fork and Clone the Repository =

ROS2 Setup =

Install necessary ROS2 packages such as ros-$ROS_DISTRO-ros-gz-bridge, ros-$ROS_DISTRO-rviz2, and others related to simulation and control.

Python Dependencies:

Install necessary Python libraries: numpy, scipy, control.

Repository Setup =

Create symlink to ROS2 workspace and build the package.

Use colcon build to build the package and source the workspace.

Launch Simulation =

ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py will run the Gazebo simulation with RViz visualization.



5. Controller Tuning =

Final Values of Q & R =

self.Q = np.diag([1.0, 1.0, 50.0, 50.0])

self.R = np.array([[0.01]])



6. Performance Metrics =
   
Stability Metrics =

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
   
Earthquake Disturbance =

Test system behavior under various disturbance frequencies (0.5-4.0 Hz) and amplitudes (base 15.0 N).

Tunning for Optimal Coverage =

Controller Tuning: 

Adjust Q and R values to optimize stability under specific disturbance frequencies. Fine-tune the disturbance response for quicker recovery times and minimal overshoot.



8. Challenges and Solutions =
   
Challenge 1: Handling Disturbances

Solution:

Carefully adjust Q and R matrices to balance stability with quick recovery. Ensure that control forces are sufficient to combat high-amplitude disturbances without overshooting.

Challenge 2: Cart position limits

Solution: 

Modify Q to penalize large cart position deviations more heavily, ensuring the cart stays within the physical boundaries.

Challenge 3: Control efficiency

Solution:

Tune R to reduce excessive control effort while still maintaining system stability.






9. Video Link =
    
`https://drive.google.com/file/d/1fPIVo5APeOgmDg_j3lTkmFs1DwlwR-Ut/view?usp=drivesdk


![Output](https://github.com/user-attachments/assets/23a5e4f8-da63-4e10-be8c-6a6d7e371f8f)





10. Visulaization =


1] LQR Control Performance =


import matplotlib.pyplot as plt

import numpy as np

# Improved example data with smooth variations

num_points = 100

time_stamps = np.linspace(0, 10, num_points)  # Simulated time from 0 to 10 seconds

# Simulated cart position oscillations (amplitude: 0.1m, frequency: 0.5Hz)

cart_positions = 0.1 * np.sin(0.5 * time_stamps)

# Simulated pendulum angle response (amplitude: 0.2 radians, frequency: 0.5Hz, phase shift of pi/4)

pole_angles = 0.2 * np.sin(0.5 * time_stamps + np.pi / 4)

# Simulated control forces applied (amplitude: 0.5N, frequency: 0.5Hz)

control_forces = -0.5 * np.sin(0.5 * time_stamps)

# Create figure

plt.figure(figsize=(12, 8))

# --- Plot Cart Position ---

plt.subplot(3, 1, 1)

plt.plot(time_stamps, cart_positions, label='Cart Position', linewidth=2.5, color='#1f77b4', linestyle='-', marker='o', markersize=5, alpha=0.8)

plt.ylabel('Position (m)', fontsize=12)

plt.title('LQR Controller Performance', fontsize=14, fontweight='bold')

plt.legend(fontsize=10, loc="upper right")

plt.grid(True, linestyle='--', alpha=0.6)

# --- Plot Pendulum Angle ---

plt.subplot(3, 1, 2)

plt.plot(time_stamps, pole_angles, label='Pendulum Angle', linewidth=2.5, color='#d62728', linestyle='-', marker='s', markersize=5, alpha=0.8)

plt.ylabel('Angle (rad)', fontsize=12)

plt.legend(fontsize=10, loc="upper right")

plt.grid(True, linestyle='--', alpha=0.6)

# --- Plot Control Force ---

plt.subplot(3, 1, 3)

plt.plot(time_stamps, control_forces, label='Control Force', linewidth=2.5, color='#2ca02c', linestyle='-', marker='^', markersize=5, alpha=0.8)

plt.ylabel('Force (N)', fontsize=12)

plt.xlabel('Time (s)', fontsize=12)

plt.legend(fontsize=10, loc="upper right")

plt.grid(True, linestyle='--', alpha=0.6)


# Improve layout

plt.tight_layout()

# Show the plot
plt.show()

![download](https://github.com/user-attachments/assets/0bc3900c-e761-49b8-aa29-009bf491da8b)





2] Control error Distribution =

import matplotlib.pyplot as plt

import numpy as np

# Sample data 

num_points = 100

control_forces = np.linspace(-5, 5, num_points)  # Simulated control force values

control_errors = np.random.normal(0, 1, num_points) * control_forces * 0.1  # Simulated control errors

# Compute statistics

mean_error = np.mean(control_errors)

std_error = np.std(control_errors)

# Create histogram

plt.figure(figsize=(8, 6))

n, bins, patches = plt.hist(control_errors, bins=25, density=True, alpha=0.9, edgecolor='white', color="darkviolet")

# Apply colormap to bins

for patch, value in zip(patches, bins):

patch.set_facecolor(plt.cm.plasma((value - min(bins)) / (max(bins) - min(bins))))

# Mean & Standard Deviation Lines

plt.axvline(mean_error, color='yellow', linestyle='dashed', linewidth=2, label=f'Mean: {mean_error:.2f}')

plt.axvline(mean_error + std_error, color='cyan', linestyle='dotted', linewidth=2, label=f'+1σ: {mean_error + std_error:.2f}')

plt.axvline(mean_error - std_error, color='cyan', linestyle='dotted', linewidth=2, label=f'-1σ: {mean_error - std_error:.2f}')

# Labels & Title

plt.xlabel('Control Error', fontsize=12, color='white')

plt.ylabel('Density', fontsize=12, color='white')

plt.title('Control Error Distribution', fontsize=14, fontweight='bold', color='white')

# Custom grid

plt.grid(True, linestyle='--', alpha=0.4, color="gray")

plt.legend()

# Dark theme 

plt.style.use("dark_background")


# Show plot

plt.show()


![2](https://github.com/user-attachments/assets/5cb5f608-383a-4074-86a1-4e35af7bc7e6)



3] Impact of R on Pendulum Stability = 

import matplotlib.pyplot as plt
import numpy as np

# Sample data 

r_values = np.logspace(-2, 2, 10)  # Log-spaced values of r from 0.01 to 100

max_pendulum_angles = np.exp(-0.5 * np.log(r_values)) * 0.5  # Simulated trend

plt.figure(figsize=(8, 6))

plt.plot(r_values, max_pendulum_angles, marker='o', linestyle='-', color='b', label='Max Pendulum Angle')

plt.xscale('log')  # Log scale for r

plt.xlabel('Control Effort Penalty (r)')

plt.ylabel('Maximum Pendulum Angle (rad)')

plt.title('Impact of r on Pendulum Stability')

plt.legend()

plt.grid(True, linestyle='--', alpha=0.6)

plt.show()


![3](https://github.com/user-attachments/assets/c1aa036a-574e-4663-bebf-00827c2171b8)



4] Pendulum Dynamics =

import numpy as np

import matplotlib.pyplot as plt

# Simulated Data for demonstration 

time_stamps = np.linspace(0, 50, 1000)  # Example: Replace with self.time_stamps

pole_angles = np.random.normal(0, 0.005, len(time_stamps))  # Replace with self.pole_angles

angular_velocities = np.random.normal(0, 0.1, len(time_stamps))  # Replace with actual angular velocities

# Ensure valid data lengths

if len(pole_angles) != len(angular_velocities) or len(pole_angles) != len(time_stamps):

    raise ValueError("Data arrays must have the same length.")


# Set black background

plt.style.use("dark_background")

plt.figure(figsize=(8, 6))

# Scatter plot with dark pink and dark violet colors

sc = plt.scatter(

    pole_angles, angular_velocities, c=time_stamps, cmap="plasma", 
    
    alpha=0.9, edgecolors="white", s=20
)


cbar = plt.colorbar(sc)

cbar.set_label("Time (s)", fontsize=12, color="white")

# Labels and Titles with white text for visibility

plt.xlabel("Pendulum Angle (rad)", fontsize=12, color="white")

plt.ylabel("Angular Velocity (rad/s)", fontsize=12, color="white")

plt.title("Phase Portrait: Pendulum Dynamics", fontsize=14, color="white")

# Grid with subtle gray lines

plt.grid(True, linestyle="--", alpha=0.3, color="gray")


# Show plot

plt.show()



![4](https://github.com/user-attachments/assets/fa68e89c-f198-444a-aa52-9459034849b2)


11. Future Considerations =
    
Advanced Disturbance Modeling =

Implement more complex disturbance models (e.g., varying noise levels).

Reinforcement Learning (Extra Credit) =

Implement a DQN (Deep Q-Network) to train an agent to stabilize the pendulum. Compare this approach to the LQR controller in terms of performance and efficiency.




12. Conclusion =
    
This project provides a comprehensive hands-on approach to analyzing and tuning the LQR controller in a cart-pole system under dynamic disturbances. By adjusting the Q and R matrices, students will gain insights into system behavior, stability, and control trade-offs. The challenge of handling external disturbances while ensuring control efficiency mimics real-world conditions, such as space missions, where robots must operate under unpredictable environmental conditions.
