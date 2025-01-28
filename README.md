First-Order Boustrophedon Navigator (Lawnmower pattern) using ROS2

The First-Order Boustrophedon Navigator (Lawnmower pattern) is a motion planning algorithm used to cover an area such as a floor or a field by moving back and forth in a grid-like manner, much like the pattern of a lawnmower. In the context of robotics, it’s commonly used for tasks like exploration, mapping, or cleaning, where the robot needs to cover an entire area in an efficient, systematic way.

The ROS2 (Robot Operating System 2) part is a framework for writing robot software, and it provides tools for creating robot behaviors like navigation, control, and sensor integration. It is used by developers to implement algorithms, control robot hardware, and connect different software components.


1] Project Overview =
This project implements a First-Order Boustrophedon Navigator (Lawnmower pattern) for a robot using ROS2. The goal is to make the robot follow a precise boustrophedon pattern to cover a given area while minimizing cross-track error and maintaining smooth motion. The robot's motion is controlled using a Proportional-Derivative (PD) controller, and the parameters of the controller are tuned to achieve optimal performance. The system is tested using real-time parameter tuning and performance evaluation, including trajectory tracking accuracy, coverage efficiency, and smoothness of motion.


2] Objective =

    Optimize PD controller parameters for precise tracking of the boustrophedon pattern.
    Minimize cross-track error while ensuring smooth and efficient motion.
    Optimize the boustrophedon pattern parameters for coverage efficiency.
    Analyze performance and document the results, including the methodology and challenges encountered during the tuning process.



3] System Used =
    Ubuntu 22.04 + ROS2 Humble

    Required ROS2 Packages:
        ros-$ROS_DISTRO-turtlesim
        ros-$ROS_DISTRO-rqt*

    Python Dependencies:
        numpy
        matplotlib


4] Installation and Setup =
1. Fork the Course Repository =
    Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
    Click "Fork" in the top-right corner and select your GitHub account as the destination.


2. Clone Your Fork =
Clone your repository to your local machine:
cd ~/
git clone https://github.com/sanskar-sunil-patil/RAS-SES-598-Space-Robotics-and-AI.git


3. Create a Symlink to the Assignment in Your ROS2 Workspace =
   cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .


4. Build the Package =
   cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash


5. Launch =
   ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py


6. Monitor Performance =
   ros2 topic echo /cross_track_error


7. Visualize Performance =
   ros2 run rqt_plot rqt_plot


5] Controller Tuning =
The PD controller is used to control the robot’s motion along the boustrophedon pattern. The following parameters are adjustable for fine-tuning the robot's performance =

1. Kp_linear: Proportional gain for linear velocity = 3.0
2. Kd_linear: Derivative gain for linear velocity = 0.1
3. Kp_angular: Proportional gain for angular velocity = 12.0
4. Kd_angular: Derivative gain for angular velocity = 0.0
5. Spacing = 1.0

Objective = Minimize the cross-track error, smooth out velocity profiles, and achieve good cornering behavior. Use rqt_reconfigure to tune these parameters in real-time. Start with low values for each gain and gradually increase them while monitoring the impact on the performance metrics.


6] Performance Metrics =

1. Average Cross-Track Error: The average deviation of the robot from its planned path.
2. Maximum Cross-Track Error: The largest deviation from the path.
3. Smoothness of Motion: How smoothly the robot moves, without sudden jerks or changes in velocity.
4. Cornering Performance: How well the robot handles turns at the end of each row.


7] Pattern Optimization =
The boustrophedon pattern is optimized for =

1. Spacing between lines (spacing): The distance between parallel rows in the boustrophedon pattern.
2. Coverage Efficiency: The ability of the robot to cover the entire area without unnecessary overlaps or gaps.
3. Pattern Completeness: The robot must cover the entire designated area.

Tuning the spacing between lines and ensuring complete coverage is crucial for performance.


8] Grading Rubric =

    Controller Performance (60%):
        Average cross-track error < 0.2 units: 25%
        Maximum cross-track error < 0.5 units: 15%
        Smooth velocity profiles: 10%
        Clean cornering behavior: 10%

    Pattern Quality (20%):
        Even spacing between lines.
        Complete coverage of target area.
        Efficient use of space.

    Documentation (20%):
        Clear explanation of tuning process.
        Well-presented performance metrics.
        Thoughtful analysis of results.


9] What is Kp_linear, Kp_angular, Kd_linear, Kd_angular, Spacing ?
1. Proportional Gain (Kp) or Kp_linear =

    Purpose = The proportional gain for linear velocity controls how much the robot adjusts its forward motion based on the cross-track error (the deviation from the target path in terms of distance).
    What it does =
   1. If the robot is off the desired path (to the left or right), the proportional term will cause the robot to adjust its linear velocity to correct the position.
   2. A higher Kp_linear means that the robot will correct errors more aggressively (move faster to return to the path).
   3. A lower Kp_linear means that the robot will be slower to react to its position error.

2. Kp_angular =

    Purpose = The proportional gain for angular velocity controls how much the robot adjusts its turning speed based on the angular error (the difference between the robot’s current orientation and its desired orientation).
    What it does =
   1. If the robot is facing the wrong direction (not aligned with the path), the proportional term will cause the robot to adjust its angular velocity (how fast it turns) to correct its orientation.
   2. A higher Kp_angular makes the robot turn more quickly to align itself with the correct orientation.
   3. A lower Kp_angular results in slower turning and less aggressive corrections.

3. Derivative Gain (Kd) or Kd_linear =

    Purpose = The derivative gain for linear velocity is used to smooth out the robot’s movement by dampening the rate of change in its linear velocity.
    What it does =
   1. It looks at the rate of change of the robot’s position error. If the robot is moving too quickly in one direction (even if it is correcting an error), the derivative term will apply a counteracting force to prevent
        overshooting and reduce oscillations.
   2. A higher Kd_linear helps dampen sudden movements and makes the robot move in a smoother, more controlled way, especially when close to the desired path.
   3. A lower Kd_linear might result in more jerky or oscillatory movements if the robot overshoots its corrections.

4. Kd_angular =

    Purpose = The derivative gain for angular velocity works similarly for turning. It dampens the robot's angular velocity to prevent excessive oscillation or overshooting during turns.
    What it does =
   1. It looks at how quickly the robot is turning (how fast the angular error is changing). If the robot is turning too fast in one direction, the derivative term helps slow down the turning motion to prevent overshoot or jerky motion.
   2. A higher Kd_angular adds damping to the turning behavior, leading to smoother cornering and reducing oscillation after turns.
   3. A lower Kd_angular means that the robot’s turns may become sharper and less controlled, possibly causing the robot to overshoot its target orientation or experience jerky motion at the corners.


10] How These Parameters Affect the motion ?

1.Linear Motion (Forward Motion) =

 1.1. Kp_linear =
 If this gain is high, the robot will try to aggressively correct its position when it deviates from the path. However, if it is too high, it may overshoot the target position and oscillate before settling. If it is too low, the robot will
 correct its position slowly and may drift off the path more easily.

1.2. Kd_linear =
This gain controls how smoothly the robot adjusts its linear velocity. If this gain is high, it helps reduce overshoot and makes the robot’s movements more stable, as it dampens the robot’s motion to avoid oscillations. If the robot doesn't have enough Kd_linear, it might keep oscillating back and forth, especially when trying to stabilize after corrections.


2. Angular Motion (Turning and Orientation) =

2.1.Kp_angular =
If this gain is high, the robot will turn aggressively to align itself with the desired orientation. This can make the robot’s turns faster but might also lead to overshooting the target orientation, especially at sharp
turns. If Kp_angular is too low, the robot may turn slowly, which could make the robot less responsive to course corrections when navigating turns or switching rows in the boustrophedon pattern.

2.2.Kd_angular =
If this gain is high, it helps smooth out turns by slowing the robot’s angular velocity as it gets closer to the correct orientation, preventing overshoot. It helps the robot make controlled, smooth turns without jerking from one side to another. If it’s too low or zero (as in your case), the robot’s turns could become more abrupt, leading to sharp, jerky movements when changing direction.


11] Summary of Each Term =

1. Kp_linear = How quickly the robot reacts to being off the desired path (cross-track error).
2. Kd_linear = How much damping is applied to smooth the robot's motion as it tries to correct its position.
3. Kp_angular = How quickly the robot turns to face the correct direction (during turns).
4. Kd_angular = How much damping is applied to smooth the robot's turns and prevent jerky motion.

By adjusting these parameters, you can control how aggressively and smoothly the robot navigates its path and turns. The challenge is finding the right balance to minimize both cross-track errors and jerky motion, ensuring smooth, efficient navigation across the area. Just by puting all values to 0.0 and slightly increasing it to get correct value combination.


12] Observation =

1. Min Cross-Track Error (< 0.2 units) =
   Kp_linear = 3.0 and Kd_linear = 0.1 provided a strong, responsive correction to small deviations from the desired path. The proportional gain Kp_linear ensured that the robot corrected quickly when off course, while the derivative term Kd_linear dampened any oscillations, resulting in the average cross-track error being well below 0.2 units.

3. Max Cross-Track Error (< 0.5 units) =

    The robot maintained excellent path accuracy with a maximum cross-track error that stayed under 0.5 units. The choice of Kp_angular = 12.0 ensured that the robot could quickly align itself during turns without overshooting or deviating too far from its intended course. While Kd_angular = 0.0 allowed the robot to make sharp turns without excessive damping, it helped keep the maximum error in check, especially during transitions between rows in the boustrophedon pattern.

4. Smoothness of Motion and Cornering =

    The robot demonstrated smooth motion thanks to the low Kd_angular value, which helped avoid sudden changes in direction during turns. This allowed for smooth transitions between lines in the boustrophedon pattern, contributing to overall cornering performance. The system exhibited clean cornering behavior with no noticeable jerking or sharp deviations during direction changes.

5. Pattern Optimization =

    With spacing = 1.0, the robot covered the target area effectively, maintaining a balance between coverage efficiency and pattern completeness. The path was neither too close nor too far apart, ensuring no overlap while covering the area uniformly.

Challenges and Solutions:

    Challenge: Overshoot and oscillations during turns due to high Kp_angular.
        Solution: By adjusting the Kp_angular to 12.0 and leaving Kd_angular at 0.0, the robot achieved fast, efficient turns without oscillating or overshooting.
    Challenge: Maintaining smooth motion while ensuring fast corrections.
        Solution: The combination of Kp_linear at 3.0 and Kd_linear at 0.1 resulted in a responsive, but controlled movement that reduced excessive motion while ensuring the robot followed the pattern accurately.

13] Future Considerations =

    Additional testing in varied environments could help refine the controller further, especially in handling obstacles or more complex terrains.
    Experimenting with a slightly higher Kd_angular might improve the robot's behavior in more complex turns without compromising the overall responsiveness of the system.


14] Conclusion = 

By carefully tuning the PD controller parameters and optimizing the boustrophedon pattern, we were able to significantly improve the turtlebot path-following accuracy, smoothness, and overall performance. The insights gained from this project can be applied to other robotic navigation tasks, providing a solid foundation for further experimentation and improvement.
