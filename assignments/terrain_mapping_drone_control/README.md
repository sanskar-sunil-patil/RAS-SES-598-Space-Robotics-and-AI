# Terrain Mapping Drone Control

This ROS2 package implements a drone control system for terrain mapping of the Bishop Fault Scarp using ORBSLAM3 and PX4 SITL simulation.

# 1. Abstract =

This report presents the development and implementation of a ROS2-based drone control system for terrain mapping of the Bishop Fault Scarp. The system utilizes PX4 SITL for simulation and ORBSLAM3 for real-time SLAM (Simultaneous Localization and Mapping). A structured lawnmower pattern is employed to ensure full area coverage while maintaining a consistent altitude. The system is capable of autonomous takeoff, flight control, data acquisition, and return to the launch point. This document details the system's architecture, methodology, implementation, results, and potential improvements.

# 2. Introduction =

## 2.1 Background =

Accurate terrain mapping is essential in geospatial studies, disaster response, and environmental monitoring. Unmanned Aerial Vehicles (UAVs) equipped with vision-based SLAM provide an efficient solution for 3D terrain reconstruction.

## 2.2 Problem Statement =

Current manual surveying techniques are labor-intensive and prone to inaccuracies. A fully autonomous drone-based mapping system enhances efficiency, precision, and repeatability.

## 2.3 Objectives =

* Develop a ROS2-based autonomous drone system for terrain mapping.

* Integrate PX4 SITL for simulation and ORBSLAM3 for real-time mapping.

* Implement a lawnmower flight pattern for complete area coverage.

* Enable configurable flight parameters for flexible survey planning.

* Provide data outputs in forms of 3D point clouds, trajectory logs, and images.


# 3. System Architecture =

## 3.1 High-Level Overview =

The system consists of the following components:

* PX4 SITL Simulator: Simulates UAV dynamics.

* ROS2-based Controller: Manages drone navigation and data acquisition.

* ORBSLAM3: Performs real-time SLAM using camera inputs.

* Gazebo: Provides a 3D simulation environment.



## 3.2 System Diagram =







# 4. Methodology =

## 4.1 Flight Pattern =

The drone follows a structured lawnmower pattern over the target area. Key parameters include:

* Mapping Height: Configurable flight altitude.

* Survey Speed: Drone velocity during mapping.

* Area Dimensions: Defined by width and length.

* Strip Spacing: Distance between adjacent survey lines.


  ## 4.2 SLAM Integration =
  
* ORBSLAM3 processes camera images for real-time localization and map generation.

* The system records trajectory data and generates a 3D point cloud.


  # 5. Implementation =

  ## 5.1 Prerequisites =

* Software: ROS2 Humble, PX4 SITL, ORBSLAM3, OpenCV, Python 3.8+

* Hardware: Simulated or real UAV with a compatible camera


## 5.2 Installation Steps =

### 1. Clone the Repository =


cd ~/ros2_ws/src

git clone <repository_url>


### 2. Install Dependencies =

cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y


### 2. Install Dependencies =
