<div align="center">

# ESVIO: Event-based Stereo Visual Inertial Odometry

</div>

## Introduction
<div style="text-align: justify;">

**ESVIO** is the first stereo event-based visual inertial odometry framework, including ESIO (purely event-based) and ESVIO (event with image-aided). The stereo event-corner features are temporally and spatially associated through an event-based representation with spatio-temporal and exponential decay kernel. The stereo event tracker are then tightly coupled into a sliding windows graph-based optimization framework for the estimation of ego-motion.

</div>

<div align="center">
<!--     <img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/ESVIO_hdr_flight_gif.gif" width = 80% > -->
    <a href="https://kwanwaipang.github.io/ESVIO/" target="_blank"><img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/ESVIO_hdr_flight_gif.gif" width="80%" /></a>
    <p>Onboard Quadrotor Flight using Our ESVIO as State Estimator in HDR Scenarios</p>
    <p>(click the gif to open the project website)</p>
</div>

### Developers:
[Peiyu Chen](https://scholar.google.com/citations?hl=zh-CN&user=rYj8xaoAAAAJ),   [Weipeng Guan](https://kwanwaipang.github.io/)

### Related papers
ESVIO is published in IEEE RA-L with IROS2023 presentation option. (The IEEE RA-L pdf is available [here](https://ieeexplore.ieee.org/document/10107754) and the arxiv pdf is available [here](https://arxiv.org/pdf/2212.13184.pdf)).
~~~
@article{ESVIO,
  title={ESVIO: Event-based Stereo Visual Inertial Odometry},
  author={Chen, Peiyu and Guan, Weipeng and Lu, Peng},
  journal={IEEE Robotics and Automation Letters},
  year={2023},
  volume={8},
  number={6},
  pages={3661-3668},
  publisher={IEEE}
}
~~~
~~~
@inproceedings{EIO,
  title={Monocular Event Visual Inertial Odometry based on Event-corner using Sliding Windows Graph-based Optimization},
  author={Guan, Weipeng and Lu, Peng},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={2438-2445},
  year={2022},
  organization={IEEE}
}
~~~

If you feel like ESVIO has indeed helped in your current research or work,  a simple star or citation of our works should be the best affirmation for us. :blush: 

### Our accompanying videos

Our **video demos** are available on **Bilibili** (click below images to open) and **YouTube**<sup>[1]( https://www.youtube.com/watch?v=XqAm1q0alNY&t=47s)</sup>.

<div align="center">
<a href="https://www.bilibili.com/video/BV1ve4y1M7v4/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank"><img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/RAL_cover.png" alt="video" width="48%" /></a>
<a href="https://www.bilibili.com/video/BV1Ju411P7sq/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank"><img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/IROS_cover.png" alt="video" width="48%" /></a>
</div>

<!-- <div style="display: flex; justify-content: space-around;">
  <div style="text-align: center;">
    <a href="https://www.bilibili.com/video/BV1ve4y1M7v4/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank">
      <img src="pics/RAL_cover.png" alt="video" width="48%" />
      <br>IEEE RA-L
    </a>
  </div>
  <div style="text-align: center;">
    <a href="https://www.bilibili.com/video/BV1Ju411P7sq/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7" target="_blank">
      <img src="pics/IROS_cover.png" alt="video" width="48%" />
      <br>IROS2023
    </a>
  </div>
</div> -->



</br>

The performance evaluation videos in various datasets (including the comparisons with [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion), and [Ultimate-SLAM](https://github.com/uzh-rpg/rpg_ultimate_slam_open)) as well as testing of outdoor large-scale and onboard drone flights can also be found on **Bilibili**<sup>
[1](https://www.bilibili.com/video/BV168411778U/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7)
[2](https://www.bilibili.com/video/BV13G411T7eY/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7)
[3](https://www.bilibili.com/video/BV1pg411H7Yo/?spm_id_from=333.999.0.0&vd_source=a88e426798937812a8ffc1a9be5a3cb7)
</sup>.
<div align="center">
    <img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/ESVIO_comparision_with_SOTA.gif" width = 80% >
    <p>Compare ESIO with ORB-SLAM3, and Ultimate SLAM</p>
</div>

<!-- </br> -->

<!-- <div align="center">
<a target="_blank"><img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/ESIO_ESVIO_DSEC.png" alt="image" width="80%" /></a>
<p> (a) ESIO and (b) ESVIO Evaluation in DSEC (Driving Sequences
zurich_city_04_a to zurich_city_04_f) </p>
</div> -->


## 1. Prerequisites
1.1 Ubuntu 20.04 with ROS Noetic.


1.2 Ceres Solver Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to make install and use the version 1.14.0 ([Download Link](dependences/ceres-solver-1.14.0.zip)).


1.3 we use catkin build, and all the dependency files are stored within the folder `dependences`.

## 2. Build
~~~
mkdir -p catkin_ws_dvs/src
cd catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws_dvs/src
git clone git@github.com:arclab-hku/ESVIO.git --recursive
~~~

You should modifie your `.bashrc` file through `gedit ~/.bashrc`, add the following codes in it:
~~~
source ~/catkin_ws_dvs/devel/setup.bash
alias esviobuild='cd ~/catkin_ws_dvs/src && catkin build esvio_estimator feature_tracker pose_graph -DCMAKE_BUILD_TYPE=Release -j8'
~~~

After that, run the `source ~/.bashrc ` and `esviobuild` command in your terminal.

## 3. Run on Dataset

### 3.1 Run on HKU-dataset
#### 3.1.1 Download our rosbag files ([HKU-dataset](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM))
Our datasets for evaluation can be download from our One-drive or Baidu-Disk. 
We have released totally 9 rosbag files for evaluating ESVIO, with the introduction of these datasets can be found on this [page](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM?tab=readme-ov-file#Dataset-for-stereo-evio).
</br>
For the convenience of the community, we also release the raw results of our methods in the form of rosbag ([link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/Results_for_comparison.md)). 

#### 3.1.2 Run our examples
After you have downloaded our bag files, you can now run our example:
* For the ESIO (event+imu) version:
~~~
roslaunch esvio_estimator esio.launch 
rosbag play YOUR_DOWNLOADED.bag
~~~

* For the ESVIO (event+image+imu) version:
~~~
roslaunch esvio_estimator esvio.launch 
rosbag play YOUR_DOWNLOADED.bag
~~~

### 3.2 Run on Your Event Camera
#### 3.2.1 Driver Installation
We thanks the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) and [DV ROS](https://gitlab.com/inivation/dv/dv-ros) for their intructions of event camera driver.
We add some modification for the code, and the driver code of the event camera is available in [link](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/tree/main/driver_code).
User can choose either one.

##### For rpg_dvs_ros
* Step 1: Install libcaer (add required repositories as per [iniVation documentation](https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html#ubuntu-linux) first):
~~~
sudo apt-get install libcaer-dev
~~~

*Step 2: Create a catkin workspace and copy the driver code:
~~~
mkdir -p ~/catkin_ws_dvs/src
cd ~/catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`
cd ~/catkin_ws_dvs/src
~~~

And then copy the code from our link, or directly use the driver code in the dependences folder

* Step 3: Build the packages:
~~~
catkin build davis_ros_driver  (if you are using the DAVIS)
catkin build dvxplorer_ros_driver  (if you are using the DVXplorer)

source ~/catkin_ws_dvs/devel/setup.bash
~~~

* Step 4: After source your environment, you can open your event camera:
~~~
roslaunch dvs_renderer davis_mono.launch` (if you are using the DAVIS)
roslaunch dvs_renderer dvxplorer_mono.launch` (if you are using the DVXplorer)
~~~

##### For DV ROS
* Step 1: Instalizing DV software libraries:
~~~
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing dv-runtime-dev gcc-10 g++-10
~~~

* Step 2: It is build using catkin tools, run the following commands from your catkin workspace:
~~~
mkdir -p ~/catkin_ws_dvs/src
cd ~/catkin_ws_dvs
catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`
cd ~/catkin_ws_dvs/src
~~~

And then copy the code from our link.

* Step 3: Modifying your `.bashrc` file, add the following codes in it:
~~~
source ~/catkin_dvs_ws/devel/setup.bash

alias dvsbuild='cd ~/catkin_dvs_ws && catkin build dv_ros_accumulation dv_ros_capture dv_ros_imu_bias dv_ros_messaging dv_ros_runtime_modules dv_ros_tracker dv_ros_visualization -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10'

alias dvsrun='cd ~/catkin_dvs_ws/src/Event_based_VO-VIO-SLAM/driver_code/dv-ros-master/script && sh run.sh'
~~~

* Step 4: the user can directly run the following command `dvsbuild` or `dvsrun` in the terminal to build the project and run your event camera, respectively.

**Tips**: Users need to adjust the lens of the camera, such as the focal length, aperture.
Filters are needed for avoiding the interfere from infrared light under the motion capture system.
For the dvxplorer, the sensitive of event generation should be set, e.g. `bias_sensitivity`.
Users can visualize the event streams to see whether it is similiar to the edge map of the testing environments, and then fine-tune it.


#### 3.2.2 Sensor calibration
In order to launch ESVIO on your own hardware setup, you need to have a carefully calibration of the extrinsic among Event, Image and IMU. We recommend you using the following the link ([DVS-IMU Calibration and Synchronization](https://arclab-hku.github.io/ecmd/calibration/)) to kindly calibrate your sensors.
</br>
Following that, you can execute the provided roslaunch file to initialize the stereo event cameras and commence our ESVIO:
~~~
roslaunch esvio_estimator stereo_davis_open.launch
roslaunch esviio_estimator esvio.launch
~~~

## 4.Implementation on UAV
In order to validate the robustness and computational efficiency of ESVIO in actual robots, we build a quadrotor which can carry stereo event cameras, as shown in below.

<div align="center">
<a target="_blank"><img src="https://github.com/arclab-hku/Event_based_VO-VIO-SLAM/blob/main/ESVIO/quadrotor_flight.jpg" alt="image" width="60%" /></a>
</div>


## 5.Acknowledgments 
This work was supported by General Research Fund under Grant 17204222, and in part by the Seed Fund for Collaborative Research and General Funding Scheme-HKU-TCL Joint Research Center for Artificial Intelligence.
We use ([VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)) as our baseline code. Thanks Dr. Qin Tong, Prof. Shen, etc. very much.

## 6. License
The source code is released under GPLv3 license. 
We are still working on improving the code reliability. 
If you are interested in our project for commercial purposes, please contact [Dr. Peng LU](https://arclab.hku.hk/People.html) for further communication.
