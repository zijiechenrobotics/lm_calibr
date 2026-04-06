# LM-Calibr Guide

To facilitate testing, we provide datasets for both simulated and real-world scenarios ([Baidu Netdisk](https://pan.baidu.com/s/1hZgSJ6Y6L0QfLroh_AOyRg?pwd=yncu)). Please place the downloaded datasets into the `/root/workspace/data` directory.

> [!NOTE]
> The following commands all use the default workspace as `/root/workspace`. If you are not using Docker to start the application, please replace the workspace path accordingly.

## 0. Compile
```bash
cd /root/workspace
colcon build
```

## 1. Calibration in Simulation

### 1.1. Single-Scene Calibration

```bash
source install/setup.bash
ros2 launch lm_calibr sim_calib.py
```

* Configuration file: `/root/workspace/src/lm_calibr/config/sim_calib.yaml`
* Calibration results: `/root/workspace/install/lm_calibr/share/lm_calibr/result`

### 1.2. Multi-Scene Joint Calibration

Joint calibration effectively prevents LM-Calibr from overfitting to a single scene, thereby significantly improving overall calibration accuracy.

```bash
source install/setup.bash 
ros2 launch lm_calibr sim_joint_calib.py
```

* Configuration file: `/root/workspace/src/lm_calibr/config/sim_joint_calib.yaml`
* Calibration results: `/root/workspace/install/lm_calibr/share/lm_calibr/result`

## 2. Calibration in the Real World

### 2.1. Check the Initial Guess (Important!!!)

LM-Calibr guarantees accuracy and convergence when **the initial angle error is less than 0.2 rad and the translation error is less than 0.2 m**. Therefore, before running the calibration, it is crucial to check whether the error of your initial guess is too large (i.e., whether the Denavit-Hartenberg (DH) parameters are completely incorrect). You can use the following script to verify this:

```bash
source install/setup.bash
# Generate the uncalibrated point cloud
ros2 launch lm_calibr check_dh.py
# Visualize the uncalibrated point cloud
pcl_viewer install/lm_calibr/share/lm_calibr/result/encoder_cloud.pcd
```

**Correct DH Parameters**

As shown in the figure below, if the provided DH parameters are **roughly correct**, the uncalibrated point cloud will exhibit distortion. In this case, LM-Calibr utilizes a coarse-to-fine strategy to guarantee both accuracy and convergence.

![framework](figures/calibration_result.jpg)

**Incorrect DH Parameters**

Conversely, as shown in the figure below, if the provided DH parameters are **completely incorrect**, the point cloud will collapse into a disorganized cluster. As a result, LM-Calibr cannot extract valid planar features from such a chaotic point cloud.

![framework](figures/error_dh.jpg)

### 2.2. Single-Scene Calibration

```bash
source install/setup.bash 
ros2 launch lm_calibr calib.py
```

* Configuration file: `/root/workspace/src/lm_calibr/config/calib.yaml`
* Calibration results: `/root/workspace/install/lm_calibr/share/lm_calibr/result`

### 2.3. Multi-Scene Joint Calibration

```bash
source install/setup.bash 
ros2 launch lm_calibr joint_calib.py
```

* Configuration file: `/root/workspace/src/lm_calibr/config/joint_calib.yaml`
* Calibration results: `/root/workspace/install/lm_calibr/share/lm_calibr/result`

# 3. Transform Point Cloud from LiDAR Frame to Base Frame

After calibration, copy the calibration results from `calib_results/newest_calib_result.yaml` into `config/transform_cloud.yaml`. Then, the `transform_cloud_node` can be used to transform laser points from the LiDAR frame to the base frame.

This node subscribes to the `lidar_topic` and `encoder_topic`, and then publishes the transformed laser points in the base frame via the `/calibrated_livox` topic. This topic can be directly used as the LiDAR input for any LIO system (e.g., Fast-LIO2).

```bash
source install/setup.bash 
ros2 launch lm_calibr transform_cloud.py
```