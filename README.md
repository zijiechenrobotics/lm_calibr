<p align="center">
  <h1 align="center"> Accurate Calibration and Robust Localization for Spinning Actuated LiDAR Systems </h1>
  <p align="center">
    <a href="https://github.com/zijiechenrobotics">Zijie Chen</a>
    , 
    Xiaowei Liu
    ,
    <a href="https://teacher.gdut.edu.cn/xuyong/zh_CN/index.htm">Yong Xu</a>
    ,
    <a href="https://snakehaihai.github.io/">Shenghai Yuan</a>
    ,
    <a href="https://github.com/kafeiyin00">Jianping Li</a>
    ,
    <a href="https://dr.ntu.edu.sg/entities/person/Xie-Lihua">Lihua Xie</a>
  </p>
  <p align="center"> <strong>IEEE Robotics and Automation Letters (RA-L, 2026) </strong></p>
  <p align="center">
    <a href="https://arxiv.org/pdf/2601.15946">
      <img src="https://img.shields.io/badge/arXiv-2601.15946-B31B1B?logo=arxiv" alt="arXiv">
    </a>
    &nbsp;
    <a href="https://opensource.org/license/gpl-3-0">
      <img src="https://img.shields.io/badge/License-GPLv3-blue" alt="License: GPLv3">
    </a>
    &nbsp;
    <a href="https://www.youtube.com/watch?v=cZyyrkmeoSk&feature=youtu.be">
      <img src="https://img.shields.io/badge/YouTube-Video-FF0000?logo=youtube" alt="YouTube">
    </a>
    &nbsp;
    <a href="https://www.youtube.com/watch?v=cZyyrkmeoSk&feature=youtu.be">
      <img src="https://img.shields.io/badge/Bilibili-视频-00A1D6?logo=bilibili" alt="Bilibili">
    </a>
  </p>
</h3>

## 📃 Abstract

Accurate calibration and robust localization are fundamental for downstream tasks in spinning actuated LiDAR applications. Existing methods, however, require parameterizing extrinsic parameters based on different mounting configurations, limiting their generalizability. Additionally, spinning actuated LiDAR inevitably scans featureless regions,  which complicates the balance between scan coverage and localization robustness. To address these challenges, this letter presents a targetless LiDAR-motor calibration (LM-Calibr) on the basis of the Denavit-Hartenberg convention and an environmental adaptive LiDAR-inertial odometry (EVA-LIO). **LM-Calibr supports calibration of LiDAR-motor systems with various mounting configurations.** Extensive experiments demonstrate its accuracy and convergence across different scenarios, mounting angles, and initial values. Additionally, **EVA-LIO adaptively selects downsample rates and map resolutions according to spatial scale**. This adaptivity enables the actuator to operate at maximum speed, thereby enhancing scan completeness while ensuring robust localization, even when LiDAR briefly scans featureless areas.

![framework](figures/framework.png)

## 📦 Setup

### Build from Docker

(TODO)

### Build from Source

Prerequisites

* Ubuntu == 18.04 or 20.04
* ROS
* gcc & g++ >= 9

(TODO)

## 🚀 Run

### LM-Calibr

(TODO)

### EVA-LIO

(TODO)

## 🔧 Hardware Design

(TODO)

## 📝 Citation

If you find our work and/or code useful, please cite our paper:

```
@ARTICLE{11386907,
  author={Chen, Zijie and Liu, Xiaowei and Xu, Yong and Yuan, Shenghai and Li, Jianping and Xie, Lihua},
  journal={IEEE Robotics and Automation Letters}, 
  title={Accurate Calibration and Robust LiDAR-Inertial Odometry for Spinning Actuated LiDAR Systems}, 
  year={2026},
  volume={11},
  number={3},
  pages={3772-3779},
  doi={10.1109/LRA.2026.3662643}}
```

