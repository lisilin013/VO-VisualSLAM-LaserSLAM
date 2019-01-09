# visual_odometry

This software is built for learning visual odometry.

# Reference
- [Visual_Odometry_Tutorial](http://rpg.ifi.uzh.ch/docs/Visual_Odometry_Tutorial.pdf)
- [vo part I and part II](http://rpg.ifi.uzh.ch/visual_odometry_tutorial.html)

# 1 Introduction
## VO Four Assumptions
- 环境光照足够连续
- 足够多的纹理保证能够提取到明显的运动
- Dominance of static scene over moving objects（静态场景比移动物体更容易提取？）
- 连续帧之间有足够的场景重叠（变化不能太剧烈，否则会找不到特征点之间的匹配？）

## Why VO?
- 相比于轮式里程计不受限与地形并且精度更好（位置误差0.1% - 2%,真的假的？我需要试一试）
- vo可作为一个补充
  - wheel odometry
  - GPS
  - IMU
  - laser odometry
- 在无GPS区域内作用很大，比如水下，aerial（空中？什么鬼？）

## Working Principle
通过匹配连续两帧图像的特征点，来恢复相机的运动，求出R，t

具体怎么求解？

## VO Flow Chart
- image sequence
- feature detection
- feature matching
- motion estimation（R,t）
  - 2d-2d(三角测量?)
  - 3d-3d(icp)
  - 3d-2d(pnp)
- local optimization

## VO Drift
运动误差是累计的

## VO or Structure From Motion（SFM）
- SFM比VO更general，用于解决三维重建问题，包括从undered image sets中恢复出structure和camera poses. SFM最终的structure和camera poses可以使用offline optimizaiton（比如BA优化），计算时间随着图像数量的增长变大
- VO是SFM的particular case，注重于估计3d motion of camera sequentially and in real time. BA（optional）可以用来优化轨迹的局部估计

## VO and Visual SLAM
- SLAM的目标是获得一个global， consistent的机器人路径估计， 这可以通过loop closures完成。检测到loop closure 时， 可以利用回环信息降低地图和相机路径的漂移（global BA）
- VO致力于recovering the path incrementally, pose after pose, and optimize only over the last m poses path(windowed BA)。 VO仅针对轨迹的局部一致性。
- VO 可以用来作为building block of SLAM. VO is SLAM before closing the loop. VO将实时性能的一致性进行折衷，而无需跟踪相机的所有历史记录

# 2 Brief history of VO
2004年VO用在了NASA的Mars mission的机器人中
同年，Nister«Visual Odometry»论文在学术环境中revived VO， VO 流传开来。

# 3 Problem formulation
# Camera modeling and calibration
# Motion estimation
# Robust estimation
# Error propagation
# Camera-pose optimization (bundle adjustment)
# Discussion
