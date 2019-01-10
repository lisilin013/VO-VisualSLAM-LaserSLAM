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
  - 2d-2d(三角测量)
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
设想一个robot在某个环境下运动，它的camera system和机器人本体刚性固定（rigidly-attached），在离散时间下拍摄一系列图片。
相邻两帧之间的变换用R，t表示，这样可以求出从初始时刻到现在的一系列变换矩阵set，最后求出一系列camera poses，需要用到初始时刻的相机坐标系位置。
通过一个m-pose windowed BA来refine camera轨迹。

# 4 Camera modeling and calibration
- Pin-Hole Approximation
- Omnidirectional Camera Model（全景相机）
可以通过Spherical Model来等价Perspective 和 Omnidirectional Model
全景相机的model还不清楚？

# 5 Motion estimation
Motion estimation是VO system的核心计算步骤，计算previous image 和current image之间的R，t（also called T），通过把这个single movements 连接起来，recover出full trajectory of camera.

## 有两种方法计算T
- Apperance-based
 使用intensity信息，计算more expensive
- Featured-based
 使用feature信息，需要match，more effective

## 具体求解T有三种情况
- 2d-2d(三角测量，epipolar geometry, conputing Essential Matrix)
  The minimal-case solution involves 5-point correspondences.
  relative scale computation， 从两帧图像是无法计算绝对尺度的，但是可以计算相对尺度。为了robustness，scale ratios通常由多对图像计算完后取均值。
  both for monocular and stereo
- 3d-3d(icp)
  it is necessary to triangulate 3D points. 
  only for stereo
- 3d-2d(pnp)
  both for monocular and stereo.
  In the monocular case, the 3D structure needs to be triangulated from two adjacent camera views (e.g., 𝐼𝑘−2 and 𝐼𝑘−1) and then matched to 2D image features in a third view (e.g., 𝐼𝑘).

## Triangulation and Keyframe Selection
Triangulated 3D points are determined by intersecting backprojected rays from 2D image correspondences of at least two image frames.
啥意思？？（通过交叉来自至少两个图像帧的2D图像对应的反投影光线来确定三角化3D点）

In reality, they never intersect（相交） due to
- image noise,
- camera model and calibration errors,
- and feature matching uncertainty

The point at minimal distance from all intersecting rays can be taken as an estimate of the 3D point position.
(距离所有相交光线的最小距离处的点可以被视为3D点位置的估计)

When frames are taken at nearby positions compared to the scene distance, 3D points will exibit large uncertainty

**Therefore, 3D-3D motion estimation methods will drift much more quickly than 3D-2D and 2D-2D methods**

In fact, the uncertainty introduced by triangulation affects the motion estimation. In fact, in the 3D-to-3D case the 3D position error is minimized, while in the 3D-to-2D and 2D-to-2D cases is the image reprojection error.

One way to avoid this consists of skipping frames until the average uncertainty of the 3D points decreases below a certain threshold. The selected frames are called keyframes.

KeyFrame Selection 在VO中是非常重要的一步，需要在update motion之前就完成。

## summary
- 在stereo情况下， 3d-2d方法比3d-3d方法drift更少
- stereo比monocular在absolute scale上的motion和structure计算更有优势， drift也更少
- 当场景距离比stereo baseline更大的时候，stereo degenerates(退化) into monocular VO
- KeyFrame Seclection应该小心谨慎，可以降低drift
- 无论选择什么计算方法, local BA (over the last m frames) 总会使得轨迹估计更准确. After BA, 会减弱运动估计方法的影响 (as long as the initialization is close to the solution)


# 6 Robust estimation
匹配点里面包含outliers，也就是包含错误匹配，Robust Estimation的工作就是剔除这些outliers。
造成outliers的原因：

- image noise
- occlusions(闭塞？啥意思？)
- blur
- changes in view point and illumination, feature detector 和descriptor的数学模型没有考虑这些变化

## RANSAC Example: Line Extraction Algorithm Steps
- select sample of 2 points at random
- calculate model params that fit the data in the sample
- calculate error function for every point left in the sample
- select data that support current hypothesis, store the inliers number
- repeat sampling and do the same thing, until we find a large enough inliners number or iterations number（eg: 1000） is enough, then keep the corresponding selected data that fits hypothesis.
- finally, use the inliers last step to estimate the real model

Fishler & Bollers 1981已经建立了存在outliers的运动估计标准方法.

需要进行的迭代次数有如下公式计算：
Ｎ = log(1-p)/log(1-(1-epsilon)^s)

- s: 实例化模型需要的点的数量（eg: 比如估计６dof相机运动可以用5-points RANSAC，大概需要迭代1000次, 通常需要的points num = dof - 1）
- epsilon: outliers占数据总量的百分比
- p: 要求计算成功的准确率
- N: 需要迭代的次数

RANSAC每次算出的结果都有不同，但是对着迭代次数的增多会变得稳定，为了鲁棒性，通常迭代次数会用上述公式再乘以10倍。
更高级的算法实现有RANSAC estimate the fraction of inliers adaptively.

## 如何降低实例化模型需要的点的数量呢？
可以通过运动约束。
比如在二维平面，只需要估计３个参数，也就是需要2-points,大概需要迭代100次

## exploit the vehicle non-holonomic constraints
比如在二维平面，利用Ackerman转向原理，车子转向可以只用２个参数描述（1-point needed）

## is it really better to use minimal sets in RANSAC?
- 如果对算法速度有要求，那么尽量使用minimal sets
- 如果图像噪声很大，对实时性要求没那么苛刻，使用non-minimal set会更好

# 7 Error propagation

# 8 Camera-pose optimization (bundle adjustment)
# 9 Discussion
