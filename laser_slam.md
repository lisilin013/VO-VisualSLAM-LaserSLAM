# Laser SLAM
目前详细了解了NDT算法，并且在无人车项目中进行了定位实现。下一步，沿着这个方向了解学习一下当前激光SLAM领域内的经典算法，看看是否可以迁移到无人车项目中。

# Motivation
- 高精度激光定位
- 多传感器融合定位，包括lidar，imu，gps，wheel encoder(optinal)，camera(optinal)
- 激光的高精度定位基本都是依赖高精度地图的；或者不依赖地图把激光当作一个odometry，提供一个粗糙的比较好的定位

# Plan
## LOAM
就定位而言需要研究一下LOAM算法的论文和各种不同的代码实现，包括：

- [LOAM_NOTED: LOAM的中文注释版以及相关搜集的论文](https://github.com/cuitaixiang/LOAM_NOTED)
- [A-LOAM: 港中文沈老师组修改的LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
- [LeGO-LOAM: 也是一个非常牛逼的LOAM实现，看他们的实现效果，貌似比较适合在校园里面跑](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
- [loam_velodyne: 根据原始版本修改后的一个版本，高星，拿来试一试](https://github.com/daobilige-su/loam_velodyne.git)
- [LOAM-multi-thread: 多线程版本的LOAM，不需要ros即可运行](https://github.com/tiger20/LOAM-multi-thread.git)

## Sensor Fusion
另外一个方面就是涉及到多传感器融合的，包括：

- [貌似是一个利用LOAM标定gps的](https://github.com/cuitaixiang/gpsCalibration)
我的理解是可以用来标定低精度的gps的，如果面对的是一个经常丢失信号的gps，估计很难做到
- [eth的一个基于EKF的传感器融合框架](https://github.com/ethz-asl/ethzasl_sensor_fusion)
- [另外一个传感器融合方案，github高星](https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion.git)

## 相关Laser SLAM
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam.git) hdl里面也实现了一个使用imu的ukf给ndt匹配做预测的pose_estimator，但是貌似直接用不好用，没有仔细研究，存疑。此外，hdl是一套东西，包括SLAM，Localization，people tracking三部分
- [cartographer](https://github.com/googlecartographer/cartographer.git) 值得注意的是，cartographer里面也包括定位的功能
- [cartographer_ros](https://github.com/googlecartographer/cartographer_ros.git)
