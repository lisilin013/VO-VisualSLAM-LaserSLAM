# Visual SLAM Learning Sources

- [一起学ORBSLAM2-CSDN源码分析](https://blog.csdn.net/qq_30356613/article/category/6897125)
- [翻译：ORB-SLAM: a Versatile and Accurate Monocular SLAM System](https://blog.csdn.net/super_mice/article/details/50972992)
- [ORB-SLAM 与图优化](https://zhuanlan.zhihu.com/p/29682514)
- [ORBSLAM 冯兵视频分享](https://www.bilibili.com/video/av7102994/)

# 深入g2o无法自拔
- [X] [CSDN g2o tutorial](https://blog.csdn.net/heyijia0327/article/details/47813405)

- [ ] [graph slam 代码](https://github.com/versatran01/graphslam)
- [ ] [双目视觉slam代码](https://github.com/SiuKeungm/stereoVO)
- [ ] [simple_slam_loop_closure代码](https://github.com/nicolov/simple_slam_loop_closure)
- [ ] [VisualOdometry代码](https://github.com/ldq9526/VisualOdometry)
- [x] [g2o入门详解--知乎](https://zhuanlan.zhihu.com/p/47315608)
- [ ] g2o paper reading

目前slam14讲中公式推到马马虎虎地可以推导个差不多，g2o代码实现也能够大致看明白了。

# Kalman Filter
- [ ] [KF视频讲解](https://www.bilibili.com/video/av24225243/?spm_id_from=333.788.videocard.6)

# VO 模块技能包
- [x] 李代数推导使用
- [ ] 常用非线性优化方法
  - SVD等方法推导理解
- [x] 2d-2d对极几何,求解相机运动两种方法
  - 本质矩阵，基础矩阵
  - 单应矩阵
- [x] 2d-3d
  - 直接线性变换法DLT(一般不用，利用的点太少)
  - PnP求解
  - 构建李代数形式，进行BA优化，使用g2o实现
- [x] 3d-3d
  - SVD方法，可以求出解析解
  - 构建李代数形式，进行BA优化，使用g2o实现，一次迭代误差基本就不变了，说明SVD方法可以求解解析解
