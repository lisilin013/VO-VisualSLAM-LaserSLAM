# 三维激光雷达和相机的标定
## Method1: 3D-3D correspondence
- code [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration)

利用Arcu码获取标定板的角点在相机坐标系下的3D位置，同时使用3D激光雷达拟合标定板的四条直线，得到四个角点在激光雷达坐标系中的3D位置。
最后获取两个标定板，共8个点对，然后使用ICP求解相机雷达的R，t
![](https://raw.githubusercontent.com/lisilin013/image_bed/master/markdown20191203173031.png)

## Method2: 3D-2D correspondence
- code [plycal](https://github.com/ram-lab/plycal)

![](https://raw.githubusercontent.com/lisilin013/image_bed/master/markdown20191203173143.png)

利用两个约束构建一个优化问题最后利用ceres求解

- 首先提取出image平面的2D四边形和激光雷达坐标系下的3D四边形
- 构建约束
    - 强约束：四边形每条3D边上的点投影到图像上，到2D边的距离累加当做error
    - 弱约束：3D四边形内部的点投影到图像上，在2D四边内的称为内点，在2D四边形外的点称为外点。计算每一个外点到2D四边形中心center的距离当做error
- 利用ceres的自动求导进行求解最优化问题    