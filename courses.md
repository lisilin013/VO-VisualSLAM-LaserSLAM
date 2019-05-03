# 1 Laser SLAM
## 1.1 Robotics: Estimation and Learning 
[coursera课程链接](https://www.coursera.org/learn/robotics-learning)  

主要讲机器人的状态估计，分四个部分：

- Gaussian Model Learning
主要讲了高斯模型（一维，二维，高维），如何利用Maximum-Likelyhood-Estimate(MLE)估计高斯模型的参数。然后介绍了Gaussian-Mixture-Mode(GMM),并且如何利用Expectation-Maximization(EM)算法估计GMM的参数。
- Bayesian Estimation - Target Tracking
- Mapping 
主要讲了Occupied-Grid-Map算法原理，以及如何利用一组二维激光数据构造二维栅格地图。使用几率Odd来描述grid的占据情况，利用贝叶斯估计推导grid的状态更新方程。
- Bayesian Estimation - Localization

Soem reference docs:

- [占据栅格地图Occupied Grid Map](https://zhuanlan.zhihu.com/p/21738718)
- [公开课的笔记week1](https://blog.yxwang.me/2018/07/robotics-slam-week1/)
- [公开课的笔记week2](https://blog.yxwang.me/2018/07/robotics-slam-week2/)
- [公开课的笔记week3](https://blog.yxwang.me/2018/08/robotics-slam-week3/)
- [公开课的笔记week4](https://blog.yxwang.me/2018/08/robotics-slam-week4/)
