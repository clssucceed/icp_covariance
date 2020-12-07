# icp_covariance

## 代码架构
* app: 主程序
* 数据生成(DataGenerator): 单例
* 可视化(Visualization): 单例
* utils: 工具函数

## 实验结果
* icp_cov_from_monte_carlo如图所示 ![icp_cov_from_monte_carlo](./image/icp_cov_from_monte_carlo.png)
### icp_cov受哪些因素影响
* target size越大，icp_cov越小
* angle resolution变大或者变小，icp_cov都有可能变大
* target在ego系下的距离: 物体较远时，点数较少，受初始位姿影响较大，为了防止发散，应该增加位姿正则或者correspondence正则(相邻两帧点云中相同激光头相同激光时序的点认为是correspondence)
* target在ego下的姿态(p+q -> I-shape/L-shape)
* 多线激光雷达vs单线激光雷达
* velodyne vs livox

## TODO
* 使用相同的生成数据X_{t-1}, X_t，T_init生成图像数据，并进行of_cov的计算
