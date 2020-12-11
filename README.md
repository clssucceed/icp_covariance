# icp_covariance

## 代码架构
* app: 主程序
* 数据生成(DataGenerator): 单例
* 可视化(Visualization): 单例
* utils: 工具函数

## 实验结果
* 静止目标车锚点坐标(10, 5, 0)如图所示
* ![icp_cov_from_monte_carlo](./image/cov_for_static_anchor_point_10-5.png)
* 运动目标车锚点坐标(10, 5, 0)如图所示 
* ![icp_cov_from_monte_carlo](./image/cov_for_static_anchor_point_10-5.png)
* 静止目标车锚点坐标(10, 0.001, 0)如图所示 
* ![icp_cov_from_monte_carlo](./image/cov_for_static_anchor_point_10-0_001.png)
* 运动目标车锚点坐标(10, 0.001, 0)如图所示 
* ![icp_cov_from_monte_carlo](./image/cov_for_static_anchor_point_10-0_001.png)
### icp_cov受哪些因素影响
* target size越大，icp_cov越小
* angle resolution变大或者变小，icp_cov都有可能变大
* target在ego系下的距离: 物体较远时，点数较少，受初始位姿影响较大，为了防止发散，应该增加位姿正则或者correspondence正则(相邻两帧点云中相同激光头相同激光时序的点认为是correspondence)
* target在ego下的姿态(p+q -> I-shape/L-shape)
* 多线激光雷达vs单线激光雷达
* velodyne vs livox

## TODO
* 使用相同的生成数据X_{t-1}, X_t，T_init生成图像数据，并进行of_cov的计算
* 每一次的仿真结果都保存下来(原始数据，中间结果，计算结果，可视化图像)
* 比较直接计算绝对速度 vs 先计算相对速度再推算绝对速度两种速度计算方案对于点云噪声的响应大小
* 引入参数文件（每次加入参数需要改动的地方尽量少）
* 实现StateManager类管理自车和目标车位姿单例，方便在全局任何需要的地方访问到，比如用于计算速度
