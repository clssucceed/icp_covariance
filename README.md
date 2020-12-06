# icp_covariance

## 代码架构
* app: 主程序
* 数据生成(DataGenerator): 单例
* 可视化(Visualization): 单例
* utils: 工具函数

## 实验结果
* icp_cov_from_monte_carlo如图所示 ![icp_cov_from_monte_carlo](./image/icp_cov_from_monte_carlo.png)
* target size越大，icp_cov越小
* angle resolution变大或者变小，icp_cov都有可能变大