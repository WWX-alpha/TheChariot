# 更改日志
## 3.24
完成step()和reset()的编写
## 3.25
加入了PPO模块
## 3.29
改动step()结构，优化数据可视化
## 4.1
引入对于仰角的观测，更新reset()策略
## 4.2
* 为yolo检测添加了四阶EKF
* 为俯仰动作添加了二阶EKF
* 将高低机电压控制改为速度控制
## 4.8
* 去除对于实际转速的观测
## 4.9
* policyV2:更改了状态空间和动作空间
## 4.10
* 将摄像头与高低机解耦，使状态观测更加稳定
## 4.11
* 训练使得转速和仰角均能接近最大值
* policyV3:更改了动作空间
* 添加代码注释
* 尝试使用SAC训练
## 4.15
* 加入运行代价
* 加入边界碰撞惩罚
* 自动定时触发扳机
* 改回V2

## 4.17
* SolidWorks导出URDF模型
* URDF导入webots并创建proto
## 4.18
* 验证机器人joint设置正确性
* webots摩擦行为研究，建立底盘控制器
## 4.19
* 研究supervisor编写方法
## 4.20
* 试图用supervisor实现抛体运动

## 4.23
* 测试飞盘弹道
## 4.26
* 测试飞盘弹道并拟合数据
## 4.28
* 将弹道解算下放到香橙派进行，不再通过tcp连接电脑
## 4.29
* 调整底盘代码，增加了速度缩放功能，修正最大速度
## 4.30
* 云台yaw轴参数调整

# 动作空间与状态空间
## V3
* n_state_space = 4  # [横宽 顶部 仰角 转速]
* n_action_space = 3  # [升降信号 转速信号 射击信号]
## V2
* n_state_space = 4  # [横宽 顶部 仰角 转速]
* n_action_space = 2  # [升降信号 转速信号]
## V1
* n_state_space = 3  # [横宽 顶部 仰角]
* n_action_space = 3  # [抬升 下降 转速]
## yolo格式
* top, left, right, bottom

# 数学模型
## 底盘运动学
$$
  \begin{bmatrix}
    v_{1}\\
    v_{2}\\
    v_{3}\\
    v_{4}\\
  \end{bmatrix}=
  \begin{bmatrix}
    -1/sin\theta&1/cos\theta&L\\
    -1/sin\theta&-1/cos\theta&L\\
    1/sin\theta&-1/cos\theta&L\\
    1/sin\theta&1/cos\theta&L\\
  \end{bmatrix}
  \begin{bmatrix}
    \dot{x}\\
    \dot{y}\\
    \dot{\theta}\\
  \end{bmatrix}
$$