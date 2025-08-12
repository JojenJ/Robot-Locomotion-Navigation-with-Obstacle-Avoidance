# Robot-Locomotion-Navigation-with-Obstacle-Avoidance
## Unitree GO2 project
code is in construction

here are a few tests:

![Demo Animation](gifs/test_inoffice.gif)

![Demo Animation](gifs/rviz.gif)




## Open duck project
This project integrates **Open Duck** bipedal locomotion with **deep reinforcement learning (DRL)**-based navigation and obstacle avoidance.  
It combines multiple components:  
- **Open Duck locomotion control** (URDF + Mujoco/Isaac Gym simulation + real-world deployment)  
- **DRL-Robot-Navigation** (SAC/TD3 reinforcement learning)  
- **Pocket-based point cloud & pose transmission**  
- **ROS-based real-time inference & control**  

The goal is to achieve **autonomous navigation** in dynamic environments using **LiDAR + pose estimation**, with sim-to-real transfer for deployment.

---

## 🔹 Features
- **Custom robot URDF** with ODIN1 module for realistic mass distribution.
- **Reinforcement learning navigation** (SAC/TD3) trained in Mujoco/Isaac Gym.
- **ROS-based real-time control** for linear & angular velocity commands.
- **PointCloud2 & pose data transmission** from Pocket devices to RL module.
- **Sim-to-real scaling** for sensor readings.
- **Modular control pipeline** (Socket + ROS Topics).

---

## 🔹 Project Structure(waiting for update)
├── open_duck/ # Modified Open Duck locomotion
├── DRL-robot-navigation/ # Reinforcement learning navigation module
├── pocket_communication/ # Point cloud & pose transmission (ROS)
└── docs/ # Diagrams, setup notes    

![Demo Animation](gifs/duck_test.gif)


**日常使用**
实机连接：
ifconfig找无线网口名
sudo arp-scan -I wlp0s20f3 --localnet 
ssh -Y lxkj@192.168.31.196

重启：
sudo reboot

先连手柄，否则会报设备错误
bluetooth
scan on
再运行程序（指onnx模型的位置）


**改动**
添加了控制指令的接收程序，在启动v2_mujoco那个程序后会waiting PC端的server_client去发布控制
控制信号是 *linear speed 和 angular speed*

暂时disable了关于扬声器的串口，alsa无法打开这个设备，会报错


**训练**
对于原本open_duck的urdf，添加了ODIN1的模型，重新训练可得到更好的效果（考虑了ODIN1部署上去导致重心的改变），训练在mujoco和issac_gym上均可进行

——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

DRL-robot-navigation：（https://github.com/reiniscimurs/DRL-robot-navigation.git）

**训练**
使用SAC模型收敛速度要显著快于TD3，但是TD3算法较先进
奖励函数上，主要是goal +100 ，collision -100  ， 然后鼓励线速度，惩罚角速度，以及靠近障碍物时的惩罚
奖励函数设置越复杂，可能收敛会越慢，可以适当增加一些时间或者路程的惩罚，但是没必要

存在很大缺点：gazebo不支持多线程训练，没有多个虚拟环境训练会导致训练效率极低


**推理**
去除了gazebo框架，以及一些用于方便演示的reset操作，创建了接受rviz拉点的订阅者，实现实时接受拉点规划


**部署**
该模型的RL，主要输入了（环境的state和robot state），环境state包含了20个角度的雷达最近距离数据，robot则是与目标的距离、偏角、线速度和角速度
所以雷达方面不需要考虑很多硬件层、数据格式的问题，只要在/scan话题上能收到 pointcloud2 类型的数据即可

sim2real：数据尺度不同，在gazebo训练的数据尺度需要加上一个scale去match现实的，比如雷达信号的distance乘以一个系数

模型输入的控制信号不能很好得适配linear speed 和 angular speed, 训练时输入的信号应该是随机的linear_vel_x、linear_vel_y、angular_vel,
其中环境获取（mujoco_playground）：    
def get_global_linvel(self, data: mjx.Data) -> jax.Array:
        """Return the linear velocity of the robot in the world frame."""
        return mjx_env.get_sensor_data(
            self.mj_model, data, constants.GLOBAL_LINVEL_SENSOR
        )
奖励计算：
def reward_tracking_lin_vel(
    commands: jax.Array,
    local_vel: jax.Array,
    tracking_sigma: float,
) -> jax.Array:
    # lin_vel_error = jp.sum(jp.square(commands[:2] - local_vel[:2]))
    # return jp.nan_to_num(jp.exp(-lin_vel_error / self._config.reward_config.tracking_sigma))
    y_tol = 0.1
    error_x = jp.square(commands[0] - local_vel[0])
    error_y = jp.clip(jp.abs(local_vel[1] - commands[1]) - y_tol, 0.0, None)
    lin_vel_error = error_x + jp.square(error_y)
    return jp.nan_to_num(jp.exp(-lin_vel_error / tracking_sigma))

所以可知这个x，y的速度大概率是基于环境的速度，而我目前设置的linear_x和angular是车辆运动学的描述方式（差速小车）
接下来应该在这进行修改

————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
Pocket通信：

**PointCloud_Transmission改ROS**：
将x,y,z 加上frame_id 时间戳等等，变为pointcloud2形式（对接RL的项目）
以及位姿，发送出来

**Pocket+导航启动**：
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_PORT_SIM=11311
export GAZEBO_RESOURCE_PATH=~/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch

cd ~/DRL-robot-navigation/catkin_ws
source devel_isolated/setup.bash


source ~/DRL-robot-navigation/catkin_ws/devel/setup.bash 

conda activate Omni_env
rosrun radar_pose lidar_receiver_node


cd ~/DRL-robot-navigation/TD3
python3 test_velodyne_td3.py


┌─────────────┐    Socket     ┌─────────────┐    ROS Topic    ┌─────────────┐
│   Pocket    │──────────────▶│    电脑     │────────────────▶│ RL避障算法    │
│    设备     │   点云+位姿    │             │    数据发布       │             │
└─────────────┘               └─────────────┘                 └─────────────┘
                                                                      │
                                                                      │ 推理
                                                                      ▼
┌─────────────┐               ┌─────────────┐    Socket      ┌─────────────┐
│    电机      │◀──────────────│   树莓派    │◀───────────────│  控制信息      │
│             │   运动信号     │ 本地模型推理  │   控制指令      │              │
└─────────────┘               └─────────────┘                └─────────────┘




TO DO:
解决控制信号转换问题，用新训好的模型检验效果

在效果完全验证之后，可以考虑部署ODIN1




