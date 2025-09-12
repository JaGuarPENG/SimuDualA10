# SimuDualA10: Simulation of dualarm A10 robot in ros rviz

## 说明：
本项目用于在虚拟机环境（Ubuntu 22.04.4 / VMware 17 PRO）中可视化仿真佳安A10双臂机器人。
依赖的库包括 ROS2，Rviz，aris。

ROS2 版本为 Humble，推荐使用鱼香ros安装，安装过程会一同安装 python 与 vscode：
https://fishros.com/d2lros2

aris安装过程请参考：
https://github.com/py0330/Aris

代码中默认 aris 安装版本为 aris-2.3.10.240826，如有不同请自行修改 cmakelist

Rviz 一般已跟随 ROS 安装，但仍需使用命令行安装 gui 调试窗口：
```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

## 使用：
本项目文件为 ROS2 下的工作空间，建议使用 vscode 进行编译及后续操作。
在 vscode 中推荐安装的插件包括：
1. C++ 及其扩展相关
2. Cmake 相关工具
3. Python 相关支持
4. ROS 微软官方依赖库（必须）

安装完依赖后在工作空间目录下执行编译指令：
```bash
colcon build
```

之后执行安装指令：
```bash
source install/setup.bash
```

建议打开两个终端窗口以便同时操作仿真程序以及 Rviz。

打开仿真程序指令如下：
```bash
ros2 run a10_dual a10_dual
```

打开 Rviz 指令如下：
```bash
ros2 launch dualarm display.launch.py
```

值得注意的是，打开新的终端时需要执行上述安装指令，同时在 Rviz 打开后请关闭 joint-state-publisher-gui，否则会影响使用指令仿真。

当上述程序成功打开后，终端会显示 Control Server Init，此时机器人控制器已成功启动，可以输入编写好的指令操控仿真。

## 注意事项：
若执行程序时报错找不到 libaris.so 动态库，使用下述指令写入 aris 安装路径中的 .so 文件的路径：
```bash
sudo gedit /etc/ld.so.conf
```
再执行以下指令：
```bash
sudo ldconfig
```

## 初步教程：
本项目代码主要包括 src/dualarm 以及 src/a10_dual 两处。其中，dualarm 下为 rviz 可视化所需配置资源，无需修改。a10_dual 下存储了相关的示例指令可供参考，相关代码内容简介如下：
1. aris_node 将 aris 包装为 ros 下的一个节点，并以此向 ros 中发送信息。
2. basic_cmd 为基础指令及教程，包含正逆运动学解算，机械臂电机控制，梯形曲线末端控制简单实现等。
3. force_cmd 为基础力控指令，包括实现的导纳控制器以及阻抗控制器。由于仿真中无法引入接触力，需要用户自定义接触力模拟。
4. gravcomp 为实现的机械臂末端重力补偿算法类
5. t_curve 为梯形曲线类

basic_cmd 类包含有详细的 aris 指令实现细节注释，请仔细参考。force_cmd 中的控制器指令均已在实机上部署测试过，可以作为力控实现的简单思路参考。
