##  一、本项目依赖安装(统一用ubuntu20.04.06版本)

#### 1.安装大恒相机和海康相机SDK

1. 请注意区分x86框架和arm框架

2. 下载地址：[大恒相机](https://www.daheng-imaging.com/downloads/)      [海康相机](https://www.hikrobotics.com/cn) ：选择liunx版本   

   安装：MVS-2.1.2_x86_64_20221208.deb  

   一定得阅读说明文档：

   使用USB相机建议设置usb缓存（**set_usbfs_memory_size.sh**）

   设置udev规则，使之能在非管理员权限下使用u3相机（**set_usb_priority.sh**）

#### 2.安装yaml

1. github进行下载：[yaml地址](https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.7.0)

 需要修改cmakelist文件，看到报错百度即可

#### 3.安装ROS2及相关依赖

1. 安装ROS2 galactic（对应ubuntu20.04)   明年可以换humble版本使用docker即可（或者直接换成ubuntu22.04)

```
wget http://fishros.com/install -O fishros && . fishros 
```

2. 安装ROS2依赖

```
1. sudo apt-get install ros-galactic-xacro ros-galactic-joint-state-publisher ros-galactic-joint-state-publisher-gui ros-galactic-serial-driver ros-galactic-imu-tools ros-galactic-camera-info-manager ros-galactic-camera-calibration ros-galactic-rosbridge-server 
2. (可选) sudo apt-get install ros-galactic-rqt-tf-tree
```

#### 4.安装SSH

```
1.sudo apt-get install openssh-server
2.sudo systemctl start sshd
3.sudo systemctl status sshd 查看是否正常运行
```

4. 设置好主从机之间的IP地址，即可使用网口进行远程调试

#### 5.哨兵依赖*

1. 安装Serial包*：github进行下载[serial地址](https://github.com/wjwwood/serial)
2. Cartographer+Cartographer_ros包*    galactic版本
3. octomap相关

```
sudo apt-get install ros-galactic-octomap-ros 
sudo apt-get install ros-galactic-octomap-server
sudo apt-get install ros-galactic-octomap-rviz-plugins
```

4. ros2bag

```
sudo apt-get install ros-galactic-rosbag2-storage-default-plugins

use:
ros2 bag record <topic_name>
ros2 bag record <topic_name>  <topic_name> -o <bag_file_name>
ros2 bag info <bag_file_name>
ros2 bag play <bag_file_name>
```



#### 6.Cartographer+Cartographer_ros包*

#### 7.

## 二、推荐软件安装

#### 1.vscode

强烈建议官网下载，snap下载版本为阉割版

#### 2.Typora

ones上有安装包（官网无法下载该版本）

#### 3.foxglove调试

1. 主机[安装foxglove](https://foxglove.dev/download)（在从机调试也可，主机比较方便）

2. 根据foxglove教程学习如何使用

#### 4.安装输入法（google-pinyin)

#### 5.cutecom串口调试

```
sudo apt-get install cutecom 
```


#### 6.terminator终端

```
sudo apt-get install terminator
```

记得解决ctrl+shift+e冲突问题

#### 7.VLC

​	一款多媒体播放器，方便录制调试视频后进行观看。若安装系统的时候选择最小安装，则不会预装媒体软件，因此需要自己安装。

#### 8.qv4l2

​	linux下相机驱动的图形界面，在Ubuntu软件商店可以找到，方便调节普通USB相机的参数。

```
sudo apt-get install libv4l-dev
```

#### 9.其他

比如zip、rar等压缩解压包；locate包；

```
sudo apt-get update
sudo apt-get install build-essential //将安装gcc/g++/gdb/make 等基本编程工具
sudo apt-get install unzip unrar 
sudo apt-get install exfat-utils
sudo apt-get install git 
sudo apt-get install libeigen3-dev  
sudo apt-get install net-tools 
```

## 三、代码使用

1. galaxy_camera和hik_camera为相机包，具体看使用什么相机进行测试。如要使用视频测试，需另外单独运行roborts_test中视频播放节点

```
ros2 run roborts_test video_test 
```

2. auto_aim为自瞄代码；rm_energy为能量机关代码。两者均含有三个功能包，分别为识别节点(detector)，处理节点(processor)，消息功能包(interface，用于定义发布的数据)。其中处理节点均使用正常的ros2中的rclcpp::Node进行编写；识别节点由于需要根据比赛内容进行切换，采用ros2中的生命节点(rclcpp_lifecycle::LifecycleNode)进行管理，可以实现节点的开关功能。具体功能基本上保持不变，请自行阅读代码。

3. 以上自瞄代码和能量机关代码均使用rm_lifecycle_manager包进行管理，实现自瞄状态和能量机关模式的切换。如需设置初始模式，请修改lifecycle_manager.cpp中的state_值为enum{ARMOR_STATE = 0, ENERGY_STATE = 1}即可。

4. jlcv_serial_driver包为串口通讯包，实现和电控进行通信。具体结构体定义请看packet.hpp。主程序为jlcv_serial_driver.cpp。

5. 以上除rm_lifecycle_manager包外，所有功能包参数均可在vision_bringup功能包中进行设置。其中config文件夹中含有三个参数文件：

   1. camera_info.yaml定义了相机标定参数（内参）
   2. launch_params.yaml定义了相机种类、世界坐标系到相机坐标系之间变换关系。
   3. node_params.yaml定义了所有节点有必要进行调节的参数。经测试，目前该文件中所有参数设置均有效***！！！！更改后保存即可，无需再次进行编译，！！！！***具体参数请自行阅读此文件。  //TODO 重要参数补充说明文档

   其中launch文件夹为程序运行launch启动文件

   1. common.py为一些初始设置节点，唯一调节参数为declare_use_serial_cmd，含义：True为使用串口（即进行下位机通信），False为不使用串口，使用roborts_test功能包模拟串口数据。
   2. 其他两个launch文件目前效果一样，随意使用。

6. start.sh文件为自启动脚本文件，里面内容需要根据具体电脑进行修改。
7. 具体进行测试时，建议使用vision_bringup功能包

```
ros2 launch vision_bringup vision_bringup.launch.py 
```



## 四、学习ros2

#### 1.日志调试功能

```
ros2 run package-name node-name --ros-args --log-level debug
```

#### 2.检查依赖

It’s good practice to run `rosdep` in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro galactic -y
```

#### 3.显示tf2-tree

```
ros2 run tf2_tools view_frames
```

#### 4.杀死gazebo进程

```
killall gzserver
killall gzclient
```

#### 5.键盘映射

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 6.nav2地图相关

```
1.保存地图
  ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
 
2.打开保存的地图
  ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=map/fishbot_map.yaml
```

#### 7.静态转换 Publisher
```
发布静态TF变换
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link base_laser 

获取传感器数据对应的frame_id
ros2 topic echo /scan | grep frame_id

输出两个TF坐标系之间的变换关系
ros2 run tf2_ros tf2_echo base_link base_laser
输出结果如下：
At time 0.0
- Translation: [0.100, 0.000, 0.200]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]

```

#### 8.相机标定
```
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.035 image:=/image_raw camera:=/my_camera
```
数据保存到 "/tmp/calibrationdata.tar.gz"   


```
// 提高代码效率
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 可直接在cmakelist中设置编译为release版本

//监视所有tf关系
ros2 run tf2_ros tf2_monitor
```

#### 9.foxglove软件配置

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### 10.ROS2 debug过程

```
1.在cmakelist文件中添加 add_compile_options(-g) 表示使用gdb进行调试
2.ros2 run --prefix 'gdb -ex run --args' <package name> <node name> 
eg:  ros2 run --prefix 'gdb -ex run --args' energy_processor energy_processor_node

eg: 如果要使用vscode进行gdb单步调试，请自行查阅教程
```
#### 11.Ubuntu录屏时间

```
gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 300
```



colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release



sudo ifconfig lo multicast

