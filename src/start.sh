#!/bin/bash



# 确保DISPLAY变量正确设置
export DISPLAY=:0
source /home/dji/.bashrc 
echo 'dji' | sudo -S chmod +x /home/dji/infantry/src/start.sh

echo 'dji' | sudo -S chmod 777 /dev/ttyUSB0

export MVCAM_SDK_PATH=/opt/MVS

export MVCAM_COMMON_RUNENV=/opt/MVS/lib

export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol

export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH

##echo 'dji' | sudo -S ifconfig lo multicast

export ROS_HOME="/home/dji/infantry"
export ROS_DOMAIN_ID=24
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export ROS_LOCALHOST_ONLY=0

source /opt/ros/humble/setup.bash

source /home/dji/.bashrc

cd /home/dji/infantry
source ./install/setup.bash

# 设置需要启动和监视的ros命令
declare -a commands=("echo 'dji' | sudo -S chmod 777 /dev/ttyUSB0;source /home/dji/.bashrc ;ros2 launch vision_bringup vision_bringup.launch.py" "ros2 run armor_processor armor_processor_node")

# 定义函数来启动ros命令
start_commands() {
  for cmd in "${commands[@]}"
  do
	gnome-terminal -- bash -c "source /home/dji/.bashrc; source /opt/intel/openvino_2024/setupvars.sh; source /opt/ros/humble/setup.bash;echo 'dji' | sudo -S chmod 777 /dev/ttyUSB0; cd $(pwd); source install/setup.bash; $cmd; exec bash;"
  done
}

# 定义函数来监视ros进程是否在运行
watch_commands() {
  while true
  do
    for cmd in "${commands[@]}"
    do
      echo $cmd
      process_name=$(echo $cmd | awk '{print $1}') # 提取进程名
      if ! pgrep -f "$cmd" > /dev/null
      then
        echo "$cmd 未在运行, 重新启动"
	gnome-terminal -- bash -c "source /home/dji/.bashrc; source /opt/intel/openvino_2024/setupvars.sh; source /opt/ros/humble/setup.bash; cd $(pwd); source install/setup.bash; $cmd; exec bash;"
      fi
    done
    sleep 5 # 设置睡眠时间，以便在下一次检查之前给进程足够的时间运行
  done
}

# 启动ros命令和监视进程
start_commands
watch_commands

