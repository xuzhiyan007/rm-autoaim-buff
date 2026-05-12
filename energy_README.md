// 发布当前模式（0=SMALL_ENERGY, 1=BIG_ENERGY, 2=ARMOR）

只开能量机关
ros2 launch fire energy.launch.py

自瞄、能量机关同时开监控脚本
bash /home/jlcv/infantry/src/energy_fire/launch/full_system.sh

自瞄、能量机关同时开launch指令
ros2 launch fire full_system.launch.py

采集打包与解压缩指令
tar -czf 20260510_1048_big_red_01.tar 20260510_1048_big_red_01/

zip -r project.zip my_project/

ros2 run camera_calibration cameracalibrator \
  --size 11x8 --square 0.035 \
  --no-service-check \
  --ros-args --remap image:=/image_raw


