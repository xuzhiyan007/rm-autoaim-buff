std_msgs/Header header
bool tracking
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw                  
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
uint8 task_mode                 # 判断是自瞄还是小陀螺模式   0：Armor  1：Outpost  2:Balance_infantry


# last version by yjq
# std_msgs/Header header
# uint8 id                        # 目标装甲板id
# uint8 task_mode                 # 判断是自瞄还是小陀螺模式   0：自瞄  1：小陀螺
# bool tracking                   # 是否在跟踪状态
# bool suggest_fire               # 是否开火
# geometry_msgs/Point position    # 目标装甲板位置  需要确认一下具体坐标系关系
# geometry_msgs/Vector3 velocity  # 目标装甲板速度 
# float32 yaw                     # yaw值   考虑了飞行延时后的yaw值
# float32 pitch                   # pitch值 考虑了飞行延时后的pitch值 