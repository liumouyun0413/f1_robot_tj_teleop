# 转向灯（下发整车控制用）
uint8 TURN_LIGHT_OFF = 1           # 关闭
uint8 TURN_LIGHT_LEFT = 2          # 左转向灯
uint8 TURN_LIGHT_RIGHT = 3         # 右转向灯
uint8 TURN_LIGHT_DOUBLE_FLASH = 4  # 双闪灯

# 远近光灯（下发整车控制用）
uint8 BEAM_LIGHT_OFF = 1   # 关闭
uint8 BEAM_LIGHT_LOW = 2   # 近光灯
uint8 BEAM_LIGHT_HIGH = 3  # 远光灯

std_msgs/Header header

bool ad_enable  # 申请进入自驾模式

float32 target_front_wheel_angle_rad         # 前轮转角，单位弧度
float32 target_front_wheel_angle_rate_rad_s  # 前轮转角速度，单位弧度/s

float32 target_speed_mps   # 目标速度，单位m/s
float32 target_accel_mps2  # 目标加速度，单位m/s^2

uint8 target_gear        # 档位，见common功能包常量定义
uint8 target_turn_light  # 转向灯
uint8 target_beam_light  # 远近光灯

float32 position_err     # 位置控制误差，单位m
float32 heading_err      # 姿态控制误差，单位deg
float32 speed_err        # 速度控制误差，单位km/h

float32 execute_lat_err  # 底盘横向执行误差，单位deg
float32 execute_long_err # 底盘纵向执行误差，单位km/h