std_msgs/Header header

uint8 mode                   # 当前控制模式，见common功能包常量定义
float32 speed_mps            # 车速，单位m/s
float32 acceleration_mps2    # 加速度，单位m/s^2

uint8 gear                   # 档位，见common功能包通用常量定义
uint8 park                   # 驻车状态，见common功能包通用常量定义

uint8 left_turn_light        # 转向灯，见common功能包通用常量定义
uint8 right_turn_light       # 转向灯，见common功能包通用常量定义
uint8 double_flash_light     # 双闪灯，见common功能包通用常量定义

uint8 high_beam_light         # 远光灯，见common功能包通用常量定义
uint8 low_beam_light          # 近光灯-日行灯，见common功能包通用常量定义
bool back_lamp_st             # 倒车灯开关状态
bool brk_lamp_st              # 制动灯开关状态
bool width_lamp_st            # 示廓灯开关状态
bool strip_lamp_st            # 迎宾灯开关状态
WheelsSpeedInfo wheels_speed  # 轮速

float32 front_wheel_angle_rad  # 前轮转角，单位弧度
float32 battery_soc            # 电量
string vin_number              # 车辆vin码
float64 accumulated_mileage    # 累计里程
float64 trip_mileage           # trip累计里程
float64 motor_speed            # 电机转速
TirePressure tire_pressure     # 实时胎压
WaterTankStatus water_tank_status        # 水箱状态
GarbageStatus garbage_status             # 垃圾状态
BatteryStatus battery_status             # 动力电池状态
BatteryStatus lv_battery_status          # 小电池状态

string software_version  # 软件版本
string hardware_version  # 硬件版本

uint8 error_level    # 故障等级
uint8 error_code     # 错误码列表