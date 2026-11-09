# 通用常量定义
uint8 INVALID = 0  # 无效
uint8 DEFAULT = 0  # 默认
uint8 OFF = 1      # 关闭
uint8 ON = 2       # 打开
uint8 ERROR = 3    # 故障，特殊情况使用

# 底盘控制模式
uint8 STANDBY_MODE = 1            # 待机
uint8 MANUAL_DRIVING = 2          # 人工驾驶，机器人没有这个状态
uint8 REMOTE_DRIVING = 3          # 远程驾驶
uint8 AUTOMATIC_DRIVING = 4       # 自动驾驶
uint8 NFC_DRIVING = 5             # 手柄/WIFI控制
uint8 EMERGENCY_STOP = 6          # 紧急停车（如碰撞胶条触发），暂时退出自驾

# 故障等级
uint8 ERROR_LEVEL1 = 1  # 故障等级1
uint8 ERROR_LEVEL2 = 2  # 故障等级2
uint8 ERROR_LEVEL3 = 3  # 故障等级3
uint8 ERROR_LEVEL4 = 4  # 故障等级4

# 档位
uint8 GEAR_N = 1  # 空档
uint8 GEAR_D = 2  # 前进档
uint8 GEAR_R = 3  # 倒车档
uint8 GEAR_P = 4  # 驻车档

# 目标物类别
uint8 UNKNOWN = 0               # 未知类别
uint8 PEDESTRIAN = 1            # 行人
uint8 BICYCLE = 2               # 非机动车
uint8 CAR = 3                   # 小车
uint8 TRUCK = 4                 # 大车
uint8 SPV = 5                   # 特种车辆，如救护车、警车、消防车灯
uint8 REJECTED_JOB_TRASH = 6    # 阻碍作业类垃圾，但不影响通行（非作业状态），如地面细水管
uint8 BIG_WASTE = 7             # 大垃圾（尺寸需要定义），需要打开垃圾抽吸口处挡板
uint8 LITTLE_WASTE = 8          # 小垃圾

# 坐标系
uint8 VCS = 1    # 车体坐标系，x向前，y向左，z向上
uint8 UTM = 2    # 大地坐标系，x向东，y向北
uint8 WGS84 = 3  # 84经纬度坐标系

# 硬件系统或者模块
uint8 HM_CHASSIS = 1  # 线控底盘（整车）
uint8 HM_ACU = 2      # 自动驾驶域控
uint8 HM_TBOX = 3     # 联网控制器

# 软件模块名称
uint8 PROCESS_RECORDER = 1            # 数据记录
uint8 PROCESS_MODEM = 2               # 通信模块
uint8 PROCESS_CHASSIS = 3             # 底盘抽象模块
uint8 PROCESS_OTA = 4                 # OTA
uint8 PROCESS_DIAG = 5                # 诊断模块
uint8 PROCESS_SCHED = 6               # 调度服务
uint8 PROCESS_ROAD_PLANNING = 7       # 常规规划
uint8 PROCESS_OPENSPACE_PLANNING = 8  # OPENSPACE规划
uint8 PROCESS_CONTROL = 9             # 自驾控制
uint8 PROCESS_LOCATION = 10           # 自驾定位
uint8 PROCESS_PERCEPTION = 11         # 自驾感知
uint8 PROCESS_PREDICTION = 12         # 自驾预测

uint8 HW_ABST_BSDL = 13   # 补盲激光雷达抽象（含IMU）
uint8 HW_ABST_LIDAR = 14  # 主激光雷达抽象

#调度任务类型
uint8 ENABLE_AUTODRIVING = 1             # 进入自动驾驶
uint8 DISABLE_AUTODRIVING = 2            # 退出自动驾驶
uint8 SET_TARGET_STATION = 3             # 设置目标站点
uint8 CANCLE_SCHEDULE = 4                # 取消当前调度任务
uint8 ENABLE_EMERGENCY_BRAKE = 5         # 紧急停车
uint8 DISABLE_EMERGENCY_BRAKE = 6        # 取消紧急停车
uint8 SET_LIMIT_SPEED = 7                # 设置限速
uint8 ROADPLANNING_DISPATCH_QUERY = 8    # 状态查询
uint8  AUTO_WATER_COMPLETE=9              #加水完成
uint8  AUTO_CHARGING_COMPLETE=10          #充电完成

#作业类型
uint8 LEFT_EDGE_CLEANING = 1         # 左贴边
uint8 RIGHT_EDGE_CLEANING = 2        # 右贴边
uint8 CLOSE_EDGE_CLEANING = 3        # 不贴边

#站点类型
uint8 CHARGING_STATION = 1           # 充电站
uint8 WATER_STATION = 2              # 加水站
uint8 GARBAGE_STATION = 3            # 垃圾站
uint8 PARKING_STATION = 4            # 停车站点
uint8 NORMAL_STATION = 5             # 常规站点
uint8 STARTT_WORKING_POINT=6         #作业起点
uint8 COMPLETE_WORKING_POINT=7       #作业终点

#语音播报                            
uint8 VOICE_LEFT_TURN = 1            # 车辆左传
uint8 VOICE_RIGHT_TURN = 2           # 车辆右转
uint8 VOICE_REVERSE = 3              # 倒车
uint8 VOICE_STAART_MOVE = 4          # 车辆起步
uint8 VOICE_WORING = 5               # 车辆作业
uint8 ENTER_AUTO_DRIVE = 6           # 进入自动驾驶
uint8 EXIT_AUTO_DRIVE = 7            # 推出自动驾驶

# openspace调度类型
uint8 OPENSPACE_DISPATCH_QUERY = 0          # 状态查询
uint8 OPENSPACE_DISPATCH_PARK_VERTICAL = 1  # 垂直/斜列车位泊入
uint8 OPENSPACE_DISPATCH_PARK_PARALLEL = 2  # 水平泊入
uint8 OPENSPACE_DISPATCH_LEFT_PARK_OUT = 3  # 左侧泊出
uint8 OPENSPACE_DISPATCH_RIGHT_PARK_OUT = 4 # 右侧泊出
uint8 OPENSPACE_DISPATCH_CANCEL = 5         # 任务取消
uint8 OPENSPACE_DISPATCH_STOP = 6           # 紧急停车
uint8 OPENSPACE_DISPATCH_CHARGE = 7         # 充电
uint8 OPENSPACE_DISPATCH_DUMPAGE = 8        # 倾倒垃圾
uint8 OPENSPACE_DISPATCH_ADD_WATER = 9      # 加水
uint8 OPENSPACE_DISPATCH_OTHER = 10         # 普通站点

# openspace状态
uint8 OPENSPACE_STATUS_IDLE = 0           # 空闲
uint8 OPENSPACE_STATUS_PLANNING_FAIL = 1  # 规划失败
uint8 OPENSPACE_STATUS_RUNNING = 2        # 规划成功，且正在执行中
uint8 OPENSPACE_STATUS_BLOCKING = 3       # 任务执行受阻，申请重规划
uint8 OPENSPACE_STATUS_COMPLETE = 4       # 任务完成

#roadplanning状态
uint8 ROADPLANNING_STATUS_IDLE = 0           # 空闲
uint8 ROADPLANNING_STATUS_PLANNING_FAIL = 1  # 规划失败
uint8 ROADPLANNING_STATUS_RUNNING = 2        # 规划成功，且正在执行中
uint8 ROADPLANNING_STATUS_COMPLETE = 3       # 任务完成


#控制指令类型
uint8  VEHICLE_CONTROL=1                    #车辆控制指令
uint8  SET_STATION_POINT=2                  #设置目标站点指令
uint8  JOB_CONTROL=3                         #上装控制指令


#行驶类型
uint8  STATUS_IDLE =1                          #空闲   
uint8 VEHICLE_DRIVE_DRIVING =2                #行驶中
uint8 VEHICLE_DRIVE_PARK = 3                   #泊车中
uint8 VEHICLE_DRIVE_WORK = 4                   #作业中
                     


