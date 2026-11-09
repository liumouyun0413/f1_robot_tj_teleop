std_msgs/Header header

string planner      # "road_planner" or "openspace_planner"

bool ad_enable            # 申请进入自驾模式
uint8 coordinate_system   # 0: NA, 1: VCS, 2: UTM
TrajectoryPoint[] points  # 轨迹序列

uint8 target_gear        # 档位，见common功能包常量定义
uint8 target_turn_light  # 转向灯，见common功能包常量定义
uint8 target_beam_light  # 远近光灯，见common功能包常量定义