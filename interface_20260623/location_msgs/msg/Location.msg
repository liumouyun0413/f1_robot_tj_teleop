std_msgs/Header header

uint8 coordinate_system  # 坐标系，见common功能包常量定义

float64 position_x              # in meters
float64 position_y              # in meters
float64 position_z              # in meters
float64[9] position_covariance  # 协方差

float32 roll                       # in radians
float32 pitch                      # in radians
float32 yaw                        # in radians -pi~pi
float32[9] orientation_covariance  # 协方差