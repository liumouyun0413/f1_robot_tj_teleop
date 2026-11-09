#pragma once

/*  driver interface */

/* -------- 数据上传接口 ------- */
/* ------   Topic  -----*/
//激光雷达IMU数据上传
// 类型: sensor_msgs/Imu
#define DRIVER_LIDAR1_IMU_DATA "driver/lidar/lidar1/imu/Data"
#define DRIVER_LIDAR2_IMU_DATA "driver/lidar/lidar1/imu/Data"

//激光雷达点云数据上传
// 类型: sensor_msgs/PointCloud2
#define DRIVER_LIDAR2_POINT_DATA "driver/lidar/lidar2/point_cloud/Data"
#define DRIVER_LIDAR2_POINT_DATA "driver/lidar/lidar2/point_cloud/Data"

//融合后激光雷达点云数据上传
// 类型: sensor_msgs/PointCloud2
#define DRIVER_LIDAR_POINT_DATA "driver/lidar/point_cloud/Data"


//相机点云数据上传
// 类型: sensor_msgs/PointCloud2
#define DRIVER_CAMERA1_POINT_DATA "driver/camera/camera1/point_cloud/Data"


//融合后相机点云数据上传
// 类型: sensor_msgs/PointCloud2
#define DRIVER_CAMERA_POINT_DATA "driver/camera/point_cloud/Data"

//超声波数据上传
// 类型: driver/RadarsData
#define DRIVER_RADAR_DATA "driver/radar/Data"

//电机数据上传
// 类型: driver/MotorsData
#define DRIVER_MOTOR_DATA "driver/motor/Data"

//立柱电机数据上传
// 类型: driver/ShaftMotorData
#define DRIVER_SHAFT_MOTOR_DATA "driver/shaft/motor/Data"

//电池数据上传
// 类型: driver/BatteriesData
#define DRIVER_BATTERY_DATA "driver/battery/Data"

//IMU数据上传
// 类型: driver/ImuData
#define DRIVER_IMU_DATA "driver/imu/Data"

//IO数据上传
// 类型: driver/IoData
#define DRIVER_IO_DATA "driver/io/Data"

//STM32数据上传
// 类型: driver/Stm32Data
#define DRIVER_IO_DATA "driver/stm32/Data"

/*  -------- 控制接口 -------*/
//IO控制
/* Service  */
// 类型: driver/SetIoOutput
#define DRIVER_IO_SET_OUTPUT "driver/io/SetOutput"


//电机控制
/* Topic */
// driver/motor/SetMotorCmd
#define DRIVER_MOTOR_SET_MOTOR_CMD "driver/motor/SetMotorCmd"


//立柱电机控制
/* Topic */
// 类型: driver/SetShaftMotorCmd
#define DRIVER_SHAFT_MOTOR_SET_MOTOR_CMD "driver/shaft/motor/SetMotorCmd"


//灯光控制
/* Service */
// driver/light/SetLight
#define DRIVER_LIGHT_SET_LIGHT "driver/light/SetLight"


//部件版本查询
/* Service */
// 类型: driver/GetChassisDeviceInfo
#define DRIVER_CHASSIS_GET_DEVICE_INFO "driver/chassis/GetDeviceInfo"


//电源控制
/* Service */
// 类型: driver/ControlPower
#define DRIVER_CONTROL_POWER "driver/ControlPower"


//软件急停控制
/* Service */
// 类型: driver/SetScram
#define DRIVER_SET_SCRAM "driver/SetScram"


//软件急停控制
/* Service */
// 类型: driver/Reset
#define DRIVER_RESET "driver/Reset"

