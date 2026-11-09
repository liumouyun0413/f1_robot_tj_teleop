#ifndef TOPIC_DEF_ROS_H
#define TOPIC_DEF_ROS_H

// 底盘
#define TOPIC_CHASSIS_RAW_CAN "chassis_raw_can" // 原始CAN报文主题
#define TOPIC_CHASSIS_INFO "chassis_info"       // 底盘信息主题
#define TOPIC_CHASSIS_CONTROL "chassis_control" // 底盘控制主题

// 激光雷达传感器
#define TOPIC_LIDAR_POINT_CLOUD_LF "lidar_point_left_front"  // 左前激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_MF "lidar_point_mid_front"   // 中前激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_RF "lidar_point_right_front" // 右前激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_LM "lidar_point_left_mid"    // 左中激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_MM "lidar_point_mid_mid"     // 中中激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_RM "lidar_point_right_mid"   // 右中激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_LB "lidar_point_left_back"   // 左后激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_MB "lidar_point_mid_back"    // 中后激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_RB "lidar_point_right_back"  // 右后激光雷达点云
#define TOPIC_LIDAR_POINT_CLOUD_360 "lidar_point_cloud_360"  // 360补盲激光雷达点云

// 视觉传感器
#define TOPIC_CAMERA_FRONT_T "camera_front_telephoto"         // 前视长焦相机
#define TOPIC_CAMERA_FRONT_MT "camera_front_medium_telephoto" // 前视中焦相机
#define TOPIC_CAMERA_FRONT_W "camera_front_wide_angle"        // 前视广角相机

#define TOPIC_CAMERA_MFD "camera_mid_front_down" // 前视补盲相机
#define TOPIC_CAMERA_MF "camera_mid_front"       // 前视相机
#define TOPIC_CAMERA_MB "camera_mid_back"        // 后视相机
#define TOPIC_CAMERA_LF "camera_left_front"      // 左前相机
#define TOPIC_CAMERA_LB "camera_left_back"       // 右前相机
#define TOPIC_CAMERA_RF "camera_right_front"     // 左后相机
#define TOPIC_CAMERA_RB "camera_right_back"      // 右后相机

// imu
#define TOPIC_IMU_DATA "imu_data" // imu数据主题

// 版本
#define TOPIC_VERSION "version" // 版本主题

// openspace
#define TOPIC_OPENSPACE_SRV "openspace_srv"         // openspace srv主题
#define TOPIC_OPENSPACE_MARKER "openspace_marker"   // openspace marker主题
#define TOPIC_OPENSPACE_MONITOR "openspace_monitor" // openspace monitor主题

// roadplanning
#define TOPIC_ROADPLANNING_SRV "roadplanning_srv" // roadplannning srv

// 感知、定位及规划
#define TOPIC_PLANNING "planning_info"     // 规划下行主题
#define TOPIC_LOCATION "location_info"     // 定位主题
#define TOPIC_PERCEPTION "perception_info" // 融合感知主题

// dtc
#define TOPIC_DTC_REPORT "dtc_report" // DTC发布主题
#define TOPIC_DTC_INFO "dtc_info"     // 系统当前DTC广播

// 网联
#define TOPIC_SCHEDULE_CMD "schedule_cmd" // 调度任务下发主题
#define TOPIC_SCHEDULE_RSP "schedule_rsp" // 调度执行情况反馈
// 作业
#define TOPIC_JOB "job_info"                    // 上装状态信息主题
#define TOPIC_JOB_CMD "job_cmd"                 // 上装控制主题
#define TOPIC_VOICE_BROADCAST "voice_broadcast" // 语音广播主题

// OTA

// FRAME_ID
#define FRAME_MAP "map"                                                             // 全局坐标系，定位系统建立的
#define FRAME_BODY "body"                                                           // 以后桥中心为坐标原点，前为x，左为y，上为z的车体坐标系
#define FRAME_IMU "imu"                                                             // imu坐标系
#define FRAME_MID_FRONT_LIDAR "mid_front_lidar"                                     // 中前激光雷达坐标系
#define FRAME_LEFT_FRONT_LIDAR "left_front_lidar"                                   // 左前激光雷达坐标系
#define FRAME_RIGHT_FRONT_LIDAR "right_front_lidar"                                 // 右前激光雷达坐标系
#define FRAME_LEFT_BACK_LIDAR "left_back_lidar"                                     // 左后激光雷达坐标系
#define FRAME_RIGHT_BACK_LIDAR "right_back_lidar"                                   // 右后激光雷达坐标系
#define FRAME_MID_FRONT_TELEPHOTO_CAMERA "mid_front_telephoto_camera"               // 中前长焦相机坐标系
#define FRAME_MID_FRONT_MEDIUM_TELEPHOTO_CAMERA "mid_front_medium_telephoto_camera" // 中前中焦相机坐标系
#define FRAME_MID_FRONT_WIDE_ANGLE_CAMERA "mid_front_wide_angle_camera"             // 中前广角相机坐标系
#define FRAME_MID_BACK_CAMERA "mid_back_camera"                                     // 中后相机坐标系
#define FRAME_LEFT_FRONT_CAMERA "left_front_camera"                                 // 左前相机坐标系
#define FRAME_RIGHT_FRONT_CAMERA "right_front_camera"                               // 右前相机坐标系
#define FRAME_LEFT_BACK_CAMERA "left_back_camera"                                   // 左后相机坐标系
#define FRAME_RIGHT_BACK_CAMERA "right_back_camera"                                 // 右后相机坐标系

#endif
