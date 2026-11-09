#pragma once

/* -------- 数据上传接口 ------- */
/* ------   Topic  -----*/

// 定位数据上传
// 类型: slam/LocationData
#define SLAM_LOCATION_DATA "slam/LocationData"


/*  -------- 控制接口 -------*/

//手动重定位
/* Service  */
// 类型: slam/Relocation
#define SLAM_RELOCATION "slam/Relocation"


//设置地图接口
/* Service  */
// 类型: slam/SetMap
#define SLAM_SET_MAP "slam/SetMap"


//开始建图接口
/* Service */
// 类型: slam/StartMapping
#define SLAM_START_MAPPING "slam/StartMapping"

//停止建图接口
/* Action */
// 类型: slam/StopMapping
#define SLAM_STOP_MAPPING "slam/StopMapping"


//辅助定位接口
/* Action */
// 类型: slam/AuxiliaryLocation
#define SLAM_AUXILIARY_LOCATION "slam/AuxiliaryLocation"

