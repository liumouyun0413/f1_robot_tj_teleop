#pragma once

/* -------- 数据上传接口 ------- */
/* ------   Topic  -----*/
//状态上传接口(包含速度里程计等信息) 
// 类型: move/State
#define MOVE_STATE "move/State"


/*  -------- 控制接口 -------*/

//软手/自动模式切换接口
/* Service  */
// 类型: move/SetControlMode
#define MOVE_SET_CONTROL_MODE "move/SetControlMode"


//驻车模式控制接口
/* Service  */
// 类型: move/SetParkingMode
#define MOVE_SET_PARKING_MODE "move/SetParkingMode"


//软手动运动控制接口
/* Topic */
// 类型: geometry_msgs/TwistStamped
#define MOVE_MANUAL_MOVE_CMD "move/ManualMoveCmd"

//自动运动控制接口
/* Topic */
// 类型: geometry_msgs/TwistStamped
#define MOVE_AUTO_MOVE_CMD "move/AutoMoveCmd"


