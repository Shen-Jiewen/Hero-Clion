//
// Created by Rick on 2024/12/10.
//

#include "imu.h"

FusionAhrs ahrs;
FusionOffset offset;
FusionEuler euler;

FusionVector gyroscope;        // replace this with actual gyroscope data in degrees/s
FusionVector accelerometer;    // replace this with actual accelerometer data in g

/**
  * @brief          获取惯性导航系统（INS）计算的欧拉角数据点.
  * @param[in]      none
  * @retval         返回指向欧拉角数据（euler.array）的指针，类型为 fp32*。
  */
fp32* get_INS_angle_point(void){
	return euler.array;
}

/**
  * @brief          获取陀螺仪数据点.
  * @param[in]      none
  * @retval         返回指向陀螺仪数据（gyroscope.array）的指针，类型为 fp32*。
  */
fp32* get_gyro_data_point(void){
	return gyroscope.array;
}