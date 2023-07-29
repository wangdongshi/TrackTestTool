/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : calibrator.h
 * Project  : Track Test Tool
 * Date     : 2023/7/29
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

// Encoder
#define MILEAGE_WHEEL_DIAMETER              63.66f  // Unit : mm

// Gyro
#define GYRO_SCALE_FACTOR1                89931.8f  // Unit : degree/s
#define GYRO_SCALE_FACTOR2                89942.9f  // Unit : degree/s

// Tilt sensor
#define TILT_SCALE_FACTOR                   17.96f
#define TILT_AXIS_MISALIGNMENT_ANGLE          0.0f

#endif // __CALIBRATOR_H__
