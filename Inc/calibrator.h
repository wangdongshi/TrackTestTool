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

#include <math.h>

// Calibration parameter struct
#define CAL_POINTS        20

typedef struct _pair {
  float meas;
  float real;
} CAL_PAIR;

typedef enum {
  CAL_DIST_COMPENSATION = 0,
  CAL_HEIGHT,
  CAL_DIP,
  CAL_DISTANCE,
  CAL_ITEMS
} ADC_CAL;

typedef CAL_PAIR CAL_TBL[CAL_ITEMS][CAL_POINTS];

// Encoder
#define MILEAGE_WHEEL_DIAMETER              63.66f  // Unit : mm

// Gyro
#define GYRO_SCALE_FACTOR1                89931.8f  // Unit : degree/s
#define GYRO_SCALE_FACTOR2                89942.9f  // Unit : degree/s

// Tilt sensor
#define TILT_SCALE_FACTOR                   17.96f
#define TILT_AXIS_MISALIGNMENT_ANGLE          0.0f

// Calibration data
static const CAL_TBL tbl = {
  // distance compensation
  {
          // vol, distance
    /* 1*/ {0.0f, 0.0f},
    /* 2*/ {0.0f, 0.0f},
    /* 3*/ {0.0f, 0.0f},
    /* 4*/ {0.0f, 0.0f},
    /* 5*/ {0.0f, 0.0f},
    /* 6*/ {NAN, NAN},
    /* 7*/ {NAN, NAN},
    /* 8*/ {NAN, NAN},
    /* 9*/ {NAN, NAN},
    /*10*/ {NAN, NAN},
    /*11*/ {NAN, NAN},
    /*12*/ {NAN, NAN},
    /*13*/ {NAN, NAN},
    /*14*/ {NAN, NAN},
    /*15*/ {NAN, NAN},
    /*16*/ {NAN, NAN},
    /*17*/ {NAN, NAN},
    /*18*/ {NAN, NAN},
    /*19*/ {NAN, NAN},
    /*20*/ {NAN, NAN}
  },
  
  // height
  {
          // vol, height
    /* 1*/ {0.0f, 0.0f},
    /* 2*/ {0.0f, 0.0f},
    /* 3*/ {0.0f, 0.0f},
    /* 4*/ {0.0f, 0.0f},
    /* 5*/ {0.0f, 0.0f},
    /* 6*/ {NAN, NAN},
    /* 7*/ {NAN, NAN},
    /* 8*/ {NAN, NAN},
    /* 9*/ {NAN, NAN},
    /*10*/ {NAN, NAN},
    /*11*/ {NAN, NAN},
    /*12*/ {NAN, NAN},
    /*13*/ {NAN, NAN},
    /*14*/ {NAN, NAN},
    /*15*/ {NAN, NAN},
    /*16*/ {NAN, NAN},
    /*17*/ {NAN, NAN},
    /*18*/ {NAN, NAN},
    /*19*/ {NAN, NAN},
    /*20*/ {NAN, NAN}
  },
  
  // dip
  {
         // angle, angle
    /* 1*/ {0.0f, 0.0f},
    /* 2*/ {0.0f, 0.0f},
    /* 3*/ {0.0f, 0.0f},
    /* 4*/ {0.0f, 0.0f},
    /* 5*/ {0.0f, 0.0f},
    /* 6*/ {NAN, NAN},
    /* 7*/ {NAN, NAN},
    /* 8*/ {NAN, NAN},
    /* 9*/ {NAN, NAN},
    /*10*/ {NAN, NAN},
    /*11*/ {NAN, NAN},
    /*12*/ {NAN, NAN},
    /*13*/ {NAN, NAN},
    /*14*/ {NAN, NAN},
    /*15*/ {NAN, NAN},
    /*16*/ {NAN, NAN},
    /*17*/ {NAN, NAN},
    /*18*/ {NAN, NAN},
    /*19*/ {NAN, NAN},
    /*20*/ {NAN, NAN}
  },
  
  // distance
  {
          // vol, distance
    /* 1*/ {0.0f, 0.0f},
    /* 2*/ {0.0f, 0.0f},
    /* 3*/ {0.0f, 0.0f},
    /* 4*/ {0.0f, 0.0f},
    /* 5*/ {0.0f, 0.0f},
    /* 6*/ {NAN, NAN},
    /* 7*/ {NAN, NAN},
    /* 8*/ {NAN, NAN},
    /* 9*/ {NAN, NAN},
    /*10*/ {NAN, NAN},
    /*11*/ {NAN, NAN},
    /*12*/ {NAN, NAN},
    /*13*/ {NAN, NAN},
    /*14*/ {NAN, NAN},
    /*15*/ {NAN, NAN},
    /*16*/ {NAN, NAN},
    /*17*/ {NAN, NAN},
    /*18*/ {NAN, NAN},
    /*19*/ {NAN, NAN},
    /*20*/ {NAN, NAN}
  }
};

#endif // __CALIBRATOR_H__
