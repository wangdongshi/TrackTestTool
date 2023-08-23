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
#define GYRO_SCALE_FACTOR1                89931.8f  // gyro1 : use for yaw angle test   (unit : degree/s)
#define GYRO_SCALE_FACTOR2                89942.9f  // gyro2 : use for pitch angle test (unit : degree/s)

// Tilt sensor
#define TILT_SCALE_FACTOR                   17.96f
#define TILT_AXIS_MISALIGNMENT_ANGLE          0.0f

// Calibration data
#define CAL_TBL_DATA {                   \
  /* distance compensation */            \
  {                                      \
          /* vol, distance */            \
    /* 1*/ {0.0f, 0.001332f},            \
    /* 2*/ {1.0f, 0.001347f},            \
    /* 3*/ {2.0f, 0.001352f},            \
    /* 4*/ {3.0f, 0.001358f},            \
    /* 5*/ {4.0f, 0.001362f},            \
    /* 6*/ {5.0f, 0.001365f},            \
    /* 7*/ {NAN, NAN},                   \
    /* 8*/ {NAN, NAN},                   \
    /* 9*/ {NAN, NAN},                   \
    /*10*/ {NAN, NAN},                   \
    /*11*/ {NAN, NAN},                   \
    /*12*/ {NAN, NAN},                   \
    /*13*/ {NAN, NAN},                   \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN}                    \
  },                                     \
                                         \
  /* height */                           \
  {                                      \
          /* vol, height */              \
    /* 1*/ {0.0f, 0.1732f},              \
    /* 2*/ {1.0f, 0.1747f},              \
    /* 3*/ {2.0f, 0.1752f},              \
    /* 4*/ {3.0f, 0.1758f},              \
    /* 5*/ {4.0f, 0.1762f},              \
    /* 6*/ {5.0f, 0.1765f},              \
    /* 7*/ {NAN, NAN},                   \
    /* 8*/ {NAN, NAN},                   \
    /* 9*/ {NAN, NAN},                   \
    /*10*/ {NAN, NAN},                   \
    /*11*/ {NAN, NAN},                   \
    /*12*/ {NAN, NAN},                   \
    /*13*/ {NAN, NAN},                   \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN}                    \
  },                                     \
										                     \
  /* dip */                              \
  {                                      \
         /* arcsin,arcsin */             \
    /* 1*/ {-1.0f, -0.99f},              \
    /* 2*/ {-0.5f, -0.55f},              \
    /* 3*/ {-0.2f, -0.21f},              \
    /* 4*/ {0.0f, 0.01f},                \
    /* 5*/ {0.2f, 0.19f},                \
    /* 6*/ {0.5f, 0.49f},                \
    /* 7*/ {1.0f, 0.98f},                \
    /* 8*/ {NAN, NAN},                   \
    /* 9*/ {NAN, NAN},                   \
    /*10*/ {NAN, NAN},                   \
    /*11*/ {NAN, NAN},                   \
    /*12*/ {NAN, NAN},                   \
    /*13*/ {NAN, NAN},                   \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN}                    \
  },                                     \
										                     \
  /* distance */                         \
  {                                      \
          /* vol, distance */            \
    /* 1*/ {0.0f, 1.409f},               \
    /* 2*/ {1.0f, 1.419f},               \
    /* 3*/ {2.0f, 1.430f},               \
    /* 4*/ {3.0f, 1.440f},               \
    /* 5*/ {4.0f, 1.451f},               \
    /* 6*/ {5.0f, 1.462f},               \
    /* 7*/ {NAN, NAN},                   \
    /* 8*/ {NAN, NAN},                   \
    /* 9*/ {NAN, NAN},                   \
    /*10*/ {NAN, NAN},                   \
    /*11*/ {NAN, NAN},                   \
    /*12*/ {NAN, NAN},                   \
    /*13*/ {NAN, NAN},                   \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN}                    \
  }                                      \
}

#endif // __CALIBRATOR_H__
