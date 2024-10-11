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
#define CAL_POINTS        21

typedef struct _pair {
  float real;
  float meas;
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
#define GYRO_SCALE_FACTOR1                89931.8f  // gyro1 : use for pitch angle test   (unit : degree/s)
#define GYRO_SCALE_FACTOR2                89942.9f  // gyro2 : use for yaw angle test     (unit : degree/s)

// Tilt sensor
#define TILT_SCALE_FACTOR                     18.0355f
#define TILT_AXIS_MISALIGNMENT_ANGLE          0.0f

// Calibration data
#define CAL_TBL_DATA {                   \
  /* distance compensation */            \
  {                                      \
          /* comp, vol */                \
    /* 1*/ {-3.0f, 1.748f},             \
    /* 2*/ {-1.5f, 1.985f},             \
    /* 3*/ {0.0f,  2.214f},             \
    /* 4*/ {1.5f,  2.450f},             \
    /* 5*/ {3.0f,  2.688f},             \
    /* 6*/ {NAN, NAN},                   \
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
    /*20*/ {NAN, NAN},                   \
    /*21*/ {NAN, NAN}                    \
  },                                     \
                                         \
  /* height compensation */              \
  {                                      \
          /* comp, vol */                \
    /* 1*/ {3.0f,  1.945f},             \
    /* 2*/ {1.5f,  2.180f},             \
    /* 3*/ {0.0f,  2.412f},             \
    /* 4*/ {-1.5f, 2.653f},             \
    /* 5*/ {-3.0f, 2.886f},             \
    /* 6*/ {NAN, NAN},                   \
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
    /*20*/ {NAN, NAN},                   \
    /*21*/ {NAN, NAN}                    \
  },                                     \
                                         \
  /* track height */                     \
                                         \
  /* Standard calibration block height */\
  /* 1: 15.39 mm */                      \
  /* 2: 20.46 mm */                      \
  /* 3: 40.47 mm */                      \
  /* 4: 50.40 mm */                      \
  /* 5: 75.28 mm */                      \
  /* 6: 100.26 mm */                     \
  /* 7: 125.27 mm */                     \
  /* 8: 150.40 mm */                     \
  /* 9: 200.16 mm */                     \
                                         \
  {                                      \
         /* measured, actual */          \
    /* 1*/ {-200.45f+0.1f, -200.70f},         \
    /* 2*/ {-150.65f+0.1f, -151.32f},         \
    /* 3*/ {-100.45f+0.1f, -101.53f},          \
    /* 4*/ {-50.60f+0.1f,  -51.99f},          \
    /* 5*/ {-0.20f+0.1f,   -1.96f},            \
    /* 6*/ { 0.00f+0.1f,   -1.80f},            \
    /* 7*/ { 50.25f+0.1f,   48.23f},          \
    /* 8*/ { 100.20f+0.1f,  97.78f},         \
    /* 9*/ { 150.40f+0.1f,  147.55f},         \
    /*10*/ { 200.10f+0.1f,  196.77f},         \
    /*11*/ {NAN, NAN},                   \
    /*12*/ {NAN, NAN},                   \
    /*13*/ {NAN, NAN},                   \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN},                   \
    /*21*/ {NAN, NAN}                    \
  },                                     \
                                         \
  /* track distance */                   \
  {                                      \
          /* distance, vol */            \
    /* 1*/ {1470.0f, 0.383f},            \
    /* 2*/ {1465.0f, 0.703f},            \
    /* 3*/ {1460.0f, 1.026f},            \
    /* 4*/ {1455.0f, 1.347f},            \
    /* 5*/ {1450.0f, 1.668f},            \
    /* 6*/ {1445.0f, 1.989f},            \
    /* 7*/ {1440.0f, 2.309f},            \
    /* 8*/ {1435.0f, 2.632f},            \
    /* 9*/ {1430.0f, 2.953f},            \
    /*10*/ {1425.0f, 3.274f},            \
    /*11*/ {1420.0f, 3.596f},            \
    /*12*/ {1415.0f, 3.918f},            \
    /*13*/ {1410.0f, 4.239f},            \
    /*14*/ {NAN, NAN},                   \
    /*15*/ {NAN, NAN},                   \
    /*16*/ {NAN, NAN},                   \
    /*17*/ {NAN, NAN},                   \
    /*18*/ {NAN, NAN},                   \
    /*19*/ {NAN, NAN},                   \
    /*20*/ {NAN, NAN},                   \
    /*21*/ {NAN, NAN}                    \
  }                                      \
}

#endif // __CALIBRATOR_H__
