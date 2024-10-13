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
    /* 1*/ {-3.0f, 1.857f},             \
    /* 2*/ {-1.5f, 2.089f},             \
    /* 3*/ {0.0f,  2.330f},             \
    /* 4*/ {1.5f,  2.557f},             \
    /* 5*/ {3.0f,  2.796f},             \
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
    /* 1*/ {3.0f,  1.888f},             \
    /* 2*/ {1.5f,  2.125f},             \
    /* 3*/ {0.0f,  2.355f},             \
    /* 4*/ {-1.5f, 2.596f},             \
    /* 5*/ {-3.0f, 2.835f},             \
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
    /* 1*/ {-200.35f+0.075f, -199.17f},         \
    /* 2*/ {-150.60f+0.075f, -149.92f},         \
    /* 3*/ {-100.45f+0.075f, -100.26f},          \
    /* 4*/ {-50.50f+0.075f,  -50.83f},          \
    /* 5*/ {-0.15f+0.075f,   -0.93f},            \
    /* 6*/ { 0.00f+0.075f,   -0.80f},            \
    /* 7*/ { 50.35f+0.075f,   49.11f},          \
    /* 8*/ { 100.25f+0.075f,  98.53f},         \
    /* 9*/ { 150.40f+0.075f,  148.18f},         \
    /*10*/ { 200.10f+0.075f,  197.28f},         \
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
    /* 1*/ {1470.0f, 0.290f},            \
    /* 2*/ {1465.0f, 0.610f},            \
    /* 3*/ {1460.0f, 0.930f},            \
    /* 4*/ {1455.0f, 1.249f},            \
    /* 5*/ {1450.0f, 1.570f},            \
    /* 6*/ {1445.0f, 1.889f},            \
    /* 7*/ {1440.0f, 2.209f},            \
    /* 8*/ {1435.0f, 2.529f},            \
    /* 9*/ {1430.0f, 2.849f},            \
    /*10*/ {1425.0f, 3.169f},            \
    /*11*/ {1420.0f, 3.490f},            \
    /*12*/ {1415.0f, 3.809f},            \
    /*13*/ {1410.0f, 4.134f},            \
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
