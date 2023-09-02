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
#define GYRO_SCALE_FACTOR1                89931.8f  // gyro1 : use for yaw angle test   (unit : degree/s)
#define GYRO_SCALE_FACTOR2                89942.9f  // gyro2 : use for pitch angle test (unit : degree/s)

// Tilt sensor
#define TILT_SCALE_FACTOR                   17.96f
#define TILT_AXIS_MISALIGNMENT_ANGLE          0.0f

// Calibration data
#define CAL_TBL_DATA {                   \
  /* distance compensation */            \
  {                                      \
          /* comp, vol */                \
    /* 1*/ {-3.0f, 1.8799f},             \
    /* 2*/ {-1.5f, 2.1074f},             \
    /* 3*/ {0.0f,  2.3499f},             \
    /* 4*/ {1.5f,  2.5801f},             \
    /* 5*/ {3.0f,  2.8273f},             \
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
    /* 1*/ {3.0f,  1.9066f},             \
    /* 2*/ {1.5f,  2.1463f},             \
    /* 3*/ {0.0f,  2.3700f},             \
    /* 4*/ {-1.5f, 2.6161f},             \
    /* 5*/ {-3.0f, 2.8563f},             \
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
  /* 4: 50.41 mm */                      \
  /* 5: 75.28 mm */                      \
  /* 6: 100.57 mm */                     \
  /* 7: 125.27 mm */                     \
  /* 8: 150.78 mm */                     \
  /* 9: 200.08 mm */                     \
                                         \
  {                                      \
         /* measured, actual */          \
    /* 1*/ {-200.08f, -191.14f},         \
    /* 2*/ {-150.78f, -143.91f},         \
    /* 3*/ {-100.57f, -95.77f},          \
    /* 4*/ {-50.41f,  -47.73f},          \
    /* 5*/ {-20.46f,  -19.04f},          \
    /* 6*/ {0.0f,     0.64f},            \
    /* 7*/ {20.46f,   20.28f},           \
    /* 8*/ {50.41f,   48.98f},           \
    /* 9*/ {100.57f,  97.01f},           \
    /*10*/ {150.78f,  145.02f},          \
    /*11*/ {200.08f,  192.09f},          \
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
    /* 1*/ {1469.0f, 0.3799f},           \
    /* 2*/ {1465.0f, 0.6362f},           \
    /* 3*/ {1455.0f, 1.2834f},           \
    /* 4*/ {1445.0f, 1.9282f},           \
    /* 5*/ {1435.0f, 2.5745f},           \
    /* 6*/ {1425.0f, 3.2209f},           \
    /* 7*/ {1415.0f, 3.8668f},           \
    /* 8*/ {1405.0f, 4.4974f},           \
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
  }                                      \
}

#endif // __CALIBRATOR_H__
