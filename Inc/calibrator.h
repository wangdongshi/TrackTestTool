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
          /* vol, distance comp */       \
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
  /* height compensation */              \
  {                                      \
          /* vol, height comp */         \
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
  /* height */                           \
  {                                      \
         /* height, height */            \
    /* 1*/ {0.0f, 0.64f},                \
    /* 2*/ {20.46f, 20.28f},             \
    /* 3*/ {50.41f, 48.98f},             \
    /* 4*/ {100.57f, 97.01f},            \
    /* 1*/ {NAN, NAN/*-200.08f, NAN*/},  \
    /* 2*/ {NAN, NAN/*-175.11f, NAN*/},  \
    /* 3*/ {NAN, NAN/*-150.78f, NAN*/},  \
    /* 4*/ {NAN, NAN/*-125.27f, NAN*/},  \
    /* 5*/ {NAN, NAN/*-100.57f, NAN*/},  \
    /* 6*/ {NAN, NAN/*-75.28f, NAN*/},   \
    /* 7*/ {NAN, NAN/*-50.41f, NAN*/},   \
    /* 8*/ {NAN, NAN/*-40.47f, NAN*/},   \
    /* 9*/ {NAN, NAN/*-20.46f, NAN*/},   \
    /*10*/ {NAN, NAN/*-15.39f, NAN*/},   \
    /*11*/ {NAN, NAN/*0.0f, 0.64f*/},    \
    /*12*/ {NAN, NAN/*15.39f, NAN*/},    \
    /*13*/ {NAN, NAN/*20.46f, 20.28f*/}, \
    /*14*/ {NAN, NAN/*40.47f, NAN*/},    \
    /*15*/ {NAN, NAN/*50.41f, 48.98f*/}, \
    /*20*/ {NAN, NAN/*175.11f, NAN*/}    \
  },                                     \
										                     \
  /* distance */                         \
  {                                      \
          /* vol, distance */            \
    /* 1*/ {1445.0f, 0.9470f},           \
    /* 2*/ {1435.0f, 2.2367f},           \
    /* 3*/ {1425.0f, 3.5300f},           \
    /* 4*/ {NAN, NAN},                   \
    /* 5*/ {NAN, NAN},                   \
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
    /*20*/ {NAN, NAN}                    \
  }                                      \
}

#endif // __CALIBRATOR_H__
