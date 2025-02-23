/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : calibrate.h
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
  CAL_TRACK_DIST_COMP = 0,
  CAL_ULTRA_HIGH_COMP,
  CAL_TRACK_DIST,
  CAL_ULTRA_HIGH,
  CAL_ITEMS
} ADC_CAL;

//typedef CAL_PAIR CAL_TBL[CAL_ITEMS][CAL_POINTS];
typedef struct _CAL_TBL {
  CAL_PAIR   adc_cal[CAL_ITEMS][CAL_POINTS];
  float      wheel_diameter;
  float      gyro1_scale;
  float      gyro2_scale;
  float      tilt_scale;
} CAL_TBL;

// Calibration data
#define CAL_TBL_DATA {                     \
  {                                        \
    /* distance compensation */            \
    {                                      \
            /* comp, vol */                \
      /* 1*/ {-3.0f, 1.748f},              \
      /* 2*/ {-1.5f, 1.985f},              \
      /* 3*/ {0.0f,  2.214f},              \
      /* 4*/ {1.5f,  2.450f},              \
      /* 5*/ {3.0f,  2.688f},              \
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
      /* 1*/ {3.0f,  1.945f},              \
      /* 2*/ {1.5f,  2.180f},              \
      /* 3*/ {0.0f,  2.412f},              \
      /* 4*/ {-1.5f, 2.653f},              \
      /* 5*/ {-3.0f, 2.886f},              \
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
    /* track distance */                   \
    {                                      \
            /* distance, vol */            \
      /* 1*/ {1470.0f, 0.383f},            \
      /* 2*/ {1455.0f, 1.347f},            \
      /* 3*/ {1435.0f, 2.632f},            \
      /* 4*/ {1420.0f, 3.596f},            \
      /* 5*/ {1410.0f, 4.239f},            \
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
           /* actual,   measured */        \
      /* 1*/ {-200.00f, -200.00f},         \
      /* 2*/ {-100.00f, -100.00f},         \
      /* 3*/ { -50.00f,  -50.00f},         \
      /* 4*/ { -15.00f,  -15.00f},         \
      /* 5*/ {   0.00f,   +0.00f},         \
      /* 6*/ { +15.00f,  +15.00f},         \
      /* 7*/ { +50.00f,  +50.00f},         \
      /* 8*/ {+100.00f, +100.00f},         \
      /* 9*/ {+200.00f, +200.00f},         \
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
  },                                       \
  63.66f,   /* wheel diameter */           \
  89931.8f, /* gyro1(pitch)(degree/s) */   \
  89942.9f, /* gyro2(yaw)  (degree/s) */   \
  18.0355f  /* tilt scale */               \
}

int initCalibrateData(void);
void insertCalADCRecord(
  const ADC_CAL type,
  const unsigned short seq,
  const float standVal,
  const float calibVal
);
void updateCalWheelDiameter(const float diameter);
void updateCalGyroScale(const unsigned short no, const float scale);
void updateCalTiltScale(const float scale);
int writeCalibrateData(void);
int eraseCalibrateData(void);

#endif // __CALIBRATOR_H__
