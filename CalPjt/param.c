/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : param.c
 * Project  : Param
 * Date     : 2023/8/23
 * Author   : WangYu
 *
 **********************************************************************/
#include <math.h>

// Calibration parameter struct
#define CAL_POINTS        21

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
const CAL_TBL tbl __attribute__((section(".ARM.__at_0x08060000"))) = {
  /* distance compensation */            
  {                                      
          /* vol, distance comp */       
    /* 1*/ {0.0f, 0.001332f},            
    /* 2*/ {1.0f, 0.001347f},            
    /* 3*/ {2.0f, 0.001352f},            
    /* 4*/ {3.0f, 0.001358f},            
    /* 5*/ {4.0f, 0.001362f},            
    /* 6*/ {5.0f, 0.001365f},            
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
    /*20*/ {NAN, NAN},                   
    /*21*/ {NAN, NAN}                    
  },                                     
                                         
  /* height compensation */              
  {                                      
          /* vol, height comp */         
    /* 1*/ {0.0f, 0.1732f},              
    /* 2*/ {1.0f, 0.1747f},              
    /* 3*/ {2.0f, 0.1752f},              
    /* 4*/ {3.0f, 0.1758f},              
    /* 5*/ {4.0f, 0.1762f},              
    /* 6*/ {5.0f, 0.1765f},              
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
    /*20*/ {NAN, NAN},                   
    /*21*/ {NAN, NAN}                    
  },                                     
									
  /* track height */                     
                                         
  /* Standard calibration block height */
  /* 1: 15.39 mm */                      
  /* 2: 20.46 mm */                      
  /* 3: 40.47 mm */                      
  /* 4: 50.41 mm */                      
  /* 5: 75.28 mm */                      
  /* 6: 100.57 mm */                     
  /* 7: 125.27 mm */                     
  /* 8: 150.78 mm */                     
  /* 9: 200.08 mm */                     
                                         
  {                                      
         /* measured, actual */          
    /* 1*/ {-200.08f, -191.14f},         
    /* 2*/ {-150.78f, -143.91f},         
    /* 3*/ {-100.57f, -95.77f},          
    /* 4*/ {-50.41f,  -47.73f},          
    /* 5*/ {-20.46f,  -19.04f},          
    /* 6*/ {0.0f,     0.64f},            
    /* 7*/ {20.46f,   20.28f},           
    /* 8*/ {50.41f,   48.98f},           
    /* 9*/ {100.57f,  97.01f},           
    /*10*/ {150.78f,  145.02f},          
    /*11*/ {200.08f,  192.09f},          
    /*12*/ {NAN, NAN},                   
    /*13*/ {NAN, NAN},                   
    /*14*/ {NAN, NAN},                   
    /*15*/ {NAN, NAN},                   
    /*16*/ {NAN, NAN},                   
    /*17*/ {NAN, NAN},                   
    /*18*/ {NAN, NAN},                   
    /*19*/ {NAN, NAN},                   
    /*20*/ {NAN, NAN},                   
    /*21*/ {NAN, NAN}                    
  },                                     

  /* track distance */                   
  {                                      
          /* vol, distance */            
    /* 1*/ {1445.0f, 0.9470f},           
    /* 2*/ {1435.0f, 2.2367f},           
    /* 3*/ {1425.0f, 3.5300f},           
    /* 4*/ {NAN, NAN},                   
    /* 5*/ {NAN, NAN},                   
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
    /*20*/ {NAN, NAN},                   
    /*21*/ {NAN, NAN}                    
  }                        
};

void SystemInit(void)
{
}

int main(void)
{
  return 0;
}
