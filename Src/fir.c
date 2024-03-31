/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : FIR.c
 * Project  : Param
 * Date     : 2024/3/30
 * Author   : WangYu
 *
 **********************************************************************/

#define __FPU_PRESENT        1U

#include "debug.h"
#include "arm_fir_init_f32.c"
#include "arm_fir_f32.c"

#define TEST_LENGTH_SAMPLES  200
#define BLOCK_SIZE           32

#define NUM_TAPS             11
#define NUM_TAPS_ARRAY_SIZE  11

const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.018161596335410146,
  -0.022750945545202503,
  0.026960154865719534,
  0.14940224353886064,
  0.290005928547412,
  0.35294963112469024,
  0.290005928547412,
  0.14940224353886064,
  0.026960154865719534,
  -0.022750945545202503,
  -0.018161596335410146
};

static float32_t testInput[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES] = {0.0f};
  
static uint32_t blockSize = BLOCK_SIZE;
static uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

void fir_test(void)
{
  arm_fir_instance_f32 S;
  float32_t *inputF32, *outputF32;

  /* Initialize input and output buffer pointers */
  inputF32 = &testInput[0];
  outputF32 = &testOutput[0];

  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

  /* ----------------------------------------------------------------------
  ** Call the FIR process function for every blockSize samples
  ** ------------------------------------------------------------------- */

  for(uint32_t i=0; i < numBlocks; i++) {
    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
  }
  
  /* Print FIR result */
  PRINTF("Output Array = \r\n");
  for(uint32_t i=0; i < TEST_LENGTH_SAMPLES; i++) {
    PRINTF("%6.5f\r\n", testOutput[i]);
  }
}

static float32_t testInput[TEST_LENGTH_SAMPLES] = {
-5.8    ,
-5.58   ,
-6.04   ,
-5.55   ,
-5.78   ,
-5.36   ,
-4.88   ,
-5.16   ,
-5.93   ,
-6.14   ,
-5.17   ,
-4.78   ,
-4.39   ,
-4.76   ,
-5.63   ,
-4.87   ,
-4.83   ,
-4.03   ,
-4.79   ,
-4.71   ,
-4.43   ,
-3.56   ,
-3.66   ,
-4.31   ,
-5.66   ,
-5.71   ,
-4.59   ,
-3.94   ,
-4.3    ,
-5.45   ,
-6.13   ,
-5.67   ,
-4.84   ,
-3.86   ,
-4.33   ,
-5.04   ,
-4.95   ,
-4.46   ,
-3.95   ,
-4.09   ,
-4.37   ,
-3.98   ,
-3.92   ,
-3.97   ,
-3.92   ,
-5.1    ,
-4.93   ,
-5.22   ,
-3.75   ,
-4.53   ,
-4.85   ,
-5.9    ,
-4.7    ,
-5.71   ,
-3.53   ,
-5      ,
-4.3    ,
-4.25   ,
-4.24   ,
-4.47   ,
-4.26   ,
-5.05   ,
-4.38   ,
-3.8    ,
-4.26   ,
-4.76   ,
-5.37   ,
-5.37   ,
-5.43   ,
-5.29   ,
-5.3    ,
-5.47   ,
-5.74   ,
-4.9    ,
-5.38   ,
-3.89   ,
-5.28   ,
-4.3    ,
-2.38   ,
-2.14   ,
-2.99   ,
-1.34   ,
-2.82   ,
-1.58   ,
0.08    ,
0.42    ,
-0.42   ,
0.17    ,
0.64    ,
-1.08   ,
2.09    ,
-30.43  ,
28.35   ,
-1.67   ,
-3.31   ,
2.31    ,
3.74    ,
-27.92  ,
14.05   ,
-5.15   ,
-1.59   ,
4.56    ,
2.19    ,
-0.63   ,
-0.34   ,
-0.36   ,
0.37    ,
-0.19   ,
-0.95   ,
-0.77   ,
-0.54   ,
-0.82   ,
-0.75   ,
-1.17   ,
-0.8    ,
-1.45   ,
-1.64   ,
-1.41   ,
-1.23   ,
-1.18   ,
-0.68   ,
-1.55   ,
-0.7    ,
-1.61   ,
-1.13   ,
-0.99   ,
-0.29   ,
-0.43   ,
-0.51   ,
-1.74   ,
-1.95   ,
-1.46   ,
-1.44   ,
-1.36   ,
-2.08   ,
-1.78   ,
-2.7    ,
-1.9    ,
-2.28   ,
-2.65   ,
-2.39   ,
-1.74   ,
-1.66   ,
-2.01   ,
-2.23   ,
-2.43   ,
-1.09   ,
-0.28   ,
0.35    ,
-0.33   ,
-0.72   ,
-1.7    ,
-0.17   ,
-0.22   ,
-0.24   ,
-0.45   ,
-0.14   ,
-0.46   ,
0.35    ,
-0.27   ,
-0.75   ,
-0.86   ,
0.25    ,
1.07    ,
1.19    ,
0.47    ,
0.29    ,
1.17    ,
1.94    ,
3.05    ,
2.75    ,
2.71    ,
2.57    ,
3.57    ,
3.4     ,
3.69    ,
3.04    ,
3.5     ,
4.26    ,
5.28    ,
4.88    ,
4.2     ,
3.23    ,
4.29    ,
5.49    ,
6.39    ,
6.11    ,
4.93    ,
5.35    ,
6.81    ,
8.24    ,
6.18    ,
2.47    ,
23.14   ,
2.18    ,
5.5     ,
5.95    ,
1.68    ,
4.34    ,
4.1     
};
