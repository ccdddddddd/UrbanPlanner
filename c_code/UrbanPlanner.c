/*
 * File: UrbanPlanner.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 28-Apr-2022 15:38:16
 */

/* Include Files */
#include "UrbanPlanner.h"
#include "UrbanPlanner_emxutil.h"
#include "UrbanPlanner_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <float.h>
#include <math.h>
#include <string.h>

#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "UrbanPlanner_emxutil.h"
#include "UrbanPlanner_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>
#include <string.h>

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/* Type Definitions */
#ifndef typedef_cell_wrap_1
#define typedef_cell_wrap_1

typedef struct {
  double f1[6];
} cell_wrap_1;

#endif                                 /*typedef_cell_wrap_1*/

#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3

typedef struct {
  double f1[3];
} cell_wrap_3;

#endif                                 /*typedef_cell_wrap_3*/

#ifndef typedef_anonymous_function
#define typedef_anonymous_function

typedef struct {
  cell_wrap_1 tunableEnvironment[1];
} anonymous_function;

#endif                                 /*typedef_anonymous_function*/

#ifndef typedef_b_anonymous_function
#define typedef_b_anonymous_function

typedef struct {
  cell_wrap_3 tunableEnvironment[1];
} b_anonymous_function;

#endif                                 /*typedef_b_anonymous_function*/

/* Function: rtGetInf ==================================================================
 * Abstract:
 * Initialize rtInf needed by the generated code.
 */
real_T rtGetInf(void)
{
  real_T inf = 0.0;
  uint16_T one = 1U;
  enum {
    LittleEndian,
    BigEndian
  } machByteOrder = (*((uint8_T *) &one) == 1U) ? LittleEndian : BigEndian;
  switch (machByteOrder) {
   case LittleEndian:
    {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
      break;
    }

   case BigEndian:
    {
      union {
        BigEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
      break;
    }
  }

  return inf;
}

/* Function: rtGetInfF =================================================================
 * Abstract:
 * Initialize rtInfF needed by the generated code.
 */
real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/* Function: rtGetMinusInf =============================================================
 * Abstract:
 * Initialize rtMinusInf needed by the generated code.
 */
real_T rtGetMinusInf(void)
{
  real_T minf = 0.0;
  uint16_T one = 1U;
  enum {
    LittleEndian,
    BigEndian
  } machByteOrder = (*((uint8_T *) &one) == 1U) ? LittleEndian : BigEndian;
  switch (machByteOrder) {
   case LittleEndian:
    {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
      break;
    }

   case BigEndian:
    {
      union {
        BigEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
      break;
    }
  }

  return minf;
}

/* Function: rtGetMinusInfF ============================================================
 * Abstract:
 * Initialize rtMinusInfF needed by the generated code.
 */
real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * File trailer for rtGetInf.c
 *
 * [EOF]
 */

/* Function: rtGetNaN ======================================================================
 * Abstract:
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
  real_T nan = 0.0;
  uint16_T one = 1U;
  enum {
    LittleEndian,
    BigEndian
  } machByteOrder = (*((uint8_T *) &one) == 1U) ? LittleEndian : BigEndian;
  switch (machByteOrder) {
   case LittleEndian:
    {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
      break;
    }

   case BigEndian:
    {
      union {
        BigEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FFFFFFFU;
      tmpVal.bitVal.words.wordL = 0xFFFFFFFFU;
      nan = tmpVal.fltVal;
      break;
    }
  }

  return nan;
}

/* Function: rtGetNaNF =====================================================================
 * Abstract:
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0 } };

  uint16_T one = 1U;
  enum {
    LittleEndian,
    BigEndian
  } machByteOrder = (*((uint8_T *) &one) == 1U) ? LittleEndian : BigEndian;
  switch (machByteOrder) {
   case LittleEndian:
    {
      nanF.wordL.wordLuint = 0xFFC00000U;
      break;
    }

   case BigEndian:
    {
      nanF.wordL.wordLuint = 0x7FFFFFFFU;
      break;
    }
  }

  return nanF.wordL.wordLreal;
}

/*
 * File trailer for rtGetNaN.c
 *
 * [EOF]
 */

/* Function: rt_InitInfAndNaN ==================================================
 * Abstract:
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
void rt_InitInfAndNaN()
{
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Function: rtIsInf ==================================================
 * Abstract:
 * Test if value is infinite
 */
boolean_T rtIsInf(real_T value)
{
  return ((value==rtInf || value==rtMinusInf) ? true : false);
}

/* Function: rtIsInfF =================================================
 * Abstract:
 * Test if single-precision value is infinite
 */
boolean_T rtIsInfF(real32_T value)
{
  return ((value==rtInfF || value==rtMinusInfF) ? true : false);
}

/* Function: rtIsNaN ==================================================
 * Abstract:
 * Test if value is not a number
 */
boolean_T rtIsNaN(real_T value)
{
  return ((value!=value) ? true : false);
}

/* Function: rtIsNaNF =================================================
 * Abstract:
 * Test if single-precision value is not a number
 */
boolean_T rtIsNaNF(real32_T value)
{
  return ((value!=value) ? true : false);
}

/*
 * File trailer for rt_nonfinite.c
 *
 * [EOF]
 */

/* Function Definitions */
/*
 * Arguments    : emxArray_boolean_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_int16_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(short));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(short) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (short *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_int32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_uint16_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(unsigned short));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(unsigned short) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (unsigned short *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_uint32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = calloc((unsigned int)i, sizeof(unsigned int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(unsigned int) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (unsigned int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 * Return Type  : void
 */
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_int16_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int16_T(emxArray_int16_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int16_T *)NULL) {
    if (((*pEmxArray)->data != (short *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int16_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_uint16_T **pEmxArray
 * Return Type  : void
 */
void emxFree_uint16_T(emxArray_uint16_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint16_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned short *)NULL) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint16_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_uint32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_uint32_T(emxArray_uint32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint32_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned int *)NULL) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint32_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_int16_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_int16_T(emxArray_int16_T **pEmxArray, int numDimensions)
{
  emxArray_int16_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int16_T *)malloc(sizeof(emxArray_int16_T));
  emxArray = *pEmxArray;
  emxArray->data = (short *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_uint16_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions)
{
  emxArray_uint16_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint16_T *)malloc(sizeof(emxArray_uint16_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned short *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_uint32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_uint32_T(emxArray_uint32_T **pEmxArray, int numDimensions)
{
  emxArray_uint32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint32_T *)malloc(sizeof(emxArray_uint32_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * File trailer for UrbanPlanner_emxutil.c
 *
 * [EOF]
 */

/* Variable Definitions */
static boolean_T isInitialized_UrbanPlanner = false;

/* Function Declarations */
static double ACC(double v_max, double v_soll, double d_ist, double speed,
                  double b_wait, double CalibrationVars_ACC_a_max, double
                  CalibrationVars_ACC_a_min, double
                  CalibrationVars_ACC_a_min_com, double
                  CalibrationVars_ACC_tau_v_com, double
                  CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                  double CalibrationVars_ACC_tau_v_bre, double
                  CalibrationVars_ACC_tau_v_emg, double
                  CalibrationVars_ACC_tau_d_emg, double
                  CalibrationVars_ACC_t_acc);
static double ACClowSpeed(double v_max, double v_soll, double d_ist, double
  speed, double c_CalibrationVars_ACClowSpeed_a, double
  d_CalibrationVars_ACClowSpeed_a, double e_CalibrationVars_ACClowSpeed_a,
  double c_CalibrationVars_ACClowSpeed_t, double d_CalibrationVars_ACClowSpeed_t,
  double e_CalibrationVars_ACClowSpeed_t, double f_CalibrationVars_ACClowSpeed_t,
  double g_CalibrationVars_ACClowSpeed_t, double h_CalibrationVars_ACClowSpeed_t,
  double i_CalibrationVars_ACClowSpeed_t, double j_CalibrationVars_ACClowSpeed_t);
static void AEBDecision(short *AEBActive, double d_veh2cross, double speed,
  double d_veh2stopline, double d_veh2waitingArea, double s_veh[6], double
  v_veh[6], const double d_veh2conflict[6], double s_veh1apostrophe[6], double
  d_veh2int, double greenLight, TypeGlobVars *GlobVars, double
  c_CalibrationVars_SpeedPlanTraf, double c_CalibrationVars_SpeedPlanAvoi,
  double Parameters_w_veh, double Parameters_l_veh);
static void LaneCenterCal(short CurrentLane, double pos_l_CurrentLane, double
  WidthOfLaneCurrent, double WidthOfGap, const double WidthOfLanesOpposite[6],
  short NumOfLanesOpposite, double LaneCenterline[7]);
static double LaneIndexJudge(short CurrentLane, double pos_l_CurrentLane, double
  WidthOfLaneCurrent, double WidthOfGap, const double WidthOfLanesOpposite[6],
  short NumOfLanesOpposite, double traj_y);
static short LaneSelectionWithBlockedLanes(const double WidthOfLanes[6], const
  short LanesWithFail[6], short *TargetLaneIndex, short CurrentLaneIndex);
static void PathPlanTurnAround(double s_turnround_border, double w_veh, double R,
  double D_safe, double l_current, double l_targetlane, double l_boundry, double
  dec2line, double R1[2], double R2[2], double R3[2], double pos_start[2],
  double p1[4], double p2[4], double pos_end[2]);
static double SpeedPlanAvoidVehicle(double speed, double d_veh2int, double
  d_veh2stopline, double s_a, double v_a, double s_b, double v_b, double s_c,
  double v_c, TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanAvoi,
  double d_CalibrationVars_SpeedPlanAvoi, double e_CalibrationVars_SpeedPlanAvoi,
  double f_CalibrationVars_SpeedPlanAvoi, double g_CalibrationVars_SpeedPlanAvoi,
  double h_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC,
  double Parameters_l_veh);
static double SpeedPlanTrafficLight(double speed, double d_veh2int, double s_b,
  double v_b, double greenLight, double time2nextSwitch, TypeGlobVars *GlobVars,
  double Parameters_l_veh, double c_CalibrationVars_SpeedPlanTraf, double
  d_CalibrationVars_SpeedPlanTraf, double e_CalibrationVars_SpeedPlanTraf,
  double f_CalibrationVars_SpeedPlanTraf, double g_CalibrationVars_SpeedPlanTraf,
  double h_CalibrationVars_SpeedPlanTraf, const CalibACC *CalibrationVars_ACC,
  const CalibACCcust CalibrationVars_ACCcust);
static void TrajPlanLaneChange(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double LeftLaneBehindDis, double LeftLaneBehindVel,
  double LeftLaneFrontDis, double LeftLaneFrontVel, double RightLaneBehindDis,
  double RightLaneBehindVel, double RightLaneFrontDis, double RightLaneFrontVel,
  double speed, double pos_s, double pos_l_CurrentLane, short CurrentLaneIndex,
  short TargetLaneIndex, short BackupTargetLaneIndex, double d_veh2int, const
  double WidthOfLanes[6], double v_max, const short LanesWithFail[6],
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters, double *a_soll, double traj_s[80], double traj_l[80],
  double traj_psi[80], double traj_vs[80], double traj_vl[80], double
  traj_omega[80]);
static void TrajPlanLaneChange_RePlan(double speed, double pos_s, double pos_l,
  double pos_l_CurrentLane, const double WidthOfLanes[6], short CurrentLaneIndex,
  double CurrentLaneFrontDis, double CurrentLaneFrontVel, TypeGlobVars *GlobVars,
  double c_CalibrationVars_TrajPlanLaneC, double d_CalibrationVars_TrajPlanLaneC,
  double e_CalibrationVars_TrajPlanLaneC, double f_CalibrationVars_TrajPlanLaneC,
  double Parameters_l_veh, double traj_s[80], double traj_l[80], double
  traj_psi[80], double traj_vs[80], double traj_vl[80], double traj_omega[80]);
static void TrajPlanTurnAround(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double speed, double pos_l_CurrentLane, double pos_s,
  double pos_l, short NumOfLanesOpposite, const double WidthOfLanesOpposite[6],
  double WidthOfGap, const double WidthOfLanes[6], double s_turnaround_border,
  const short IndexOfLaneOppositeCar[20], const double SpeedOppositeCar[20],
  const double PosSOppositeCar[20], const short IndexOfLaneCodirectCar[10],
  const double SpeedCodirectCar[10], const double PosSCodirectCar[10], short
  CurrentLane, double v_max, double a_soll, short CurrentGear, short
  *TurnAroundActive, short *AEBactive, TypeGlobVars *GlobVars, const
  TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters, double *
  a_soll_TrajPlanTurnAround, double traj_s[80], double traj_l[80], double
  traj_psi[80], double traj_vs[80], double traj_vl[80], double traj_omega[80],
  short *TargetGear);
static double anon(const anonymous_function fun_x_tunableEnvironment[1], const
                   double S_traj[41], double i_traj, double x);
static double b_ACC(double v_max, double v_soll, double d_ist, double speed,
                    short b_wait, double CalibrationVars_ACC_a_max, double
                    CalibrationVars_ACC_a_min, double
                    CalibrationVars_ACC_a_min_com, double
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc);
static double b_anon(const b_anonymous_function fun_x_tunableEnvironment[1],
                     const double S_traj[80], double i_traj, double x);
static void b_cosd(double *x);
static void b_do_vectors(const emxArray_int16_T *a, const emxArray_real_T *b,
  emxArray_int16_T *c, emxArray_int32_T *ia, int ib_size[1]);
static void b_eml_float_colon(double a, double b, emxArray_real_T *y);
static void b_fzero(const b_anonymous_function c_FunFcn_tunableEnvironment_f1_[1],
                    const double FunFcn_tunableEnvironment_f2[80], double
                    FunFcn_tunableEnvironment_f3, const double x[2], double *b,
                    double *fval, double *exitflag);
static boolean_T b_local_ismember(short a, const emxArray_int16_T *s);
static double b_maximum(const double x[3]);
static double b_minimum(const double x[3]);
static void b_mldivide(const double A[36], double B[6]);
static void b_sind(double *x);
static void c_fzero(const double FunFcn_tunableEnvironment_f1[6], double
                    FunFcn_tunableEnvironment_f2, double
                    FunFcn_tunableEnvironment_f3, double
                    FunFcn_tunableEnvironment_f4, const double
                    FunFcn_tunableEnvironment_f5[6], double
                    FunFcn_tunableEnvironment_f6, double
                    FunFcn_tunableEnvironment_f7, const double x[2], double *b,
                    double *fval, double *exitflag);
static short c_maximum(const short x[2]);
static short c_minimum(const short x[2]);
static void c_mldivide(const double A[16], double B[4]);
static void do_vectors(const emxArray_real_T *a, const emxArray_real_T *b,
  emxArray_real_T *c, emxArray_int32_T *ia, int ib_size[1]);
static void eml_float_colon(double a, double d, double b, emxArray_real_T *y);
static void eml_integer_colon_dispatcher(double a, short b, emxArray_int16_T *y);
static void fzero(const double c_FunFcn_tunableEnvironment_f1_[1], double
                  FunFcn_tunableEnvironment_f2, double
                  FunFcn_tunableEnvironment_f3, double *b, double *fval, double *
                  exitflag);
static void linspace(double d2, double y[100]);
static boolean_T local_ismember(short a, const emxArray_real_T *s);
static double maximum(const double x[2]);
static double median(const double x[3]);
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static double minimum(const double x[2]);
static void mldivide(const double A[9], const double B[3], double Y[3]);
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_remd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static double skip_to_last_equal_value(int *k, const emxArray_real_T *x);
static void sort(emxArray_real_T *x);
static double sum(const double x_data[], const int x_size[2]);
static double trapz(const double x[100], const double y[100]);

/* Function Definitions */
/*
 * Arguments    : double v_max
 *                double v_soll
 *                double d_ist
 *                double speed
 *                double b_wait
 *                double CalibrationVars_ACC_a_max
 *                double CalibrationVars_ACC_a_min
 *                double CalibrationVars_ACC_a_min_com
 *                double CalibrationVars_ACC_tau_v_com
 *                double CalibrationVars_ACC_tau_v
 *                double CalibrationVars_ACC_tau_d
 *                double CalibrationVars_ACC_tau_v_bre
 *                double CalibrationVars_ACC_tau_v_emg
 *                double CalibrationVars_ACC_tau_d_emg
 *                double CalibrationVars_ACC_t_acc
 * Return Type  : double
 */
static double ACC(double v_max, double v_soll, double d_ist, double speed,
                  double b_wait, double CalibrationVars_ACC_a_max, double
                  CalibrationVars_ACC_a_min, double
                  CalibrationVars_ACC_a_min_com, double
                  CalibrationVars_ACC_tau_v_com, double
                  CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                  double CalibrationVars_ACC_tau_v_bre, double
                  CalibrationVars_ACC_tau_v_emg, double
                  CalibrationVars_ACC_tau_d_emg, double
                  CalibrationVars_ACC_t_acc)
{
  double b_speed[3];
  double b_accel[2];
  double accel;
  double d_soll;

  /* 2.5; */
  /* -4; */
  /* -1.5; */
  /* 4; */
  /* 2; */
  /* 5; */
  /* 1; */
  /* 0.5; */
  /* 2; */
  /* 2; */
  accel = 100.0;
  if (d_ist < 100.0) {
    if (b_wait == -1.0) {
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 17.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    } else {
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 9.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    }

    if (speed * speed / d_ist / 2.0 > -CalibrationVars_ACC_a_min_com) {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v_bre;
    } else if (fabs(speed - v_soll) < 2.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v_com;
    } else {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v;
    }

    if (d_ist < 9.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
    } else {
      if (d_ist < 12.0) {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_emg;
      }
    }
  }

  b_accel[0] = -2.5;
  b_accel[1] = (v_max - speed) / CalibrationVars_ACC_tau_v;
  d_soll = maximum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_soll;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_max;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_min;
  return maximum(b_accel);
}

/*
 * Arguments    : double v_max
 *                double v_soll
 *                double d_ist
 *                double speed
 *                double c_CalibrationVars_ACClowSpeed_a
 *                double d_CalibrationVars_ACClowSpeed_a
 *                double e_CalibrationVars_ACClowSpeed_a
 *                double c_CalibrationVars_ACClowSpeed_t
 *                double d_CalibrationVars_ACClowSpeed_t
 *                double e_CalibrationVars_ACClowSpeed_t
 *                double f_CalibrationVars_ACClowSpeed_t
 *                double g_CalibrationVars_ACClowSpeed_t
 *                double h_CalibrationVars_ACClowSpeed_t
 *                double i_CalibrationVars_ACClowSpeed_t
 *                double j_CalibrationVars_ACClowSpeed_t
 * Return Type  : double
 */
static double ACClowSpeed(double v_max, double v_soll, double d_ist, double
  speed, double c_CalibrationVars_ACClowSpeed_a, double
  d_CalibrationVars_ACClowSpeed_a, double e_CalibrationVars_ACClowSpeed_a,
  double c_CalibrationVars_ACClowSpeed_t, double d_CalibrationVars_ACClowSpeed_t,
  double e_CalibrationVars_ACClowSpeed_t, double f_CalibrationVars_ACClowSpeed_t,
  double g_CalibrationVars_ACClowSpeed_t, double h_CalibrationVars_ACClowSpeed_t,
  double i_CalibrationVars_ACClowSpeed_t, double j_CalibrationVars_ACClowSpeed_t)
{
  double b_speed[3];
  double b_accel[2];
  double accel;
  double d_soll;
  double speed_tmp;

  /* 2.5; */
  /* -4; */
  /* -1.5; */
  /* 4; */
  /* 2; */
  /* 5; */
  /* 1; */
  /* 0.5; */
  /* 2; */
  /* 5/2; */
  /* 2; */
  accel = 100.0;
  if (d_ist < 100.0) {
    b_speed[0] = speed * j_CalibrationVars_ACClowSpeed_t;
    b_speed[1] = 9.0;
    speed_tmp = speed * speed;
    b_speed[2] = (v_soll * v_soll - speed_tmp) / (2.0 *
      d_CalibrationVars_ACClowSpeed_a);
    d_soll = b_maximum(b_speed);
    if (speed_tmp / d_ist / 2.0 > -e_CalibrationVars_ACClowSpeed_a) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               e_CalibrationVars_ACClowSpeed_t) /
        f_CalibrationVars_ACClowSpeed_t;
    } else if (fabs(speed - v_soll) < 2.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               i_CalibrationVars_ACClowSpeed_t) /
        c_CalibrationVars_ACClowSpeed_t;
    } else {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               i_CalibrationVars_ACClowSpeed_t) /
        d_CalibrationVars_ACClowSpeed_t;
    }

    if (d_ist < 9.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               h_CalibrationVars_ACClowSpeed_t) /
        g_CalibrationVars_ACClowSpeed_t;
    } else {
      if (d_ist < 11.0) {
        accel = ((v_soll - speed) + (d_ist - d_soll) /
                 e_CalibrationVars_ACClowSpeed_t) /
          g_CalibrationVars_ACClowSpeed_t;
      }
    }
  }

  b_accel[0] = -2.5;
  b_accel[1] = (v_max - speed) / d_CalibrationVars_ACClowSpeed_t;
  speed_tmp = maximum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = speed_tmp;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = c_CalibrationVars_ACClowSpeed_a;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_CalibrationVars_ACClowSpeed_a;
  return maximum(b_accel);
}

/*
 * ,
 * Arguments    : short *AEBActive
 *                double d_veh2cross
 *                double speed
 *                double d_veh2stopline
 *                double d_veh2waitingArea
 *                double s_veh[6]
 *                double v_veh[6]
 *                const double d_veh2conflict[6]
 *                double s_veh1apostrophe[6]
 *                double d_veh2int
 *                double greenLight
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_SpeedPlanTraf
 *                double c_CalibrationVars_SpeedPlanAvoi
 *                double Parameters_w_veh
 *                double Parameters_l_veh
 * Return Type  : void
 */
static void AEBDecision(short *AEBActive, double d_veh2cross, double speed,
  double d_veh2stopline, double d_veh2waitingArea, double s_veh[6], double
  v_veh[6], const double d_veh2conflict[6], double s_veh1apostrophe[6], double
  d_veh2int, double greenLight, TypeGlobVars *GlobVars, double
  c_CalibrationVars_SpeedPlanTraf, double c_CalibrationVars_SpeedPlanAvoi,
  double Parameters_w_veh, double Parameters_l_veh)
{
  double s_max[6];
  double b_v_veh[2];
  double d;
  int i;
  short wait_AvoidVehicle;
  short wait_TrafficLight;
  short wait_ped;
  boolean_T exitg1;

  /* -------------------------------------------------------------------------------------------------------------------------------------------------------- */
  wait_TrafficLight = GlobVars->SpeedPlanTrafficLight.wait_TrafficLight;
  wait_ped = GlobVars->SpeedPlanAvoidPedestrian.wait_ped;
  wait_AvoidVehicle = GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle;

  /*  ����ͣ������ */
  /* 30/3.6; */
  /* 2 */
  if (*AEBActive == 0) {
    /*  if PrePedestrianActive==1 && PedestrianActive==0 && wait_ped==1 && speed>0 */
    if ((d_veh2cross <= 0.0) && (GlobVars->SpeedPlanAvoidPedestrian.wait_ped ==
         1) && (speed > 0.0)) {
      /*  �������˾��� �� AEB */
      *AEBActive = 1;
      wait_ped = 0;
    }

    if ((d_veh2stopline <= 0.0) &&
        (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1) && (speed > 0.0)
        && (*AEBActive == 0)) {
      /*  ����ͬ�������� �� AEB */
      *AEBActive = 2;
      wait_AvoidVehicle = 0;
    }

    /*  if d_veh2int<=0 && d_veh2int>=-0.5*l_veh && greenLight==0 && speed>0 && AEBActive==0 % ���̵�ͨ�о��� �� AEB */
    if ((d_veh2int <= 0.0) && ((0.0 - speed * speed) / -8.0 - d_veh2int <= 10.0)
        && (greenLight == 0.0) && (speed > 0.0) && (*AEBActive == 0)) {
      /*  ���̵�ͨ�о��� �� AEB */
      *AEBActive = 4;
      wait_TrafficLight = 0;
    }

    if ((d_veh2waitingArea <= 0.0) && (speed > 0.0) && (*AEBActive == 0)) {
      /*  ���ö��������� �� AEB */
      /*          if wait_avoidOncomingVehicle==1 */
      /*              AEBActive=3; */
      /*              wait_avoidOncomingVehicle=0; */
      /*          else */
      /*              d_veh1=max([(d_veh2cross1+l_veh)/max([speed 0.00001])*v_veh1+0.5*w_veh+l_veh 0]); */
      /*              d_veh2=max([(d_veh2cross2+l_veh)/max([speed 0.00001])*v_veh2+0.5*w_veh+l_veh 0]); */
      /*              d_veh3=max([(d_veh2cross3+l_veh)/max([speed 0.00001])*v_veh3+0.5*w_veh+l_veh 0]); */
      /*              if s_veh1<=d_veh1 || s_veh1apostrophe1>-l_veh || s_veh2<=d_veh2 || s_veh1apostrophe2>-l_veh || s_veh3<=d_veh3 || s_veh1apostrophe3>-l_veh */
      /*                  AEBActive=3; */
      /*              end */
      for (i = 0; i < 6; i++) {
        if (d_veh2conflict[i] == 0.0) {
          s_veh[i] = 200.0;
          v_veh[i] = 0.0;
          s_veh1apostrophe[i] = -200.0;
        }

        b_v_veh[0] = v_veh[i];
        b_v_veh[1] = 1.0E-5;
        d = maximum(b_v_veh);
        b_v_veh[0] = 0.0;
        b_v_veh[1] = ((s_veh[i] - 0.5 * Parameters_w_veh) -
                      c_CalibrationVars_SpeedPlanAvoi) / d;
        d = maximum(b_v_veh);
        b_v_veh[0] = speed + 1.5 * d;
        b_v_veh[1] = c_CalibrationVars_SpeedPlanTraf;
        s_max[i] = 0.5 * (minimum(b_v_veh) + speed) * d;
      }

      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 6)) {
        if ((s_max[i] > d_veh2conflict[i] + Parameters_l_veh) &&
            (s_veh1apostrophe[i] < -Parameters_l_veh)) {
          i++;
        } else {
          *AEBActive = 3;
          exitg1 = true;
        }
      }

      /*          end */
    }
  }

  GlobVars->SpeedPlanTrafficLight.wait_TrafficLight = wait_TrafficLight;
  GlobVars->SpeedPlanAvoidPedestrian.wait_ped = wait_ped;
  GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle = wait_AvoidVehicle;
}

/*
 * Arguments    : short CurrentLane
 *                double pos_l_CurrentLane
 *                double WidthOfLaneCurrent
 *                double WidthOfGap
 *                const double WidthOfLanesOpposite[6]
 *                short NumOfLanesOpposite
 *                double LaneCenterline[7]
 * Return Type  : void
 */
static void LaneCenterCal(short CurrentLane, double pos_l_CurrentLane, double
  WidthOfLaneCurrent, double WidthOfGap, const double WidthOfLanesOpposite[6],
  short NumOfLanesOpposite, double LaneCenterline[7])
{
  double Lane_boundary[24];
  double WidthOfLanes[8];
  double d;
  double d1;
  double y;
  int b_i;
  int i;
  int i1;
  int k;
  i = -CurrentLane;
  if (-CurrentLane > 32767) {
    i = 32767;
  }

  WidthOfLanes[0] = WidthOfLaneCurrent;
  WidthOfLanes[1] = WidthOfGap;
  for (i1 = 0; i1 < 6; i1++) {
    WidthOfLanes[i1 + 2] = WidthOfLanesOpposite[i1];
  }

  memset(&Lane_boundary[0], 0, 24U * sizeof(double));
  for (i1 = 0; i1 < 7; i1++) {
    LaneCenterline[i1] = 0.0;
  }

  for (b_i = 0; b_i < 8; b_i++) {
    /*      if i==1 */
    /*          Lane_boundary(i,1)=WidthOfLanes(length(WidthOfLanes)); */
    /*      else */
    /*          Lane_boundary(i,1)=Lane_boundary(i-1,1)+WidthOfLanes(length(WidthOfLanes)+1-i); */
    /*      end */
    y = WidthOfLanes[0];
    for (k = 2; k <= b_i + 1; k++) {
      y += WidthOfLanes[k - 1];
    }

    Lane_boundary[b_i] = y;
    i1 = b_i + (short)i;
    if (i1 > 32767) {
      i1 = 32767;
    }

    Lane_boundary[b_i + 16] = i1;
  }

  y = (WidthOfLanes[0] * 0.5 + pos_l_CurrentLane) - Lane_boundary[0];
  for (k = 0; k < 8; k++) {
    d = Lane_boundary[k] + y;
    Lane_boundary[k + 8] = d;
    d1 = d - WidthOfLanes[k];
    Lane_boundary[k] = d1;
    WidthOfLanes[k] = 0.5 * (d + d1);
  }

  i = NumOfLanesOpposite + 2;
  if (NumOfLanesOpposite + 2 > 32767) {
    i = 32767;
  }

  if (3 > (short)i) {
    i1 = 0;
    i = 0;
  } else {
    i1 = 2;
    i = (short)i;
  }

  k = i - i1;
  for (i = 0; i < k; i++) {
    LaneCenterline[i] = WidthOfLanes[i1 + i];
  }

  LaneCenterline[k] = WidthOfLanes[0];

  /*  WidthOfLaneCurrent=3.1; */
  /*  WidthOfLanesOpposite=[3.2 3.3 3.4 3.5 3.6 0]; */
  /*  WidthOfGap=1; */
  /*  CurrentLane=1; */
  /*  pos_l_CurrentLane=0; */
  /*  NumOfLanesOpposite=5; */
  /*  LaneCenterline=LaneCenterCal(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite) */
}

/*
 * Arguments    : short CurrentLane
 *                double pos_l_CurrentLane
 *                double WidthOfLaneCurrent
 *                double WidthOfGap
 *                const double WidthOfLanesOpposite[6]
 *                short NumOfLanesOpposite
 *                double traj_y
 * Return Type  : double
 */
static double LaneIndexJudge(short CurrentLane, double pos_l_CurrentLane, double
  WidthOfLaneCurrent, double WidthOfGap, const double WidthOfLanesOpposite[6],
  short NumOfLanesOpposite, double traj_y)
{
  double Lane_boundary[24];
  double WidthOfLanes[8];
  double LaneIndexOfPoint;
  double d;
  double y;
  int b_i;
  int i;
  int k;
  signed char tmp_data[8];
  i = -CurrentLane;
  if (-CurrentLane > 32767) {
    i = 32767;
  }

  WidthOfLanes[0] = WidthOfLaneCurrent;
  WidthOfLanes[1] = WidthOfGap;
  for (k = 0; k < 6; k++) {
    WidthOfLanes[k + 2] = WidthOfLanesOpposite[k];
  }

  memset(&Lane_boundary[0], 0, 24U * sizeof(double));
  for (b_i = 0; b_i < 8; b_i++) {
    /*      if i==1 */
    /*          Lane_boundary(i,1)=WidthOfLanes(length(WidthOfLanes)); */
    /*      else */
    /*          Lane_boundary(i,1)=Lane_boundary(i-1,1)+WidthOfLanes(length(WidthOfLanes)+1-i); */
    /*      end */
    y = WidthOfLanes[0];
    for (k = 2; k <= b_i + 1; k++) {
      y += WidthOfLanes[k - 1];
    }

    Lane_boundary[b_i] = y;
    k = b_i + (short)i;
    if (k > 32767) {
      k = 32767;
    }

    Lane_boundary[b_i + 16] = k;
  }

  y = (WidthOfLanes[0] * 0.5 + pos_l_CurrentLane) - Lane_boundary[0];
  for (k = 0; k < 8; k++) {
    d = Lane_boundary[k] + y;
    Lane_boundary[k + 8] = d;
    Lane_boundary[k] = d - WidthOfLanes[k];
  }

  /*  CurrentTargetLaneIndex1=-1; */
  d = Lane_boundary[8] - WidthOfLanes[0];
  if ((traj_y >= d) && (traj_y < Lane_boundary[15])) {
    k = 0;
    for (b_i = 0; b_i < 8; b_i++) {
      if (Lane_boundary[b_i + 8] - traj_y >= 0.0) {
        tmp_data[k] = (signed char)(b_i + 1);
        k++;
      }
    }

    LaneIndexOfPoint = Lane_boundary[tmp_data[0] + 15];
  } else if (traj_y < d) {
    LaneIndexOfPoint = Lane_boundary[16] - 1.0;
  } else if (traj_y >= Lane_boundary[15]) {
    LaneIndexOfPoint = NumOfLanesOpposite;
  } else {
    LaneIndexOfPoint = 0.0;

    /* 20220324 */
  }

  /*  WidthOfLaneCurrent=3.1; */
  /*  WidthOfLanesOpposite=[3.2 3.3 3.4 3.5 3.6 0]; */
  /*  WidthOfGap=1; */
  /*  CurrentLane=1; */
  /*  pos_l_CurrentLane=0; */
  /*  NumOfLanesOpposite=5; */
  /*  traj_y=-1.5; */
  /*  LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,traj_y) */
  return LaneIndexOfPoint;
}

/*
 * NumOfLanes=2; % ��������
 *  LanesWithFail=[1 2]; % ���ϳ����ڳ�����ţ����������С�������У�����೵�����Ϊ1
 *  TargetLaneIndex=2; % ԭĿ�공��
 *  CurrentLaneIndex=1; �������ڳ���
 * Arguments    : const double WidthOfLanes[6]
 *                const short LanesWithFail[6]
 *                short *TargetLaneIndex
 *                short CurrentLaneIndex
 * Return Type  : short
 */
static short LaneSelectionWithBlockedLanes(const double WidthOfLanes[6], const
  short LanesWithFail[6], short *TargetLaneIndex, short CurrentLaneIndex)
{
  emxArray_boolean_T *c_x;
  emxArray_int32_T *ii;
  emxArray_int32_T *r;
  emxArray_uint16_T *LaneInfos;
  emxArray_uint32_T *Neighbor2Current;
  emxArray_uint32_T *b_Neighbor2Current;
  double d;
  int i;
  int idx;
  int n;
  int nx;
  int nz;
  unsigned int u;
  short BackupTargetLaneIndex;
  short b_x;
  short d_x;
  boolean_T x[6];
  boolean_T exitg1;
  boolean_T guard1 = false;
  for (i = 0; i < 6; i++) {
    x[i] = (WidthOfLanes[i] != 0.0);
  }

  nz = x[0];
  for (idx = 0; idx < 5; idx++) {
    nz += x[idx + 1];
  }

  emxInit_uint16_T(&LaneInfos, 2);
  i = LaneInfos->size[0] * LaneInfos->size[1];
  LaneInfos->size[0] = nz;
  LaneInfos->size[1] = 2;
  emxEnsureCapacity_uint16_T(LaneInfos, i);
  n = nz << 1;
  for (i = 0; i < n; i++) {
    LaneInfos->data[i] = 0U;
  }

  /*  Failfalg Dis2Tar+Dis2Cur */
  for (idx = 0; idx < 6; idx++) {
    b_x = LanesWithFail[idx];
    if (b_x != 0) {
      LaneInfos->data[b_x - 1] = 1U;
    }
  }

  for (idx = 0; idx < nz; idx++) {
    d = ((double)idx + 1.0) - (double)*TargetLaneIndex;
    if (d < 32768.0) {
      b_x = (short)d;
    } else {
      b_x = MAX_int16_T;
    }

    d = ((double)idx + 1.0) - (double)CurrentLaneIndex;
    if (d < 32768.0) {
      d_x = (short)d;
    } else {
      d_x = MAX_int16_T;
    }

    if (b_x < 0) {
      n = -b_x;
    } else {
      n = b_x;
    }

    if (d_x < 0) {
      nx = -d_x;
    } else {
      nx = d_x;
    }

    i = n + nx;
    if (i > 32767) {
      i = 32767;
    } else {
      if (i < -32768) {
        i = -32768;
      }
    }

    LaneInfos->data[idx + LaneInfos->size[0]] = (unsigned short)i;

    /*      LaneInfos(i,2)=abs(i-TargetLaneIndex); */
    /*      LaneInfos(i,3)=abs(i-CurrentLaneIndex); */
  }

  emxInit_uint32_T(&Neighbor2Current, 2);
  emxInit_boolean_T(&c_x, 1);
  Neighbor2Current->size[0] = 0;
  Neighbor2Current->size[1] = 0;
  n = LaneInfos->size[0];
  i = c_x->size[0];
  c_x->size[0] = LaneInfos->size[0];
  emxEnsureCapacity_boolean_T(c_x, i);
  for (i = 0; i < n; i++) {
    c_x->data[i] = (LaneInfos->data[i] == 0);
  }

  nx = c_x->size[0] - 1;
  n = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (c_x->data[idx]) {
      n++;
    }
  }

  emxInit_int32_T(&ii, 1);
  i = ii->size[0];
  ii->size[0] = n;
  emxEnsureCapacity_int32_T(ii, i);
  n = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (c_x->data[idx]) {
      ii->data[n] = idx + 1;
      n++;
    }
  }

  if (ii->size[0] != 0) {
    n = LaneInfos->size[0];
    i = c_x->size[0];
    c_x->size[0] = LaneInfos->size[0];
    emxEnsureCapacity_boolean_T(c_x, i);
    for (i = 0; i < n; i++) {
      c_x->data[i] = (LaneInfos->data[i] == 0);
    }

    nx = c_x->size[0] - 1;
    n = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (c_x->data[idx]) {
        n++;
      }
    }

    emxInit_int32_T(&r, 1);
    i = r->size[0];
    r->size[0] = n;
    emxEnsureCapacity_int32_T(r, i);
    n = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (c_x->data[idx]) {
        r->data[n] = idx + 1;
        n++;
      }
    }

    n = r->size[0];
    if (r->size[0] <= 2) {
      if (r->size[0] == 1) {
        nx = LaneInfos->data[(r->data[0] + LaneInfos->size[0]) - 1];
      } else {
        nx = LaneInfos->data[(r->data[0] + LaneInfos->size[0]) - 1];
        i = LaneInfos->data[(r->data[1] + LaneInfos->size[0]) - 1];
        if (nx > i) {
          nx = i;
        }
      }
    } else {
      nx = LaneInfos->data[(r->data[0] + LaneInfos->size[0]) - 1];
      for (idx = 2; idx <= n; idx++) {
        i = LaneInfos->data[(r->data[idx - 1] + LaneInfos->size[0]) - 1];
        if (nx > i) {
          nx = i;
        }
      }
    }

    emxFree_int32_T(&r);
    n = LaneInfos->size[0];
    i = c_x->size[0];
    c_x->size[0] = LaneInfos->size[0];
    emxEnsureCapacity_boolean_T(c_x, i);
    for (i = 0; i < n; i++) {
      c_x->data[i] = ((LaneInfos->data[i + LaneInfos->size[0]] == nx) &&
                      (LaneInfos->data[i] == 0));
    }

    nx = c_x->size[0];
    idx = 0;
    i = ii->size[0];
    ii->size[0] = c_x->size[0];
    emxEnsureCapacity_int32_T(ii, i);
    n = 0;
    exitg1 = false;
    while ((!exitg1) && (n <= nx - 1)) {
      if (c_x->data[n]) {
        idx++;
        ii->data[idx - 1] = n + 1;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          n++;
        }
      } else {
        n++;
      }
    }

    if (c_x->size[0] == 1) {
      if (idx == 0) {
        ii->size[0] = 0;
      }
    } else {
      i = ii->size[0];
      if (1 > idx) {
        ii->size[0] = 0;
      } else {
        ii->size[0] = idx;
      }

      emxEnsureCapacity_int32_T(ii, i);
    }

    emxInit_uint32_T(&b_Neighbor2Current, 1);
    i = b_Neighbor2Current->size[0];
    b_Neighbor2Current->size[0] = ii->size[0];
    emxEnsureCapacity_uint32_T(b_Neighbor2Current, i);
    n = ii->size[0];
    for (i = 0; i < n; i++) {
      b_Neighbor2Current->data[i] = (unsigned int)ii->data[i];
    }

    i = Neighbor2Current->size[0] * Neighbor2Current->size[1];
    Neighbor2Current->size[0] = b_Neighbor2Current->size[0];
    Neighbor2Current->size[1] = 1;
    emxEnsureCapacity_uint32_T(Neighbor2Current, i);
    n = b_Neighbor2Current->size[0];
    for (i = 0; i < n; i++) {
      Neighbor2Current->data[i] = b_Neighbor2Current->data[i];
    }

    emxFree_uint32_T(&b_Neighbor2Current);
  }

  emxFree_int32_T(&ii);
  emxFree_boolean_T(&c_x);

  /*  Neighbor2Current=[]; */
  /*  if isempty(Neighbor2Target)==0 */
  /*      if length(Neighbor2Target)==2 */
  /*          if LaneInfos(Neighbor2Target(1),3)> LaneInfos(Neighbor2Target(2),3) */
  /*              Neighbor2Current=Neighbor2Target(2); */
  /*          elseif LaneInfos(Neighbor2Target(1),3)< LaneInfos(Neighbor2Target(2),3) */
  /*              Neighbor2Current=Neighbor2Target(1); */
  /*          else */
  /*              Neighbor2Current=Neighbor2Target; */
  /*          end */
  /*      elseif length(Neighbor2Target)==1 */
  /*          Neighbor2Current=Neighbor2Target; */
  /*      end     */
  /*  end */
  BackupTargetLaneIndex = -1;
  if ((Neighbor2Current->size[0] != 0) && (Neighbor2Current->size[1] != 0)) {
    n = Neighbor2Current->size[0];
    if (n <= 1) {
      n = 1;
    }

    guard1 = false;
    if (n == 2) {
      guard1 = true;
    } else {
      n = Neighbor2Current->size[0];
      if (n <= 1) {
        n = 1;
      }

      if (n == 1) {
        guard1 = true;
      }
    }

    if (guard1) {
      u = Neighbor2Current->data[0];
      if (Neighbor2Current->data[0] > 32767U) {
        u = 32767U;
      }

      *TargetLaneIndex = (short)u;
    }

    n = Neighbor2Current->size[0];
    if (n <= 1) {
      n = 1;
    }

    if (n == 2) {
      u = Neighbor2Current->data[1];
      if (Neighbor2Current->data[1] > 32767U) {
        u = 32767U;
      }

      BackupTargetLaneIndex = (short)u;
    }
  }

  emxFree_uint32_T(&Neighbor2Current);
  if ((BackupTargetLaneIndex == -1) && (*TargetLaneIndex > CurrentLaneIndex)) {
    i = CurrentLaneIndex - 1;
    if (CurrentLaneIndex - 1 < -32768) {
      i = -32768;
    }

    if ((short)i >= 1) {
      i = CurrentLaneIndex - 1;
      if (CurrentLaneIndex - 1 < -32768) {
        i = -32768;
      }

      if (LaneInfos->data[(short)i - 1] == 0) {
        i = CurrentLaneIndex - 1;
        if (CurrentLaneIndex - 1 < -32768) {
          i = -32768;
        }

        BackupTargetLaneIndex = (short)i;
      }
    }
  }

  if ((BackupTargetLaneIndex == -1) && (*TargetLaneIndex < CurrentLaneIndex)) {
    i = CurrentLaneIndex + 1;
    if (CurrentLaneIndex + 1 > 32767) {
      i = 32767;
    }

    if ((short)i <= nz) {
      i = CurrentLaneIndex + 1;
      if (CurrentLaneIndex + 1 > 32767) {
        i = 32767;
      }

      if (LaneInfos->data[(short)i - 1] == 0) {
        i = CurrentLaneIndex + 1;
        if (CurrentLaneIndex + 1 > 32767) {
          i = 32767;
        }

        BackupTargetLaneIndex = (short)i;
      }
    }
  }

  emxFree_uint16_T(&LaneInfos);

  /*  TargetLaneIndex ��Ŀ�공�� */
  /*  BackupTargetLaneIndex % �ֱ���Ŀ�공���������������һ��������Աܿ����ϳ�ʱ���ڣ���ʱTargetLaneIndexΪ��೵����BackupTargetLaneIndexΪ�Ҳ೵����������Ϊ-1 */
  return BackupTargetLaneIndex;
}

/*
 * Arguments    : double s_turnround_border
 *                double w_veh
 *                double R
 *                double D_safe
 *                double l_current
 *                double l_targetlane
 *                double l_boundry
 *                double dec2line
 *                double R1[2]
 *                double R2[2]
 *                double R3[2]
 *                double pos_start[2]
 *                double p1[4]
 *                double p2[4]
 *                double pos_end[2]
 * Return Type  : void
 */
static void PathPlanTurnAround(double s_turnround_border, double w_veh, double R,
  double D_safe, double l_current, double l_targetlane, double l_boundry, double
  dec2line, double R1[2], double R2[2], double R3[2], double pos_start[2],
  double p1[4], double p2[4], double pos_end[2])
{
  double a;
  double l_p1;
  double l_p2;
  double lr1;
  double lr2;
  double lr3;
  double s_p1;
  double s_p2;
  double sr1_tmp;
  double sr2;
  double sr3;
  double sr3_tmp;
  l_boundry -= dec2line;
  sr1_tmp = ((s_turnround_border - D_safe) - w_veh / 2.0) - R;
  lr1 = l_current + R;
  l_p1 = l_boundry - lr1;
  if (l_p1 > 0.0) {
    l_p1 = lr1 + l_p1 * (R / (R + w_veh / 2.0));
    a = l_p1 - lr1;
    s_p1 = sr1_tmp + sqrt(R * R - a * a);
  } else if (l_p1 < 0.0) {
    l_p1 = lr1 + l_p1 * ((R + w_veh / 2.0) / R);
    a = l_p1 - lr1;
    s_p1 = sr1_tmp + sqrt(R * R - a * a);
  } else {
    l_p1 = lr1 + R;
    s_p1 = sr1_tmp;
  }

  /*  l_p1=1.5*l_targetlane-0.5*l_current; */
  /*  l_p1=l_boundry; */
  /*  s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2); */
  /*  Dpo=max(0.2,abs(w_veh/2*sin(atan2(l_p1-lr1,s_p1-sr1)))); */
  /*  l_p1=l_p1-Dpo; */
  /*  s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2); */
  sr2 = 2.0 * s_p1 - sr1_tmp;
  lr2 = 2.0 * l_p1 - lr1;
  lr3 = l_targetlane - R;
  a = lr2 - lr3;
  sr3_tmp = R * R;
  sr3 = sr2 - sqrt(4.0 * sr3_tmp - a * a);
  a = l_targetlane - lr3;
  l_p2 = 0.5 * (lr3 - lr2) + lr2;
  s_p2 = 0.5 * (sr3 - sr2) + sr2;
  R1[0] = sr1_tmp;
  R1[1] = lr1;
  R2[0] = sr2;
  R2[1] = lr2;
  R3[0] = sr3;
  R3[1] = lr3;

  /*  theta1=-theta1; */
  p1[0] = s_p1;
  p1[1] = l_p1;
  p1[2] = atan((l_p1 - lr1) / (s_p1 - sr1_tmp));
  p1[3] = 0.0;

  /*  theta2=-theta2; */
  p2[0] = s_p2;
  p2[1] = l_p2;
  p2[2] = atan((l_p2 - lr2) / (s_p2 - sr2));
  p2[3] = 0.0;
  pos_start[0] = sr1_tmp;
  pos_start[1] = l_current;
  pos_end[0] = sr3 + sqrt(sr3_tmp - a * a);
  pos_end[1] = l_targetlane;

  /*  figure(1) */
  /*  r = R;%�뾶 */
  /*  a1 = sr1;%Բ�ĺ����� */
  /*  b1 = lr1;%Բ�������� */
  /*  theta = 0:pi/20:2*pi; %�Ƕ�[0,2*pi]  */
  /*  theta1=-pi/2:pi/20:atan((l_p1-lr1)/(s_p1-sr1)); */
  /*  x1 = a1+r*cos(theta); */
  /*  y1= b1+r*sin(theta); */
  /*  x11 = a1+r*cos(theta1); */
  /*  y11= b1+r*sin(theta1); */
  /*  plot(x1,y1,'g--',x11,y11,'r-') */
  /*  %plot(x1,y1,'g--',x1(s_start:s_p1),y1(l_start:l_p1),'b-') */
  /*  hold on */
  /*  a2 = sr2;%Բ�ĺ����� */
  /*  b2 = lr2;%Բ�������� */
  /*  theta2=(atan((l_p1-lr2)/(s_p1-sr2))+pi):pi/20:(atan((l_p2-lr2)/(s_p2-sr2))+pi); */
  /*  x2 = a2+r*cos(theta); */
  /*  y2= b2+r*sin(theta); */
  /*  x22 = a2+r*cos(theta2); */
  /*  y22= b2+r*sin(theta2); */
  /*  plot(x2,y2,'g--',x22,y22,'r-') */
  /*  hold on  */
  /*  a3 = sr3;%Բ�ĺ����� */
  /*  b3 = lr3;%Բ�������� */
  /*  theta3=atan((l_p2-lr3)/(s_p2-sr3)):pi/20:pi/2; */
  /*  x3 = a3+r*cos(theta); */
  /*  y3= b3+r*sin(theta); */
  /*  x33 = a3+r*cos(theta3); */
  /*  y33= b3+r*sin(theta3); */
  /*  plot(x3,y3,'g--',x33,y33,'r-') */
  /*  %������ */
  /*  hold on */
  /*  %Ŀ�공���� */
  /*  plot([0,s_turnround_border],[l_targetlane,l_targetlane],'r--') */
  /*  %��ǰ������ */
  /*  hold on  */
  /*  plot([0,s_turnround_border],[l_current,l_current],'r--') */
  /*  %�����߽� */
  /*  hold on */
  /*  plot([0,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_current-0.5*l_targetlane],'r-') */
  /*  hold on  */
  /*  plot([0,s_turnround_border],[1.5*l_targetlane-0.5*l_current,1.5*l_targetlane-0.5*l_current],'r-') */
  /*  hold on */
  /*  plot([0,s_turnround_border-6],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r-',[s_turnround_border-6,s_turnround_border],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r--') */
  /*  %��ͷ�߽� */
  /*  plot([s_turnround_border,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_targetlane-0.5*l_current],'r-') */
  /*  hold on */
  /*  %������ */
  /*  plot(s_start,l_start,'r*',s_end,l_end,'r*',s_p1,l_p1,'r*',s_p2,l_p2,'r*',sr1,lr1,'r.',sr2,lr2,'r.',sr3,lr3,'r.') */
  /*  axis equal */
  /*   */
}

/*
 * CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 * Arguments    : double speed
 *                double d_veh2int
 *                double d_veh2stopline
 *                double s_a
 *                double v_a
 *                double s_b
 *                double v_b
 *                double s_c
 *                double v_c
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_SpeedPlanAvoi
 *                double d_CalibrationVars_SpeedPlanAvoi
 *                double e_CalibrationVars_SpeedPlanAvoi
 *                double f_CalibrationVars_SpeedPlanAvoi
 *                double g_CalibrationVars_SpeedPlanAvoi
 *                double h_CalibrationVars_SpeedPlanAvoi
 *                const CalibACC *CalibrationVars_ACC
 *                double Parameters_l_veh
 * Return Type  : double
 */
static double SpeedPlanAvoidVehicle(double speed, double d_veh2int, double
  d_veh2stopline, double s_a, double v_a, double s_b, double v_b, double s_c,
  double v_c, TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanAvoi,
  double d_CalibrationVars_SpeedPlanAvoi, double e_CalibrationVars_SpeedPlanAvoi,
  double f_CalibrationVars_SpeedPlanAvoi, double g_CalibrationVars_SpeedPlanAvoi,
  double h_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC,
  double Parameters_l_veh)
{
  double dv[3];
  double b_speed[2];
  double a_soll;
  double b_u0;
  double d_ist;
  double s_b_end;
  double t_a2int;
  double t_b2int;
  double t_c2int;
  double u0;
  double u1;
  double u1_tmp;
  double u1_tmp_tmp;
  double v_soll;
  short b_wait;
  short dec_bre;
  short dec_fol;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T prereq4;
  dec_fol = GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle;
  dec_bre = GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle;
  b_wait = GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle;
  b_speed[0] = 1.0E-5;
  b_speed[1] = v_a;
  v_a = maximum(b_speed);
  b_speed[0] = 1.0E-5;
  b_speed[1] = v_b;
  v_b = maximum(b_speed);
  b_speed[0] = 1.0E-5;
  b_speed[1] = v_c;
  v_c = maximum(b_speed);

  /* CalibrationVariable---------------------------------------------------------------------------------------------------------------- */
  /* -1.5; */
  /* 1.5; */
  /* -3; */
  /* 40/3.6; */
  /* 1.5; */
  /* 2; */
  /* Parameters-------------------------------------------------------------------------------------------------------------------------- */
  /*  w_veh=1.8; */
  /*  t_acc=1.5; */
  d_ist = s_a;
  v_soll = v_a;

  /*  ����ģʽ�ж� */
  if ((GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle == 0) &&
      (GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle == 0) &&
      (d_veh2stopline > 0.0) && (d_veh2stopline <= (0.0 - speed * speed) / (2.0 *
        c_CalibrationVars_SpeedPlanAvoi))) {
    dec_fol = 1;
  }

  if (GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle == 0) {
    /*  if d_veh2stopline<=min([d_bre+15 20]) && d_veh2stopline>0 */
    b_speed[0] = (0.0 - speed * speed) / (2.0 * e_CalibrationVars_SpeedPlanAvoi)
      + 10.0;
    b_speed[1] = 15.0;
    if ((d_veh2stopline <= minimum(b_speed)) && (d_veh2stopline > 0.0)) {
      dec_bre = 1;
    }
  } else {
    if ((d_veh2stopline <= 0.0) ||
        (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1)) {
      dec_bre = 0;
    }
  }

  if ((dec_fol == 1) && (dec_bre == 1)) {
    dec_fol = 0;
  }

  t_a2int = (d_veh2int - s_a) / v_a;
  t_b2int = (d_veh2int - s_b) / v_b;
  t_c2int = (d_veh2int - s_c) / v_c;

  /*  �������� */
  if (dec_fol == 1) {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    if (t_a2int < t_b2int) {
      /*  a��b�� */
      s_b_end = s_a + t_b2int * v_a;

      /*  s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_b2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_b2int;

      /*  s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2); */
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_b2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_b2int;
      u1_tmp_tmp = v_b * g_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = u1_tmp_tmp / h_CalibrationVars_SpeedPlanAvoi;
      u1 = (d_veh2int + Parameters_l_veh) + u1_tmp;
      u1_tmp = (s_b_end - Parameters_l_veh) - u1_tmp;
      dv[0] = 0.0;
      dv[1] = u1_tmp_tmp;
      dv[2] = (v_a * v_a - v_b * v_b) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
      if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
          b_maximum(dv)) {
        if ((b_u0 > u1) || rtIsNaN(u1)) {
          u1 = b_u0;
        }

        if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
          u1_tmp = u0;
        }

        if (!(u1 < u1_tmp)) {
          guard4 = true;
        } else {
          /*  ǰ��=a */
        }
      } else {
        guard4 = true;
      }
    } else {
      /*  b��a�� */
      s_b_end = s_b + t_a2int * v_b;

      /*  s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2); */
      /*  s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_a2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_a2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_a2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_a2int;
      u1_tmp_tmp = v_a * g_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = u1_tmp_tmp / h_CalibrationVars_SpeedPlanAvoi;
      u1 = (d_veh2int + Parameters_l_veh) + u1_tmp;
      u1_tmp = (s_b_end - Parameters_l_veh) - u1_tmp;
      dv[0] = 0.0;
      dv[1] = u1_tmp_tmp;
      u1_tmp_tmp = v_a * v_a;
      dv[2] = (v_b * v_b - u1_tmp_tmp) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
      if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
          b_maximum(dv)) {
        if ((b_u0 > u1) || rtIsNaN(u1)) {
          u1 = b_u0;
        }

        if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
          u1_tmp = u0;
        }

        if (u1 < u1_tmp) {
          /*  ǰ��=b */
          d_ist = s_b;
          v_soll = v_b;
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    }

    if (guard4) {
      s_b_end = s_b + t_c2int * v_b;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_c2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_c2int;

      /*  b��c�� */
      u1 = (d_veh2int + Parameters_l_veh) + v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      if (t_b2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * g_CalibrationVars_SpeedPlanAvoi;
        dv[2] = (v_b * v_b - v_c * v_c) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
        if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
            b_maximum(dv)) {
          if ((b_u0 > u1) || rtIsNaN(u1)) {
            u1 = b_u0;
          }

          if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
            u1_tmp = u0;
          }

          if (u1 < u1_tmp) {
            /*  ǰ��=b */
            d_ist = s_b;
            v_soll = v_b;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    }

    if (guard3) {
      s_b_end = s_a + t_c2int * v_a;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_c2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_c2int;

      /*  a��c�� */
      u1 = (d_veh2int + Parameters_l_veh) + v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      if (t_a2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * g_CalibrationVars_SpeedPlanAvoi;
        dv[2] = (u1_tmp_tmp - v_c * v_c) / (2.0 *
          e_CalibrationVars_SpeedPlanAvoi);
        if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
            b_maximum(dv)) {
          if ((b_u0 > u1) || rtIsNaN(u1)) {
            u1 = b_u0;
          }

          if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
            u1_tmp = u0;
          }

          if (!(u1 < u1_tmp)) {
            guard1 = true;
          } else {
            /*  ǰ��=a */
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard2) {
      /*  ǰ��=c */
      d_ist = s_c;
      v_soll = v_c;
    }

    if (guard1) {
      /*  ǰ��=c */
      d_ist = s_c;
      v_soll = v_c;
    }

    /*      s_a,v_a,s_b,v_b,s_c,v_c */
    /*                      d_ist */
    /*                      v_soll */
    /*                  v_soll */
  }

  /*  ͣ������ */
  if (dec_bre == 1) {
    guard1 = false;
    guard2 = false;
    if (t_a2int < t_b2int) {
      /*  a��b�� */
      s_b_end = s_a + t_b2int * v_a;

      /*  s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2); */
      /*  s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_b2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_b2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_b2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_b2int;
      u1 = (d_veh2int + Parameters_l_veh) + v_b *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_b *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      dv[0] = 0.0;
      dv[1] = v_b * g_CalibrationVars_SpeedPlanAvoi;
      dv[2] = (v_a * v_a - v_b * v_b) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
      if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
          b_maximum(dv)) {
        if ((b_u0 > u1) || rtIsNaN(u1)) {
          u1 = b_u0;
        }

        if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
          u1_tmp = u0;
        }

        if (u1 < u1_tmp) {
          /*              prereq1 */
          /*              prereq2 */
          /*  ǰ��=a */
          d_ist = s_a;
          v_soll = v_a;
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      /*  b��a�� */
      s_b_end = s_b + t_a2int * v_b;

      /*  s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2); */
      /*  s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_a2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_a2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_a2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_a2int;
      u1 = (d_veh2int + Parameters_l_veh) + v_a *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_a *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      dv[0] = 0.0;
      dv[1] = v_a * g_CalibrationVars_SpeedPlanAvoi;
      dv[2] = (v_b * v_b - v_a * v_a) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
      if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
          b_maximum(dv)) {
        if ((b_u0 > u1) || rtIsNaN(u1)) {
          u1 = b_u0;
        }

        if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
          u1_tmp = u0;
        }

        if (u1 < u1_tmp) {
          /*  ǰ��=b */
          d_ist = s_b;
          v_soll = v_b;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard2) {
      s_b_end = s_b + t_c2int * v_b;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_c2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_c2int;

      /*  b��c�� */
      u1 = (d_veh2int + Parameters_l_veh) + v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      if (t_b2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * g_CalibrationVars_SpeedPlanAvoi;
        dv[2] = (v_b * v_b - v_c * v_c) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
        if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
            b_maximum(dv)) {
          if ((b_u0 > u1) || rtIsNaN(u1)) {
            u1 = b_u0;
          }

          if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
            u1_tmp = u0;
          }

          if (u1 < u1_tmp) {
            /*  ǰ��=b */
            d_ist = s_b;
            v_soll = v_b;
          } else {
            /*  �޽� */
            b_wait = 1;
          }
        } else {
          /*  �޽� */
          b_wait = 1;
        }
      } else {
        /*  �޽� */
        b_wait = 1;
      }
    }

    if (guard1) {
      s_b_end = s_a + t_c2int * v_a;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
      u0 = 0.5 * (minimum(b_speed) + speed) * t_c2int;
      b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_c2int;
      b_speed[1] = 0.0;
      b_u0 = 0.5 * (maximum(b_speed) + speed) * t_c2int;

      /*  a��c�� */
      u1 = (d_veh2int + Parameters_l_veh) + v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      u1_tmp = (s_b_end - Parameters_l_veh) - v_c *
        g_CalibrationVars_SpeedPlanAvoi / h_CalibrationVars_SpeedPlanAvoi;
      if (t_a2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * g_CalibrationVars_SpeedPlanAvoi;
        dv[2] = (v_a * v_a - v_c * v_c) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
        if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
            b_maximum(dv)) {
          if ((b_u0 > u1) || rtIsNaN(u1)) {
            u1 = b_u0;
          }

          if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
            u1_tmp = u0;
          }

          if (u1 < u1_tmp) {
            /*  ǰ��=a */
            d_ist = s_a;
            v_soll = v_a;
          } else {
            /*  �޽� */
            b_wait = 1;
          }
        } else {
          /*  �޽� */
          b_wait = 1;
        }
      } else {
        /*  �޽� */
        b_wait = 1;
      }
    }
  }

  /*  �𲽾��� */
  if (b_wait == 1) {
    s_b_end = s_b + t_c2int * v_b;

    /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
    /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
    b_speed[0] = speed + d_CalibrationVars_SpeedPlanAvoi * t_c2int;
    b_speed[1] = f_CalibrationVars_SpeedPlanAvoi;
    u0 = 0.5 * (minimum(b_speed) + speed) * t_c2int;
    b_speed[0] = speed + e_CalibrationVars_SpeedPlanAvoi * t_c2int;
    b_speed[1] = 0.0;
    b_u0 = 0.5 * (maximum(b_speed) + speed) * t_c2int;

    /*  b��c�� */
    u1_tmp_tmp = v_c * g_CalibrationVars_SpeedPlanAvoi;
    u1_tmp = u1_tmp_tmp / h_CalibrationVars_SpeedPlanAvoi;
    u1 = (d_veh2int + Parameters_l_veh) + u1_tmp;
    u1_tmp = (s_b_end - Parameters_l_veh) - u1_tmp;
    if ((s_a > 10.0) && (d_veh2stopline < 10.0)) {
      prereq4 = true;
    } else {
      prereq4 = false;
    }

    /*  prereq4=(s_a>10)&&(d_veh2stopline<15); */
    if (t_b2int < t_c2int) {
      dv[0] = 0.0;
      dv[1] = u1_tmp_tmp;
      dv[2] = (v_b * v_b - v_c * v_c) / (2.0 * e_CalibrationVars_SpeedPlanAvoi);
      if (((s_b_end - d_veh2int) - Parameters_l_veh) - Parameters_l_veh >
          b_maximum(dv)) {
        if ((b_u0 > u1) || rtIsNaN(u1)) {
          u1 = b_u0;
        }

        if ((u0 < u1_tmp) || rtIsNaN(u1_tmp)) {
          u1_tmp = u0;
        }

        if ((u1 < u1_tmp) && prereq4) {
          /*  ǰ��=b */
          d_ist = s_b;
          v_soll = v_b;
          b_wait = 0;
        }
      }
    }
  }

  /*  ACC�ٶȹ滮 */
  /*  a_acc=min([ACC(v_max,v_soll,d_ist,speed) ACC(v_max,v_a,s_a,speed)]); */
  u1_tmp = b_ACC(f_CalibrationVars_SpeedPlanAvoi, v_a, s_a, speed, b_wait,
                 CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                 CalibrationVars_ACC->a_min_com, CalibrationVars_ACC->tau_v_com,
                 CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                 CalibrationVars_ACC->tau_v_bre, CalibrationVars_ACC->tau_v_emg,
                 CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc);
  b_speed[0] = b_ACC(f_CalibrationVars_SpeedPlanAvoi, v_soll, d_ist, speed,
                     b_wait, CalibrationVars_ACC->a_max,
                     CalibrationVars_ACC->a_min, CalibrationVars_ACC->a_min_com,
                     CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                     CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc);
  b_speed[1] = u1_tmp;
  a_soll = minimum(b_speed);
  if (b_wait == 0) {
    if (d_veh2stopline <= 0.0) {
      b_speed[0] = a_soll;
      b_speed[1] = b_ACC(f_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed, 0,
                         CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                         CalibrationVars_ACC->a_min_com,
                         CalibrationVars_ACC->tau_v_com,
                         CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                         CalibrationVars_ACC->tau_v_bre,
                         CalibrationVars_ACC->tau_v_emg,
                         CalibrationVars_ACC->tau_d_emg,
                         CalibrationVars_ACC->t_acc);
      a_soll = minimum(b_speed);
    }
  } else {
    /*  ͣ����ͨ��״̬���ٶȹ滮 */
    b_speed[0] = 0.0;
    b_speed[1] = d_veh2stopline + 9.0;
    u1_tmp_tmp = maximum(b_speed);
    b_speed[0] = b_ACC(f_CalibrationVars_SpeedPlanAvoi, 0.0, u1_tmp_tmp, speed,
                       b_wait, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->a_min_com,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc);
    b_speed[1] = u1_tmp;
    a_soll = minimum(b_speed);
  }

  if (dec_fol == 1) {
    b_speed[0] = a_soll;
    b_speed[1] = -2.0;
    a_soll = maximum(b_speed);
  }

  GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle = dec_fol;
  GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle = dec_bre;
  GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle = b_wait;
  return a_soll;
}

/*
 * greenLight��0 ���
 *  greenLight��1 �̵�
 *  greenLight��2 �Ƶ�
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 * Arguments    : double speed
 *                double d_veh2int
 *                double s_b
 *                double v_b
 *                double greenLight
 *                double time2nextSwitch
 *                TypeGlobVars *GlobVars
 *                double Parameters_l_veh
 *                double c_CalibrationVars_SpeedPlanTraf
 *                double d_CalibrationVars_SpeedPlanTraf
 *                double e_CalibrationVars_SpeedPlanTraf
 *                double f_CalibrationVars_SpeedPlanTraf
 *                double g_CalibrationVars_SpeedPlanTraf
 *                double h_CalibrationVars_SpeedPlanTraf
 *                const CalibACC *CalibrationVars_ACC
 *                const CalibACCcust CalibrationVars_ACCcust
 * Return Type  : double
 */
static double SpeedPlanTrafficLight(double speed, double d_veh2int, double s_b,
  double v_b, double greenLight, double time2nextSwitch, TypeGlobVars *GlobVars,
  double Parameters_l_veh, double c_CalibrationVars_SpeedPlanTraf, double
  d_CalibrationVars_SpeedPlanTraf, double e_CalibrationVars_SpeedPlanTraf,
  double f_CalibrationVars_SpeedPlanTraf, double g_CalibrationVars_SpeedPlanTraf,
  double h_CalibrationVars_SpeedPlanTraf, const CalibACC *CalibrationVars_ACC,
  const CalibACCcust CalibrationVars_ACCcust)
{
  double c_speed[3];
  double b_speed[2];
  double a_soll;
  double accel;
  double d_ist;
  double d_soll;
  int decel;
  short b_wait;
  short dec_bre;
  short dec_fol;
  dec_fol = GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight;
  dec_bre = GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight;
  b_wait = GlobVars->SpeedPlanTrafficLight.wait_TrafficLight;

  /* CalibrationVariable---------------------------------------------------------------------------------------------------------------- */
  /* -1.5; */
  /* 2.5; */
  /* -3; */
  /* 14; */
  /* 30/3.6; */
  /* 1.5; */
  /* Parameters-------------------------------------------------------------------------------------------------------------------------- */
  /*  w_veh=Parameters.w_veh; */
  decel = 0;
  b_speed[0] = 0.0;
  b_speed[1] = d_veh2int;
  d_veh2int = maximum(b_speed);

  /*  ����ģʽ�ж� */
  if ((GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight == 0) &&
      (GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight == 0) && (d_veh2int >
       10.0) && (d_veh2int < (0.0 - speed * speed) / (2.0 *
        c_CalibrationVars_SpeedPlanTraf))) {
    dec_fol = 1;
  }

  if (GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight == 0) {
    if ((d_veh2int <= (0.0 - speed * speed) / (2.0 *
          e_CalibrationVars_SpeedPlanTraf) + 10.0) && (d_veh2int > 1.0)) {
      /*  1ԭΪ10 12.31�޸� */
      dec_bre = 1;
    }
  } else {
    if ((d_veh2int <= 1.0) || (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight
         == 1)) {
      /*  1ԭΪ10 12.31�޸� */
      dec_bre = 0;
    }
  }

  if ((dec_fol == 1) && (dec_bre == 1)) {
    dec_fol = 0;
  }

  /*  ���پ��� */
  if (dec_fol == 1) {
    if (greenLight == 1.0) {
      if (!(d_veh2int / speed < time2nextSwitch)) {
        decel = 1;
      }
    } else {
      decel = 1;
    }
  }

  /*  ͣ������ */
  if (dec_bre == 1) {
    if (greenLight == 1.0) {
      /*          if d_veh2int/min([0.5*(v_max_int+speed) speed])>=time2nextSwitch */
      if (d_veh2int / speed >= time2nextSwitch) {
        b_wait = 1;
      }
    } else {
      b_wait = 1;
    }
  }

  /*  �𲽾��� */
  if ((b_wait == 1) && (greenLight == 1.0) && (d_veh2int < 30.0)) {
    /*  12.31�޸� */
    b_speed[0] = ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, 0.0,
                     CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                     CalibrationVars_ACC->a_min_com,
                     CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                     CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc);
    b_speed[1] = d_CalibrationVars_SpeedPlanTraf;
    d_soll = minimum(b_speed);
    b_speed[0] = speed + d_soll * time2nextSwitch;
    b_speed[1] = g_CalibrationVars_SpeedPlanTraf;
    if (0.5 * (minimum(b_speed) + speed) * time2nextSwitch > d_veh2int) {
      b_wait = 0;

      /*  12.31�޸� */
      /*  decel=0; */
      /*  dec_fol=0; */
      /*  dec_bre=0; */
    }
  }

  /*  ACC�ٶȹ滮 */
  if ((dec_fol == 0) && (dec_bre == 0) && (d_veh2int > 10.0) && (b_wait == 0)) {
    a_soll = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, 0,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->a_min_com,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc);
  } else if (decel == 1) {
    d_ist = (d_veh2int + 3.5) + Parameters_l_veh;

    /* 4; */
    /* 2; */
    /* 5; */
    accel = 100.0;
    if (d_ist < 100.0) {
      c_speed[0] = speed * h_CalibrationVars_SpeedPlanTraf;
      c_speed[1] = 11.0;
      c_speed[2] = (0.0 - speed * speed) / (2.0 *
        c_CalibrationVars_SpeedPlanTraf);
      d_soll = b_maximum(c_speed);
      if (fabs(speed) < 2.0) {
        accel = ((0.0 - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACCcust.tau_d) /
          CalibrationVars_ACCcust.tau_v_com;
      } else {
        accel = ((0.0 - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACCcust.tau_d) / CalibrationVars_ACCcust.tau_v;
      }
    }

    b_speed[0] = -2.5;
    b_speed[1] = (g_CalibrationVars_SpeedPlanTraf - speed) /
      CalibrationVars_ACCcust.tau_v;
    d_soll = maximum(b_speed);
    b_speed[0] = accel;
    b_speed[1] = d_soll;
    accel = minimum(b_speed);
    b_speed[0] = accel;
    b_speed[1] = d_CalibrationVars_SpeedPlanTraf;
    accel = minimum(b_speed);
    b_speed[0] = accel;
    b_speed[1] = c_CalibrationVars_SpeedPlanTraf;
    accel = maximum(b_speed);
    b_speed[0] = b_ACC(g_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->a_min_com,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc);
    b_speed[1] = accel;
    a_soll = minimum(b_speed);
  } else if (b_wait == 1) {
    b_speed[0] = b_ACC(g_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, 1,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->a_min_com,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc);
    b_speed[1] = b_ACC(g_CalibrationVars_SpeedPlanTraf, 0.0, (d_veh2int + 3.5) +
                       Parameters_l_veh, speed, 1, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->a_min_com,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc);
    a_soll = minimum(b_speed);

    /*  a_soll */
    /*  greenLight */
  } else {
    a_soll = b_ACC(g_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->a_min_com,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc);
  }

  GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight = dec_fol;
  GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight = dec_bre;
  GlobVars->SpeedPlanTrafficLight.wait_TrafficLight = b_wait;
  return a_soll;
}

/*
 * ,
 * Arguments    : double CurrentLaneFrontDis
 *                double CurrentLaneFrontVel
 *                double LeftLaneBehindDis
 *                double LeftLaneBehindVel
 *                double LeftLaneFrontDis
 *                double LeftLaneFrontVel
 *                double RightLaneBehindDis
 *                double RightLaneBehindVel
 *                double RightLaneFrontDis
 *                double RightLaneFrontVel
 *                double speed
 *                double pos_s
 *                double pos_l_CurrentLane
 *                short CurrentLaneIndex
 *                short TargetLaneIndex
 *                short BackupTargetLaneIndex
 *                double d_veh2int
 *                const double WidthOfLanes[6]
 *                double v_max
 *                const short LanesWithFail[6]
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 *                double *a_soll
 *                double traj_s[80]
 *                double traj_l[80]
 *                double traj_psi[80]
 *                double traj_vs[80]
 *                double traj_vl[80]
 *                double traj_omega[80]
 * Return Type  : void
 */
static void TrajPlanLaneChange(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double LeftLaneBehindDis, double LeftLaneBehindVel,
  double LeftLaneFrontDis, double LeftLaneFrontVel, double RightLaneBehindDis,
  double RightLaneBehindVel, double RightLaneFrontDis, double RightLaneFrontVel,
  double speed, double pos_s, double pos_l_CurrentLane, short CurrentLaneIndex,
  short TargetLaneIndex, short BackupTargetLaneIndex, double d_veh2int, const
  double WidthOfLanes[6], double v_max, const short LanesWithFail[6],
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters, double *a_soll, double traj_s[80], double traj_l[80],
  double traj_psi[80], double traj_vs[80], double traj_vl[80], double
  traj_omega[80])
{
  b_anonymous_function e_this_tunableEnvironment_f1_tu[1];
  cell_wrap_3 fun_S_tunableEnvironment[1];
  double traj[720];
  double x1[100];
  double y[100];
  double S_traj[80];
  double V_traj[80];
  double X_traj[80];
  double dv1[16];
  double dv[9];
  double para_ST[4];
  double b_S_b_end[3];
  double b_speed[2];
  double c_this_tunableEnvironment_f1_tu[1];
  double d_this_tunableEnvironment_f1_tu[1];
  double S_b_end;
  double S_c_end;
  double S_end;
  double S_min_dyn;
  double S_tlc;
  double TargetLaneBehindDis;
  double TargetLaneBehindVel;
  double TargetLaneFrontDis;
  double TargetLaneFrontVel;
  double V_c_end;
  double V_end;
  double a;
  double d;
  double s_d;
  double s_e;
  double speed_tmp;
  double t_lc;
  double t_lc_traj;
  double t_mid;
  double v_d;
  double v_e;
  double w_lane;
  double w_lane_left;
  double w_lane_right;
  int SwitchACC;
  int b_wait;
  int count_1;
  int i;
  int i3;
  int traj_tmp;
  short b_CurrentLaneIndex[2];
  short CountLaneChange;
  short DurationLaneChange;
  short count_2;
  short i1;
  short i2;
  short i4;
  short i5;
  short i6;
  short i7;
  short i8;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T prereq5;

  /* , */
  /* globalVariable---------------------------------------------------------------------------------------------------------------------- */
  CountLaneChange = GlobVars->TrajPlanLaneChange.CountLaneChange;
  DurationLaneChange = GlobVars->TrajPlanLaneChange.DurationLaneChange;
  t_lc_traj = GlobVars->TrajPlanLaneChange.t_lc_traj;
  i = CurrentLaneIndex - 1;
  if (CurrentLaneIndex - 1 < -32768) {
    i = -32768;
  }

  S_tlc = 0.5 * WidthOfLanes[CurrentLaneIndex - 1];
  if ((short)i < 1) {
    i = 1;
  } else {
    i = (short)i;
  }

  w_lane_left = 0.5 * WidthOfLanes[i - 1] + S_tlc;
  if ((short)(CurrentLaneIndex + 1) > 6) {
    b_wait = 6;
  } else {
    b_wait = (short)(CurrentLaneIndex + 1);
  }

  w_lane_right = 0.5 * WidthOfLanes[b_wait - 1] + S_tlc;

  /*  w_lane=3.2; */
  if (TargetLaneIndex <= CurrentLaneIndex) {
    TargetLaneBehindDis = LeftLaneBehindDis;
    TargetLaneBehindVel = LeftLaneBehindVel;
    TargetLaneFrontDis = LeftLaneFrontDis;
    TargetLaneFrontVel = LeftLaneFrontVel;
  } else {
    TargetLaneBehindDis = RightLaneBehindDis;
    TargetLaneBehindVel = RightLaneBehindVel;
    TargetLaneFrontDis = RightLaneFrontDis;
    TargetLaneFrontVel = RightLaneFrontVel;
  }

  if (BackupTargetLaneIndex != -1) {
    s_d = RightLaneBehindDis;
    v_d = RightLaneBehindVel;
    s_e = RightLaneFrontDis;
    v_e = RightLaneFrontVel;
    b_CurrentLaneIndex[0] = (short)(CurrentLaneIndex + 1);
    b_CurrentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = c_minimum(b_CurrentLaneIndex);
    b_CurrentLaneIndex[0] = (short)(CurrentLaneIndex - 1);
    b_CurrentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = c_maximum(b_CurrentLaneIndex);
  } else {
    s_d = -200.0;
    v_d = -20.0;
    s_e = 200.0;
    v_e = 20.0;
  }

  /*  TargetLaneIndex=TargetLaneIndex-1; */
  /*  CurrentLaneIndex=CurrentLaneIndex-1; */
  /* 30/3.6; */
  /* 1; */
  /* 3; */
  /* 0.5; */
  /* 0.5; */
  /* 1; */
  /* -3.5; */
  /* 2.5; */
  /* -1; */
  *a_soll = 100.0;

  /*  t_lc=max([2-0.05*(speed-10) 1.7]); */
  /*  t_lc=min([t_lc 2.3]); */
  b_speed[0] = 2.0 - 0.04 * (speed - 15.0);
  b_speed[1] = 2.0;
  d = maximum(b_speed);
  b_speed[0] = d;
  b_speed[1] = 2.5;
  t_lc = ceil(minimum(b_speed) / 0.1) * 0.1;
  memset(&traj[0], 0, 720U * sizeof(double));

  /*  w_veh=1.8; */
  b_CurrentLaneIndex[0] = (short)(CurrentLaneIndex + 1);
  b_CurrentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = c_minimum(b_CurrentLaneIndex);
  b_CurrentLaneIndex[0] = (short)(CurrentLaneIndex - 1);
  b_CurrentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = c_maximum(b_CurrentLaneIndex);
  memset(&S_traj[0], 0, 80U * sizeof(double));
  memset(&X_traj[0], 0, 80U * sizeof(double));
  memset(&V_traj[0], 0, 80U * sizeof(double));
  b_wait = 0;
  for (SwitchACC = 0; SwitchACC < 6; SwitchACC++) {
    if (CurrentLaneIndex == LanesWithFail[SwitchACC]) {
      b_wait = -1;
    }
  }

  /*  Lane change decision */
  S_end = 0.0;

  /* 2020324,����c�������ӳ�ʼֵ */
  V_end = 0.0;
  if ((GlobVars->TrajPlanLaneChange.CountLaneChange == 0) && (CurrentLaneIndex
       != TargetLaneIndex) && (CurrentLaneIndex != BackupTargetLaneIndex)) {
    /*      if TargetLaneIndex<=CurrentLaneIndex */
    /*          w_lane=w_lane_left; */
    /*      else */
    /*          w_lane=w_lane_right; */
    /*      end */
    w_lane = S_tlc + 0.5 * WidthOfLanes[TargetLaneIndex - 1];

    /* 20220328 */
    if (speed < 5.0) {
      b_speed[0] = 10.0;
      b_speed[1] = t_lc * speed;
      S_end = maximum(b_speed);
      d_this_tunableEnvironment_f1_tu[0] = speed;
      fzero(d_this_tunableEnvironment_f1_tu, S_end, w_lane, &S_b_end, &S_tlc,
            &S_min_dyn);
      b_speed[0] = t_lc;
      b_speed[1] = S_b_end;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      b_speed[1] = (0.0 - speed * speed) / (2.0 *
        CalibrationVars->TrajPlanLaneChange.a_min);
      a = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_min_dyn = maximum(b_speed);
      speed_tmp = w_lane * w_lane;
      b_speed[0] = sqrt(a * a - speed_tmp);
      b_speed[1] = sqrt(S_min_dyn * S_min_dyn - speed_tmp);
      S_min_dyn = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_speed[0] = ACC(v_max, TargetLaneFrontVel, TargetLaneFrontDis -
                       TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.a_min_com,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      d = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + d *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (TargetLaneFrontVel +
        TargetLaneBehindVel) * t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel_strich*a_max_comfort)*t_lc; */
      /* , */
      d = TargetLaneBehindVel * t_lc;
      guard1 = false;
      if (-TargetLaneBehindDis > d) {
        a = speed * t_lc;
        if (TargetLaneFrontDis <= a) {
          /*                  b�������� c������ */
          /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
          V_end = sqrt(S_end * S_end + speed_tmp) / t_lc * 2.0 - speed;
          b_speed[0] = TargetLaneFrontVel - V_end;
          b_speed[1] = 0.0;
          b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          TargetLaneBehindVel = V_end * V_end;
          b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                           CalibrationVars->TrajPlanLaneChange.t_re) +
                          (TargetLaneBehindVel - TargetLaneFrontVel *
                           TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
          b_S_b_end[2] = S_min_dyn;
          w_lane = b_maximum(b_S_b_end);
          a += 0.5 * CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
          S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = S_min_dyn - Parameters.l_veh;
          b_S_b_end[1] = (S_min_dyn - (V_c_end * V_c_end - TargetLaneBehindVel) /
                          (2.0 * CalibrationVars->TrajPlanLaneChange.a_min)) -
            Parameters.l_veh;
          b_S_b_end[2] = sqrt(a * a - speed_tmp);
          TargetLaneBehindVel = b_minimum(b_S_b_end);
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        if ((-TargetLaneBehindDis <= d) && (TargetLaneFrontDis > speed * t_lc))
        {
          /* , */
          /*                  b������ c�������� */
          /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
          V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
          b_speed[0] = TargetLaneFrontVel - V_end;
          b_speed[1] = 0.0;
          b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
          b_S_b_end[2] = S_min_dyn;
          w_lane = b_maximum(b_S_b_end);
          a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
            t_lc * t_lc;
          b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                          V_end) - Parameters.l_veh;
          b_S_b_end[1] = ((S_c_end - V_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
            V_c_end - V_end * V_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
          b_S_b_end[2] = sqrt(a * a - speed_tmp);
          TargetLaneBehindVel = b_minimum(b_S_b_end);
        } else if ((-TargetLaneBehindDis > d) && (TargetLaneFrontDis > speed *
                    t_lc)) {
          /* , */
          /*                  b�������� c�������� */
          V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
          b_speed[0] = TargetLaneFrontVel - V_end;
          b_speed[1] = 0.0;
          b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
          b_S_b_end[2] = S_min_dyn;
          w_lane = b_maximum(b_S_b_end);
          a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
            t_lc * t_lc;
          b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                          V_end) - Parameters.l_veh;
          b_S_b_end[1] = ((S_c_end - V_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
            V_c_end - V_end * V_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
          b_S_b_end[2] = sqrt(a * a - speed_tmp);
          TargetLaneBehindVel = b_minimum(b_S_b_end);
        } else {
          /*                  b������ c������ */
          /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
          V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
          b_speed[0] = TargetLaneFrontVel - V_end;
          b_speed[1] = 0.0;
          b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
          b_S_b_end[2] = S_min_dyn;
          w_lane = b_maximum(b_S_b_end);
          a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
            t_lc * t_lc;
          b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                          V_end) - Parameters.l_veh;
          b_S_b_end[1] = ((S_c_end - V_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
            V_c_end - V_end * V_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
          b_S_b_end[2] = sqrt(a * a - speed_tmp);
          TargetLaneBehindVel = b_minimum(b_S_b_end);
        }
      }

      if ((TargetLaneBehindVel > S_end) && (S_end > w_lane)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  ����·�ڹ���ʱ���������� */
      if (((S_c_end - S_b_end) - Parameters.l_veh) - Parameters.l_veh >
          TargetLaneFrontVel * CalibrationVars->TrajPlanLaneChange.t_re +
          (V_c_end * V_c_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
           CalibrationVars->TrajPlanLaneChange.a_min)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_lc > 0.5 * ((S_end + Parameters.l_veh) + Parameters.l_veh)) &&
            (V_end > speed + CalibrationVars->TrajPlanLaneChange.a_min * t_lc) &&
            (V_end < speed + CalibrationVars->TrajPlanLaneChange.a_max * t_lc) &&
            prereq5 && (d_veh2int >= S_end +
                        CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int
                        * Parameters.l_veh) && (TargetLaneBehindDis <=
             -Parameters.l_veh) && (TargetLaneFrontDis >= Parameters.l_veh)) {
          b_speed[0] = 0.0;
          b_speed[1] = CurrentLaneFrontVel +
            CalibrationVars->TrajPlanLaneChange.index_accel *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
          if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) +
                CurrentLaneFrontVel) * t_mid > 0.5 * S_end + Parameters.l_veh) &&
              (CurrentLaneFrontDis >= Parameters.l_veh + 0.25 * t_lc * (0.75 *
                speed + 0.25 * V_end))) {
            CountLaneChange = 1;
            GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
              TargetLaneIndex;
          }
        }
      }
    } else {
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_speed[0] = ACC(v_max, TargetLaneFrontVel, TargetLaneFrontDis -
                       TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.a_min_com,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      d = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + d *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (TargetLaneFrontVel +
        TargetLaneBehindVel) * t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel*index_accel_strich)*t_lc; */
      /* , */
      d = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > d) && (TargetLaneFrontDis <= speed * t_lc)) {
        /*                  b�������� c������ */
        b_speed[0] = 0.5 * (speed + V_c_end);
        b_speed[1] = speed;
        V_end = minimum(b_speed);
        TargetLaneBehindVel = S_b_end + TargetLaneFrontVel *
          CalibrationVars->TrajPlanLaneChange.t_re;
        b_S_b_end[0] = TargetLaneBehindVel + Parameters.l_veh;
        S_min_dyn = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = (TargetLaneBehindVel + (V_end * V_end -
          TargetLaneFrontVel * TargetLaneFrontVel) / S_min_dyn) +
          Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);
        TargetLaneBehindVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re
          * V_end;
        b_S_b_end[0] = TargetLaneBehindVel - Parameters.l_veh;
        b_S_b_end[1] = (TargetLaneBehindVel - (V_c_end * V_c_end - V_end * V_end)
                        / S_min_dyn) - Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneBehindVel = b_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = TargetLaneBehindVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= d) && (TargetLaneFrontDis > speed *
                  t_lc)) {
        /* , */
        /*                  b������ c�������� */
        b_speed[0] = 0.5 * (speed + TargetLaneFrontVel);
        b_speed[1] = speed;
        V_end = minimum(b_speed);
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);
        TargetLaneBehindVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re
          * V_end;
        b_S_b_end[0] = TargetLaneBehindVel - Parameters.l_veh;
        b_S_b_end[1] = (TargetLaneBehindVel - (V_c_end * V_c_end - V_end * V_end)
                        / (2.0 * CalibrationVars->TrajPlanLaneChange.a_min)) -
          Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneBehindVel = b_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = TargetLaneBehindVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else {
        guard1 = false;
        if (-TargetLaneBehindDis > d) {
          S_end = speed * t_lc;
          if (TargetLaneFrontDis > S_end) {
            /* , */
            /*                  b�������� c�������� */
            V_end = speed;
            b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            TargetLaneBehindVel = speed * speed;
            S_min_dyn = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                             CalibrationVars->TrajPlanLaneChange.t_re) +
                            (TargetLaneBehindVel - TargetLaneFrontVel *
                             TargetLaneFrontVel) / S_min_dyn) + Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            w_lane = b_maximum(b_S_b_end);
            S_tlc = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
            b_S_b_end[0] = S_tlc - Parameters.l_veh;
            b_S_b_end[1] = (S_tlc - (V_c_end * V_c_end - TargetLaneBehindVel) /
                            S_min_dyn) - Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            TargetLaneBehindVel = b_minimum(b_S_b_end);
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          /*                  b������ c������ */
          b_speed[0] = speed;
          b_speed[1] = 0.5 * (TargetLaneFrontVel + V_c_end);
          V_end = minimum(b_speed);
          b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          w_lane = b_maximum(b_S_b_end);
          b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                          V_end) - Parameters.l_veh;
          b_S_b_end[1] = ((S_c_end - V_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
            V_c_end - V_end * V_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          TargetLaneBehindVel = b_minimum(b_S_b_end);
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = TargetLaneBehindVel;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        }
      }

      /*  �μ�TrajPlanLaneChange_S_max_withAccel.bmp */
      b_speed[0] = V_end;
      b_speed[1] = speed;
      S_min_dyn = fabs(V_end - speed);
      a = (CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc - S_min_dyn)
        / 2.0;

      /*  ����������ֻ�����ȼ��ٺ����ٻ�ֻ�����ȼ��ٺ����� -> ���λ��  */
      S_tlc = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_min_dyn = (S_tlc - S_min_dyn) / 2.0;

      /*  ����������ֻ�����ȼ��ٺ����ٻ�ֻ�����ȼ��ٺ����� -> ��Сλ�� */
      if (TargetLaneBehindVel >= w_lane) {
        d = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        if ((S_end <= (t_lc * minimum(b_speed) + d) - a * a /
             CalibrationVars->TrajPlanLaneChange.a_max_comfort) && (S_end >=
             (t_lc * maximum(b_speed) - d) + S_min_dyn * S_min_dyn /
             CalibrationVars->TrajPlanLaneChange.a_max_comfort)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  ����·�ڹ���ʱ���������� */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % �ٶȽϵ�ʱ�����·�ڹ���ʱ���������� */
      if (((S_c_end - S_b_end) - Parameters.l_veh) - Parameters.l_veh >
          TargetLaneFrontVel * CalibrationVars->TrajPlanLaneChange.t_re +
          (V_c_end * V_c_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
           CalibrationVars->TrajPlanLaneChange.a_min)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_lc > 0.5 * ((S_end + Parameters.l_veh) + Parameters.l_veh)) &&
            (V_end >= speed + CalibrationVars->TrajPlanLaneChange.a_min_comfort *
             t_lc) && (V_end <= speed + S_tlc) && prereq5 && (d_veh2int >= S_end
             + CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
             Parameters.l_veh) && (TargetLaneBehindDis <= -Parameters.l_veh) &&
            (TargetLaneFrontDis >= Parameters.l_veh)) {
          b_speed[0] = 0.0;
          b_speed[1] = CurrentLaneFrontVel +
            CalibrationVars->TrajPlanLaneChange.index_accel *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
          if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) +
                CurrentLaneFrontVel) * t_mid > 0.5 * S_end + Parameters.l_veh) &&
              (CurrentLaneFrontDis >= Parameters.l_veh + 0.25 * t_lc * (0.75 *
                speed + 0.25 * V_end))) {
            CountLaneChange = 1;
            GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
              TargetLaneIndex;
          }
        }
      }
    }
  }

  if ((CountLaneChange == 0) && (CurrentLaneIndex != TargetLaneIndex) &&
      (CurrentLaneIndex != BackupTargetLaneIndex) && (-1 !=
       BackupTargetLaneIndex)) {
    if (BackupTargetLaneIndex <= CurrentLaneIndex) {
      w_lane = w_lane_left;
    } else {
      w_lane = w_lane_right;
    }

    if (speed < 5.0) {
      b_speed[0] = 10.0;
      b_speed[1] = t_lc * speed;
      S_end = maximum(b_speed);
      c_this_tunableEnvironment_f1_tu[0] = speed;
      fzero(c_this_tunableEnvironment_f1_tu, S_end, w_lane, &S_b_end, &S_tlc,
            &S_min_dyn);
      b_speed[0] = t_lc;
      b_speed[1] = S_b_end;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      TargetLaneBehindDis = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_speed[1] = (0.0 - speed * speed) / TargetLaneBehindDis;
      a = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_min_dyn = maximum(b_speed);
      S_tlc = w_lane * w_lane;
      b_speed[0] = sqrt(a * a - S_tlc);
      b_speed[1] = sqrt(S_min_dyn * S_min_dyn - S_tlc);
      S_min_dyn = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_e + CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = s_e + 0.5 * (V_c_end + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.a_min_com,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      d = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + d * CalibrationVars->TrajPlanLaneChange.a_max_comfort *
        t_lc;
      S_b_end = maximum(b_speed);
      TargetLaneFrontVel = s_d + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      d = v_d * t_lc;
      if ((-s_d > d) && (s_e <= speed * t_lc)) {
        /*                  b�������� c������ */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / TargetLaneBehindDis) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);
        a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        TargetLaneBehindVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re
          * V_end;
        b_S_b_end[0] = TargetLaneBehindVel - Parameters.l_veh;
        b_S_b_end[1] = (TargetLaneBehindVel - (V_c_end * V_c_end - V_end * V_end)
                        / TargetLaneBehindDis) - Parameters.l_veh;
        b_S_b_end[2] = sqrt(a * a - S_tlc);
        TargetLaneBehindVel = b_minimum(b_S_b_end);
      } else if ((-s_d <= d) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b������ c�������� */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / TargetLaneBehindDis) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);
        a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                        V_end) - Parameters.l_veh;
        b_S_b_end[1] = ((S_c_end - V_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
        b_S_b_end[2] = sqrt(a * a - S_tlc);
        TargetLaneBehindVel = b_minimum(b_S_b_end);
      } else if ((-s_d > d) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b�������� c�������� */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);
        a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                        V_end) - Parameters.l_veh;
        b_S_b_end[1] = ((S_c_end - V_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) - Parameters.l_veh;
        b_S_b_end[2] = sqrt(a * a - S_tlc);
        TargetLaneBehindVel = b_minimum(b_S_b_end);
      } else {
        /*                  b������ c������ */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / TargetLaneBehindDis) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);
        a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                        V_end) - Parameters.l_veh;
        b_S_b_end[1] = ((S_c_end - V_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / TargetLaneBehindDis) - Parameters.l_veh;
        b_S_b_end[2] = sqrt(a * a - S_tlc);
        TargetLaneBehindVel = b_minimum(b_S_b_end);
      }

      if ((TargetLaneBehindVel > S_end) && (S_end > w_lane)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  ����·�ڹ���ʱ���������� */
      if (((S_c_end - TargetLaneFrontVel) - Parameters.l_veh) - Parameters.l_veh
          > S_b_end * CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end *
           V_c_end - S_b_end * S_b_end) / (2.0 *
           CalibrationVars->TrajPlanLaneChange.a_min)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_lc > 0.5 * ((S_end + Parameters.l_veh) + Parameters.l_veh)) &&
            (V_end > speed + CalibrationVars->TrajPlanLaneChange.a_min * t_lc) &&
            (V_end < speed + CalibrationVars->TrajPlanLaneChange.a_max * t_lc) &&
            prereq5 && (d_veh2int >= S_end +
                        CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int
                        * Parameters.l_veh) && (s_d <= -Parameters.l_veh) &&
            (s_e >= Parameters.l_veh)) {
          b_speed[0] = 0.0;
          b_speed[1] = CurrentLaneFrontVel +
            CalibrationVars->TrajPlanLaneChange.index_accel *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
          if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) +
                CurrentLaneFrontVel) * t_mid > 0.5 * S_end + Parameters.l_veh) &&
              (CurrentLaneFrontDis >= Parameters.l_veh + 0.25 * t_lc * (0.75 *
                speed + 0.25 * V_end))) {
            CountLaneChange = 1;
            TargetLaneIndex = BackupTargetLaneIndex;
            GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
              BackupTargetLaneIndex;
          }
        }
      }
    } else {
      b_speed[0] = 0.0;
      TargetLaneBehindDis = CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort;
      speed_tmp = TargetLaneBehindDis * t_lc;
      b_speed[1] = v_e + speed_tmp;
      V_c_end = maximum(b_speed);
      S_c_end = s_e + 0.5 * (V_c_end + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.a_min_com,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      d = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + d * CalibrationVars->TrajPlanLaneChange.a_max_comfort *
        t_lc;
      S_b_end = maximum(b_speed);
      TargetLaneFrontVel = s_d + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      d = v_d * t_lc;
      guard1 = false;
      guard2 = false;
      if (-s_d > d) {
        a = speed * t_lc;
        if (s_e <= a) {
          /*                  b�������� c������ */
          b_speed[0] = 0.5 * (speed + V_c_end);
          b_speed[1] = speed;
          V_end = minimum(b_speed);
          TargetLaneBehindVel = TargetLaneFrontVel + S_b_end *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = TargetLaneBehindVel + Parameters.l_veh;
          S_min_dyn = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = (TargetLaneBehindVel + (V_end * V_end - S_b_end *
            S_b_end) / S_min_dyn) + Parameters.l_veh;
          b_S_b_end[2] = a + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          w_lane = b_maximum(b_S_b_end);
          TargetLaneBehindVel = S_c_end -
            CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = TargetLaneBehindVel - Parameters.l_veh;
          b_S_b_end[1] = (TargetLaneBehindVel - (V_c_end * V_c_end - V_end *
            V_end) / S_min_dyn) - Parameters.l_veh;
          b_S_b_end[2] = a + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          TargetLaneBehindVel = b_minimum(b_S_b_end);
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = TargetLaneBehindVel;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        if ((-s_d <= d) && (s_e > speed * t_lc)) {
          /* , */
          /*                  b������ c�������� */
          b_speed[0] = 0.5 * (speed + S_b_end);
          b_speed[1] = speed;
          V_end = minimum(b_speed);
          b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          TargetLaneBehindVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = ((TargetLaneFrontVel + S_b_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - S_b_end * S_b_end) / TargetLaneBehindVel) + Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          w_lane = b_maximum(b_S_b_end);
          b_S_b_end[0] = (S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
                          V_end) - Parameters.l_veh;
          b_S_b_end[1] = ((S_c_end - V_end *
                           CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
            V_c_end - V_end * V_end) / TargetLaneBehindVel) - Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          TargetLaneBehindVel = b_minimum(b_S_b_end);
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = TargetLaneBehindVel;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else if (-s_d > d) {
          S_end = speed * t_lc;
          if (s_e > S_end) {
            /* , */
            /*                  b�������� c�������� */
            V_end = speed;
            b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            TargetLaneBehindVel = speed * speed;
            S_min_dyn = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((TargetLaneFrontVel + S_b_end *
                             CalibrationVars->TrajPlanLaneChange.t_re) +
                            (TargetLaneBehindVel - S_b_end * S_b_end) /
                            S_min_dyn) + Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            w_lane = b_maximum(b_S_b_end);
            S_tlc = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
            b_S_b_end[0] = S_tlc - Parameters.l_veh;
            b_S_b_end[1] = (S_tlc - (V_c_end * V_c_end - TargetLaneBehindVel) /
                            S_min_dyn) - Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            TargetLaneBehindVel = b_minimum(b_S_b_end);
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /*                  b������ c������ */
        b_speed[0] = speed;
        b_speed[1] = 0.5 * (S_b_end + V_c_end);
        V_end = minimum(b_speed);
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + S_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);
        TargetLaneBehindVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re
          * V_end;
        b_S_b_end[0] = TargetLaneBehindVel - Parameters.l_veh;
        b_S_b_end[1] = (TargetLaneBehindVel - (V_c_end * V_c_end - V_end * V_end)
                        / (2.0 * CalibrationVars->TrajPlanLaneChange.a_min)) -
          Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneBehindVel = b_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = TargetLaneBehindVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      }

      b_speed[0] = V_end;
      b_speed[1] = speed;
      S_tlc = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_min_dyn = (S_tlc - fabs(V_end - speed)) / 2.0;

      /*  ����������ֻ�����ȼ��ٺ����ٻ�ֻ�����ȼ��ٺ����� -> ���λ�� */
      /*  ����������ֻ�����ȼ��ٺ����ٻ�ֻ�����ȼ��ٺ����� -> ��Сλ�� */
      if (TargetLaneBehindVel >= w_lane) {
        d = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        a = S_min_dyn * S_min_dyn /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * minimum(b_speed) + d) - a) && (S_end >= (t_lc *
              maximum(b_speed) - d) + a)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  ����·�ڹ���ʱ���������� */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % �ٶȽϵ�ʱ�����·�ڹ���ʱ���������� */
      if (((S_c_end - TargetLaneFrontVel) - Parameters.l_veh) - Parameters.l_veh
          > S_b_end * CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end *
           V_c_end - S_b_end * S_b_end) / (2.0 *
           CalibrationVars->TrajPlanLaneChange.a_min)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel + speed_tmp;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_lc > 0.5 * ((S_end + Parameters.l_veh) + Parameters.l_veh)) &&
            (V_end >= speed + CalibrationVars->TrajPlanLaneChange.a_min_comfort *
             t_lc) && (V_end <= speed + S_tlc) && prereq5 && (d_veh2int >= S_end
             + CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
             Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (s_e >=
             Parameters.l_veh)) {
          b_speed[0] = 0.0;
          b_speed[1] = CurrentLaneFrontVel + TargetLaneBehindDis * t_mid;
          if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) +
                CurrentLaneFrontVel) * t_mid > 0.5 * S_end + Parameters.l_veh) &&
              (CurrentLaneFrontDis >= Parameters.l_veh + 0.25 * t_lc * (0.75 *
                speed + 0.25 * V_end))) {
            CountLaneChange = 1;
            TargetLaneIndex = BackupTargetLaneIndex;
            GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
              BackupTargetLaneIndex;
          }
        }
      }
    }
  }

  /*  para=[]; */
  /*  path generation */
  if (CountLaneChange == 1) {
    t_lc_traj = t_lc;
    dv[0] = 3.0;
    dv[3] = 4.0 * S_end;
    dv[6] = 5.0 * S_end * S_end;
    dv[1] = 6.0;
    dv[4] = 12.0 * S_end;
    dv[7] = 20.0 * S_end * S_end;
    dv[2] = rt_powd_snf(S_end, 3.0);
    dv[5] = rt_powd_snf(S_end, 4.0);
    dv[8] = rt_powd_snf(S_end, 5.0);
    b_S_b_end[0] = 0.0;
    b_S_b_end[1] = 0.0;
    if (TargetLaneIndex <= CurrentLaneIndex) {
      w_lane_right = w_lane_left;
    }

    b_S_b_end[2] = w_lane_right * 1.0E+6;
    mldivide(dv, b_S_b_end, fun_S_tunableEnvironment[0].f1);

    /*  SpeedPlanner */
    linspace(S_end, x1);
    a = 3.0 * fun_S_tunableEnvironment[0].f1[0];
    S_min_dyn = 4.0 * fun_S_tunableEnvironment[0].f1[1];
    S_tlc = 5.0 * fun_S_tunableEnvironment[0].f1[2];
    for (SwitchACC = 0; SwitchACC < 100; SwitchACC++) {
      d = x1[SwitchACC];
      d = ((a * (d * d) + S_min_dyn * rt_powd_snf(d, 3.0)) + S_tlc * rt_powd_snf
           (d, 4.0)) / 1.0E+6;
      y[SwitchACC] = sqrt(d * d + 1.0);
    }

    S_tlc = trapz(x1, y);
    para_ST[0] = 0.0;
    para_ST[1] = speed;
    para_ST[2] = S_tlc;
    para_ST[3] = V_end;
    dv1[0] = 1.0;
    dv1[1] = 0.0;
    dv1[4] = 0.0;
    dv1[5] = 1.0;
    dv1[8] = 0.0;
    dv1[9] = 0.0;
    dv1[12] = 0.0;
    dv1[13] = 0.0;
    dv1[2] = 1.0;
    dv1[6] = t_lc;
    d = t_lc * t_lc;
    dv1[10] = d;
    dv1[14] = rt_powd_snf(t_lc, 3.0);
    dv1[3] = 0.0;
    dv1[7] = 1.0;
    dv1[11] = 2.0 * t_lc;
    dv1[15] = 3.0 * d;
    c_mldivide(dv1, para_ST);

    /*  ST�������켣 */
    d = rt_roundd_snf(t_lc / 0.05);
    i = (int)(d + 1.0);
    if (0 <= (int)(d + 1.0) - 1) {
      e_this_tunableEnvironment_f1_tu[0].tunableEnvironment[0] =
        fun_S_tunableEnvironment[0];
      b_speed[0] = 0.0;
      b_speed[1] = S_tlc;
    }

    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      S_tlc = 0.05 * (((double)SwitchACC + 1.0) - 1.0);
      S_min_dyn = S_tlc * S_tlc;
      S_traj[SwitchACC] = ((para_ST[0] + para_ST[1] * S_tlc) + para_ST[2] *
                           S_min_dyn) + para_ST[3] * rt_powd_snf(S_tlc, 3.0);
      V_traj[SwitchACC] = (para_ST[1] + para_ST[2] * 2.0 * S_tlc) + para_ST[3] *
        3.0 * S_min_dyn;
      b_fzero(e_this_tunableEnvironment_f1_tu, S_traj, (double)SwitchACC + 1.0,
              b_speed, &X_traj[SwitchACC], &S_tlc, &S_min_dyn);
    }

    i = (int)(t_lc / 0.05);
    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      a = X_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC] = pos_s + a;
      i3 = CurrentLaneIndex - TargetLaneIndex;
      S_tlc = rt_powd_snf(a, 3.0);
      S_min_dyn = rt_powd_snf(a, 4.0);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 120] =
        pos_l_CurrentLane + (double)i3 * ((fun_S_tunableEnvironment[0].f1[0] *
        S_tlc + fun_S_tunableEnvironment[0].f1[1] * S_min_dyn) +
        fun_S_tunableEnvironment[0].f1[2] * rt_powd_snf(a, 5.0)) * 1.0E-6;
      a = atan(((fun_S_tunableEnvironment[0].f1[0] * 3.0 * (a * a) +
                 fun_S_tunableEnvironment[0].f1[1] * 4.0 * S_tlc) +
                fun_S_tunableEnvironment[0].f1[2] * 5.0 * S_min_dyn) * 1.0E-6);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 240] = 90.0 -
        (double)i3 * 180.0 / 3.1415926535897931 * a;
      a *= 57.295779513082323;
      S_tlc = a;
      b_cosd(&S_tlc);
      S_min_dyn = V_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 360] = S_min_dyn *
        S_tlc;
      b_sind(&a);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 480] = (double)i3 *
        S_min_dyn * a;
      if (SwitchACC + 1U == 1U) {
        GlobVars->TrajPlanLaneChange.LaneChangePath[600] = 0.0;
      } else {
        GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 600] =
          (GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 240] -
           GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 239]) / 0.05;
      }
    }

    if (GlobVars->TrajPlanLaneChange.LaneChangePath[(int)d + 359] == 0.0) {
      GlobVars->TrajPlanLaneChange.LaneChangePath[(int)rt_roundd_snf(t_lc / 0.05)
        + 359] = V_traj[(int)(rt_roundd_snf(t_lc / 0.05) + 1.0) - 1];
    }

    i = GlobVars->TrajPlanLaneChange.DurationLaneChange + 1;
    if (i > 32767) {
      i = 32767;
    }

    DurationLaneChange = (short)i;
    CountLaneChange = 2;
  }

  if ((DurationLaneChange > 0) && (DurationLaneChange <= rt_roundd_snf(10.0 *
        t_lc_traj))) {
    /*  for count_1=1:1:2*(21-DurationLaneChange) */
    d = rt_roundd_snf(t_lc_traj / 0.1);
    a = (d + 1.0) - (double)DurationLaneChange;
    if (a < 32768.0) {
      if (a >= -32768.0) {
        i1 = (short)a;
      } else {
        i1 = MIN_int16_T;
      }
    } else if (a >= 32768.0) {
      i1 = MAX_int16_T;
    } else {
      i1 = 0;
    }

    if (i1 > 16383) {
      i1 = MAX_int16_T;
    } else if (i1 <= -16384) {
      i1 = MIN_int16_T;
    } else {
      i1 <<= 1;
    }

    i = i1;
    if (0 <= i - 1) {
      if (DurationLaneChange > 16383) {
        i2 = MAX_int16_T;
        count_2 = MAX_int16_T;
        i4 = MAX_int16_T;
        i5 = MAX_int16_T;
        i6 = MAX_int16_T;
        i7 = MAX_int16_T;
      } else {
        i2 = (short)(DurationLaneChange << 1);
        count_2 = (short)(DurationLaneChange << 1);
        i4 = (short)(DurationLaneChange << 1);
        i5 = (short)(DurationLaneChange << 1);
        i6 = (short)(DurationLaneChange << 1);
        i7 = (short)(DurationLaneChange << 1);
      }
    }

    for (count_1 = 0; count_1 < i; count_1++) {
      i3 = (i2 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      SwitchACC = 6 * count_1;
      traj[SwitchACC] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 - 3];
      i3 = (count_2 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[SwitchACC + 1] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 + 117];
      i3 = (i4 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[SwitchACC + 2] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 + 237];
      i3 = (i5 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[SwitchACC + 3] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 + 357];
      i3 = (i6 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[SwitchACC + 4] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 + 477];
      i3 = (i7 + count_1) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[SwitchACC + 5] = GlobVars->TrajPlanLaneChange.LaneChangePath[i3 + 597];
    }

    /*  for count_2=2*(21-DurationLaneChange)+1:1:80 */
    i = i1 + 1;
    if (i1 + 1 > 32767) {
      i = 32767;
    }

    i2 = (short)i;
    if (i2 <= 120) {
      traj_tmp = (int)(2.0 * rt_roundd_snf(t_lc_traj / 0.1));
      a = (rt_roundd_snf(t_lc_traj / 0.1) + 1.0) - (double)DurationLaneChange;
      if (a < 32768.0) {
        if (a >= -32768.0) {
          count_2 = (short)a;
        } else {
          count_2 = MIN_int16_T;
        }
      } else if (a >= 32768.0) {
        count_2 = MAX_int16_T;
      } else {
        count_2 = 0;
      }

      if (count_2 > 16383) {
        i8 = MAX_int16_T;
      } else if (count_2 <= -16384) {
        i8 = MIN_int16_T;
      } else {
        i8 = (short)(count_2 << 1);
      }
    }

    for (count_2 = i2; count_2 < 121; count_2++) {
      /*  traj(1,count_2)=LaneChangePath(40,1)+traj(4,2*(21-DurationLaneChange))*0.05*double(count_2-2*(21-DurationLaneChange)); */
      /*  traj(2,count_2)=LaneChangePath(40,2); */
      /*  traj(3,count_2)=LaneChangePath(40,3);  */
      /*  traj(4,count_2)=traj(4,2*(21-DurationLaneChange)); */
      i = count_2 - i1;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      SwitchACC = 6 * (count_2 - 1);
      traj[SwitchACC] = GlobVars->TrajPlanLaneChange.LaneChangePath[(int)(2.0 *
        d) - 1] + traj[6 * (i1 - 1) + 3] * 0.05 * (double)i;
      traj[SwitchACC + 1] = GlobVars->TrajPlanLaneChange.LaneChangePath[traj_tmp
        + 119];
      traj[SwitchACC + 2] = GlobVars->TrajPlanLaneChange.LaneChangePath[traj_tmp
        + 239];
      traj[SwitchACC + 3] = traj[6 * (i8 - 1) + 3];
      traj[SwitchACC + 4] = 0.0;
      traj[SwitchACC + 5] = 0.0;
    }

    i = DurationLaneChange + 1;
    if (DurationLaneChange + 1 > 32767) {
      i = 32767;
    }

    DurationLaneChange = (short)i;
    SwitchACC = 0;
  } else if (DurationLaneChange == 0) {
    SwitchACC = 1;
  } else if (DurationLaneChange > 10.0 * t_lc_traj) {
    SwitchACC = 1;
    CountLaneChange = 0;
    DurationLaneChange = 0;
  } else {
    SwitchACC = 1;
  }

  /*          ACC Function */
  if (SwitchACC != 0) {
    if ((CurrentLaneIndex != TargetLaneIndex) && (d_veh2int <
         (CalibrationVars->TrajPlanLaneChange.v_max_int
          * CalibrationVars->TrajPlanLaneChange.v_max_int - v_max * v_max) /
         (2.0 * CalibrationVars->TrajPlanLaneChange.a_min) +
         CalibrationVars->TrajPlanLaneChange.v_max_int
         * CalibrationVars->TrajPlanLaneChange.t_permit)) {
      /*  if 0 */
      *a_soll = ACC(8.3333333333333339, CurrentLaneFrontVel, CurrentLaneFrontDis,
                    speed, b_wait, CalibrationVars->ACC.a_max,
                    CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
                    CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                    CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                    CalibrationVars->ACC.tau_v_emg,
                    CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    } else {
      *a_soll = ACC(v_max, CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                    b_wait, CalibrationVars->ACC.a_max,
                    CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
                    CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                    CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                    CalibrationVars->ACC.tau_v_emg,
                    CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    }
  }

  for (i = 0; i < 80; i++) {
    traj_s[i] = traj[6 * i];
    traj_l[i] = traj[6 * i + 1];
    traj_psi[i] = traj[6 * i + 2];
    traj_vs[i] = traj[6 * i + 3];
    traj_vl[i] = traj[6 * i + 4];
    traj_omega[i] = traj[6 * i + 5];
  }

  GlobVars->TrajPlanLaneChange.CountLaneChange = CountLaneChange;
  GlobVars->TrajPlanLaneChange.DurationLaneChange = DurationLaneChange;
  GlobVars->TrajPlanLaneChange.t_lc_traj = t_lc_traj;

  /*  if SwitchACC */
  /*      a_soll=ACC(v_max,v_a,s_a,speed,wait); */
  /*      for count_1=1:1:2 */
  /*          t_count_1=0.05*count_1; */
  /*          traj(1,count_1)=pos_s+V_0*t_count_1+0.5*a_soll*t_count_1.^2; */
  /*          traj(2,count_1)=pos_l; */
  /*          traj(3,count_1)=90; */
  /*          traj(4,count_1)=V_0+a_soll*t_count_1; */
  /*          traj(5,count_1)=0; */
  /*          traj(6,count_1)=0; */
  /*      end */
  /*      jerk_2=(0-a_soll)/(4-0.1); */
  /*      for count_2=3:1:80 */
  /*          t_count_2=0.05*(count_2-2); */
  /*          if (a_soll*0.1+V_0)+a_soll*t_count_2+0.5*jerk_2*t_count_2.^2<=0 */
  /*              traj(1,count_2)=traj(1,count_2-1); */
  /*          else */
  /*              traj(1,count_2)=traj(1,2)+(a_soll*0.1+V_0)*t_count_2+0.5*a_soll*t_count_2.^2+1/6*jerk_2*t_count_2.^3; */
  /*          end */
  /*          traj(2,count_2)=pos_l; */
  /*          traj(3,count_2)=90; */
  /*          traj(4,count_2)=max([(a_soll*0.1+V_0)+a_soll*t_count_2+0.5*jerk_2*t_count_2.^2 0]); */
  /*          traj(5,count_2)=0; */
  /*          traj(6,count_2)=0; */
  /*      end */
  /*      traci.vehicle.setSpeed('S0',traj(4,2)); */
  /*  end */
  /*  �������� */
  /*  postion_veh=traci.vehicle.getPosition('S0'); */
  /*  CurrentLaneIndex=traci.vehicle.getLaneIndex('S0'); */
  /*  CountLaneChange=0; */
  /*  DurationLaneChange=0; */
  /*  LaneChangePath=zeros([6/0.05 6]); */
  /*  TargetLaneIndex=1; */
  /*  d_veh2int=100; */
  /*  pos_s=postion_veh(2); */
  /*  pos_l=postion_veh(1); */
  /*  w_lane=3.2; */
  /*  t_lc=2; */
  /*  function [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj]=..., */
  /*      TrajPlanLaneChange(CurrentLaneFrontDis,CurrentLaneFrontVel,TargetLaneBehindDis,TargetLaneBehindVel,TargetLaneFrontDis,TargetLaneFrontVel,speed,pos_s,pos_l,CurrentLaneIndex,CountLaneChange,..., */
  /*      DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,w_lane) */
  /*  if a_soll==100 */
  /*      traci.vehicle.moveToXY('S0','12', 2, traj_l(2), traj_s(2),traj_psi(2),2); */
  /*  else */
  /*      traci.vehicle.setSpeed('S0',a_soll*0.1+speed); */
  /*      traci.vehicle.changeLane('S0',CurrentLaneIndex,2); */
  /*  ��֤������ٶ��ڷ�Χ�� */
  /*  clear all */
  /*  i=0; */
  /*  for V_0=3:1:15 */
  /*      i=i+1; */
  /*      tlc(i)=max([2-0.04*(V_0-15) 2]); */
  /*      tlc(i)=min([tlc(i) 2.5]); */
  /*      tlc(i)=ceil(tlc(i)/0.1)*0.1; */
  /*      w_lane=3.2; */
  /*      S_end=tlc(i)*V_0; */
  /*      para=[3 4*S_end 5*S_end*S_end;6 12*S_end 20*S_end*S_end;S_end.^3 S_end.^4 S_end.^5]\[0;0;w_lane*10.^6]; */
  /*      aaa=0:0.1:S_end; */
  /*      bbb=abs((6*para(1)*aaa+12*para(2)*aaa.^2+20*para(3)*aaa.^3)/(10.^6))/..., */
  /*          (((1+(3*para(1)*aaa.^2+4*para(2)*aaa.^3+5*para(3)*aaa.^4)/(10.^6)).^2).^1.5); */
  /*      amax(i)=V_0.^2*max(bbb); */
  /*  end */
  /*  tlc */
  /*  amax */
}

/*
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 * Arguments    : double speed
 *                double pos_s
 *                double pos_l
 *                double pos_l_CurrentLane
 *                const double WidthOfLanes[6]
 *                short CurrentLaneIndex
 *                double CurrentLaneFrontDis
 *                double CurrentLaneFrontVel
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_TrajPlanLaneC
 *                double d_CalibrationVars_TrajPlanLaneC
 *                double e_CalibrationVars_TrajPlanLaneC
 *                double f_CalibrationVars_TrajPlanLaneC
 *                double Parameters_l_veh
 *                double traj_s[80]
 *                double traj_l[80]
 *                double traj_psi[80]
 *                double traj_vs[80]
 *                double traj_vl[80]
 *                double traj_omega[80]
 * Return Type  : void
 */
static void TrajPlanLaneChange_RePlan(double speed, double pos_s, double pos_l,
  double pos_l_CurrentLane, const double WidthOfLanes[6], short CurrentLaneIndex,
  double CurrentLaneFrontDis, double CurrentLaneFrontVel, TypeGlobVars *GlobVars,
  double c_CalibrationVars_TrajPlanLaneC, double d_CalibrationVars_TrajPlanLaneC,
  double e_CalibrationVars_TrajPlanLaneC, double f_CalibrationVars_TrajPlanLaneC,
  double Parameters_l_veh, double traj_s[80], double traj_l[80], double
  traj_psi[80], double traj_vs[80], double traj_vl[80], double traj_omega[80])
{
  static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };

  static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };

  static const signed char iv2[6] = { 0, 0, 2, 0, 0, 0 };

  anonymous_function tunableEnvironment[1];
  cell_wrap_1 fun_S_tunableEnvironment[1];
  double traj[480];
  double c[100];
  double x1[100];
  double z1[100];
  double S_traj[41];
  double V_traj[41];
  double X_traj[41];
  double dv[36];
  double dv1[16];
  double para_ST[4];
  double S_a_end[3];
  double x[2];
  double L_end;
  double S_end;
  double V_a_end;
  double V_end;
  double a;
  double b_c;
  double d;
  double e;
  double fb;
  double fc;
  double m;
  double s;
  double t_lc;
  double toler;
  int b_iac;
  int ia;
  int iac;
  int ix;
  int traj_tmp;
  short DurationLaneChange_RePlan;
  short count_2;
  short i;
  short i1;
  short i2;
  short i3;
  short i4;
  short i5;
  short i6;
  boolean_T exitg1;

  /* , */
  DurationLaneChange_RePlan =
    GlobVars->TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan;
  t_lc = GlobVars->TrajPlanLaneChange_RePlan.t_lc_RePlan;

  /* 3.2; */
  memset(&S_traj[0], 0, 41U * sizeof(double));
  memset(&X_traj[0], 0, 41U * sizeof(double));
  memset(&V_traj[0], 0, 41U * sizeof(double));
  memset(&traj[0], 0, 480U * sizeof(double));

  /* 0.5; */
  /* 0.5; */
  /* -1; */
  /* -3.5; */
  if (GlobVars->TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan == 0) {
    x[0] = 0.4;
    x[1] = fabs(pos_l - pos_l_CurrentLane) / WidthOfLanes[CurrentLaneIndex - 1] *
      2.5;
    s = maximum(x);
    x[0] = s;
    x[1] = 1.2;
    t_lc = rt_roundd_snf(minimum(x) / 0.1) * 0.1;

    /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
    x[0] = 0.0;
    x[1] = CurrentLaneFrontVel + d_CalibrationVars_TrajPlanLaneC *
      e_CalibrationVars_TrajPlanLaneC * t_lc;
    V_a_end = maximum(x);
    x[0] = 0.5 * (speed + V_a_end);
    x[1] = speed;
    s = minimum(x);
    x[0] = s;
    x[1] = speed + t_lc * e_CalibrationVars_TrajPlanLaneC;
    V_end = maximum(x);
    L_end = pos_l_CurrentLane - pos_l;

    /*  L_end=-L_end; */
    a = 0.5 * (speed + V_end) * t_lc;
    S_end = (CurrentLaneFrontDis + 0.5 * (V_a_end + CurrentLaneFrontVel) * t_lc)
      - c_CalibrationVars_TrajPlanLaneC * V_end;
    S_a_end[0] = S_end - Parameters_l_veh;
    S_a_end[1] = (S_end - (V_a_end * V_a_end - V_end * V_end) / (2.0 *
      f_CalibrationVars_TrajPlanLaneC)) - Parameters_l_veh;
    S_a_end[2] = sqrt(a * a - L_end * L_end);
    S_end = b_minimum(S_a_end);

    /* , */
    fun_S_tunableEnvironment[0].f1[0] = 0.0;
    fun_S_tunableEnvironment[0].f1[1] = 0.0;
    fun_S_tunableEnvironment[0].f1[2] = 0.0;
    fun_S_tunableEnvironment[0].f1[3] = L_end;
    fun_S_tunableEnvironment[0].f1[4] = 0.0;
    fun_S_tunableEnvironment[0].f1[5] = 0.0;
    for (ia = 0; ia < 6; ia++) {
      dv[6 * ia] = iv[ia];
      dv[6 * ia + 1] = iv1[ia];
      dv[6 * ia + 2] = iv2[ia];
    }

    dv[3] = 1.0;
    dv[9] = S_end;
    s = S_end * S_end;
    dv[15] = s;
    L_end = rt_powd_snf(S_end, 3.0);
    dv[21] = L_end;
    toler = rt_powd_snf(S_end, 4.0);
    dv[27] = toler;
    dv[33] = rt_powd_snf(S_end, 5.0);
    dv[4] = 0.0;
    dv[10] = 1.0;
    dv[16] = 2.0 * S_end;
    dv[22] = 3.0 * s;
    dv[28] = 4.0 * L_end;
    dv[34] = 5.0 * toler;
    dv[5] = 0.0;
    dv[11] = 0.0;
    dv[17] = 2.0;
    dv[23] = 6.0 * S_end;
    dv[29] = 12.0 * s;
    dv[35] = 20.0 * L_end;
    b_mldivide(dv, fun_S_tunableEnvironment[0].f1);
    x1[99] = S_end;
    x1[0] = 0.0;
    if (0.0 == -S_end) {
      for (ix = 0; ix < 98; ix++) {
        x1[ix + 1] = S_end * ((2.0 * ((double)ix + 2.0) - 101.0) / 99.0);
      }
    } else if ((S_end < 0.0) && (fabs(S_end) > 8.9884656743115785E+307)) {
      S_end /= 99.0;
      for (ix = 0; ix < 98; ix++) {
        x1[ix + 1] = S_end * ((double)ix + 1.0);
      }
    } else {
      S_end /= 99.0;
      for (ix = 0; ix < 98; ix++) {
        x1[ix + 1] = ((double)ix + 1.0) * S_end;
      }
    }

    a = 2.0 * fun_S_tunableEnvironment[0].f1[2];
    V_a_end = 3.0 * fun_S_tunableEnvironment[0].f1[3];
    L_end = 4.0 * fun_S_tunableEnvironment[0].f1[4];
    S_end = 5.0 * fun_S_tunableEnvironment[0].f1[5];
    for (ix = 0; ix < 100; ix++) {
      s = x1[ix];
      s = (((fun_S_tunableEnvironment[0].f1[1] + a * s) + V_a_end * (s * s)) +
           L_end * rt_powd_snf(s, 3.0)) + S_end * rt_powd_snf(s, 4.0);
      z1[ix] = sqrt(s * s + 1.0);
    }

    c[0] = 0.5 * (x1[1] - x1[0]);
    for (ix = 0; ix < 98; ix++) {
      c[ix + 1] = 0.5 * (x1[ix + 2] - x1[ix]);
    }

    c[99] = 0.5 * (x1[99] - x1[98]);
    S_end = 0.0;
    ix = 0;
    for (iac = 0; iac < 100; iac++) {
      b_iac = iac + 1;
      for (ia = b_iac; ia <= b_iac; ia++) {
        S_end += z1[ia - 1] * c[ix];
      }

      ix++;
    }

    para_ST[0] = 0.0;
    para_ST[1] = speed;
    para_ST[2] = S_end;
    para_ST[3] = V_end;
    dv1[0] = 1.0;
    dv1[1] = 0.0;
    dv1[4] = 0.0;
    dv1[5] = 1.0;
    dv1[8] = 0.0;
    dv1[9] = 0.0;
    dv1[12] = 0.0;
    dv1[13] = 0.0;
    dv1[2] = 1.0;
    dv1[6] = t_lc;
    s = t_lc * t_lc;
    dv1[10] = s;
    dv1[14] = rt_powd_snf(t_lc, 3.0);
    dv1[3] = 0.0;
    dv1[7] = 1.0;
    dv1[11] = 2.0 * t_lc;
    dv1[15] = 3.0 * s;
    c_mldivide(dv1, para_ST);
    ia = (int)(t_lc / 0.05 + 1.0);
    if (0 <= ia - 1) {
      tunableEnvironment[0].tunableEnvironment[0] = fun_S_tunableEnvironment[0];
      x[1] = S_end + 1.0;
    }

    for (ix = 0; ix < ia; ix++) {
      S_end = 0.05 * (((double)ix + 1.0) - 1.0);
      V_a_end = S_end * S_end;
      S_traj[ix] = ((para_ST[0] + para_ST[1] * S_end) + para_ST[2] * V_a_end) +
        para_ST[3] * rt_powd_snf(S_end, 3.0);
      V_traj[ix] = (para_ST[1] + para_ST[2] * 2.0 * S_end) + para_ST[3] * 3.0 *
        V_a_end;
      a = -1.0;
      V_end = x[1];
      L_end = anon(tunableEnvironment, S_traj, (double)ix + 1.0, -1.0);
      fb = anon(tunableEnvironment, S_traj, (double)ix + 1.0, x[1]);
      if (L_end == 0.0) {
        V_end = -1.0;
      } else {
        if (!(fb == 0.0)) {
          fc = fb;
          b_c = x[1];
          e = 0.0;
          d = 0.0;
          exitg1 = false;
          while ((!exitg1) && ((fb != 0.0) && (a != V_end))) {
            if ((fb > 0.0) == (fc > 0.0)) {
              b_c = a;
              fc = L_end;
              d = V_end - a;
              e = d;
            }

            if (fabs(fc) < fabs(fb)) {
              a = V_end;
              V_end = b_c;
              b_c = a;
              L_end = fb;
              fb = fc;
              fc = L_end;
            }

            m = 0.5 * (b_c - V_end);
            V_a_end = fabs(V_end);
            if (!(V_a_end > 1.0)) {
              V_a_end = 1.0;
            }

            toler = 4.4408920985006262E-16 * V_a_end;
            if ((fabs(m) <= toler) || (fb == 0.0)) {
              exitg1 = true;
            } else {
              if ((fabs(e) < toler) || (fabs(L_end) <= fabs(fb))) {
                d = m;
                e = m;
              } else {
                s = fb / L_end;
                if (a == b_c) {
                  V_a_end = 2.0 * m * s;
                  L_end = 1.0 - s;
                } else {
                  L_end /= fc;
                  S_end = fb / fc;
                  V_a_end = s * (2.0 * m * L_end * (L_end - S_end) - (V_end - a)
                                 * (S_end - 1.0));
                  L_end = (L_end - 1.0) * (S_end - 1.0) * (s - 1.0);
                }

                if (V_a_end > 0.0) {
                  L_end = -L_end;
                } else {
                  V_a_end = -V_a_end;
                }

                if ((2.0 * V_a_end < 3.0 * m * L_end - fabs(toler * L_end)) &&
                    (V_a_end < fabs(0.5 * e * L_end))) {
                  e = d;
                  d = V_a_end / L_end;
                } else {
                  d = m;
                  e = m;
                }
              }

              a = V_end;
              L_end = fb;
              if (fabs(d) > toler) {
                V_end += d;
              } else if (V_end > b_c) {
                V_end -= toler;
              } else {
                V_end += toler;
              }

              fb = anon(tunableEnvironment, S_traj, (double)ix + 1.0, V_end);
            }
          }
        }
      }

      X_traj[ix] = V_end;
    }

    /*      plot(S_traj); */
    s = rt_roundd_snf(t_lc / 0.05);
    ia = (int)s;
    for (ix = 0; ix < ia; ix++) {
      L_end = X_traj[ix + 1];
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix] = pos_s +
        L_end;
      toler = L_end * L_end;
      S_end = rt_powd_snf(L_end, 3.0);
      V_a_end = rt_powd_snf(L_end, 4.0);
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 120] =
        pos_l + (((((fun_S_tunableEnvironment[0].f1[0] +
                     fun_S_tunableEnvironment[0].f1[1] * L_end) +
                    fun_S_tunableEnvironment[0].f1[2] * toler) +
                   fun_S_tunableEnvironment[0].f1[3] * S_end) +
                  fun_S_tunableEnvironment[0].f1[4] * V_a_end) +
                 fun_S_tunableEnvironment[0].f1[5] * rt_powd_snf(L_end, 5.0));
      L_end = 57.295779513082323 * atan((((fun_S_tunableEnvironment[0].f1[1] +
        2.0 * fun_S_tunableEnvironment[0].f1[2] * L_end) + 3.0 *
        fun_S_tunableEnvironment[0].f1[3] * toler) + 4.0 *
        fun_S_tunableEnvironment[0].f1[4] * S_end) + 5.0 *
        fun_S_tunableEnvironment[0].f1[5] * V_a_end);
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 240] = 90.0
        - L_end;
      toler = L_end;
      b_cosd(&toler);
      S_end = V_traj[ix + 1];
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 360] =
        S_end * toler;
      b_sind(&L_end);
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 480] =
        S_end * L_end;
      if (ix + 1U == 1U) {
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[600] = 0.0;
      } else {
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 600] =
          (GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 240] -
           GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 239]) /
          0.05;
      }
    }

    if (GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[(int)s + 359] ==
        0.0) {
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[(int)
        rt_roundd_snf(t_lc / 0.05) + 359] = V_traj[(int)s];
    }

    /*  plot(LaneChangePath_RePlan(1:round(t_lc/0.05),1),LaneChangePath_RePlan(1:round(t_lc/0.05),2)); */
    DurationLaneChange_RePlan = 1;
  }

  if ((DurationLaneChange_RePlan > 0) && (DurationLaneChange_RePlan <= 10.0 *
       t_lc)) {
    s = rt_roundd_snf(t_lc / 0.1);
    L_end = (s + 1.0) - (double)DurationLaneChange_RePlan;
    if (L_end < 32768.0) {
      if (L_end >= -32768.0) {
        i = (short)L_end;
      } else {
        i = MIN_int16_T;
      }
    } else if (L_end >= 32768.0) {
      i = MAX_int16_T;
    } else {
      i = 0;
    }

    if (i > 16383) {
      i = MAX_int16_T;
    } else if (i <= -16384) {
      i = MIN_int16_T;
    } else {
      i <<= 1;
    }

    ia = i;
    if (0 <= ia - 1) {
      if (DurationLaneChange_RePlan > 16383) {
        i1 = MAX_int16_T;
        count_2 = MAX_int16_T;
        i2 = MAX_int16_T;
        i3 = MAX_int16_T;
        i4 = MAX_int16_T;
        i5 = MAX_int16_T;
      } else {
        i1 = (short)(DurationLaneChange_RePlan << 1);
        count_2 = (short)(DurationLaneChange_RePlan << 1);
        i2 = (short)(DurationLaneChange_RePlan << 1);
        i3 = (short)(DurationLaneChange_RePlan << 1);
        i4 = (short)(DurationLaneChange_RePlan << 1);
        i5 = (short)(DurationLaneChange_RePlan << 1);
      }
    }

    for (iac = 0; iac < ia; iac++) {
      ix = (i1 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      b_iac = 6 * iac;
      traj[b_iac] = GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix
        - 3];
      ix = (count_2 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      traj[b_iac + 1] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 117];
      ix = (i2 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      traj[b_iac + 2] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 237];
      ix = (i3 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      traj[b_iac + 3] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 357];
      ix = (i4 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      traj[b_iac + 4] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 477];
      ix = (i5 + iac) + 1;
      if (ix > 32767) {
        ix = 32767;
      }

      traj[b_iac + 5] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 597];
    }

    ia = i + 1;
    if (i + 1 > 32767) {
      ia = 32767;
    }

    i1 = (short)ia;
    if (i1 <= 80) {
      traj_tmp = (int)(2.0 * rt_roundd_snf(t_lc / 0.1));
      L_end = (rt_roundd_snf(t_lc / 0.1) + 1.0) - (double)
        DurationLaneChange_RePlan;
      if (L_end < 32768.0) {
        if (L_end >= -32768.0) {
          count_2 = (short)L_end;
        } else {
          count_2 = MIN_int16_T;
        }
      } else if (L_end >= 32768.0) {
        count_2 = MAX_int16_T;
      } else {
        count_2 = 0;
      }

      if (count_2 > 16383) {
        i6 = MAX_int16_T;
      } else if (count_2 <= -16384) {
        i6 = MIN_int16_T;
      } else {
        i6 = (short)(count_2 << 1);
      }
    }

    for (count_2 = i1; count_2 < 81; count_2++) {
      /*          traj(1,count_2)=LaneChangePath_2(2*(round((t_lc)/0.1)),1)+LaneChangePath_2(2*(round((t_lc)/0.1)),4)*0.05*double(count_2-2*(round(t_lc/0.1)+1-DurationLaneChange_2)); */
      /*          traj(1,count_2)=LaneChangePath_2(2*(round((t_lc)/0.1)),1)+V_0*0.05*double(count_2-2*(round(t_lc/0.1)+1-DurationLaneChange_2)); */
      ia = count_2 - i;
      if (ia > 32767) {
        ia = 32767;
      } else {
        if (ia < -32768) {
          ia = -32768;
        }
      }

      b_iac = 6 * (count_2 - 1);
      traj[b_iac] = GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan
        [(int)(2.0 * s) - 1] + traj[6 * (i - 1) + 3] * 0.05 * (double)ia;
      traj[b_iac + 1] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[traj_tmp + 119];
      traj[b_iac + 2] =
        GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[traj_tmp + 239];
      traj[b_iac + 3] = traj[6 * (i6 - 1) + 3];
      traj[b_iac + 4] = 0.0;
      traj[b_iac + 5] = 0.0;
    }

    /*      traci.vehicle.moveToXY('S0','M0', 2, LaneChangePath(DurationLaneChange,1), LaneChangePath(DurationLaneChange,2),..., */
    /*          LaneChangePath(DurationLaneChange,3),2); */
    /*  %         traci.vehicle.moveToXY('S0','M0', 2, traj(1,2), traj(2,2),traj(3,2),2); */
    ia = DurationLaneChange_RePlan + 1;
    if (DurationLaneChange_RePlan + 1 > 32767) {
      ia = 32767;
    }

    DurationLaneChange_RePlan = (short)ia;
  }

  if (DurationLaneChange_RePlan > 10.0 * t_lc) {
    DurationLaneChange_RePlan = 0;
  }

  for (ia = 0; ia < 80; ia++) {
    traj_s[ia] = traj[6 * ia];
    traj_l[ia] = traj[6 * ia + 1];
    traj_psi[ia] = traj[6 * ia + 2];
    traj_vs[ia] = traj[6 * ia + 3];
    traj_vl[ia] = traj[6 * ia + 4];
    traj_omega[ia] = traj[6 * ia + 5];
  }

  GlobVars->TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan =
    DurationLaneChange_RePlan;
  GlobVars->TrajPlanLaneChange_RePlan.t_lc_RePlan = t_lc;
}

/*
 * ,
 * ,
 * Arguments    : double CurrentLaneFrontDis
 *                double CurrentLaneFrontVel
 *                double speed
 *                double pos_l_CurrentLane
 *                double pos_s
 *                double pos_l
 *                short NumOfLanesOpposite
 *                const double WidthOfLanesOpposite[6]
 *                double WidthOfGap
 *                const double WidthOfLanes[6]
 *                double s_turnaround_border
 *                const short IndexOfLaneOppositeCar[20]
 *                const double SpeedOppositeCar[20]
 *                const double PosSOppositeCar[20]
 *                const short IndexOfLaneCodirectCar[10]
 *                const double SpeedCodirectCar[10]
 *                const double PosSCodirectCar[10]
 *                short CurrentLane
 *                double v_max
 *                double a_soll
 *                short CurrentGear
 *                short *TurnAroundActive
 *                short *AEBactive
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 *                double *a_soll_TrajPlanTurnAround
 *                double traj_s[80]
 *                double traj_l[80]
 *                double traj_psi[80]
 *                double traj_vs[80]
 *                double traj_vl[80]
 *                double traj_omega[80]
 *                short *TargetGear
 * Return Type  : void
 */
static void TrajPlanTurnAround(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double speed, double pos_l_CurrentLane, double pos_s,
  double pos_l, short NumOfLanesOpposite, const double WidthOfLanesOpposite[6],
  double WidthOfGap, const double WidthOfLanes[6], double s_turnaround_border,
  const short IndexOfLaneOppositeCar[20], const double SpeedOppositeCar[20],
  const double PosSOppositeCar[20], const short IndexOfLaneCodirectCar[10],
  const double SpeedCodirectCar[10], const double PosSCodirectCar[10], short
  CurrentLane, double v_max, double a_soll, short CurrentGear, short
  *TurnAroundActive, short *AEBactive, TypeGlobVars *GlobVars, const
  TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters, double *
  a_soll_TrajPlanTurnAround, double traj_s[80], double traj_l[80], double
  traj_psi[80], double traj_vs[80], double traj_vl[80], double traj_omega[80],
  short *TargetGear)
{
  emxArray_int16_T *Lanes2Search;
  emxArray_int16_T *b_OccupiedLanes;
  emxArray_int32_T *b_ia;
  emxArray_int32_T *c_ia;
  emxArray_real_T *OccupiedLanes;
  emxArray_real_T *OccupiedLanesPosMid1;
  emxArray_real_T *OccupiedLanesPosMid2;
  emxArray_real_T *ia;
  double IndexOfLaneOppositeCarFront[6];
  double PosSOppositeCarFront[6];
  double PosSOppositeCarRear[6];
  double SpeedOppositeCarFront[6];
  double SpeedOppositeCarRear[6];
  double posOfLaneCenterline[6];
  double WidthOfLanesOpposite_data[5];
  double d_veh2cross[5];
  double dv1[3];
  double b_speed[2];
  double dv[2];
  double D_safe2;
  double TurningRadius;
  double WidthOfLaneCurrent_tmp;
  double a_max_com;
  double a_predict;
  double b;
  double d;
  double d_cur2pot_tar;
  double k;
  double l_veh;
  double passedPerimeter;
  double pos_l_TargetLane;
  double s_max;
  double timeGap;
  double v_max_turnAround;
  double w_veh;
  int WidthOfLanesOpposite_size[2];
  int b_WidthOfLanesOpposite_size[2];
  int ib_size[1];
  int b_i;
  int i;
  int j;
  int loop_ub;
  int trueCount;
  short iv[2];
  short TargetLaneIndexOpposite;
  short TurnAroundState;
  short TypeOfTurnAround;
  short dec_trunAround;
  short i1;
  short wait_turnAround;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard1 = false;
  boolean_T tf;

  /* , */
  /* -------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* Parameters */
  TurningRadius = Parameters.TurningRadius;
  w_veh = Parameters.w_veh;
  l_veh = Parameters.l_veh;

  /* --------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* CalibrationVariable */
  /*  D_safe1=0.5; %D_safeSboder2HostVeh */
  /*  D_safe2=2;   %D_safeEvVeh2HostVeh */
  /*  dec2line=0.2; */
  /*  a_min=-2; */
  /*  a_max_com=1.5; */
  /*  v_max_turnAround=5; */
  D_safe2 = CalibrationVars->TrajPlanTurnAround.D_safe2;
  a_max_com = CalibrationVars->TrajPlanTurnAround.a_max_com;
  v_max_turnAround = CalibrationVars->TrajPlanTurnAround.v_max_turnAround;

  /* --------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* globalVariable */
  dec_trunAround = GlobVars->TrajPlanTurnAround.dec_trunAround;
  wait_turnAround = GlobVars->TrajPlanTurnAround.wait_turnAround;
  TypeOfTurnAround = GlobVars->TrajPlanTurnAround.TypeOfTurnAround;
  TurnAroundState = GlobVars->TrajPlanTurnAround.TurnAroundState;
  TargetLaneIndexOpposite = GlobVars->TrajPlanTurnAround.TargetLaneIndexOpposite;

  /* --------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* intermediateVars */
  for (i = 0; i < 5; i++) {
    d_veh2cross[i] = 0.0;
  }

  memset(&traj_s[0], 0, 80U * sizeof(double));
  memset(&traj_l[0], 0, 80U * sizeof(double));
  memset(&traj_psi[0], 0, 80U * sizeof(double));
  memset(&traj_vs[0], 0, 80U * sizeof(double));
  memset(&traj_vl[0], 0, 80U * sizeof(double));
  memset(&traj_omega[0], 0, 80U * sizeof(double));
  *TargetGear = CurrentGear;
  for (i = 0; i < 6; i++) {
    IndexOfLaneOppositeCarFront[i] = 0.0;
    SpeedOppositeCarFront[i] = 0.0;
    PosSOppositeCarFront[i] = 200.0;
    SpeedOppositeCarRear[i] = 0.0;
    PosSOppositeCarRear[i] = -200.0;
  }

  /*  Ŀ�공��ѡ�� ----(�жϵ�ͷһ�ζ��Σ�ѡ���ͷĿ�공����ֻ����һ��) */
  pos_l_TargetLane = 0.0;

  /* 20220324��Ϊֻ��һ�Σ����Կ������ֵ */
  WidthOfLaneCurrent_tmp = WidthOfLanes[CurrentLane - 1];
  if (GlobVars->TrajPlanTurnAround.TypeOfTurnAround == 0) {
    /*  d_cur2tar=0.5*(WidthOfLanesOpposite(1)-w_veh)+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(1); */
    TargetLaneIndexOpposite = 1;
    TypeOfTurnAround = 2;

    /*  1Ϊһ��˳����ͷ��2Ϊ����˳����ͷ */
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= NumOfLanesOpposite - 1)) {
      s_max = WidthOfLanesOpposite[i];
      d_cur2pot_tar = (0.5 * WidthOfLaneCurrent_tmp + WidthOfGap) + 0.5 * s_max;
      if (i + 1 > 1) {
        b_i = i + 1;
        for (j = 0; j <= b_i - 2; j++) {
          d_cur2pot_tar += WidthOfLanesOpposite[j];
        }
      }

      if (d_cur2pot_tar >= 2.0 * TurningRadius) {
        TargetLaneIndexOpposite = (short)(i + 1);
        TypeOfTurnAround = 1;
        pos_l_TargetLane = pos_l_CurrentLane + d_cur2pot_tar;
        exitg1 = true;
      } else if ((0.5 * (s_max - w_veh) -
                  CalibrationVars->TrajPlanTurnAround.dec2line) + d_cur2pot_tar >=
                 2.0 * TurningRadius) {
        /*  d_cur2tar=0.5*(WidthOfLanesOpposite(i)-w_veh)-dec2line+d_cur2pot_tar; */
        TargetLaneIndexOpposite = (short)(i + 1);
        TypeOfTurnAround = 1;
        pos_l_TargetLane = pos_l_CurrentLane + 2.0 * TurningRadius;
        exitg1 = true;
      } else {
        i++;
      }
    }

    /*  һ��˳����ͷ·������ */
    if (TypeOfTurnAround == 1) {
      d = ((s_turnaround_border - Parameters.TurningRadius) - 0.5 *
           Parameters.w_veh) - CalibrationVars->TrajPlanTurnAround.D_safe1;
      GlobVars->TrajPlanTurnAround.PosCircle[0] = d;
      GlobVars->TrajPlanTurnAround.PosCircle[1] = pos_l_CurrentLane +
        Parameters.TurningRadius;

      /*  s_start=PosCircle1(1); */
      GlobVars->TrajPlanTurnAround.PosCircle2[0] = d;
      GlobVars->TrajPlanTurnAround.PosCircle2[1] = pos_l_TargetLane -
        Parameters.TurningRadius;
      LaneCenterCal(CurrentLane, pos_l_CurrentLane, WidthOfLaneCurrent_tmp,
                    WidthOfGap, WidthOfLanesOpposite, NumOfLanesOpposite,
                    GlobVars->TrajPlanTurnAround.LaneCenterline);

      /*  ����������λ�� ȫ�ֱ��� */
    }

    /*  ����˳����ͷ·������ */
    if (TypeOfTurnAround == 2) {
      /* , */
      b_i = NumOfLanesOpposite - 1;
      if (NumOfLanesOpposite - 1 < -32768) {
        b_i = -32768;
      }

      if (1 > (short)b_i) {
        j = 0;
      } else {
        j = (short)b_i;
      }

      TargetLaneIndexOpposite = NumOfLanesOpposite;
      if (1 > (short)(NumOfLanesOpposite - 1)) {
        loop_ub = 0;
      } else {
        loop_ub = (short)(NumOfLanesOpposite - 1);
      }

      WidthOfLanesOpposite_size[0] = 1;
      WidthOfLanesOpposite_size[1] = j;
      if (0 <= j - 1) {
        memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], j * sizeof
               (double));
      }

      b_WidthOfLanesOpposite_size[0] = 1;
      b_WidthOfLanesOpposite_size[1] = loop_ub;
      if (0 <= loop_ub - 1) {
        memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], loop_ub *
               sizeof(double));
      }

      PathPlanTurnAround(s_turnaround_border, Parameters.w_veh,
                         Parameters.TurningRadius,
                         CalibrationVars->TrajPlanTurnAround.D_safe1,
                         pos_l_CurrentLane, (((((pos_l_CurrentLane + 0.5 *
        WidthOfLanes[CurrentLane - 1]) + WidthOfGap) + 0.5 *
        WidthOfLanesOpposite[NumOfLanesOpposite - 1]) + sum(posOfLaneCenterline,
        WidthOfLanesOpposite_size)) + 0.5 *
        (WidthOfLanesOpposite[NumOfLanesOpposite - 1] - Parameters.w_veh)) -
                         CalibrationVars->TrajPlanTurnAround.dec2line,
                         (((pos_l_CurrentLane + 0.5 * WidthOfLanes[CurrentLane -
                            1]) + WidthOfGap) +
                          WidthOfLanesOpposite[NumOfLanesOpposite - 1]) + sum
                         (WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size),
                         CalibrationVars->TrajPlanTurnAround.dec2line,
                         GlobVars->TrajPlanTurnAround.PosCircle,
                         GlobVars->TrajPlanTurnAround.PosCircle2,
                         GlobVars->TrajPlanTurnAround.PosCircle3,
                         GlobVars->TrajPlanTurnAround.pos_start,
                         GlobVars->TrajPlanTurnAround.pos_mid1,
                         GlobVars->TrajPlanTurnAround.pos_mid2,
                         GlobVars->TrajPlanTurnAround.pos_end);
      d = cos(GlobVars->TrajPlanTurnAround.pos_mid1[2]);
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] =
        GlobVars->TrajPlanTurnAround.pos_mid1[0] + sin
        (GlobVars->TrajPlanTurnAround.pos_mid1[2]) * Parameters.l_veh;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] =
        GlobVars->TrajPlanTurnAround.pos_mid1[1] - d * Parameters.l_veh;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[2] = 0.0;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] = 0.0;
      d = cos(GlobVars->TrajPlanTurnAround.pos_mid2[2]);
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] =
        GlobVars->TrajPlanTurnAround.pos_mid2[0] + sin
        (GlobVars->TrajPlanTurnAround.pos_mid2[2]) * Parameters.l_veh;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] =
        GlobVars->TrajPlanTurnAround.pos_mid2[1] - d * Parameters.l_veh;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[2] = 0.0;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] = 0.0;

      /*  OccupiedLanesPosMid1=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1_rear(2)):1:..., */
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1(2)); % ����[2 1] ȫ�ֱ��� ���򳵵����Ϊ���������Ϊ1 */
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]);
      GlobVars->TrajPlanTurnAround.pos_mid1[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid1[1]);

      /*  OccupiedLanesPosMid2=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2_rear(2)):1:..., */
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2(2)); % ����[1 -1] ȫ�ֱ��� ��ͷǰ��·�������Ϊ���������Ϊ-1 */
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]);
      GlobVars->TrajPlanTurnAround.pos_mid2[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid2[1]);
      LaneCenterCal(CurrentLane, pos_l_CurrentLane, WidthOfLanes[CurrentLane - 1],
                    WidthOfGap, WidthOfLanesOpposite, NumOfLanesOpposite,
                    GlobVars->TrajPlanTurnAround.LaneCenterline);

      /*  ����������λ�� ȫ�ֱ��� */
    }
  }

  /* ɸѡ����/һ��˳����ͷǰ���󳵵�·��Բ�ľ��� */
  for (i = 0; i < 6; i++) {
    posOfLaneCenterline[i] = 0.0;
  }

  /* 20220324 */
  if (TypeOfTurnAround == 1) {
    /*      posOfLaneCenterline=zeros([6,1]);%�����������ڵ�ͷ·���Ͻ���s���� */
    for (i = 0; i < 7; i++) {
      if ((GlobVars->TrajPlanTurnAround.LaneCenterline[i] != 0.0) && (i + 1 <=
           TargetLaneIndexOpposite)) {
        if (GlobVars->TrajPlanTurnAround.LaneCenterline[i] <
            GlobVars->TrajPlanTurnAround.PosCircle[1]) {
          passedPerimeter = GlobVars->TrajPlanTurnAround.PosCircle[1] -
            GlobVars->TrajPlanTurnAround.LaneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            passedPerimeter * passedPerimeter) +
            GlobVars->TrajPlanTurnAround.PosCircle[0];
        } else if ((GlobVars->TrajPlanTurnAround.LaneCenterline[i] >=
                    GlobVars->TrajPlanTurnAround.PosCircle[1]) &&
                   (GlobVars->TrajPlanTurnAround.LaneCenterline[i] <=
                    GlobVars->TrajPlanTurnAround.PosCircle2[1])) {
          posOfLaneCenterline[i] = TurningRadius +
            GlobVars->TrajPlanTurnAround.PosCircle[0];
        } else {
          if (GlobVars->TrajPlanTurnAround.LaneCenterline[i] >
              GlobVars->TrajPlanTurnAround.PosCircle2[1]) {
            passedPerimeter = GlobVars->TrajPlanTurnAround.PosCircle2[1] -
              GlobVars->TrajPlanTurnAround.LaneCenterline[i];
            posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
              passedPerimeter * passedPerimeter) +
              GlobVars->TrajPlanTurnAround.PosCircle2[0];
          }
        }
      }
    }
  } else {
    if ((TypeOfTurnAround == 2) && (GlobVars->TrajPlanTurnAround.TurnAroundState
         < 2)) {
      for (i = 0; i < 7; i++) {
        if ((GlobVars->TrajPlanTurnAround.LaneCenterline[i] != 0.0) && (i + 1 <=
             TargetLaneIndexOpposite)) {
          passedPerimeter = GlobVars->TrajPlanTurnAround.PosCircle[1] -
            GlobVars->TrajPlanTurnAround.LaneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            passedPerimeter * passedPerimeter) +
            GlobVars->TrajPlanTurnAround.PosCircle[0];
        }
      }
    }
  }

  if ((TypeOfTurnAround == 1) || ((TypeOfTurnAround == 2) &&
       (GlobVars->TrajPlanTurnAround.TurnAroundState < 2))) {
    j = 0;
    for (i = 0; i < 20; i++) {
      i1 = IndexOfLaneOppositeCar[i];
      if ((i1 != 0) && (i1 <= TargetLaneIndexOpposite)) {
        d = PosSOppositeCar[i] - posOfLaneCenterline[i1 - 1];
        if (d >= 0.0) {
          IndexOfLaneOppositeCarFront[j] = i1;
          PosSOppositeCarFront[j] = d;
          SpeedOppositeCarFront[j] = SpeedOppositeCar[i];
          j++;
        }
      }

      if ((i1 != 0) && (i1 <= TargetLaneIndexOpposite)) {
        d = PosSOppositeCar[i] - posOfLaneCenterline[i1 - 1];
        if ((d < 0.0) && (d > PosSOppositeCarRear[i1 - 1])) {
          PosSOppositeCarRear[i1 - 1] = d;
          SpeedOppositeCarRear[i1 - 1] = SpeedOppositeCar[i];
        }
      }
    }
  }

  /*  һ��˳����ͷ���� */
  if (TypeOfTurnAround == 1) {
    /* && pos_s<PosCircle1(1) */
    b_i = TargetLaneIndexOpposite;
    if (0 <= b_i - 1) {
      WidthOfLanesOpposite_size[0] = 1;
    }

    for (i = 0; i < b_i; i++) {
      if (1 > i) {
        j = 0;
      } else {
        j = i;
      }

      WidthOfLanesOpposite_size[1] = j;
      if (0 <= j - 1) {
        memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], j * sizeof
               (double));
      }

      d = 0.5 * WidthOfLanesOpposite[i];
      timeGap = (0.5 * WidthOfLaneCurrent_tmp + WidthOfGap) + d;
      if (timeGap + sum(posOfLaneCenterline, WidthOfLanesOpposite_size) <
          TurningRadius) {
        if (1 > i) {
          j = 0;
        } else {
          j = i;
        }

        b_WidthOfLanesOpposite_size[0] = 1;
        b_WidthOfLanesOpposite_size[1] = j;
        if (0 <= j - 1) {
          memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                 sizeof(double));
        }

        d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) +
          TurningRadius * acos((TurningRadius - (timeGap + sum
          (WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size))) /
          TurningRadius);
      } else {
        if (1 > i) {
          j = 0;
        } else {
          j = i;
        }

        b_WidthOfLanesOpposite_size[0] = 1;
        b_WidthOfLanesOpposite_size[1] = j;
        if (0 <= j - 1) {
          memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                 sizeof(double));
        }

        if (timeGap + sum(WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size)
            > (TurningRadius + GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
            GlobVars->TrajPlanTurnAround.PosCircle[1]) {
          /* , */
          if (1 > i) {
            j = 0;
          } else {
            j = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = j;
          if (0 <= j - 1) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                   sizeof(double));
          }

          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) +
            (((TurningRadius * asin(((((timeGap + sum(WidthOfLanesOpposite_data,
                     b_WidthOfLanesOpposite_size)) - TurningRadius) -
                  GlobVars->TrajPlanTurnAround.PosCircle2[1]) +
                 GlobVars->TrajPlanTurnAround.PosCircle[1]) / TurningRadius) +
               GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
              GlobVars->TrajPlanTurnAround.PosCircle[1]) + TurningRadius *
             3.1415926535897931 / 2.0);
        } else {
          if (1 > i) {
            j = 0;
          } else {
            j = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = j;
          if (0 <= j - 1) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                   sizeof(double));
          }

          d_veh2cross[i] = ((((((GlobVars->TrajPlanTurnAround.PosCircle[0] -
            pos_s) + TurningRadius * 3.1415926535897931 / 2.0) + 0.5 *
                               WidthOfLaneCurrent_tmp) + WidthOfGap) + d) + sum
                            (WidthOfLanesOpposite_data,
                             b_WidthOfLanesOpposite_size)) - TurningRadius;
        }
      }

      /*          d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % �ɸ���ȷ */
    }

    /*  ����ģʽ�ж� */
    if (GlobVars->TrajPlanTurnAround.dec_trunAround == 0) {
      d = GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s;
      if ((d <= (0.0 - speed * speed) / (2.0 *
            CalibrationVars->TrajPlanTurnAround.a_min) + 2.0 * Parameters.l_veh)
          && (d > 0.0) && (pos_l < GlobVars->TrajPlanTurnAround.PosCircle[1])) {
        dec_trunAround = 1;
      }
    } else {
      if ((GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s <= 0.0) ||
          (GlobVars->TrajPlanTurnAround.wait_turnAround == 1) || (pos_l >
           GlobVars->TrajPlanTurnAround.PosCircle[1])) {
        /* 20220225 */
        dec_trunAround = 0;
      }
    }

    /*  ͣ������ */
    if (dec_trunAround == 1) {
      /*          for i=1:TargetLaneIndexOpposite */
      /*              d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % �ɸ���ȷ */
      /*          end */
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 6)) {
        if ((IndexOfLaneOppositeCarFront[j] <= TargetLaneIndexOpposite) &&
            (IndexOfLaneOppositeCarFront[j] > 0.0)) {
          b_speed[0] = speed;
          b_speed[1] = 1.0E-5;
          d = maximum(b_speed);
          b_speed[0] = ((d_veh2cross[(int)IndexOfLaneOppositeCarFront[j] - 1] +
                         l_veh) / d * SpeedOppositeCarFront[j] + 0.5 * w_veh) +
            D_safe2;
          b_speed[1] = 0.0;
          if (PosSOppositeCarFront[j] <= maximum(b_speed)) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        } else {
          j++;
        }
      }

      if (wait_turnAround == 0) {
        iv[0] = 6;
        iv[1] = TargetLaneIndexOpposite;
        i1 = c_minimum(iv);
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j <= i1 - 1)) {
          if (PosSOppositeCarRear[j] > -l_veh) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        }
      }
    }

    /*  �𲽾��� */
    if ((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] -
         pos_s < 10.0)) {
      wait_turnAround = 0;
      a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                      CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);

      /* �Գ�Ԥ�����ٶ� */
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 6)) {
        if ((IndexOfLaneOppositeCarFront[j] <= TargetLaneIndexOpposite) &&
            (IndexOfLaneOppositeCarFront[j] > 0.0)) {
          /*  timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)/max([SpeedOppositeCarFront(j) 0.00001])]); */
          /*  timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh-TurningRadius)/max([SpeedOppositeCarFront(j) 0.00001])]); */
          loop_ub = (int)IndexOfLaneOppositeCarFront[j] - 1;
          b_speed[0] = ACC(13.888888888888889, SpeedOppositeCarRear[loop_ub],
                           PosSOppositeCarFront[j] - PosSOppositeCarRear[loop_ub],
                           SpeedOppositeCarFront[j], 0.0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.a_min_com,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc);
          b_speed[1] = 0.0;
          pos_l_TargetLane = maximum(b_speed);
          if (SpeedOppositeCarFront[j] <= 0.01) {
            pos_l_TargetLane = 0.0;
          }

          if (!(pos_l_TargetLane < 1.5)) {
            pos_l_TargetLane = 1.5;
          }

          /*  [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-(max(SpeedOppositeCarFront(j)+a_OppositeCarFront*t-v_max,0))^2/(2*a_OppositeCarFront+eps)..., */
          /*      -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
          /*      [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
          /* , */
          /* , */
          if ((SpeedOppositeCarFront[j] < v_max) || rtIsNaN(v_max)) {
            s_max = SpeedOppositeCarFront[j];
          } else {
            s_max = v_max;
          }

          b_speed[0] = 0.0;
          b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
          dv[0] = 1.0E-5;
          dv[1] = s_max;
          d = maximum(b_speed);
          b_speed[0] = -0.01;
          b_speed[1] = d / maximum(dv) + 0.01;
          c_fzero(SpeedOppositeCarFront, (double)j + 1.0, pos_l_TargetLane,
                  v_max, PosSOppositeCarFront, w_veh, D_safe2, b_speed, &timeGap,
                  &s_max, &d_cur2pot_tar);

          /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
          b_speed[0] = a_predict;
          b_speed[1] = a_max_com;
          a_predict = minimum(b_speed);
          if (a_predict > 0.0) {
            s_max = speed + a_predict * timeGap;
            d_cur2pot_tar = s_max - v_max_turnAround;
            if (!(d_cur2pot_tar > 0.0)) {
              d_cur2pot_tar = 0.0;
            }

            s_max = 0.5 * (s_max + speed) * timeGap - d_cur2pot_tar *
              d_cur2pot_tar / (2.0 * a_predict + 2.2204460492503131E-16);
          } else {
            dv1[0] = 0.0;
            dv1[1] = speed + a_predict * timeGap;
            dv1[2] = v_max_turnAround;
            s_max = 0.5 * (median(dv1) + speed) * timeGap;
          }

          if (s_max <= d_veh2cross[loop_ub] + l_veh) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        } else {
          j++;
        }
      }

      if (wait_turnAround == 0) {
        iv[0] = 6;
        iv[1] = TargetLaneIndexOpposite;
        i1 = c_minimum(iv);
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j <= i1 - 1)) {
          if (PosSOppositeCarRear[j] > -l_veh) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        }
      }
    }
  }

  /*  ����˳����ͷ���� */
  if (TypeOfTurnAround == 2) {
    /*  ����˳����ͷ״̬�ж� */
    if ((GlobVars->TrajPlanTurnAround.TurnAroundState == 0) && (pos_s >=
         GlobVars->TrajPlanTurnAround.pos_start[0] - 1.0) && (pos_l <
         GlobVars->TrajPlanTurnAround.pos_start[1] + 0.5) && (pos_l >
         GlobVars->TrajPlanTurnAround.pos_start[1] - 0.5)) {
      TurnAroundState = 1;
      *TargetGear = 4;

      /*  P R N D /1 2 3 4 */
    }

    emxInit_real_T(&OccupiedLanesPosMid2, 2);
    emxInit_real_T(&ia, 1);
    if (TurnAroundState == 1) {
      passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
      s_max = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
      emxInit_real_T(&OccupiedLanesPosMid1, 2);
      emxInit_real_T(&OccupiedLanes, 2);
      emxInit_int32_T(&b_ia, 1);
      if ((sqrt(passedPerimeter * passedPerimeter + s_max * s_max) < 0.15) &&
          (speed <= 0.05)) {
        *TargetGear = 2;
        if (CurrentGear == 2) {
          /*  �������������� % ����ǰ���� */
          /*  ��ǰλ��mid1��ͷ��β���ڳ��� �� ȷ����ռ�ݳ��� */
          s_max = GlobVars->TrajPlanTurnAround.pos_mid2[3] -
            GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
          loop_ub = ((s_max >= 0.0) << 1) - 1;
          if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) || rtIsNaN
              (GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
            b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
            OccupiedLanesPosMid2->size[0] = 1;
            OccupiedLanesPosMid2->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
            OccupiedLanesPosMid2->data[0] = rtNaN;
          } else if ((loop_ub == 0) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid2[3]) && (loop_ub < 0))
                     || ((GlobVars->TrajPlanTurnAround.pos_mid2[3] <
                          GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) &&
                         (loop_ub > 0))) {
            OccupiedLanesPosMid2->size[0] = 1;
            OccupiedLanesPosMid2->size[1] = 0;
          } else if ((rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ||
                      rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid2[3])) &&
                     (GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] ==
                      GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
            b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
            OccupiedLanesPosMid2->size[0] = 1;
            OccupiedLanesPosMid2->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
            OccupiedLanesPosMid2->data[0] = rtNaN;
          } else if ((floor(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ==
                      GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) && (((s_max
            >= 0.0) << 1) - 1 == loop_ub)) {
            b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
            OccupiedLanesPosMid2->size[0] = 1;
            j = (int)floor(s_max / (((double)loop_ub + 1.0) - 1.0));
            OccupiedLanesPosMid2->size[1] = j + 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
            for (b_i = 0; b_i <= j; b_i++) {
              OccupiedLanesPosMid2->data[b_i] =
                GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] + (double)(loop_ub
                * b_i);
            }
          } else {
            eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3],
                            ((double)loop_ub + 1.0) - 1.0,
                            GlobVars->TrajPlanTurnAround.pos_mid2[3],
                            OccupiedLanesPosMid2);
          }

          /*  ����[1 -1] ��ͷǰ��·�������Ϊ���������Ϊ-1 */
          s_max = GlobVars->TrajPlanTurnAround.pos_mid1[3] -
            GlobVars->TrajPlanTurnAround.pos_mid1_rear[3];
          loop_ub = ((s_max >= 0.0) << 1) - 1;
          if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) || rtIsNaN
              (GlobVars->TrajPlanTurnAround.pos_mid1[3])) {
            b_i = OccupiedLanesPosMid1->size[0] * OccupiedLanesPosMid1->size[1];
            OccupiedLanesPosMid1->size[0] = 1;
            OccupiedLanesPosMid1->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid1, b_i);
            OccupiedLanesPosMid1->data[0] = rtNaN;
          } else if ((loop_ub == 0) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid1[3]) && (loop_ub < 0))
                     || ((GlobVars->TrajPlanTurnAround.pos_mid1[3] <
                          GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) &&
                         (loop_ub > 0))) {
            OccupiedLanesPosMid1->size[0] = 1;
            OccupiedLanesPosMid1->size[1] = 0;
          } else if ((rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) ||
                      rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid1[3])) &&
                     (GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] ==
                      GlobVars->TrajPlanTurnAround.pos_mid1[3])) {
            b_i = OccupiedLanesPosMid1->size[0] * OccupiedLanesPosMid1->size[1];
            OccupiedLanesPosMid1->size[0] = 1;
            OccupiedLanesPosMid1->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid1, b_i);
            OccupiedLanesPosMid1->data[0] = rtNaN;
          } else if ((floor(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) ==
                      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) && (((s_max
            >= 0.0) << 1) - 1 == loop_ub)) {
            b_i = OccupiedLanesPosMid1->size[0] * OccupiedLanesPosMid1->size[1];
            OccupiedLanesPosMid1->size[0] = 1;
            j = (int)floor(s_max / (((double)loop_ub + 1.0) - 1.0));
            OccupiedLanesPosMid1->size[1] = j + 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid1, b_i);
            for (b_i = 0; b_i <= j; b_i++) {
              OccupiedLanesPosMid1->data[b_i] =
                GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] + (double)(loop_ub
                * b_i);
            }
          } else {
            eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3],
                            ((double)loop_ub + 1.0) - 1.0,
                            GlobVars->TrajPlanTurnAround.pos_mid1[3],
                            OccupiedLanesPosMid1);
          }

          /*  ����[2 1] ���򳵵����Ϊ���������Ϊ1 */
          if (rtIsNaN(OccupiedLanesPosMid2->data[0]) || rtIsNaN
              (OccupiedLanesPosMid1->data[OccupiedLanesPosMid1->size[1] - 1])) {
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            OccupiedLanes->data[0] = rtNaN;
          } else if (OccupiedLanesPosMid1->data[OccupiedLanesPosMid1->size[1] -
                     1] < OccupiedLanesPosMid2->data[0]) {
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 0;
          } else if ((rtIsInf(OccupiedLanesPosMid2->data[0]) || rtIsInf
                      (OccupiedLanesPosMid1->data[OccupiedLanesPosMid1->size[1]
                       - 1])) && (OccupiedLanesPosMid2->data[0] ==
                                  OccupiedLanesPosMid1->
                                  data[OccupiedLanesPosMid1->size[1] - 1])) {
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            OccupiedLanes->data[0] = rtNaN;
          } else if (floor(OccupiedLanesPosMid2->data[0]) ==
                     OccupiedLanesPosMid2->data[0]) {
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = (int)floor(OccupiedLanesPosMid1->
              data[OccupiedLanesPosMid1->size[1] - 1] -
              OccupiedLanesPosMid2->data[0]) + 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            j = (int)floor(OccupiedLanesPosMid1->data[OccupiedLanesPosMid1->
                           size[1] - 1] - OccupiedLanesPosMid2->data[0]);
            for (b_i = 0; b_i <= j; b_i++) {
              OccupiedLanes->data[b_i] = OccupiedLanesPosMid2->data[0] + (double)
                b_i;
            }
          } else {
            b_eml_float_colon(OccupiedLanesPosMid2->data[0],
                              OccupiedLanesPosMid1->data
                              [OccupiedLanesPosMid1->size[1] - 1], OccupiedLanes);
          }

          /*  Ŀ��λ��mid2��β���ڳ��� �� ȷ��������ĳ��� */
          do_vectors(OccupiedLanes, OccupiedLanesPosMid1, OccupiedLanesPosMid2,
                     b_ia, ib_size);
          b_i = ia->size[0];
          ia->size[0] = b_ia->size[0];
          emxEnsureCapacity_real_T(ia, b_i);
          j = b_ia->size[0];
          for (b_i = 0; b_i < j; b_i++) {
            ia->data[b_i] = b_ia->data[b_i];
          }

          sort(ia);
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = ia->size[0];
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          j = ia->size[0];
          for (b_i = 0; b_i < j; b_i++) {
            OccupiedLanesPosMid2->data[b_i] = OccupiedLanes->data[(int)ia->
              data[b_i] - 1];
          }

          /*  ������ռ�ݳ����Ľ����복��+��ռ�ݳ�����������ĳ���֮��ĳ����� */
          loop_ub = ia->size[0] - 1;
          trueCount = 0;
          for (i = 0; i <= loop_ub; i++) {
            if (OccupiedLanesPosMid2->data[i] != 0.0) {
              trueCount++;
            }
          }

          j = 0;
          for (i = 0; i <= loop_ub; i++) {
            if (OccupiedLanesPosMid2->data[i] != 0.0) {
              OccupiedLanesPosMid2->data[j] = OccupiedLanesPosMid2->data[i];
              j++;
            }
          }

          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = trueCount;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);

          /*  ������ռ�ݳ����Ľ����복��+��ռ�ݳ�����������ĳ���֮��ĳ���������Ѱǰ�� �� �ж���ײ�����ԣ��𲽾��ߣ� ��������·����Ϊpos_mid1_rear��pos_mid2_rear���߶Ρ�d_veh2cross,timegap�� */
          TurnAroundState = 2;
          a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                          CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                          CalibrationVars->ACC.a_min_com,
                          CalibrationVars->ACC.tau_v_com,
                          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                          CalibrationVars->ACC.tau_v_bre,
                          CalibrationVars->ACC.tau_v_emg,
                          CalibrationVars->ACC.tau_d_emg,
                          CalibrationVars->ACC.t_acc);
          i = 0;
          exitg1 = false;
          while ((!exitg1) && (i < 20)) {
            tf = local_ismember(IndexOfLaneOppositeCar[i], OccupiedLanesPosMid2);
            if (tf) {
              k = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]) /
                (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
              b = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] *
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] -
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] *
                   GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]) /
                (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
              d_cur2pot_tar = GlobVars->
                TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i] - 1];
              pos_l_TargetLane = PosSOppositeCar[i] - (d_cur2pot_tar - b) / k;
              if (pos_l_TargetLane > 0.0) {
                b_speed[0] = SpeedOppositeCar[i];
                b_speed[1] = 1.0E-5;
                d = maximum(b_speed);
                b_speed[0] = 0.0;
                b_speed[1] = ((pos_l_TargetLane - 0.5 * w_veh) - D_safe2) / d;
                timeGap = maximum(b_speed);
                passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]
                  - (d_cur2pot_tar - b) / k;
                s_max = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                  d_cur2pot_tar;
                b_speed[0] = a_predict;
                b_speed[1] = a_max_com;
                d = minimum(b_speed);
                b_speed[0] = speed + d * timeGap;
                b_speed[1] = v_max_turnAround;
                if (0.5 * (minimum(b_speed) + speed) * timeGap <= sqrt
                    (passedPerimeter * passedPerimeter + s_max * s_max) + l_veh)
                {
                  TurnAroundState = 1;
                  exitg1 = true;
                } else {
                  i++;
                }
              } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane > -l_veh))
              {
                TurnAroundState = 1;
                exitg1 = true;
              } else {
                i++;
              }
            } else {
              i++;
            }
          }

          if (TurnAroundState == 2) {
            /*                      SpeedCodirectCar=SpeedCodirectCar(SpeedCodirectCar>-1);%20220324 */
            /*  SpeedCodirectCarĬ��ֵΪ-1, LaneCenterline���һ��Ϊ��ͷǰ���ڳ��� */
            /*                      for j=1:length(SpeedCodirectCar) */
            trueCount = -1;
            for (i = 0; i < 10; i++) {
              if (SpeedCodirectCar[i] > -1.0) {
                trueCount++;
              }
            }

            j = 0;
            exitg1 = false;
            while ((!exitg1) && (j <= trueCount)) {
              tf = local_ismember(IndexOfLaneCodirectCar[j],
                                  OccupiedLanesPosMid2);
              if (tf) {
                k = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]) /
                  (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
                b = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] *
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] -
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] *
                     GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]) /
                  (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
                if (IndexOfLaneCodirectCar[j] == -1) {
                  pos_l_TargetLane =
                    (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k -
                    PosSCodirectCar[j];
                } else {
                  pos_l_TargetLane =
                    ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2) - b)
                    / k - PosSCodirectCar[j];
                }

                if (pos_l_TargetLane > 0.0) {
                  b_speed[0] = SpeedCodirectCar[j];
                  b_speed[1] = 1.0E-5;
                  d = maximum(b_speed);
                  b_speed[0] = 0.0;
                  b_speed[1] = ((pos_l_TargetLane - 0.5 * w_veh) - D_safe2) / d;
                  timeGap = maximum(b_speed);
                  if (IndexOfLaneCodirectCar[j] == -1) {
                    passedPerimeter = GlobVars->
                      TrajPlanTurnAround.pos_mid1_rear[0] -
                      (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k;
                    s_max = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                      GlobVars->TrajPlanTurnAround.LaneCenterline[6];
                    s_max = sqrt(passedPerimeter * passedPerimeter + s_max *
                                 s_max);
                  } else {
                    passedPerimeter = GlobVars->
                      TrajPlanTurnAround.pos_mid1_rear[0] -
                      ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2) -
                       b) / k;
                    s_max = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                             GlobVars->TrajPlanTurnAround.LaneCenterline[6]) +
                      3.2;
                    s_max = sqrt(passedPerimeter * passedPerimeter + s_max *
                                 s_max);
                  }

                  b_speed[0] = a_predict;
                  b_speed[1] = a_max_com;
                  d = minimum(b_speed);
                  b_speed[0] = speed + d * timeGap;
                  b_speed[1] = v_max_turnAround;
                  if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max +
                      l_veh) {
                    TurnAroundState = 1;
                    exitg1 = true;
                  } else {
                    j++;
                  }
                } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                            -l_veh)) {
                  TurnAroundState = 1;
                  exitg1 = true;
                } else {
                  j++;
                }
              } else {
                j++;
              }
            }
          }
        }
      }

      emxFree_int32_T(&b_ia);
      emxFree_real_T(&OccupiedLanes);
      emxFree_real_T(&OccupiedLanesPosMid1);
    } else if (TurnAroundState == 2) {
      passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid2[0] - pos_s;
      s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
      s_max = sqrt(passedPerimeter * passedPerimeter + s_max * s_max);
      if ((s_max < 0.15) && (speed <= 0.05)) {
        *TargetGear = 4;
      }

      emxInit_int16_T(&b_OccupiedLanes, 2);
      emxInit_int16_T(&Lanes2Search, 2);
      emxInit_int32_T(&c_ia, 1);
      if ((s_max < 0.15) && (CurrentGear == 4) && (speed <= 0.05)) {
        /*  ����������ǰ�� % ǰ��ǰ���� */
        /*  ��ǰλ��mid2��ͷ��β���ڳ��� �� ȷ����ռ�ݳ��� */
        s_max = GlobVars->TrajPlanTurnAround.pos_mid2[3] -
          GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
        loop_ub = ((s_max >= 0.0) << 1) - 1;
        if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) || rtIsNaN
            (GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = 1;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          OccupiedLanesPosMid2->data[0] = rtNaN;
        } else if ((loop_ub == 0) ||
                   ((GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] <
                     GlobVars->TrajPlanTurnAround.pos_mid2[3]) && (loop_ub < 0))
                   || ((GlobVars->TrajPlanTurnAround.pos_mid2[3] <
                        GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) &&
                       (loop_ub > 0))) {
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = 0;
        } else if ((rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ||
                    rtIsInf(GlobVars->TrajPlanTurnAround.pos_mid2[3])) &&
                   (GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] ==
                    GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = 1;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          OccupiedLanesPosMid2->data[0] = rtNaN;
        } else if ((floor(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ==
                    GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) && (((s_max >=
          0.0) << 1) - 1 == loop_ub)) {
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          j = (int)floor(s_max / (((double)loop_ub + 1.0) - 1.0));
          OccupiedLanesPosMid2->size[1] = j + 1;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          for (b_i = 0; b_i <= j; b_i++) {
            OccupiedLanesPosMid2->data[b_i] =
              GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] + (double)(loop_ub *
              b_i);
          }
        } else {
          eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3],
                          ((double)loop_ub + 1.0) - 1.0,
                          GlobVars->TrajPlanTurnAround.pos_mid2[3],
                          OccupiedLanesPosMid2);
        }

        /*  ����[1 -1] ��ͷǰ��·�������Ϊ���������Ϊ-1 */
        eml_integer_colon_dispatcher(OccupiedLanesPosMid2->data[0],
          TargetLaneIndexOpposite, b_OccupiedLanes);
        b_do_vectors(b_OccupiedLanes, OccupiedLanesPosMid2, Lanes2Search, c_ia,
                     ib_size);
        b_i = ia->size[0];
        ia->size[0] = c_ia->size[0];
        emxEnsureCapacity_real_T(ia, b_i);
        j = c_ia->size[0];
        for (b_i = 0; b_i < j; b_i++) {
          ia->data[b_i] = c_ia->data[b_i];
        }

        sort(ia);
        b_i = Lanes2Search->size[0] * Lanes2Search->size[1];
        Lanes2Search->size[0] = 1;
        Lanes2Search->size[1] = ia->size[0];
        emxEnsureCapacity_int16_T(Lanes2Search, b_i);
        j = ia->size[0];
        for (b_i = 0; b_i < j; b_i++) {
          Lanes2Search->data[b_i] = b_OccupiedLanes->data[(int)ia->data[b_i] - 1];
        }

        /*  ������ռ�ݳ����Ľ����복��+��ռ�ݳ�����������ĳ���֮��ĳ����� */
        loop_ub = ia->size[0] - 1;
        trueCount = 0;
        for (i = 0; i <= loop_ub; i++) {
          if (Lanes2Search->data[i] != 0) {
            trueCount++;
          }
        }

        j = 0;
        for (i = 0; i <= loop_ub; i++) {
          if (Lanes2Search->data[i] != 0) {
            Lanes2Search->data[j] = Lanes2Search->data[i];
            j++;
          }
        }

        b_i = Lanes2Search->size[0] * Lanes2Search->size[1];
        Lanes2Search->size[0] = 1;
        Lanes2Search->size[1] = trueCount;
        emxEnsureCapacity_int16_T(Lanes2Search, b_i);

        /*  ������ռ�ݳ�����Ŀ�공��+��ռ�ݳ�����Ŀ�공��֮��ĳ���������Ѱǰ�� �� �ж���ײ�����ԣ��𲽾��ߣ� ����ǰ��·����Ϊpos_mid2��pos_end���߶Ρ�d_veh2cross,timegap�� */
        TurnAroundState = 3;
        a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                        CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                        CalibrationVars->ACC.a_min_com,
                        CalibrationVars->ACC.tau_v_com,
                        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                        CalibrationVars->ACC.tau_v_bre,
                        CalibrationVars->ACC.tau_v_emg,
                        CalibrationVars->ACC.tau_d_emg,
                        CalibrationVars->ACC.t_acc);
        i = 0;
        exitg1 = false;
        while ((!exitg1) && (i < 20)) {
          if (b_local_ismember(IndexOfLaneOppositeCar[i], Lanes2Search)) {
            k = (GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                 GlobVars->TrajPlanTurnAround.pos_end[1]) /
              (GlobVars->TrajPlanTurnAround.pos_mid2[0] -
               GlobVars->TrajPlanTurnAround.pos_end[0]);
            d_cur2pot_tar = GlobVars->
              TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i] - 1];
            s_max = d_cur2pot_tar - (GlobVars->TrajPlanTurnAround.pos_mid2[0] *
              GlobVars->TrajPlanTurnAround.pos_end[1] -
              GlobVars->TrajPlanTurnAround.pos_end[0] *
              GlobVars->TrajPlanTurnAround.pos_mid2[1]) /
              (GlobVars->TrajPlanTurnAround.pos_mid2[0] -
               GlobVars->TrajPlanTurnAround.pos_end[0]);
            pos_l_TargetLane = PosSOppositeCar[i] - s_max / k;
            if (pos_l_TargetLane > 0.0) {
              /*  a_OppositeCarFront=min(a_OppositeCarFront,1.5); */
              /*  [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])),..., */
              /*      [0-0.01 0.01+max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
              b_speed[0] = SpeedOppositeCar[i];
              b_speed[1] = 1.0E-5;
              d = maximum(b_speed);
              b_speed[0] = 0.0;
              b_speed[1] = ((pos_l_TargetLane - 0.5 * w_veh) - D_safe2) / d;
              timeGap = maximum(b_speed);
              passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid2[0] - s_max
                / k;
              s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - d_cur2pot_tar;
              b_speed[0] = a_predict;
              b_speed[1] = a_max_com;
              d = minimum(b_speed);
              b_speed[0] = speed + d * timeGap;
              b_speed[1] = v_max_turnAround;
              if (0.5 * (minimum(b_speed) + speed) * timeGap <= sqrt
                  (passedPerimeter * passedPerimeter + s_max * s_max) + l_veh) {
                TurnAroundState = 2;
                exitg1 = true;
              } else {
                i++;
              }
            } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane > -l_veh))
            {
              TurnAroundState = 2;
              exitg1 = true;
            } else {
              i++;
            }
          } else {
            i++;
          }
        }
      }

      emxFree_int32_T(&c_ia);
      emxFree_int16_T(&Lanes2Search);
      emxFree_int16_T(&b_OccupiedLanes);
    } else if (TurnAroundState == 3) {
      *TargetGear = 4;
      if ((pos_s < GlobVars->TrajPlanTurnAround.pos_end[0] - 4.0) && (pos_l <
           GlobVars->TrajPlanTurnAround.pos_end[1] + 0.5) && (pos_l >
           GlobVars->TrajPlanTurnAround.pos_end[1] - 0.5)) {
        /*  TurnAroundState=0; */
        *TurnAroundActive = 0;
      }
    } else {
      *TargetGear = 4;
    }

    emxFree_real_T(&ia);
    emxFree_real_T(&OccupiedLanesPosMid2);

    /*  �����ͷ·��ǰ���� */
    if ((TurnAroundState == 0) || ((TurnAroundState == 1) &&
         (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s >= 0.0))) {
      b_i = TargetLaneIndexOpposite;
      if (0 <= b_i - 1) {
        WidthOfLanesOpposite_size[0] = 1;
      }

      for (i = 0; i < b_i; i++) {
        if (1 > i) {
          j = 0;
        } else {
          j = i;
        }

        WidthOfLanesOpposite_size[1] = j;
        if (0 <= j - 1) {
          memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], j * sizeof
                 (double));
        }

        d = (0.5 * WidthOfLaneCurrent_tmp + WidthOfGap) + 0.5 *
          WidthOfLanesOpposite[i];
        if (d + sum(posOfLaneCenterline, WidthOfLanesOpposite_size) <
            TurningRadius) {
          if (1 > i) {
            j = 0;
          } else {
            j = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = j;
          if (0 <= j - 1) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                   sizeof(double));
          }

          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) +
            TurningRadius * acos((TurningRadius - (d + sum
            (WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size))) /
            TurningRadius);
        } else {
          /* , */
          if (1 > i) {
            j = 0;
          } else {
            j = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = j;
          if (0 <= j - 1) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0], j *
                   sizeof(double));
          }

          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) +
            (TurningRadius * asin(((d + sum(WidthOfLanesOpposite_data,
                 b_WidthOfLanesOpposite_size)) - TurningRadius) / TurningRadius)
             + TurningRadius * 3.1415926535897931 / 2.0);
        }
      }

      /*  ����ģʽ�ж� */
      if (dec_trunAround == 0) {
        d = GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s;
        if ((d <= (0.0 - speed * speed) / (2.0 *
              CalibrationVars->TrajPlanTurnAround.a_min) + 2.0 *
             Parameters.l_veh) && (d > 0.0) && (pos_l <
             GlobVars->TrajPlanTurnAround.PosCircle[1])) {
          dec_trunAround = 1;
        }
      } else {
        if (((GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s <= 0.0) ||
             (wait_turnAround == 1)) && (pos_l <
             GlobVars->TrajPlanTurnAround.PosCircle[1])) {
          dec_trunAround = 0;
        }
      }

      /*  ͣ������ */
      if (dec_trunAround == 1) {
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j < 6)) {
          if ((IndexOfLaneOppositeCarFront[j] <= TargetLaneIndexOpposite) &&
              (IndexOfLaneOppositeCarFront[j] > 0.0)) {
            b_speed[0] = speed;
            b_speed[1] = 1.0E-5;
            d = maximum(b_speed);
            b_speed[0] = ((d_veh2cross[(int)IndexOfLaneOppositeCarFront[j] - 1]
                           + l_veh) / d * SpeedOppositeCarFront[j] + 0.5 * w_veh)
              + D_safe2;
            b_speed[1] = 0.0;
            if (PosSOppositeCarFront[j] <= maximum(b_speed)) {
              wait_turnAround = 1;
              exitg1 = true;
            } else {
              j++;
            }
          } else {
            j++;
          }
        }

        if (wait_turnAround == 0) {
          iv[0] = 6;
          iv[1] = TargetLaneIndexOpposite;
          i1 = c_minimum(iv);
          j = 0;
          exitg1 = false;
          while ((!exitg1) && (j <= i1 - 1)) {
            if (PosSOppositeCarRear[j] > -l_veh) {
              wait_turnAround = 1;
              exitg1 = true;
            } else {
              j++;
            }
          }
        }
      }

      /*  �𲽾��� */
      if ((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.pos_start[0] -
           pos_s < 10.0)) {
        wait_turnAround = 0;

        /*  a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0); */
        b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.a_min_com,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc);
        b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           0.0, fabs(rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
           GlobVars->TrajPlanTurnAround.PosCircle[1],
           GlobVars->TrajPlanTurnAround.pos_mid1[0] -
           GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf(pos_l -
          GlobVars->TrajPlanTurnAround.PosCircle[1], pos_s -
          GlobVars->TrajPlanTurnAround.PosCircle[0])) * Parameters.TurningRadius
                           + 9.0, speed, 0, CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.a_min_com,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc);
        a_predict = minimum(b_speed);
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j < 6)) {
          if ((IndexOfLaneOppositeCarFront[j] <= TargetLaneIndexOpposite) &&
              (IndexOfLaneOppositeCarFront[j] > 0.0)) {
            loop_ub = (int)IndexOfLaneOppositeCarFront[j] - 1;
            b_speed[0] = ACC(13.888888888888889, SpeedOppositeCarRear[loop_ub],
                             PosSOppositeCarFront[j] -
                             PosSOppositeCarRear[loop_ub],
                             SpeedOppositeCarFront[j], 0.0,
                             CalibrationVars->ACC.a_max,
                             CalibrationVars->ACC.a_min,
                             CalibrationVars->ACC.a_min_com,
                             CalibrationVars->ACC.tau_v_com,
                             CalibrationVars->ACC.tau_v,
                             CalibrationVars->ACC.tau_d,
                             CalibrationVars->ACC.tau_v_bre,
                             CalibrationVars->ACC.tau_v_emg,
                             CalibrationVars->ACC.tau_d_emg,
                             CalibrationVars->ACC.t_acc);
            b_speed[1] = 0.0;
            pos_l_TargetLane = maximum(b_speed);
            if (SpeedOppositeCarFront[j] <= 0.01) {
              pos_l_TargetLane = 0.0;
            }

            if (!(pos_l_TargetLane < 1.5)) {
              pos_l_TargetLane = 1.5;
            }

            /*                      [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
            /*                          [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
            /*                      % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)/max([SpeedOppositeCarFront(j) 0.00001])]); */
            /*                      s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            /* , */
            /* , */
            if ((SpeedOppositeCarFront[j] < v_max) || rtIsNaN(v_max)) {
              s_max = SpeedOppositeCarFront[j];
            } else {
              s_max = v_max;
            }

            b_speed[0] = 0.0;
            b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
            dv[0] = 1.0E-5;
            dv[1] = s_max;
            d = maximum(b_speed);
            b_speed[0] = -0.01;
            b_speed[1] = d / maximum(dv) + 0.01;
            c_fzero(SpeedOppositeCarFront, (double)j + 1.0, pos_l_TargetLane,
                    v_max, PosSOppositeCarFront, w_veh, D_safe2, b_speed,
                    &timeGap, &s_max, &d_cur2pot_tar);

            /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            b_speed[0] = a_predict;
            b_speed[1] = a_max_com;
            a_predict = minimum(b_speed);
            if (a_predict > 0.0) {
              s_max = speed + a_predict * timeGap;
              d_cur2pot_tar = s_max - v_max_turnAround;
              if (!(d_cur2pot_tar > 0.0)) {
                d_cur2pot_tar = 0.0;
              }

              s_max = 0.5 * (s_max + speed) * timeGap - d_cur2pot_tar *
                d_cur2pot_tar / (2.0 * a_predict + 2.2204460492503131E-16);
            } else {
              dv1[0] = 0.0;
              dv1[1] = speed + a_predict * timeGap;
              dv1[2] = v_max_turnAround;
              s_max = 0.5 * (median(dv1) + speed) * timeGap;
            }

            if (s_max <= d_veh2cross[loop_ub] + l_veh) {
              wait_turnAround = 1;
              exitg1 = true;
            } else {
              j++;
            }
          } else {
            j++;
          }
        }

        if (wait_turnAround == 0) {
          iv[0] = 6;
          iv[1] = TargetLaneIndexOpposite;
          i1 = c_minimum(iv);
          j = 0;
          exitg1 = false;
          while ((!exitg1) && (j <= i1 - 1)) {
            if (PosSOppositeCarRear[j] > -l_veh) {
              wait_turnAround = 1;
              exitg1 = true;
            } else {
              j++;
            }
          }
        }
      }
    }
  }

  /*  ACC�ٶȹ滮 */
  *a_soll_TrajPlanTurnAround = 0.0;

  /* 20220322 */
  if (TypeOfTurnAround == 1) {
    /*  һ��˳����ͷ�ٶȹ滮 */
    if (((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] >
          pos_s)) || (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
      /*  a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]); */
      b_speed[0] = 0.0;
      b_speed[1] = ((GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) + 4.0) +
        Parameters.l_veh;
      d = maximum(b_speed);
      b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                         wait_turnAround, CalibrationVars->ACC.a_max,
                         CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.a_min_com,
                         CalibrationVars->ACC.tau_v_com,
                         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                         CalibrationVars->ACC.tau_v_bre,
                         CalibrationVars->ACC.tau_v_emg,
                         CalibrationVars->ACC.tau_d_emg,
                         CalibrationVars->ACC.t_acc);
      b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         0.0, d, speed, wait_turnAround,
                         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.a_min_com,
                         CalibrationVars->ACC.tau_v_com,
                         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                         CalibrationVars->ACC.tau_v_bre,
                         CalibrationVars->ACC.tau_v_emg,
                         CalibrationVars->ACC.tau_d_emg,
                         CalibrationVars->ACC.t_acc);
      *a_soll_TrajPlanTurnAround = minimum(b_speed);
    } else if (dec_trunAround == 1) {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    } else if (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s > 2.0 *
               Parameters.l_veh) {
      *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
        CurrentLaneFrontDis, speed, wait_turnAround, CalibrationVars->ACC.a_max,
        CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc);
    } else {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    }

    b_speed[0] = *a_soll_TrajPlanTurnAround;
    b_speed[1] = a_soll;
    *a_soll_TrajPlanTurnAround = minimum(b_speed);
  } else {
    if (TypeOfTurnAround == 2) {
      /*  ����˳����ͷ�ٶȹ滮 */
      /*      if TurnAroundState==0 */
      if ((TurnAroundState == 0) || ((TurnAroundState == 1) && (pos_s <=
            GlobVars->TrajPlanTurnAround.pos_start[0]))) {
        if ((wait_turnAround == 1) ||
            (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
          b_speed[0] = 0.0;
          b_speed[1] = ((GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s) +
                        4.0) + Parameters.l_veh;
          d = maximum(b_speed);
          b_speed[0] = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
             CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
             CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
             CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
          b_speed[1] = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0, d, speed,
             wait_turnAround, CalibrationVars->ACC.a_max,
             CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
             CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
             CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
             CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
             CalibrationVars->ACC.t_acc);
          *a_soll_TrajPlanTurnAround = minimum(b_speed);
        } else if (dec_trunAround == 1) {
          *a_soll_TrajPlanTurnAround = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
             CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
             CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
             CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
          d_cur2pot_tar = ACC(v_max, 0.0, (((rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_mid1[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_start[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_start[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0])) *
            Parameters.TurningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
            - pos_s) + 9.0, speed, 0.0, CalibrationVars->ACC.a_max,
                              CalibrationVars->ACC.a_min,
                              CalibrationVars->ACC.a_min_com,
                              CalibrationVars->ACC.tau_v_com,
                              CalibrationVars->ACC.tau_v,
                              CalibrationVars->ACC.tau_d,
                              CalibrationVars->ACC.tau_v_bre,
                              CalibrationVars->ACC.tau_v_emg,
                              CalibrationVars->ACC.tau_d_emg,
                              CalibrationVars->ACC.t_acc);
          if ((d_cur2pot_tar < *a_soll_TrajPlanTurnAround) || rtIsNaN
              (*a_soll_TrajPlanTurnAround)) {
            *a_soll_TrajPlanTurnAround = d_cur2pot_tar;
          }
        } else if (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s > 2.0 *
                   Parameters.l_veh) {
          *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
            CurrentLaneFrontDis, speed, wait_turnAround,
            CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
            CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
            CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
            CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
            CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
        } else {
          *a_soll_TrajPlanTurnAround = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
             CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
             CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
             CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
          d_cur2pot_tar = ACC(v_max, 0.0, (((rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_mid1[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_start[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_start[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0])) *
            Parameters.TurningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
            - pos_s) + 9.0, speed, 0.0, CalibrationVars->ACC.a_max,
                              CalibrationVars->ACC.a_min,
                              CalibrationVars->ACC.a_min_com,
                              CalibrationVars->ACC.tau_v_com,
                              CalibrationVars->ACC.tau_v,
                              CalibrationVars->ACC.tau_d,
                              CalibrationVars->ACC.tau_v_bre,
                              CalibrationVars->ACC.tau_v_emg,
                              CalibrationVars->ACC.tau_d_emg,
                              CalibrationVars->ACC.t_acc);
          if ((d_cur2pot_tar < *a_soll_TrajPlanTurnAround) || rtIsNaN
              (*a_soll_TrajPlanTurnAround)) {
            *a_soll_TrajPlanTurnAround = d_cur2pot_tar;
          }
        }
      } else if (TurnAroundState == 1) {
        b_speed[0] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
           CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
           CalibrationVars->ACClowSpeed.a_max,
           CalibrationVars->ACClowSpeed.a_min,
           CalibrationVars->ACClowSpeed.a_min_com,
           CalibrationVars->ACClowSpeed.tau_v_com,
           CalibrationVars->ACClowSpeed.tau_v,
           CalibrationVars->ACClowSpeed.tau_d,
           CalibrationVars->ACClowSpeed.tau_v_bre,
           CalibrationVars->ACClowSpeed.tau_v_emg,
           CalibrationVars->ACClowSpeed.tau_d_emg,
           CalibrationVars->ACClowSpeed.tau_d_lowspeed,
           CalibrationVars->ACClowSpeed.t_acc);
        b_speed[1] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0,
           (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid1[1] -
                          GlobVars->TrajPlanTurnAround.PosCircle[1],
                          GlobVars->TrajPlanTurnAround.pos_mid1[0] -
                          GlobVars->TrajPlanTurnAround.PosCircle[0]) -
            rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.PosCircle[1],
                          pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0])) *
           Parameters.TurningRadius + 9.0, speed,
           CalibrationVars->ACClowSpeed.a_max,
           CalibrationVars->ACClowSpeed.a_min,
           CalibrationVars->ACClowSpeed.a_min_com,
           CalibrationVars->ACClowSpeed.tau_v_com,
           CalibrationVars->ACClowSpeed.tau_v,
           CalibrationVars->ACClowSpeed.tau_d,
           CalibrationVars->ACClowSpeed.tau_v_bre,
           CalibrationVars->ACClowSpeed.tau_v_emg,
           CalibrationVars->ACClowSpeed.tau_d_emg,
           CalibrationVars->ACClowSpeed.tau_d_lowspeed,
           CalibrationVars->ACClowSpeed.t_acc);
        *a_soll_TrajPlanTurnAround = minimum(b_speed);
        passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
        s_max = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
        if (sqrt(passedPerimeter * passedPerimeter + s_max * s_max) < 0.15) {
          s_max = speed;
          if (speed < 0.0) {
            s_max = -1.0;
          } else if (speed > 0.0) {
            s_max = 1.0;
          } else {
            if (speed == 0.0) {
              s_max = 0.0;
            }
          }

          *a_soll_TrajPlanTurnAround = -4.0 * s_max;
        }
      } else if (TurnAroundState == 2) {
        b_speed[0] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
           CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
           CalibrationVars->ACClowSpeed.a_max,
           CalibrationVars->ACClowSpeed.a_min,
           CalibrationVars->ACClowSpeed.a_min_com,
           CalibrationVars->ACClowSpeed.tau_v_com,
           CalibrationVars->ACClowSpeed.tau_v,
           CalibrationVars->ACClowSpeed.tau_d,
           CalibrationVars->ACClowSpeed.tau_v_bre,
           CalibrationVars->ACClowSpeed.tau_v_emg,
           CalibrationVars->ACClowSpeed.tau_d_emg,
           CalibrationVars->ACClowSpeed.tau_d_lowspeed,
           CalibrationVars->ACClowSpeed.t_acc);
        b_speed[1] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0, fabs
           (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                          GlobVars->TrajPlanTurnAround.PosCircle2[1],
                          GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                          GlobVars->TrajPlanTurnAround.PosCircle2[0]) -
            rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.PosCircle2[1],
                          pos_s - GlobVars->TrajPlanTurnAround.PosCircle2[0])) *
           Parameters.TurningRadius + 9.0, speed,
           CalibrationVars->ACClowSpeed.a_max,
           CalibrationVars->ACClowSpeed.a_min,
           CalibrationVars->ACClowSpeed.a_min_com,
           CalibrationVars->ACClowSpeed.tau_v_com,
           CalibrationVars->ACClowSpeed.tau_v,
           CalibrationVars->ACClowSpeed.tau_d,
           CalibrationVars->ACClowSpeed.tau_v_bre,
           CalibrationVars->ACClowSpeed.tau_v_emg,
           CalibrationVars->ACClowSpeed.tau_d_emg,
           CalibrationVars->ACClowSpeed.tau_d_lowspeed,
           CalibrationVars->ACClowSpeed.t_acc);
        *a_soll_TrajPlanTurnAround = minimum(b_speed);
        passedPerimeter = GlobVars->TrajPlanTurnAround.pos_mid2[0] - pos_s;
        s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
        if (sqrt(passedPerimeter * passedPerimeter + s_max * s_max) < 0.15) {
          s_max = speed;
          if (speed < 0.0) {
            s_max = -1.0;
          } else if (speed > 0.0) {
            s_max = 1.0;
          } else {
            if (speed == 0.0) {
              s_max = 0.0;
            }
          }

          *a_soll_TrajPlanTurnAround = -4.0 * s_max;
        }
      } else {
        if (TurnAroundState == 3) {
          *a_soll_TrajPlanTurnAround = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
             CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
             CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
             CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
        }
      }

      b_speed[0] = *a_soll_TrajPlanTurnAround;
      b_speed[1] = a_soll;
      *a_soll_TrajPlanTurnAround = minimum(b_speed);
    }
  }

  /*  AEB���� */
  if (*AEBactive == 0) {
    if ((TypeOfTurnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] -
         pos_s <= 0.0)) {
      j = 0;
      exitg4 = false;
      while ((!exitg4) && (j < 20)) {
        /*  ��IndexOfLaneOppositeCar(i)��pos_s��pos_l��LaneCenterline��PosCircle1��TurningRadius��d_veh2cross_strich(ע���ǳ�ͷ�����������ߵľ���) */
        if (IndexOfLaneOppositeCar[j] != 0) {
          d = GlobVars->
            TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j] - 1];
          timeGap = d - (GlobVars->TrajPlanTurnAround.PosCircle[1] -
                         TurningRadius);
          if (timeGap < TurningRadius) {
            s_max = GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius *
              acos((TurningRadius - timeGap) / TurningRadius);
          } else if (timeGap > (TurningRadius +
                                GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                     GlobVars->TrajPlanTurnAround.PosCircle[1]) {
            /* , */
            s_max = GlobVars->TrajPlanTurnAround.PosCircle[0] + (((TurningRadius
              * asin((((timeGap - TurningRadius) -
                       GlobVars->TrajPlanTurnAround.PosCircle2[1]) +
                      GlobVars->TrajPlanTurnAround.PosCircle[1]) / TurningRadius)
              + GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
              GlobVars->TrajPlanTurnAround.PosCircle[1]) + TurningRadius *
              3.1415926535897931 / 2.0);
          } else {
            s_max = ((GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius *
                      3.1415926535897931 / 2.0) + timeGap) - TurningRadius;
          }

          if ((pos_l < GlobVars->TrajPlanTurnAround.PosCircle[1]) && (pos_s <=
               GlobVars->TrajPlanTurnAround.PosCircle[0])) {
            d_cur2pot_tar = pos_s;
          } else if ((pos_l < GlobVars->TrajPlanTurnAround.PosCircle[1]) &&
                     (pos_s > GlobVars->TrajPlanTurnAround.PosCircle[0])) {
            d_cur2pot_tar = GlobVars->TrajPlanTurnAround.PosCircle[0] + atan
              ((pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0]) /
               ((GlobVars->TrajPlanTurnAround.PosCircle[1] +
                 2.2204460492503131E-16) - pos_l)) * TurningRadius;
          } else if ((pos_l >= GlobVars->TrajPlanTurnAround.PosCircle[1]) &&
                     (pos_l <= GlobVars->TrajPlanTurnAround.PosCircle2[1])) {
            d_cur2pot_tar = ((GlobVars->TrajPlanTurnAround.PosCircle[0] +
                              1.5707963267948966 * TurningRadius) + pos_l) -
              GlobVars->TrajPlanTurnAround.PosCircle[1];
          } else if ((pos_l > GlobVars->TrajPlanTurnAround.PosCircle2[1]) &&
                     (pos_s > GlobVars->TrajPlanTurnAround.PosCircle2[0])) {
            d_cur2pot_tar = (((GlobVars->TrajPlanTurnAround.PosCircle[0] +
                               1.5707963267948966 * TurningRadius) +
                              GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                             GlobVars->TrajPlanTurnAround.PosCircle[1]) + atan
              ((pos_l - GlobVars->TrajPlanTurnAround.PosCircle2[1]) / ((pos_s +
                 2.2204460492503131E-16) -
                GlobVars->TrajPlanTurnAround.PosCircle2[0])) * TurningRadius;
          } else {
            d_cur2pot_tar = (((((GlobVars->TrajPlanTurnAround.PosCircle[0] +
                                 1.5707963267948966 * TurningRadius) +
                                GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                               GlobVars->TrajPlanTurnAround.PosCircle[1]) +
                              1.5707963267948966 * TurningRadius) +
                             GlobVars->TrajPlanTurnAround.PosCircle2[0]) - pos_s;
          }

          s_max -= d_cur2pot_tar;

          /*  ��PosCircle1��TurningRadius��LaneCenterline��PosSOppositeCar(j)��disOppositeCar2circle2 */
          if (d < GlobVars->TrajPlanTurnAround.PosCircle[1]) {
            passedPerimeter = d - GlobVars->TrajPlanTurnAround.PosCircle[1];
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle[0] + sqrt(fabs
                (TurningRadius * TurningRadius - passedPerimeter *
                 passedPerimeter)));
          } else if ((d >= GlobVars->TrajPlanTurnAround.PosCircle[1]) && (d <=
                      GlobVars->TrajPlanTurnAround.PosCircle2[1])) {
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius);
          } else {
            passedPerimeter = d - GlobVars->TrajPlanTurnAround.PosCircle2[1];
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle2[0] + sqrt(fabs
                (TurningRadius * TurningRadius - passedPerimeter *
                 passedPerimeter)));
          }

          if ((IndexOfLaneOppositeCar[j] <= TargetLaneIndexOpposite) &&
              (IndexOfLaneOppositeCar[j] > 0) && (s_max > 0.0)) {
            if (pos_l_TargetLane > 0.0) {
              /*                      if IndexOfLaneOppositeCar(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(j)>0 && d_veh2cross_strich>0 */
              b_speed[0] = SpeedOppositeCarFront[j];
              b_speed[1] = 1.0E-5;
              d = maximum(b_speed);
              b_speed[0] = 0.0;
              b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) / d;
              timeGap = maximum(b_speed);
              b_speed[0] = speed + a_max_com * timeGap;
              b_speed[1] = v_max_turnAround;
              if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max + l_veh) {
                *AEBactive = 5;
                exitg4 = true;
              } else {
                j++;
              }
            } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane > -l_veh))
            {
              *AEBactive = 5;
              exitg4 = true;
            } else {
              j++;
            }
          } else {
            j++;
          }
        } else {
          j++;
        }
      }
    } else {
      if (TypeOfTurnAround == 2) {
        guard1 = false;
        if (TurnAroundState == 1) {
          d = GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s;
          if (d <= 0.0) {
            j = 0;
            exitg3 = false;
            while ((!exitg3) && (j < 20)) {
              if (IndexOfLaneOppositeCar[j] != 0) {
                /*  ��IndexOfLaneOppositeCar(i)��pos_s��pos_l��LaneCenterline��PosCircle1��TurningRadius��d_veh2cross_strich(ע���ǳ�ͷ�����������ߵľ���) */
                if (pos_s < GlobVars->TrajPlanTurnAround.PosCircle[0]) {
                  timeGap = GlobVars->
                    TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j]
                    - 1] - (GlobVars->TrajPlanTurnAround.PosCircle[1] -
                            TurningRadius);
                  if (timeGap < TurningRadius) {
                    s_max = d + TurningRadius * acos((TurningRadius - timeGap) /
                      TurningRadius);
                  } else {
                    s_max = d + (TurningRadius * asin((timeGap - TurningRadius) /
                      TurningRadius) + TurningRadius * 3.1415926535897931 / 2.0);
                  }
                } else {
                  s_max = (asin((GlobVars->
                                 TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar
                                 [j] - 1] -
                                 GlobVars->TrajPlanTurnAround.PosCircle[1]) /
                                TurningRadius) - rt_atan2d_snf(pos_l -
                            GlobVars->TrajPlanTurnAround.PosCircle[1], pos_s -
                            GlobVars->TrajPlanTurnAround.PosCircle[0])) *
                    TurningRadius;
                }

                /*  ��PosCircle1��TurningRadius��LaneCenterline��PosSOppositeCar(j)��disOppositeCar2circle2 */
                passedPerimeter = GlobVars->
                  TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j] -
                  1] - GlobVars->TrajPlanTurnAround.PosCircle[1];
                pos_l_TargetLane = PosSOppositeCar[j] -
                  (GlobVars->TrajPlanTurnAround.PosCircle[0] + sqrt(fabs
                    (TurningRadius * TurningRadius - passedPerimeter *
                     passedPerimeter)));
                if ((IndexOfLaneOppositeCar[j] <= TargetLaneIndexOpposite) &&
                    (IndexOfLaneOppositeCar[j] > 0) && (s_max > 0.0)) {
                  if (pos_l_TargetLane > 0.0) {
                    b_speed[0] = SpeedOppositeCarFront[j];
                    b_speed[1] = 1.0E-5;
                    timeGap = maximum(b_speed);
                    b_speed[0] = 0.0;
                    b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) /
                      timeGap;
                    timeGap = maximum(b_speed);
                    b_speed[0] = speed + a_max_com * timeGap;
                    b_speed[1] = v_max_turnAround;
                    if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max +
                        l_veh * (double)(IndexOfLaneOppositeCar[j] !=
                                         TargetLaneIndexOpposite)) {
                      *AEBactive = 5;
                      exitg3 = true;
                    } else {
                      j++;
                    }
                  } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                              -l_veh)) {
                    *AEBactive = 5;
                    exitg3 = true;
                  } else {
                    j++;
                  }
                } else {
                  j++;
                }
              } else {
                j++;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TurnAroundState == 2) {
            s_max = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
              GlobVars->TrajPlanTurnAround.pos_mid2_rear[0];
            k = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]) / s_max;
            b = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] *
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] *
                 GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]) / s_max;
            i = 0;
            exitg5 = false;
            while ((!exitg5) && (i < 20)) {
              if (IndexOfLaneOppositeCar[i] != 0) {
                /*  ��pos_s��Pos_circle2��pos_l��pos_s_rear��pos_l_rear */
                d_cur2pot_tar = atan((pos_l -
                                      GlobVars->TrajPlanTurnAround.PosCircle2[1])
                                     / (pos_s -
                  GlobVars->TrajPlanTurnAround.PosCircle2[0]));

                /*  ��IndexOfLaneOppositeCar(i)��pos_s_rear��pos_l_rear��LaneCenterline��k��b����d_veh2cross_strich(ע���ǳ�β�����������ߵľ���) */
                s_max = GlobVars->
                  TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i] -
                  1];
                pos_l_TargetLane = (s_max - b) / k;
                passedPerimeter = (pos_s + sin(d_cur2pot_tar) * l_veh) -
                  pos_l_TargetLane;
                d_cur2pot_tar = (pos_l - cos(d_cur2pot_tar) * l_veh) - s_max;
                s_max = d_cur2pot_tar;
                if (d_cur2pot_tar < 0.0) {
                  s_max = -1.0;
                } else if (d_cur2pot_tar > 0.0) {
                  s_max = 1.0;
                } else {
                  if (d_cur2pot_tar == 0.0) {
                    s_max = 0.0;
                  }
                }

                s_max *= sqrt(passedPerimeter * passedPerimeter + d_cur2pot_tar *
                              d_cur2pot_tar);
                if (IndexOfLaneOppositeCar[i] <= TargetLaneIndexOpposite) {
                  b_speed[0] = 1.0;
                  b_speed[1] = GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
                  if ((IndexOfLaneOppositeCar[i] >= maximum(b_speed)) && (s_max >
                       0.0)) {
                    pos_l_TargetLane = PosSOppositeCar[i] - pos_l_TargetLane;
                    if (pos_l_TargetLane > 0.0) {
                      b_speed[0] = SpeedOppositeCar[i];
                      b_speed[1] = 1.0E-5;
                      d = maximum(b_speed);
                      b_speed[0] = 0.0;
                      b_speed[1] = (pos_l_TargetLane - 0.5 * w_veh) / d;
                      timeGap = maximum(b_speed);
                      b_speed[0] = speed + a_max_com * timeGap;
                      b_speed[1] = v_max_turnAround;
                      if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max +
                          l_veh * (double)(IndexOfLaneOppositeCar[i] !=
                                           TargetLaneIndexOpposite)) {
                        *AEBactive = 5;
                        exitg5 = true;
                      } else {
                        i++;
                      }
                    } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                                -l_veh)) {
                      *AEBactive = 5;
                      exitg5 = true;
                    } else {
                      i++;
                    }
                  } else {
                    i++;
                  }
                } else {
                  i++;
                }
              } else {
                i++;
              }
            }

            if (*AEBactive == 0) {
              /*                  SpeedCodirectCar=SpeedCodirectCar(SpeedCodirectCar>-1);%20220324 */
              /*  SpeedCodirectCarĬ��ֵΪ-1, LaneCenterline���һ��Ϊ��ͷǰ���ڳ��� */
              /*                  for j=1:length(SpeedCodirectCar) */
              trueCount = -1;
              for (i = 0; i < 10; i++) {
                if (SpeedCodirectCar[i] > -1.0) {
                  trueCount++;
                }
              }

              j = 0;
              exitg2 = false;
              while ((!exitg2) && (j <= trueCount)) {
                /*  ��pos_s��Pos_circle2��pos_l��pos_s_rear��pos_l_rear */
                d_cur2pot_tar = atan((pos_l -
                                      GlobVars->TrajPlanTurnAround.PosCircle2[1])
                                     / (pos_s -
                  GlobVars->TrajPlanTurnAround.PosCircle2[0]));
                s_max = pos_s + sin(d_cur2pot_tar) * l_veh;
                d_cur2pot_tar = pos_l - cos(d_cur2pot_tar) * l_veh;

                /*  IndexOfLaneCodirectCar(i)��pos_s��pos_l��LaneCenterline��k��b����d_veh2cross_strich(Ĭ��ͬ��ڶ��������Ŀ���Ϊ3.2)(ע���ǳ�β�����������ߵľ���) */
                if (IndexOfLaneCodirectCar[j] == -1) {
                  passedPerimeter = s_max -
                    (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k;
                  d_cur2pot_tar -= GlobVars->TrajPlanTurnAround.LaneCenterline[6];
                  s_max = d_cur2pot_tar;
                  if (d_cur2pot_tar < 0.0) {
                    s_max = -1.0;
                  } else if (d_cur2pot_tar > 0.0) {
                    s_max = 1.0;
                  } else {
                    if (d_cur2pot_tar == 0.0) {
                      s_max = 0.0;
                    }
                  }

                  s_max *= sqrt(passedPerimeter * passedPerimeter +
                                d_cur2pot_tar * d_cur2pot_tar);
                } else {
                  passedPerimeter = s_max -
                    ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2) - b)
                    / k;
                  d_cur2pot_tar -= GlobVars->TrajPlanTurnAround.LaneCenterline[6]
                    - 3.2;
                  s_max = d_cur2pot_tar;
                  if (d_cur2pot_tar < 0.0) {
                    s_max = -1.0;
                  } else if (d_cur2pot_tar > 0.0) {
                    s_max = 1.0;
                  } else {
                    if (d_cur2pot_tar == 0.0) {
                      s_max = 0.0;
                    }
                  }

                  s_max *= sqrt(passedPerimeter * passedPerimeter +
                                d_cur2pot_tar * d_cur2pot_tar);
                }

                if ((IndexOfLaneCodirectCar[j] >=
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) && (s_max >
                     0.0)) {
                  if (IndexOfLaneCodirectCar[j] == -1) {
                    pos_l_TargetLane =
                      (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k -
                      PosSCodirectCar[j];
                  } else {
                    pos_l_TargetLane =
                      ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2) -
                       b) / k - PosSCodirectCar[j];
                  }

                  if (pos_l_TargetLane > 0.0) {
                    b_speed[0] = SpeedCodirectCar[j];
                    b_speed[1] = 1.0E-5;
                    d = maximum(b_speed);
                    b_speed[0] = 0.0;
                    b_speed[1] = (pos_l_TargetLane - 0.5 * w_veh) / d;
                    timeGap = maximum(b_speed);
                    b_speed[0] = speed + a_max_com * timeGap;
                    b_speed[1] = v_max_turnAround;
                    if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max +
                        l_veh * (double)(IndexOfLaneCodirectCar[j] !=
                                         GlobVars->TrajPlanTurnAround.pos_mid2_rear
                                         [3])) {
                      *AEBactive = 5;
                      exitg2 = true;
                    } else {
                      j++;
                    }
                  } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                              -l_veh)) {
                    *AEBactive = 5;
                    exitg2 = true;
                  } else {
                    j++;
                  }
                } else {
                  j++;
                }
              }
            }
          } else {
            if (TurnAroundState == 3) {
              s_max = GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                GlobVars->TrajPlanTurnAround.pos_end[0];
              k = (GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                   GlobVars->TrajPlanTurnAround.pos_end[1]) / s_max;
              b = (GlobVars->TrajPlanTurnAround.pos_mid2[0] *
                   GlobVars->TrajPlanTurnAround.pos_end[1] -
                   GlobVars->TrajPlanTurnAround.pos_end[0] *
                   GlobVars->TrajPlanTurnAround.pos_mid2[1]) / s_max;
              i = 0;
              exitg1 = false;
              while ((!exitg1) && (i < 20)) {
                if (IndexOfLaneOppositeCar[i] != 0) {
                  /*  ��IndexOfLaneOppositeCar(i)��pos_s��pos_l��LaneCenterline��k��b����d_veh2cross_strich(ע���ǳ�ͷ�����������ߵľ���) */
                  s_max = GlobVars->
                    TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i]
                    - 1];
                  pos_l_TargetLane = (s_max - b) / k;
                  passedPerimeter = pos_l_TargetLane - pos_s;
                  d_cur2pot_tar = s_max - pos_l;
                  s_max = d_cur2pot_tar;
                  if (d_cur2pot_tar < 0.0) {
                    s_max = -1.0;
                  } else if (d_cur2pot_tar > 0.0) {
                    s_max = 1.0;
                  } else {
                    if (d_cur2pot_tar == 0.0) {
                      s_max = 0.0;
                    }
                  }

                  s_max *= sqrt(passedPerimeter * passedPerimeter +
                                d_cur2pot_tar * d_cur2pot_tar);
                  if ((IndexOfLaneOppositeCar[i] <= TargetLaneIndexOpposite) &&
                      (IndexOfLaneOppositeCar[i] > 0) && (s_max > 0.0)) {
                    pos_l_TargetLane = PosSOppositeCar[i] - pos_l_TargetLane;
                    if (pos_l_TargetLane > 0.0) {
                      /*                          timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedOppositeCar(i) 0.00001])]); */
                      b_speed[0] = SpeedOppositeCar[i];
                      b_speed[1] = 1.0E-5;
                      d = maximum(b_speed);
                      b_speed[0] = 0.0;
                      b_speed[1] = (pos_l_TargetLane - 0.5 * w_veh) / d;
                      timeGap = maximum(b_speed);

                      /* 20220214 */
                      b_speed[0] = speed + a_max_com * timeGap;
                      b_speed[1] = v_max_turnAround;
                      if (0.5 * (minimum(b_speed) + speed) * timeGap <= s_max +
                          l_veh) {
                        *AEBactive = 5;
                        exitg1 = true;
                      } else {
                        i++;
                      }
                    } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                                -l_veh)) {
                      *AEBactive = 5;
                      exitg1 = true;
                    } else {
                      i++;
                    }
                  } else {
                    i++;
                  }
                } else {
                  i++;
                }
              }
            }
          }
        }
      }
    }
  }

  /*  ���a_soll_TrajPlanTurnAround��-4���߲��䣩��AEBactive=5 */
  if (*AEBactive == 5) {
    s_max = speed;
    if (speed < 0.0) {
      s_max = -1.0;
    } else if (speed > 0.0) {
      s_max = 1.0;
    } else {
      if (speed == 0.0) {
        s_max = 0.0;
      }
    }

    b_speed[0] = -4.0 * s_max;
    b_speed[1] = *a_soll_TrajPlanTurnAround;
    *a_soll_TrajPlanTurnAround = minimum(b_speed);
  }

  /*  һ��˳����ͷ�켣���� */
  if (TypeOfTurnAround == 1) {
    d = GlobVars->TrajPlanTurnAround.PosCircle[0] - Parameters.TurningRadius;
    if (pos_s > d) {
      passedPerimeter = pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0];
      if ((passedPerimeter > 0.0) && (pos_l <
           GlobVars->TrajPlanTurnAround.PosCircle[1])) {
        passedPerimeter = (atan((pos_l - GlobVars->TrajPlanTurnAround.PosCircle
          [1]) / passedPerimeter) + 1.5707963267948966) *
          Parameters.TurningRadius;
      } else if ((pos_l <= GlobVars->TrajPlanTurnAround.PosCircle2[1]) && (pos_l
                  >= GlobVars->TrajPlanTurnAround.PosCircle[1])) {
        passedPerimeter = (pos_l - GlobVars->TrajPlanTurnAround.PosCircle[1]) +
          Parameters.TurningRadius * 3.1415926535897931 / 2.0;
      } else {
        if (pos_l > GlobVars->TrajPlanTurnAround.PosCircle2[1]) {
          s_max = atan((pos_l - GlobVars->TrajPlanTurnAround.PosCircle2[1]) /
                       (pos_s - GlobVars->TrajPlanTurnAround.PosCircle2[0]));
          if (s_max < 0.0) {
            passedPerimeter = (((GlobVars->TrajPlanTurnAround.PosCircle2[1] -
                                 GlobVars->TrajPlanTurnAround.PosCircle[1]) +
                                Parameters.TurningRadius * 3.1415926535897931) +
                               GlobVars->TrajPlanTurnAround.PosCircle2[0]) -
              pos_s;
          } else {
            passedPerimeter = ((s_max * Parameters.TurningRadius +
                                GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                               GlobVars->TrajPlanTurnAround.PosCircle[1]) +
              Parameters.TurningRadius * 3.1415926535897931 / 2.0;
          }
        }
      }

      b_speed[0] = 0.0;
      timeGap = 3.1415926535897931 * TurningRadius / 2.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        s_max = 0.05 * ((double)loop_ub + 1.0);
        b_speed[1] = speed + *a_soll_TrajPlanTurnAround * s_max;
        b = maximum(b_speed);
        if (b == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (b + speed) * s_max / 2.0;
        }

        d_cur2pot_tar = s_max + passedPerimeter;
        if (d_cur2pot_tar < timeGap) {
          pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
        } else if ((d_cur2pot_tar <= (timeGap +
                     GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                    GlobVars->TrajPlanTurnAround.PosCircle[1]) && (d_cur2pot_tar
                    >= timeGap)) {
          pos_l_TargetLane = 0.0;
        } else {
          pos_l_TargetLane = (d_cur2pot_tar -
                              (GlobVars->TrajPlanTurnAround.PosCircle2[1] -
                               GlobVars->TrajPlanTurnAround.PosCircle[1])) /
            TurningRadius - 1.5707963267948966;
        }

        if (pos_l_TargetLane <= -1.5707963267948966) {
          traj_s[loop_ub] = pos_s + s_max;
          traj_l[loop_ub] = pos_l_CurrentLane;
          traj_psi[loop_ub] = 90.0;
          traj_vs[loop_ub] = b;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        } else if (pos_l_TargetLane < 0.0) {
          /* PI() */
          /*          targetAngle=targetAngle-pi/2; */
          s_max = cos(pos_l_TargetLane);
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] + s_max *
            TurningRadius;
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[1] + sin
            (pos_l_TargetLane) * TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = -b * sin(pos_l_TargetLane);
          traj_vl[loop_ub] = b * s_max;
          traj_omega[loop_ub] = -b / TurningRadius * 180.0 / 3.1415926535897931;
        } else if (pos_l_TargetLane == 0.0) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] +
            TurningRadius;
          traj_l[loop_ub] = (GlobVars->TrajPlanTurnAround.PosCircle[1] +
                             d_cur2pot_tar) - timeGap;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = 0.0;
          traj_vl[loop_ub] = b;
          traj_omega[loop_ub] = b;
        } else if (pos_l_TargetLane <= 1.5707963267948966) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] + cos
            (pos_l_TargetLane) * TurningRadius;
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] + sin
            (pos_l_TargetLane) * TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = -b * sin(pos_l_TargetLane);
          traj_vl[loop_ub] = b * cos(pos_l_TargetLane);
          traj_omega[loop_ub] = -b / TurningRadius * 180.0 / 3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] -
            (((d_cur2pot_tar - TurningRadius * 3.1415926535897931) -
              GlobVars->TrajPlanTurnAround.PosCircle2[1]) +
             GlobVars->TrajPlanTurnAround.PosCircle[1]);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] +
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -b;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    }

    /* ---------------------------------------------- */
    /* һ�ε�ͷ�����ж� */
    if ((traj_psi[0] == -90.0) && (pos_s <
         GlobVars->TrajPlanTurnAround.PosCircle[0] - Parameters.TurningRadius /
         2.0)) {
      *TurnAroundActive = 0;
    }

    if ((traj_psi[79] != 90.0) && (pos_s > d)) {
      *a_soll_TrajPlanTurnAround = 100.0;
    }
  }

  /*  ����˳����ͷ�켣���� */
  passedPerimeter = 0.0;

  /* 20220324 */
  if (TypeOfTurnAround == 2) {
    if ((TurnAroundState == 1) || (TurnAroundState == 0)) {
      passedPerimeter = pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0];
      if (passedPerimeter > 0.0) {
        passedPerimeter = (atan((pos_l - GlobVars->TrajPlanTurnAround.PosCircle
          [1]) / (pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0])) +
                           1.5707963267948966) * Parameters.TurningRadius;
      } else {
        if ((passedPerimeter < 0.0) && (pos_l >
             GlobVars->TrajPlanTurnAround.PosCircle[1])) {
          passedPerimeter = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s)
            + Parameters.TurningRadius * 3.1415926535897931;
        }
      }

      b_speed[0] = 0.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        s_max = 0.05 * ((double)loop_ub + 1.0);
        b_speed[1] = speed + *a_soll_TrajPlanTurnAround * s_max;
        b = maximum(b_speed);
        if (b == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (b + speed) * s_max / 2.0;
        }

        d_cur2pot_tar = s_max + passedPerimeter;
        pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
        if (pos_l_TargetLane <= -1.5707963267948966) {
          traj_s[loop_ub] = pos_s + s_max;
          traj_l[loop_ub] = pos_l;
          traj_psi[loop_ub] = 90.0;
          traj_vs[loop_ub] = b;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        } else if (pos_l_TargetLane < 1.5707963267948966) {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] + cos
            (pos_l_TargetLane) * TurningRadius;
          s_max = sin(pos_l_TargetLane);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[1] + s_max *
            TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = -b * s_max;
          traj_vl[loop_ub] = b * cos(pos_l_TargetLane);
          traj_omega[loop_ub] = -b / TurningRadius * 180.0 / 3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] -
            (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[1] +
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -b;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    } else if (TurnAroundState == 2) {
      d = pos_s - GlobVars->TrajPlanTurnAround.PosCircle2[0];
      if (d < 0.0) {
        passedPerimeter = (atan((pos_l - GlobVars->
          TrajPlanTurnAround.PosCircle2[1]) / (pos_s -
          GlobVars->TrajPlanTurnAround.PosCircle2[0])) + 1.5707963267948966) *
          Parameters.TurningRadius;
      } else {
        passedPerimeter = d + Parameters.TurningRadius * 3.1415926535897931;
      }

      b_speed[0] = 0.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        s_max = 0.05 * ((double)loop_ub + 1.0);
        b_speed[1] = speed + *a_soll_TrajPlanTurnAround * s_max;
        b = maximum(b_speed);
        if (b == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (b + speed) * s_max / 2.0;
        }

        d_cur2pot_tar = s_max + passedPerimeter;
        pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
        if (pos_l_TargetLane < 1.5707963267948966) {
          s_max = cos(pos_l_TargetLane);
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[0] - s_max *
            TurningRadius;
          d = sin(pos_l_TargetLane);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] - d *
            TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = b * d;
          traj_vl[loop_ub] = -b * s_max;
          traj_omega[loop_ub] = b / TurningRadius * 180.0 / 3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[0] +
            (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] -
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = b;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    } else {
      if (TurnAroundState == 3) {
        d = pos_s - GlobVars->TrajPlanTurnAround.PosCircle3[0];
        if (d > 0.0) {
          passedPerimeter = (atan((pos_l -
            GlobVars->TrajPlanTurnAround.PosCircle3[1]) / d) +
                             1.5707963267948966) * Parameters.TurningRadius;
        } else {
          if ((d < 0.0) && (pos_l > GlobVars->TrajPlanTurnAround.PosCircle3[1]))
          {
            passedPerimeter = (GlobVars->TrajPlanTurnAround.PosCircle3[0] -
                               pos_s) + Parameters.TurningRadius *
              3.1415926535897931;
          }
        }

        b_speed[0] = 0.0;
        for (loop_ub = 0; loop_ub < 80; loop_ub++) {
          s_max = 0.05 * ((double)loop_ub + 1.0);
          b_speed[1] = speed + *a_soll_TrajPlanTurnAround * s_max;
          b = maximum(b_speed);
          if (b == 0.0) {
            s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
              2.2204460492503131E-16);
          } else {
            s_max = (b + speed) * s_max / 2.0;
          }

          d_cur2pot_tar = s_max + passedPerimeter;
          pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
          if (pos_l_TargetLane < 1.5707963267948966) {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[0] + cos
              (pos_l_TargetLane) * TurningRadius;
            s_max = sin(pos_l_TargetLane);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[1] + s_max
              * TurningRadius;
            traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
            traj_vs[loop_ub] = -b * s_max;
            traj_vl[loop_ub] = b * cos(pos_l_TargetLane);
            traj_omega[loop_ub] = -b / TurningRadius * 180.0 /
              3.1415926535897931;
          } else {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[0] -
              (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[1] +
              TurningRadius;
            traj_psi[loop_ub] = -90.0;
            traj_vs[loop_ub] = -b;
            traj_vl[loop_ub] = 0.0;
            traj_omega[loop_ub] = 0.0;
          }
        }
      }
    }

    /*  if traj_psi(1)==-90 && (TurnAroundState==0) */
    /*      TurnAroundActive=0; */
    /*  end */
    if ((traj_psi[79] != 90.0) && (TurnAroundState != 0)) {
      *a_soll_TrajPlanTurnAround = 100.0;
    }
  }

  GlobVars->TrajPlanTurnAround.dec_trunAround = dec_trunAround;
  GlobVars->TrajPlanTurnAround.wait_turnAround = wait_turnAround;
  GlobVars->TrajPlanTurnAround.TypeOfTurnAround = TypeOfTurnAround;
  GlobVars->TrajPlanTurnAround.TurnAroundState = TurnAroundState;
  GlobVars->TrajPlanTurnAround.TargetLaneIndexOpposite = TargetLaneIndexOpposite;

  /*  if pos_s>PosCircle1(1)-TurningRadius */
  /*      if pos_s-PosCircle1(1)>0 */
  /*          passedAngle=atan((pos_s-PosCircle1(1))/(PosCircle1(2)-pos_l)); */
  /*          passedPerimeter=passedAngle*TurningRadius; */
  /*      else */
  /*          passedPerimeter=pos_s-PosCircle1(1); */
  /*      end */
  /*  end */
  /*  for count_1=1:1:80 */
  /*      t_count_1=0.05*count_1; */
  /*  */
  /*  */
  /*      if traj_vs(count_1)==0 */
  /*          adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps); */
  /*      else */
  /*          adavancedPerimeter=(traj_vs(count_1)+speed)*t_count_1/2; */
  /*      end */
  /*      targetPerimeter=adavancedPerimeter+passedPerimeter; */
  /*      targetAngle=targetPerimeter/TurningRadius; */
  /*      if targetAngle<=0 */
  /*          traj_s(count_1)=pos_s+adavancedPerimeter; */
  /*          traj_l(count_1)=pos_l; */
  /*          traj_psi(count_1)=90; */
  /*          targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]); */
  /*          traj_vs(count_1)=targetSpeed; */
  /*          traj_vl(count_1)=0; */
  /*          traj_omega(count_1)=0; */
  /*      elseif targetAngle<=PI() */
  /*          traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius; */
  /*          traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius; */
  /*          traj_psi(count_1)=90-targetAngle*180/PI(); */
  /*          targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]); */
  /*          traj_vs(count_1)=targetSpeed*cos(targetAngle); */
  /*          traj_vl(count_1)=targetSpeed*sin(targetAngle); */
  /*          traj_omega(count_1)=targetSpeed/TurningRadius*180/PI(); */
  /*      else */
  /*          traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*PI()); */
  /*          traj_l(count_1)=PosCircle1(2)+TurningRadius; */
  /*          traj_psi(count_1)=-90; */
  /*          targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]); */
  /*          traj_vs(count_1)=-targetSpeed; */
  /*          traj_vl(count_1)=0; */
  /*          traj_omega(count_1)=0; */
  /*      end */
  /*  end */
}

/*
 * Arguments    : const anonymous_function fun_x_tunableEnvironment[1]
 *                const double S_traj[41]
 *                double i_traj
 *                double x
 * Return Type  : double
 */
static double anon(const anonymous_function fun_x_tunableEnvironment[1], const
                   double S_traj[41], double i_traj, double x)
{
  double c[50];
  double varargin_1_tmp[50];
  double z1[50];
  double a;
  double b_a;
  double c_a;
  double d;
  double delta1;
  double varargout_1;
  int b_iac;
  int ia;
  int iac;
  int ix;
  varargin_1_tmp[49] = x;
  varargin_1_tmp[0] = 0.0;
  if (0.0 == -x) {
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = x * ((2.0 * ((double)ix + 2.0) - 51.0) / 49.0);
    }
  } else if ((x < 0.0) && (fabs(x) > 8.9884656743115785E+307)) {
    delta1 = x / 49.0;
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = delta1 * ((double)ix + 1.0);
    }
  } else {
    delta1 = x / 49.0;
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = ((double)ix + 1.0) * delta1;
    }
  }

  delta1 = 2.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[2];
  a = 3.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[3];
  b_a = 4.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[4];
  c_a = 5.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[5];
  for (ix = 0; ix < 50; ix++) {
    d = varargin_1_tmp[ix];
    d = (((fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[1] + delta1 * d)
          + a * (d * d)) + b_a * rt_powd_snf(d, 3.0)) + c_a * rt_powd_snf(d, 4.0);
    z1[ix] = sqrt(d * d + 1.0);
  }

  c[0] = 0.5 * (varargin_1_tmp[1] - varargin_1_tmp[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (varargin_1_tmp[ix + 2] - varargin_1_tmp[ix]);
  }

  c[49] = 0.5 * (varargin_1_tmp[49] - varargin_1_tmp[48]);
  varargout_1 = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      varargout_1 += z1[ia - 1] * c[ix];
    }

    ix++;
  }

  varargout_1 -= S_traj[(int)i_traj - 1];
  return varargout_1;
}

/*
 * Arguments    : double v_max
 *                double v_soll
 *                double d_ist
 *                double speed
 *                short b_wait
 *                double CalibrationVars_ACC_a_max
 *                double CalibrationVars_ACC_a_min
 *                double CalibrationVars_ACC_a_min_com
 *                double CalibrationVars_ACC_tau_v_com
 *                double CalibrationVars_ACC_tau_v
 *                double CalibrationVars_ACC_tau_d
 *                double CalibrationVars_ACC_tau_v_bre
 *                double CalibrationVars_ACC_tau_v_emg
 *                double CalibrationVars_ACC_tau_d_emg
 *                double CalibrationVars_ACC_t_acc
 * Return Type  : double
 */
static double b_ACC(double v_max, double v_soll, double d_ist, double speed,
                    short b_wait, double CalibrationVars_ACC_a_max, double
                    CalibrationVars_ACC_a_min, double
                    CalibrationVars_ACC_a_min_com, double
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc)
{
  double b_speed[3];
  double b_accel[2];
  double accel;
  double d_soll;

  /* 2.5; */
  /* -4; */
  /* -1.5; */
  /* 4; */
  /* 2; */
  /* 5; */
  /* 1; */
  /* 0.5; */
  /* 2; */
  /* 2; */
  accel = 100.0;
  if (d_ist < 100.0) {
    if (b_wait == -1) {
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 17.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    } else {
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 9.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    }

    if (b_wait == 1) {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v_bre;
    } else if (speed * speed / d_ist / 2.0 > -CalibrationVars_ACC_a_min_com) {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v_bre;
    } else if (fabs(speed - v_soll) < 2.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v_com;
    } else {
      accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d) /
        CalibrationVars_ACC_tau_v;
    }

    if (d_ist < 9.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
    } else {
      if (d_ist < 12.0) {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_emg;
      }
    }
  }

  b_accel[0] = -2.5;
  b_accel[1] = (v_max - speed) / CalibrationVars_ACC_tau_v;
  d_soll = maximum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_soll;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_max;
  accel = minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_min;
  return maximum(b_accel);
}

/*
 * Arguments    : const b_anonymous_function fun_x_tunableEnvironment[1]
 *                const double S_traj[80]
 *                double i_traj
 *                double x
 * Return Type  : double
 */
static double b_anon(const b_anonymous_function fun_x_tunableEnvironment[1],
                     const double S_traj[80], double i_traj, double x)
{
  double c[50];
  double varargin_1_tmp[50];
  double z1[50];
  double a;
  double b_a;
  double d;
  double delta1;
  double varargout_1;
  int b_iac;
  int ia;
  int iac;
  int ix;
  varargin_1_tmp[49] = x;
  varargin_1_tmp[0] = 0.0;
  if (0.0 == -x) {
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = x * ((2.0 * ((double)ix + 2.0) - 51.0) / 49.0);
    }
  } else if ((x < 0.0) && (fabs(x) > 8.9884656743115785E+307)) {
    delta1 = x / 49.0;
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = delta1 * ((double)ix + 1.0);
    }
  } else {
    delta1 = x / 49.0;
    for (ix = 0; ix < 48; ix++) {
      varargin_1_tmp[ix + 1] = ((double)ix + 1.0) * delta1;
    }
  }

  delta1 = 3.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[0];
  a = 4.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[1];
  b_a = 5.0 * fun_x_tunableEnvironment[0].tunableEnvironment[0].f1[2];
  for (ix = 0; ix < 50; ix++) {
    d = varargin_1_tmp[ix];
    d = ((delta1 * (d * d) + a * rt_powd_snf(d, 3.0)) + b_a * rt_powd_snf(d, 4.0))
      / 1.0E+6;
    z1[ix] = sqrt(d * d + 1.0);
  }

  c[0] = 0.5 * (varargin_1_tmp[1] - varargin_1_tmp[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (varargin_1_tmp[ix + 2] - varargin_1_tmp[ix]);
  }

  c[49] = 0.5 * (varargin_1_tmp[49] - varargin_1_tmp[48]);
  varargout_1 = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      varargout_1 += z1[ia - 1] * c[ix];
    }

    ix++;
  }

  varargout_1 -= S_traj[(int)i_traj - 1];
  return varargout_1;
}

/*
 * Arguments    : double *x
 * Return Type  : void
 */
static void b_cosd(double *x)
{
  double absx;
  signed char n;
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = rtNaN;
  } else {
    *x = rt_remd_snf(*x, 360.0);
    absx = fabs(*x);
    if (absx > 180.0) {
      if (*x > 0.0) {
        *x -= 360.0;
      } else {
        *x += 360.0;
      }

      absx = fabs(*x);
    }

    if (absx <= 45.0) {
      *x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (*x > 0.0) {
        *x = 0.017453292519943295 * (*x - 90.0);
        n = 1;
      } else {
        *x = 0.017453292519943295 * (*x + 90.0);
        n = -1;
      }
    } else if (*x > 0.0) {
      *x = 0.017453292519943295 * (*x - 180.0);
      n = 2;
    } else {
      *x = 0.017453292519943295 * (*x + 180.0);
      n = -2;
    }

    if (n == 0) {
      *x = cos(*x);
    } else if (n == 1) {
      *x = -sin(*x);
    } else if (n == -1) {
      *x = sin(*x);
    } else {
      *x = -cos(*x);
    }
  }
}

/*
 * Arguments    : const emxArray_int16_T *a
 *                const emxArray_real_T *b
 *                emxArray_int16_T *c
 *                emxArray_int32_T *ia
 *                int ib_size[1]
 * Return Type  : void
 */
static void b_do_vectors(const emxArray_int16_T *a, const emxArray_real_T *b,
  emxArray_int16_T *c, emxArray_int32_T *ia, int ib_size[1])
{
  double absx;
  double bk;
  int b_ialast;
  int exponent;
  int iafirst;
  int ialast;
  int iblast;
  int na;
  int nc;
  int nia;
  short ak;
  na = a->size[1];
  iafirst = c->size[0] * c->size[1];
  c->size[0] = 1;
  c->size[1] = a->size[1];
  emxEnsureCapacity_int16_T(c, iafirst);
  iafirst = ia->size[0];
  ia->size[0] = a->size[1];
  emxEnsureCapacity_int32_T(ia, iafirst);
  ib_size[0] = 0;
  nc = 0;
  nia = 0;
  iafirst = 0;
  ialast = 0;
  iblast = 1;
  while ((ialast + 1 <= na) && (iblast <= b->size[1])) {
    b_ialast = ialast + 1;
    ak = a->data[ialast];
    while ((b_ialast < a->size[1]) && (a->data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast - 1;
    bk = skip_to_last_equal_value(&iblast, b);
    absx = fabs(bk / 2.0);
    if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }

    if (fabs(bk - (double)ak) < absx) {
      ialast = b_ialast;
      iafirst = b_ialast;
      iblast++;
    } else if (rtIsNaN(bk) || (ak < bk)) {
      nc++;
      nia++;
      c->data[nc - 1] = ak;
      ia->data[nia - 1] = iafirst + 1;
      ialast = b_ialast;
      iafirst = b_ialast;
    } else {
      iblast++;
    }
  }

  while (ialast + 1 <= na) {
    b_ialast = ialast + 1;
    while ((b_ialast < a->size[1]) && (a->data[b_ialast] == a->data[ialast])) {
      b_ialast++;
    }

    nc++;
    nia++;
    c->data[nc - 1] = a->data[ialast];
    ia->data[nia - 1] = iafirst + 1;
    ialast = b_ialast;
    iafirst = b_ialast;
  }

  if (a->size[1] > 0) {
    iafirst = ia->size[0];
    if (1 > nia) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia;
    }

    emxEnsureCapacity_int32_T(ia, iafirst);
    iafirst = c->size[0] * c->size[1];
    if (1 > nc) {
      c->size[1] = 0;
    } else {
      c->size[1] = nc;
    }

    emxEnsureCapacity_int16_T(c, iafirst);
  }
}

/*
 * Arguments    : double a
 *                double b
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void b_eml_float_colon(double a, double b, emxArray_real_T *y)
{
  double apnd;
  double cdiff;
  double ndbl;
  double u0;
  double u1;
  int k;
  int n;
  int nm1d2;
  ndbl = floor((b - a) + 0.5);
  apnd = a + ndbl;
  cdiff = apnd - b;
  u0 = fabs(a);
  u1 = fabs(b);
  if ((u0 > u1) || rtIsNaN(u1)) {
    u1 = u0;
  }

  if (fabs(cdiff) < 4.4408920985006262E-16 * u1) {
    ndbl++;
    apnd = b;
  } else if (cdiff > 0.0) {
    apnd = a + (ndbl - 1.0);
  } else {
    ndbl++;
  }

  if (ndbl >= 0.0) {
    n = (int)ndbl;
  } else {
    n = 0;
  }

  nm1d2 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity_real_T(y, nm1d2);
  if (n > 0) {
    y->data[0] = a;
    if (n > 1) {
      y->data[n - 1] = apnd;
      nm1d2 = (n - 1) / 2;
      for (k = 0; k <= nm1d2 - 2; k++) {
        y->data[k + 1] = a + ((double)k + 1.0);
        y->data[(n - k) - 2] = apnd - ((double)k + 1.0);
      }

      if (nm1d2 << 1 == n - 1) {
        y->data[nm1d2] = (a + apnd) / 2.0;
      } else {
        y->data[nm1d2] = a + (double)nm1d2;
        y->data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }
}

/*
 * Arguments    : const b_anonymous_function c_FunFcn_tunableEnvironment_f1_[1]
 *                const double FunFcn_tunableEnvironment_f2[80]
 *                double FunFcn_tunableEnvironment_f3
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void b_fzero(const b_anonymous_function c_FunFcn_tunableEnvironment_f1_[1],
                    const double FunFcn_tunableEnvironment_f2[80], double
                    FunFcn_tunableEnvironment_f3, const double x[2], double *b,
                    double *fval, double *exitflag)
{
  double a;
  double c;
  double d;
  double e;
  double fa;
  double fc;
  double m;
  double p;
  double r;
  double s;
  double savefa;
  double savefb;
  double toler;
  boolean_T exitg1;
  *exitflag = 1.0;
  a = 0.0;
  *b = x[1];
  fa = b_anon(c_FunFcn_tunableEnvironment_f1_, FunFcn_tunableEnvironment_f2,
              FunFcn_tunableEnvironment_f3, 0.0);
  *fval = b_anon(c_FunFcn_tunableEnvironment_f1_, FunFcn_tunableEnvironment_f2,
                 FunFcn_tunableEnvironment_f3, x[1]);
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = 0.0;
    *fval = fa;
  } else {
    if (!(*fval == 0.0)) {
      fc = *fval;
      c = x[1];
      e = 0.0;
      d = 0.0;
      exitg1 = false;
      while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
        if ((*fval > 0.0) == (fc > 0.0)) {
          c = a;
          fc = fa;
          d = *b - a;
          e = d;
        }

        if (fabs(fc) < fabs(*fval)) {
          a = *b;
          *b = c;
          c = a;
          fa = *fval;
          *fval = fc;
          fc = fa;
        }

        m = 0.5 * (c - *b);
        p = fabs(*b);
        if (!(p > 1.0)) {
          p = 1.0;
        }

        toler = 4.4408920985006262E-16 * p;
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              p = 2.0 * m * s;
              fa = 1.0 - s;
            } else {
              fa /= fc;
              r = *fval / fc;
              p = s * (2.0 * m * fa * (fa - r) - (*b - a) * (r - 1.0));
              fa = (fa - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (p > 0.0) {
              fa = -fa;
            } else {
              p = -p;
            }

            if ((2.0 * p < 3.0 * m * fa - fabs(toler * fa)) && (p < fabs(0.5 * e
                  * fa))) {
              e = d;
              d = p / fa;
            } else {
              d = m;
              e = m;
            }
          }

          a = *b;
          fa = *fval;
          if (fabs(d) > toler) {
            *b += d;
          } else if (*b > c) {
            *b -= toler;
          } else {
            *b += toler;
          }

          *fval = b_anon(c_FunFcn_tunableEnvironment_f1_,
                         FunFcn_tunableEnvironment_f2,
                         FunFcn_tunableEnvironment_f3, *b);
        }
      }

      p = fabs(savefa);
      fa = fabs(savefb);
      if ((p > fa) || rtIsNaN(fa)) {
        fa = p;
      }

      if (!(fabs(*fval) <= fa)) {
        *exitflag = -5.0;
      }
    }
  }
}

/*
 * Arguments    : short a
 *                const emxArray_int16_T *s
 * Return Type  : boolean_T
 */
static boolean_T b_local_ismember(short a, const emxArray_int16_T *s)
{
  int k;
  boolean_T exitg1;
  boolean_T tf;
  tf = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= s->size[1] - 1)) {
    if (a == s->data[k]) {
      tf = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return tf;
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double b_maximum(const double x[3])
{
  double d;
  double ex;
  int idx;
  int k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      d = x[k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double b_minimum(const double x[3])
{
  double d;
  double ex;
  int idx;
  int k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double A[36]
 *                double B[6]
 * Return Type  : void
 */
static void b_mldivide(const double A[36], double B[6])
{
  double b_A[36];
  double s;
  double smax;
  int b_tmp;
  int i;
  int ijA;
  int ix;
  int iy;
  int j;
  int jA;
  int jp1j;
  int k;
  int mmj_tmp;
  signed char ipiv[6];
  signed char i1;
  memcpy(&b_A[0], &A[0], 36U * sizeof(double));
  for (i = 0; i < 6; i++) {
    ipiv[i] = (signed char)(i + 1);
  }

  for (j = 0; j < 5; j++) {
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    iy = 6 - j;
    jA = 0;
    ix = b_tmp;
    smax = fabs(b_A[b_tmp]);
    for (k = 2; k <= iy; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }

    if (b_A[b_tmp + jA] != 0.0) {
      if (jA != 0) {
        iy = j + jA;
        ipiv[j] = (signed char)(iy + 1);
        ix = j;
        for (k = 0; k < 6; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i = (b_tmp - j) + 6;
      for (jA = jp1j; jA <= i; jA++) {
        b_A[jA - 1] /= b_A[b_tmp];
      }
    }

    iy = b_tmp + 6;
    jA = b_tmp;
    for (k = 0; k <= mmj_tmp; k++) {
      smax = b_A[iy];
      if (b_A[iy] != 0.0) {
        ix = b_tmp + 1;
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (ijA = i; ijA <= jp1j; ijA++) {
          b_A[ijA - 1] += b_A[ix] * -smax;
          ix++;
        }
      }

      iy += 6;
      jA += 6;
    }

    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }

  for (k = 0; k < 6; k++) {
    iy = 6 * k;
    if (B[k] != 0.0) {
      i = k + 2;
      for (jA = i; jA < 7; jA++) {
        B[jA - 1] -= B[k] * b_A[(jA + iy) - 1];
      }
    }
  }

  for (k = 5; k >= 0; k--) {
    iy = 6 * k;
    if (B[k] != 0.0) {
      B[k] /= b_A[k + iy];
      for (jA = 0; jA < k; jA++) {
        B[jA] -= B[k] * b_A[jA + iy];
      }
    }
  }
}

/*
 * Arguments    : double *x
 * Return Type  : void
 */
static void b_sind(double *x)
{
  double absx;
  signed char n;
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = rtNaN;
  } else {
    *x = rt_remd_snf(*x, 360.0);
    absx = fabs(*x);
    if (absx > 180.0) {
      if (*x > 0.0) {
        *x -= 360.0;
      } else {
        *x += 360.0;
      }

      absx = fabs(*x);
    }

    if (absx <= 45.0) {
      *x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (*x > 0.0) {
        *x = 0.017453292519943295 * (*x - 90.0);
        n = 1;
      } else {
        *x = 0.017453292519943295 * (*x + 90.0);
        n = -1;
      }
    } else if (*x > 0.0) {
      *x = 0.017453292519943295 * (*x - 180.0);
      n = 2;
    } else {
      *x = 0.017453292519943295 * (*x + 180.0);
      n = -2;
    }

    if (n == 0) {
      *x = sin(*x);
    } else if (n == 1) {
      *x = cos(*x);
    } else if (n == -1) {
      *x = -cos(*x);
    } else {
      *x = -sin(*x);
    }
  }
}

/*
 * Arguments    : const double FunFcn_tunableEnvironment_f1[6]
 *                double FunFcn_tunableEnvironment_f2
 *                double FunFcn_tunableEnvironment_f3
 *                double FunFcn_tunableEnvironment_f4
 *                const double FunFcn_tunableEnvironment_f5[6]
 *                double FunFcn_tunableEnvironment_f6
 *                double FunFcn_tunableEnvironment_f7
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void c_fzero(const double FunFcn_tunableEnvironment_f1[6], double
                    FunFcn_tunableEnvironment_f2, double
                    FunFcn_tunableEnvironment_f3, double
                    FunFcn_tunableEnvironment_f4, const double
                    FunFcn_tunableEnvironment_f5[6], double
                    FunFcn_tunableEnvironment_f6, double
                    FunFcn_tunableEnvironment_f7, const double x[2], double *b,
                    double *fval, double *exitflag)
{
  double dv[2];
  double a;
  double b_d;
  double c;
  double d;
  double e;
  double fa;
  double fc;
  double m;
  double p;
  double r;
  double s;
  double savefa;
  double savefb;
  double toler;
  double u0_tmp;
  boolean_T exitg1;
  *exitflag = 1.0;
  a = -0.01;
  *b = x[1];
  u0_tmp = FunFcn_tunableEnvironment_f1[(int)FunFcn_tunableEnvironment_f2 - 1];
  p = u0_tmp + FunFcn_tunableEnvironment_f3 * -0.01;
  dv[0] = 0.0;
  d = (FunFcn_tunableEnvironment_f5[(int)FunFcn_tunableEnvironment_f2 - 1] - 0.5
       * FunFcn_tunableEnvironment_f6) - FunFcn_tunableEnvironment_f7;
  dv[1] = d;
  if ((!(p < FunFcn_tunableEnvironment_f4)) && (!rtIsNaN
       (FunFcn_tunableEnvironment_f4))) {
    p = FunFcn_tunableEnvironment_f4;
  }

  p = (u0_tmp + p) / 2.0;
  if (!(p > 0.0001)) {
    p = 0.0001;
  }

  fa = p * -0.01 - maximum(dv);
  p = u0_tmp + FunFcn_tunableEnvironment_f3 * x[1];
  dv[0] = 0.0;
  dv[1] = d;
  if ((!(p < FunFcn_tunableEnvironment_f4)) && (!rtIsNaN
       (FunFcn_tunableEnvironment_f4))) {
    p = FunFcn_tunableEnvironment_f4;
  }

  p = (u0_tmp + p) / 2.0;
  if (!(p > 0.0001)) {
    p = 0.0001;
  }

  *fval = p * x[1] - maximum(dv);
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = -0.01;
    *fval = fa;
  } else {
    if (!(*fval == 0.0)) {
      fc = *fval;
      c = x[1];
      e = 0.0;
      b_d = 0.0;
      exitg1 = false;
      while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
        if ((*fval > 0.0) == (fc > 0.0)) {
          c = a;
          fc = fa;
          b_d = *b - a;
          e = b_d;
        }

        if (fabs(fc) < fabs(*fval)) {
          a = *b;
          *b = c;
          c = a;
          fa = *fval;
          *fval = fc;
          fc = fa;
        }

        m = 0.5 * (c - *b);
        p = fabs(*b);
        if (!(p > 1.0)) {
          p = 1.0;
        }

        toler = 4.4408920985006262E-16 * p;
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            b_d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              p = 2.0 * m * s;
              fa = 1.0 - s;
            } else {
              fa /= fc;
              r = *fval / fc;
              p = s * (2.0 * m * fa * (fa - r) - (*b - a) * (r - 1.0));
              fa = (fa - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (p > 0.0) {
              fa = -fa;
            } else {
              p = -p;
            }

            if ((2.0 * p < 3.0 * m * fa - fabs(toler * fa)) && (p < fabs(0.5 * e
                  * fa))) {
              e = b_d;
              b_d = p / fa;
            } else {
              b_d = m;
              e = m;
            }
          }

          a = *b;
          fa = *fval;
          if (fabs(b_d) > toler) {
            *b += b_d;
          } else if (*b > c) {
            *b -= toler;
          } else {
            *b += toler;
          }

          p = u0_tmp + FunFcn_tunableEnvironment_f3 * *b;
          dv[0] = 0.0;
          dv[1] = d;
          if ((!(p < FunFcn_tunableEnvironment_f4)) && (!rtIsNaN
               (FunFcn_tunableEnvironment_f4))) {
            p = FunFcn_tunableEnvironment_f4;
          }

          p = (u0_tmp + p) / 2.0;
          if (!(p > 0.0001)) {
            p = 0.0001;
          }

          *fval = p * *b - maximum(dv);
        }
      }

      p = fabs(savefa);
      fa = fabs(savefb);
      if ((p > fa) || rtIsNaN(fa)) {
        fa = p;
      }

      if (!(fabs(*fval) <= fa)) {
        *exitflag = -5.0;
      }
    }
  }
}

/*
 * Arguments    : const short x[2]
 * Return Type  : short
 */
static short c_maximum(const short x[2])
{
  short ex;
  ex = x[0];
  if (x[0] < x[1]) {
    ex = x[1];
  }

  return ex;
}

/*
 * Arguments    : const short x[2]
 * Return Type  : short
 */
static short c_minimum(const short x[2])
{
  short ex;
  ex = x[0];
  if (x[0] > x[1]) {
    ex = x[1];
  }

  return ex;
}

/*
 * Arguments    : const double A[16]
 *                double B[4]
 * Return Type  : void
 */
static void c_mldivide(const double A[16], double B[4])
{
  double b_A[16];
  double s;
  double smax;
  int b_tmp;
  int i;
  int ijA;
  int ix;
  int j;
  int jA;
  int jp1j;
  int jy;
  int k;
  int mmj_tmp;
  signed char ipiv[4];
  signed char i1;
  memcpy(&b_A[0], &A[0], 16U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jy = 4 - j;
    jA = 0;
    ix = b_tmp;
    smax = fabs(b_A[b_tmp]);
    for (k = 2; k <= jy; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }

    if (b_A[b_tmp + jA] != 0.0) {
      if (jA != 0) {
        jy = j + jA;
        ipiv[j] = (signed char)(jy + 1);
        smax = b_A[j];
        b_A[j] = b_A[jy];
        b_A[jy] = smax;
        smax = b_A[j + 4];
        b_A[j + 4] = b_A[jy + 4];
        b_A[jy + 4] = smax;
        smax = b_A[j + 8];
        b_A[j + 8] = b_A[jy + 8];
        b_A[jy + 8] = smax;
        smax = b_A[j + 12];
        b_A[j + 12] = b_A[jy + 12];
        b_A[jy + 12] = smax;
      }

      i = (b_tmp - j) + 4;
      for (jy = jp1j; jy <= i; jy++) {
        b_A[jy - 1] /= b_A[b_tmp];
      }
    }

    jy = b_tmp + 4;
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = b_tmp + 1;
        i = jA + 6;
        k = (jA - j) + 8;
        for (ijA = i; ijA <= k; ijA++) {
          b_A[ijA - 1] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 4;
      jA += 4;
    }

    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }

  if (B[0] != 0.0) {
    for (jy = 2; jy < 5; jy++) {
      B[jy - 1] -= B[0] * b_A[jy - 1];
    }
  }

  if (B[1] != 0.0) {
    for (jy = 3; jy < 5; jy++) {
      B[jy - 1] -= B[1] * b_A[jy + 3];
    }
  }

  if (B[2] != 0.0) {
    for (jy = 4; jy < 5; jy++) {
      B[3] -= B[2] * b_A[11];
    }
  }

  if (B[3] != 0.0) {
    B[3] /= b_A[15];
    for (jy = 0; jy < 3; jy++) {
      B[jy] -= B[3] * b_A[jy + 12];
    }
  }

  if (B[2] != 0.0) {
    B[2] /= b_A[10];
    for (jy = 0; jy < 2; jy++) {
      B[jy] -= B[2] * b_A[jy + 8];
    }
  }

  if (B[1] != 0.0) {
    B[1] /= b_A[5];
    for (jy = 0; jy < 1; jy++) {
      B[0] -= B[1] * b_A[4];
    }
  }

  if (B[0] != 0.0) {
    B[0] /= b_A[0];
  }
}

/*
 * Arguments    : const emxArray_real_T *a
 *                const emxArray_real_T *b
 *                emxArray_real_T *c
 *                emxArray_int32_T *ia
 *                int ib_size[1]
 * Return Type  : void
 */
static void do_vectors(const emxArray_real_T *a, const emxArray_real_T *b,
  emxArray_real_T *c, emxArray_int32_T *ia, int ib_size[1])
{
  double absx;
  double ak;
  double bk;
  int b_ialast;
  int exponent;
  int iafirst;
  int ialast;
  int iblast;
  int na;
  int nc;
  int nia;
  boolean_T b_b;
  na = a->size[1];
  iblast = c->size[0] * c->size[1];
  c->size[0] = 1;
  c->size[1] = a->size[1];
  emxEnsureCapacity_real_T(c, iblast);
  iblast = ia->size[0];
  ia->size[0] = a->size[1];
  emxEnsureCapacity_int32_T(ia, iblast);
  ib_size[0] = 0;
  nc = 0;
  nia = 0;
  iafirst = 0;
  ialast = 1;
  iblast = 1;
  while ((ialast <= na) && (iblast <= b->size[1])) {
    b_ialast = ialast;
    ak = skip_to_last_equal_value(&b_ialast, a);
    ialast = b_ialast;
    bk = skip_to_last_equal_value(&iblast, b);
    absx = fabs(bk / 2.0);
    if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }

    if ((fabs(bk - ak) < absx) || (rtIsInf(ak) && rtIsInf(bk) && ((ak > 0.0) ==
          (bk > 0.0)))) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast++;
    } else {
      if (rtIsNaN(bk)) {
        b_b = !rtIsNaN(ak);
      } else {
        b_b = ((!rtIsNaN(ak)) && (ak < bk));
      }

      if (b_b) {
        nc++;
        nia++;
        c->data[nc - 1] = ak;
        ia->data[nia - 1] = iafirst + 1;
        ialast = b_ialast + 1;
        iafirst = b_ialast;
      } else {
        iblast++;
      }
    }
  }

  while (ialast <= na) {
    iblast = ialast;
    ak = skip_to_last_equal_value(&iblast, a);
    nc++;
    nia++;
    c->data[nc - 1] = ak;
    ia->data[nia - 1] = iafirst + 1;
    ialast = iblast + 1;
    iafirst = iblast;
  }

  if (a->size[1] > 0) {
    iblast = ia->size[0];
    if (1 > nia) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia;
    }

    emxEnsureCapacity_int32_T(ia, iblast);
    iblast = c->size[0] * c->size[1];
    if (1 > nc) {
      c->size[1] = 0;
    } else {
      c->size[1] = nc;
    }

    emxEnsureCapacity_real_T(c, iblast);
  }
}

/*
 * Arguments    : double a
 *                double d
 *                double b
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void eml_float_colon(double a, double d, double b, emxArray_real_T *y)
{
  double apnd;
  double cdiff;
  double ndbl;
  double u0;
  double u1;
  int k;
  int n;
  int nm1d2;
  ndbl = floor((b - a) / d + 0.5);
  apnd = a + ndbl * d;
  if (d > 0.0) {
    cdiff = apnd - b;
  } else {
    cdiff = b - apnd;
  }

  u0 = fabs(a);
  u1 = fabs(b);
  if ((u0 > u1) || rtIsNaN(u1)) {
    u1 = u0;
  }

  if (fabs(cdiff) < 4.4408920985006262E-16 * u1) {
    ndbl++;
    apnd = b;
  } else if (cdiff > 0.0) {
    apnd = a + (ndbl - 1.0) * d;
  } else {
    ndbl++;
  }

  if (ndbl >= 0.0) {
    n = (int)ndbl;
  } else {
    n = 0;
  }

  nm1d2 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity_real_T(y, nm1d2);
  if (n > 0) {
    y->data[0] = a;
    if (n > 1) {
      y->data[n - 1] = apnd;
      nm1d2 = (n - 1) / 2;
      for (k = 0; k <= nm1d2 - 2; k++) {
        ndbl = ((double)k + 1.0) * d;
        y->data[k + 1] = a + ndbl;
        y->data[(n - k) - 2] = apnd - ndbl;
      }

      if (nm1d2 << 1 == n - 1) {
        y->data[nm1d2] = (a + apnd) / 2.0;
      } else {
        ndbl = (double)nm1d2 * d;
        y->data[nm1d2] = a + ndbl;
        y->data[nm1d2 + 1] = apnd - ndbl;
      }
    }
  }
}

/*
 * Arguments    : double a
 *                short b
 *                emxArray_int16_T *y
 * Return Type  : void
 */
static void eml_integer_colon_dispatcher(double a, short b, emxArray_int16_T *y)
{
  int k;
  int n;
  unsigned short b_a;
  short yk;
  if (b < (short)a) {
    n = 0;
  } else {
    if (((short)a < 0) && (b >= 0)) {
      b_a = (unsigned short)((unsigned int)b - (unsigned short)(short)a);
    } else {
      b_a = (unsigned short)(b - (short)a);
    }

    n = b_a + 1;
  }

  k = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n;
  emxEnsureCapacity_int16_T(y, k);
  if (n > 0) {
    y->data[0] = (short)a;
    yk = (short)a;
    for (k = 2; k <= n; k++) {
      yk++;
      y->data[k - 1] = yk;
    }
  }
}

/*
 * Arguments    : const double c_FunFcn_tunableEnvironment_f1_[1]
 *                double FunFcn_tunableEnvironment_f2
 *                double FunFcn_tunableEnvironment_f3
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void fzero(const double c_FunFcn_tunableEnvironment_f1_[1], double
                  FunFcn_tunableEnvironment_f2, double
                  FunFcn_tunableEnvironment_f3, double *b, double *fval, double *
                  exitflag)
{
  double a;
  double c;
  double d;
  double e;
  double fa;
  double fa_tmp;
  double fc;
  double m;
  double p;
  double r;
  double s;
  double savefa;
  double savefb;
  double toler;
  boolean_T exitg1;
  *exitflag = 1.0;
  a = 1.0;
  *b = 4.0;
  fa_tmp = sqrt(FunFcn_tunableEnvironment_f2 * FunFcn_tunableEnvironment_f2 +
                FunFcn_tunableEnvironment_f3 * FunFcn_tunableEnvironment_f3);
  fa = (c_FunFcn_tunableEnvironment_f1_[0] + 1.0) - fa_tmp;
  *fval = (c_FunFcn_tunableEnvironment_f1_[0] * 4.0 + 16.0) - fa_tmp;
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = 1.0;
    *fval = fa;
  } else {
    if (!(*fval == 0.0)) {
      fc = *fval;
      c = 4.0;
      e = 0.0;
      d = 0.0;
      exitg1 = false;
      while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
        if ((*fval > 0.0) == (fc > 0.0)) {
          c = a;
          fc = fa;
          d = *b - a;
          e = d;
        }

        if (fabs(fc) < fabs(*fval)) {
          a = *b;
          *b = c;
          c = a;
          fa = *fval;
          *fval = fc;
          fc = fa;
        }

        m = 0.5 * (c - *b);
        p = fabs(*b);
        if (!(p > 1.0)) {
          p = 1.0;
        }

        toler = 4.4408920985006262E-16 * p;
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              p = 2.0 * m * s;
              fa = 1.0 - s;
            } else {
              fa /= fc;
              r = *fval / fc;
              p = s * (2.0 * m * fa * (fa - r) - (*b - a) * (r - 1.0));
              fa = (fa - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (p > 0.0) {
              fa = -fa;
            } else {
              p = -p;
            }

            if ((2.0 * p < 3.0 * m * fa - fabs(toler * fa)) && (p < fabs(0.5 * e
                  * fa))) {
              e = d;
              d = p / fa;
            } else {
              d = m;
              e = m;
            }
          }

          a = *b;
          fa = *fval;
          if (fabs(d) > toler) {
            *b += d;
          } else if (*b > c) {
            *b -= toler;
          } else {
            *b += toler;
          }

          *fval = (c_FunFcn_tunableEnvironment_f1_[0] * *b + *b * *b) - fa_tmp;
        }
      }

      p = fabs(savefa);
      fa = fabs(savefb);
      if ((p > fa) || rtIsNaN(fa)) {
        fa = p;
      }

      if (!(fabs(*fval) <= fa)) {
        *exitflag = -5.0;
      }
    }
  }
}

/*
 * Arguments    : double d2
 *                double y[100]
 * Return Type  : void
 */
static void linspace(double d2, double y[100])
{
  double delta1;
  int k;
  y[99] = d2;
  y[0] = 0.0;
  if (0.0 == -d2) {
    for (k = 0; k < 98; k++) {
      y[k + 1] = d2 * ((2.0 * ((double)k + 2.0) - 101.0) / 99.0);
    }
  } else if ((d2 < 0.0) && (fabs(d2) > 8.9884656743115785E+307)) {
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = delta1 * ((double)k + 1.0);
    }
  } else {
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = ((double)k + 1.0) * delta1;
    }
  }
}

/*
 * Arguments    : short a
 *                const emxArray_real_T *s
 * Return Type  : boolean_T
 */
static boolean_T local_ismember(short a, const emxArray_real_T *s)
{
  double absx;
  int exponent;
  int k;
  boolean_T exitg1;
  boolean_T tf;
  tf = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= s->size[1] - 1)) {
    absx = fabs(s->data[k] / 2.0);
    if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }

    if (fabs(s->data[k] - (double)a) < absx) {
      tf = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return tf;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double maximum(const double x[2])
{
  double ex;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double median(const double x[3])
{
  double y;
  int exitg1;
  int k;
  k = 0;
  do {
    exitg1 = 0;
    if (k < 3) {
      if (rtIsNaN(x[k])) {
        y = rtNaN;
        exitg1 = 1;
      } else {
        k++;
      }
    } else {
      if (x[0] < x[1]) {
        if (x[1] < x[2]) {
          k = 1;
        } else if (x[0] < x[2]) {
          k = 2;
        } else {
          k = 0;
        }
      } else if (x[0] < x[2]) {
        k = 0;
      } else if (x[1] < x[2]) {
        k = 2;
      } else {
        k = 1;
      }

      y = x[k];
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return y;
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 *                int np
 *                int nq
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  if (nq != 0) {
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork->data[j] = idx->data[iout];
      xwork->data[j] = x->data[iout];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[q]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[q];
        x->data[iout] = xwork->data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx->data[iout] = iwork->data[j - 1];
            x->data[iout] = xwork->data[j - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 *                int n
 *                int preSortLevel
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int bLen;
  int nPairs;
  int nTail;
  int tailOffset;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double minimum(const double x[2])
{
  double ex;
  if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }

  return ex;
}

/*
 * Arguments    : const double A[9]
 *                const double B[3]
 *                double Y[3]
 * Return Type  : void
 */
static void mldivide(const double A[9], const double B[3], double Y[3])
{
  double b_A[9];
  double a21;
  double maxval;
  int r1;
  int r2;
  int r3;
  int rtemp;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[r3 + 3];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - Y[2] * b_A[r1 + 6];
  Y[1] -= Y[2] * b_A[r2 + 6];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= Y[1] * b_A[r1 + 3];
  Y[0] /= b_A[r1];
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_remd_snf(double u0, double u1)
{
  double b_u1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = rtNaN;
  } else if (rtIsInf(u1)) {
    y = u0;
  } else {
    if (u1 < 0.0) {
      b_u1 = ceil(u1);
    } else {
      b_u1 = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      b_u1 = fabs(u0 / u1);
      if (!(fabs(b_u1 - floor(b_u1 + 0.5)) > DBL_EPSILON * b_u1)) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : int *k
 *                const emxArray_real_T *x
 * Return Type  : double
 */
static double skip_to_last_equal_value(int *k, const emxArray_real_T *x)
{
  double absx;
  double xk;
  int exponent;
  boolean_T exitg1;
  xk = x->data[*k - 1];
  exitg1 = false;
  while ((!exitg1) && (*k < x->size[1])) {
    absx = fabs(xk / 2.0);
    if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }

    if ((fabs(xk - x->data[*k]) < absx) || (rtIsInf(x->data[*k]) && rtIsInf(xk) &&
         ((x->data[*k] > 0.0) == (xk > 0.0)))) {
      (*k)++;
    } else {
      exitg1 = true;
    }
  }

  return xk;
}

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
static void sort(emxArray_real_T *x)
{
  emxArray_int32_T *b_iwork;
  emxArray_int32_T *iidx;
  emxArray_int32_T *iwork;
  emxArray_real_T *b_xwork;
  emxArray_real_T *vwork;
  emxArray_real_T *xwork;
  double c_xwork[256];
  double x4[4];
  double d;
  double d1;
  int c_iwork[256];
  int idx4[4];
  int b;
  int bLen;
  int b_b;
  int b_n;
  int dim;
  int exitg1;
  int i1;
  int i2;
  int i3;
  int i4;
  int iidx_tmp;
  int j;
  int k;
  int n;
  int nBlocks;
  int nNonNaN;
  int vlen;
  int vstride;
  signed char perm[4];
  dim = 0;
  if (x->size[0] != 1) {
    dim = -1;
  }

  emxInit_real_T(&vwork, 1);
  if (dim + 2 <= 1) {
    i2 = x->size[0];
  } else {
    i2 = 1;
  }

  vlen = i2 - 1;
  i1 = vwork->size[0];
  vwork->size[0] = i2;
  emxEnsureCapacity_real_T(vwork, i1);
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= x->size[0];
  }

  emxInit_int32_T(&iidx, 1);
  emxInit_int32_T(&iwork, 1);
  emxInit_real_T(&xwork, 1);
  emxInit_int32_T(&b_iwork, 1);
  emxInit_real_T(&b_xwork, 1);
  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork->data[k] = x->data[j + k * vstride];
    }

    i2 = iidx->size[0];
    iidx->size[0] = vwork->size[0];
    emxEnsureCapacity_int32_T(iidx, i2);
    dim = vwork->size[0];
    for (i2 = 0; i2 < dim; i2++) {
      iidx->data[i2] = 0;
    }

    if (vwork->size[0] != 0) {
      n = vwork->size[0];
      b_n = vwork->size[0];
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      dim = vwork->size[0];
      i2 = iwork->size[0];
      iwork->size[0] = vwork->size[0];
      emxEnsureCapacity_int32_T(iwork, i2);
      for (i2 = 0; i2 < dim; i2++) {
        iwork->data[i2] = 0;
      }

      dim = vwork->size[0];
      i2 = xwork->size[0];
      xwork->size[0] = vwork->size[0];
      emxEnsureCapacity_real_T(xwork, i2);
      for (i2 = 0; i2 < dim; i2++) {
        xwork->data[i2] = 0.0;
      }

      bLen = 0;
      dim = -1;
      for (k = 0; k < b_n; k++) {
        if (rtIsNaN(vwork->data[k])) {
          iidx_tmp = (b_n - bLen) - 1;
          iidx->data[iidx_tmp] = k + 1;
          xwork->data[iidx_tmp] = vwork->data[k];
          bLen++;
        } else {
          dim++;
          idx4[dim] = k + 1;
          x4[dim] = vwork->data[k];
          if (dim + 1 == 4) {
            dim = k - bLen;
            if (x4[0] <= x4[1]) {
              i1 = 1;
              i2 = 2;
            } else {
              i1 = 2;
              i2 = 1;
            }

            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }

            d = x4[i1 - 1];
            d1 = x4[i3 - 1];
            if (d <= d1) {
              d = x4[i2 - 1];
              if (d <= d1) {
                perm[0] = (signed char)i1;
                perm[1] = (signed char)i2;
                perm[2] = (signed char)i3;
                perm[3] = (signed char)i4;
              } else if (d <= x4[i4 - 1]) {
                perm[0] = (signed char)i1;
                perm[1] = (signed char)i3;
                perm[2] = (signed char)i2;
                perm[3] = (signed char)i4;
              } else {
                perm[0] = (signed char)i1;
                perm[1] = (signed char)i3;
                perm[2] = (signed char)i4;
                perm[3] = (signed char)i2;
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[i2 - 1] <= d1) {
                  perm[0] = (signed char)i3;
                  perm[1] = (signed char)i1;
                  perm[2] = (signed char)i2;
                  perm[3] = (signed char)i4;
                } else {
                  perm[0] = (signed char)i3;
                  perm[1] = (signed char)i1;
                  perm[2] = (signed char)i4;
                  perm[3] = (signed char)i2;
                }
              } else {
                perm[0] = (signed char)i3;
                perm[1] = (signed char)i4;
                perm[2] = (signed char)i1;
                perm[3] = (signed char)i2;
              }
            }

            iidx->data[dim - 3] = idx4[perm[0] - 1];
            iidx->data[dim - 2] = idx4[perm[1] - 1];
            iidx->data[dim - 1] = idx4[perm[2] - 1];
            iidx->data[dim] = idx4[perm[3] - 1];
            vwork->data[dim - 3] = x4[perm[0] - 1];
            vwork->data[dim - 2] = x4[perm[1] - 1];
            vwork->data[dim - 1] = x4[perm[2] - 1];
            vwork->data[dim] = x4[perm[3] - 1];
            dim = -1;
          }
        }
      }

      i3 = (b_n - bLen) - 1;
      if (dim + 1 > 0) {
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (dim + 1 == 1) {
          perm[0] = 1;
        } else if (dim + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }

        for (k = 0; k <= dim; k++) {
          iidx_tmp = perm[k] - 1;
          i1 = (i3 - dim) + k;
          iidx->data[i1] = idx4[iidx_tmp];
          vwork->data[i1] = x4[iidx_tmp];
        }
      }

      dim = (bLen >> 1) + 1;
      for (k = 0; k <= dim - 2; k++) {
        i1 = (i3 + k) + 1;
        i2 = iidx->data[i1];
        iidx_tmp = (b_n - k) - 1;
        iidx->data[i1] = iidx->data[iidx_tmp];
        iidx->data[iidx_tmp] = i2;
        vwork->data[i1] = xwork->data[iidx_tmp];
        vwork->data[iidx_tmp] = xwork->data[i1];
      }

      if ((bLen & 1) != 0) {
        i1 = i3 + dim;
        vwork->data[i1] = xwork->data[i1];
      }

      nNonNaN = n - bLen;
      i1 = 2;
      if (nNonNaN > 1) {
        if (n >= 256) {
          nBlocks = nNonNaN >> 8;
          if (nBlocks > 0) {
            for (b = 0; b < nBlocks; b++) {
              i4 = (b << 8) - 1;
              for (b_b = 0; b_b < 6; b_b++) {
                bLen = 1 << (b_b + 2);
                b_n = bLen << 1;
                n = 256 >> (b_b + 3);
                for (k = 0; k < n; k++) {
                  i2 = (i4 + k * b_n) + 1;
                  for (i1 = 0; i1 < b_n; i1++) {
                    dim = i2 + i1;
                    c_iwork[i1] = iidx->data[dim];
                    c_xwork[i1] = vwork->data[dim];
                  }

                  i3 = 0;
                  i1 = bLen;
                  dim = i2 - 1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (c_xwork[i3] <= c_xwork[i1]) {
                      iidx->data[dim] = c_iwork[i3];
                      vwork->data[dim] = c_xwork[i3];
                      if (i3 + 1 < bLen) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx->data[dim] = c_iwork[i1];
                      vwork->data[dim] = c_xwork[i1];
                      if (i1 + 1 < b_n) {
                        i1++;
                      } else {
                        dim -= i3;
                        for (i1 = i3 + 1; i1 <= bLen; i1++) {
                          iidx_tmp = dim + i1;
                          iidx->data[iidx_tmp] = c_iwork[i1 - 1];
                          vwork->data[iidx_tmp] = c_xwork[i1 - 1];
                        }

                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }

            dim = nBlocks << 8;
            i1 = nNonNaN - dim;
            if (i1 > 0) {
              merge_block(iidx, vwork, dim, i1, 2, iwork, xwork);
            }

            i1 = 8;
          }
        }

        dim = iwork->size[0];
        i2 = b_iwork->size[0];
        b_iwork->size[0] = iwork->size[0];
        emxEnsureCapacity_int32_T(b_iwork, i2);
        for (i2 = 0; i2 < dim; i2++) {
          b_iwork->data[i2] = iwork->data[i2];
        }

        i2 = b_xwork->size[0];
        b_xwork->size[0] = xwork->size[0];
        emxEnsureCapacity_real_T(b_xwork, i2);
        dim = xwork->size[0];
        for (i2 = 0; i2 < dim; i2++) {
          b_xwork->data[i2] = xwork->data[i2];
        }

        merge_block(iidx, vwork, 0, nNonNaN, i1, b_iwork, b_xwork);
      }
    }

    for (k = 0; k <= vlen; k++) {
      x->data[j + k * vstride] = vwork->data[k];
    }
  }

  emxFree_real_T(&b_xwork);
  emxFree_int32_T(&b_iwork);
  emxFree_real_T(&xwork);
  emxFree_int32_T(&iwork);
  emxFree_int32_T(&iidx);
  emxFree_real_T(&vwork);
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
static double sum(const double x_data[], const int x_size[2])
{
  double y;
  int k;
  int vlen;
  vlen = x_size[1];
  if (x_size[1] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= vlen; k++) {
      y += x_data[k - 1];
    }
  }

  return y;
}

/*
 * Arguments    : const double x[100]
 *                const double y[100]
 * Return Type  : double
 */
static double trapz(const double x[100], const double y[100])
{
  double c[100];
  double z;
  int b_iac;
  int ia;
  int iac;
  int ix;
  c[0] = 0.5 * (x[1] - x[0]);
  for (ix = 0; ix < 98; ix++) {
    c[ix + 1] = 0.5 * (x[ix + 2] - x[ix]);
  }

  c[99] = 0.5 * (x[99] - x[98]);
  z = 0.0;
  ix = 0;
  for (iac = 0; iac < 100; iac++) {
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      z += y[ia - 1] * c[ix];
    }

    ix++;
  }

  return z;
}

/*
 * ,
 * ,
 * Arguments    : const TypeBasicsInfo *BasicsInfo
 *                const TypeChassisInfo *ChassisInfo
 *                const TypeLaneChangeInfo *LaneChangeInfo
 *                const TypeAvoMainRoVehInfo *AvoMainRoVehInfo
 *                const TypeAvoPedInfo *AvoPedInfo
 *                const TypeTrafficLightInfo *TrafficLightInfo
 *                const TypeAvoOncomingVehInfo *AvoOncomingVehInfo
 *                const TypeAvoFailVehInfo *AvoFailVehInfo
 *                const TypeTurnAroundInfo *TurnAroundInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short TurnAroundActive
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters *Parameters
 *                struct0_T *Trajectory
 *                struct1_T *Decision
 * Return Type  : void
 */
void UrbanPlanner(const TypeBasicsInfo *BasicsInfo, const TypeChassisInfo
                  *ChassisInfo, const TypeLaneChangeInfo *LaneChangeInfo, const
                  TypeAvoMainRoVehInfo *AvoMainRoVehInfo, const TypeAvoPedInfo
                  *AvoPedInfo, const TypeTrafficLightInfo *TrafficLightInfo,
                  const TypeAvoOncomingVehInfo *AvoOncomingVehInfo, const
                  TypeAvoFailVehInfo *AvoFailVehInfo, const TypeTurnAroundInfo
                  *TurnAroundInfo, short LaneChangeActive, short
                  PedestrianActive, short TrafficLightActive, short
                  VehicleCrossingActive, short VehicleOncomingActive, short
                  TurnAroundActive, TypeGlobVars *GlobVars, const
                  TypeCalibrationVars *CalibrationVars, const TypeParameters
                  *Parameters, struct0_T *Trajectory, struct1_T *Decision)
{
  double LaneChangePath[720];
  double b_AvoOncomingVehInfo[6];
  double c_AvoOncomingVehInfo[6];
  double d_veh[6];
  double b_ChassisInfo[2];
  double D_safe;
  double TargetLaneFrontDis;
  double TargetLaneFrontVel;
  double V_c_end;
  double a_soll;
  double l_veh;
  double pos_s;
  double speed;
  double t_lc_traj;
  double timeGap;
  double w_veh;
  double x_tmp;
  int i;
  int i1;
  int i2;
  int k;
  short AEBActive;
  short BackupTargetLaneIndex;
  short CurrentTargetLaneIndex;
  short TargetLaneIndex;
  short dec_ped;
  short wait_ped;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T y;
  if (!isInitialized_UrbanPlanner) {
    UrbanPlanner_initialize();
  }

  /* ��� */
  pos_s = BasicsInfo->pos_s;

  /*  d_veh2int = BasicsInform.d_veh2int; */
  TargetLaneIndex = BasicsInfo->TargetLaneIndex;
  speed = ChassisInfo->speed;

  /*  d_veh2stopline = AvoMainRoVehInform.d_veh2stopline; */
  /*  s_veh2 = AvoOncomingVehInfo.s_veh2; */
  /*  v_veh2 = AvoOncomingVehInfo.v_veh2; */
  /*  d_veh2cross2 = AvoOncomingVehInfo.d_veh2cross2; */
  /*  s_veh1apostrophe2 = AvoOncomingVehInfo.s_veh1apostrophe2; */
  /*  s_veh3 = AvoOncomingVehInfo.s_veh3; */
  /*  v_veh3 = AvoOncomingVehInfo.v_veh3; */
  /*  d_veh2cross3 = AvoOncomingVehInfo.d_veh2cross3; */
  /*  s_veh1apostrophe3 = AvoOncomingVehInfo.s_veh1apostrophe3; */
  /*  d_veh2stopline = TrafficLightInform.d_veh2stopline; */
  /* --------------------------------------------------------- */
  CurrentTargetLaneIndex = GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex;
  t_lc_traj = GlobVars->TrajPlanLaneChange.t_lc_traj;
  memcpy(&LaneChangePath[0], &GlobVars->TrajPlanLaneChange.LaneChangePath[0],
         720U * sizeof(double));
  Decision->TargetGear = ChassisInfo->CurrentGear;

  /* ------------------------------------------------------- */
  memset(&Trajectory->traj_s[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_l[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_psi[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_vs[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_vl[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_omega[0], 0, 80U * sizeof(double));
  if (TurnAroundActive == 1) {
    GlobVars->TrajPlanTurnAround.TurnAroundActive = 1;
  }

  TurnAroundActive = GlobVars->TrajPlanTurnAround.TurnAroundActive;

  /*  ���ù��ϳ����ܣ���Ѱ��������linkǰ�����ϳ��� */
  BackupTargetLaneIndex = -1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if (AvoFailVehInfo->LanesWithFail[k] == 0) {
      k++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  if (y) {
    BackupTargetLaneIndex = LaneSelectionWithBlockedLanes
      (BasicsInfo->WidthOfLanes, AvoFailVehInfo->LanesWithFail, &TargetLaneIndex,
       BasicsInfo->CurrentLaneIndex);
  }

  a_soll = ACC(BasicsInfo->v_max, BasicsInfo->CurrentLaneFrontVel,
               BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed, 0.0,
               CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
               CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
               CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
               CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
               CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);

  /*  ����ͣ������ */
  /* , */
  AEBActive = GlobVars->AEBDecision.AEBActive;
  for (i = 0; i < 6; i++) {
    d_veh[i] = AvoOncomingVehInfo->s_veh[i];
  }

  for (i1 = 0; i1 < 6; i1++) {
    b_AvoOncomingVehInfo[i1] = AvoOncomingVehInfo->v_veh[i1];
  }

  for (i2 = 0; i2 < 6; i2++) {
    c_AvoOncomingVehInfo[i2] = AvoOncomingVehInfo->s_veh1apostrophe[i2];
  }

  AEBDecision(&AEBActive, AvoPedInfo->d_veh2cross, ChassisInfo->speed,
              AvoMainRoVehInfo->d_veh2stopline,
              AvoOncomingVehInfo->d_veh2waitingArea, d_veh, b_AvoOncomingVehInfo,
              AvoOncomingVehInfo->d_veh2conflict, c_AvoOncomingVehInfo,
              TrafficLightInfo->d_veh2stopline, TrafficLightInfo->greenLight,
              GlobVars, CalibrationVars->SpeedPlanTrafficLight.v_max_int,
              CalibrationVars->SpeedPlanAvoidOncomingVehicle.D_safe,
              Parameters->w_veh, Parameters->l_veh);

  /*  PrePedestrianActive=PedestrianActive; */
  if (AEBActive != 0) {
    /*  a_soll=min([ACC(0,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0),a_soll]); */
    timeGap = ChassisInfo->speed;
    if (ChassisInfo->speed < 0.0) {
      timeGap = -1.0;
    } else if (ChassisInfo->speed > 0.0) {
      timeGap = 1.0;
    } else {
      if (ChassisInfo->speed == 0.0) {
        timeGap = 0.0;
      }
    }

    b_ChassisInfo[0] = -4.0 * timeGap;
    b_ChassisInfo[1] = a_soll;
    a_soll = minimum(b_ChassisInfo);
  }

  if (PedestrianActive != 0) {
    /*  function [a_soll,dec_ped,wait_ped]=SpeedPlanAvoidPedestrian(speed,dec_ped,d_veh2cross,w_cross,wait_ped,s_ped,s_b,v_b,v_max) */
    b_ChassisInfo[0] = 0.0;
    b_ChassisInfo[1] = AvoPedInfo->d_veh2cross;
    l_veh = maximum(b_ChassisInfo);

    /* globalVariable--------------------------------------------------------------------------------------------------------------------------------------- */
    dec_ped = GlobVars->SpeedPlanAvoidPedestrian.dec_ped;
    wait_ped = GlobVars->SpeedPlanAvoidPedestrian.wait_ped;

    /*  CalibrationVariable--------------------------------------------------------------------------------------------------------------------------------- */
    /* 2.5; */
    /* -3; */
    /* v_max_in30/3.6; */
    /* v_max_int_emg=20/3.6 */
    /* Parameters------------------------------------------------------------------------------------------------------------------------------------------- */
    /* ----------------------------------------------------------------------------------------------------------------------------------------------------- */
    /*  v_ped=2; */
    b_ChassisInfo[0] = 1.0E-5;
    b_ChassisInfo[1] = 2.0 * AvoPedInfo->v_ped;
    timeGap = maximum(b_ChassisInfo);
    D_safe = fabs(AvoPedInfo->s_ped);

    /*  ����ģʽ�ж� */
    if (GlobVars->SpeedPlanAvoidPedestrian.dec_ped == 0) {
      if ((l_veh <= (0.0 - ChassisInfo->speed * ChassisInfo->speed) / (2.0 *
            CalibrationVars->SpeedPlanAvoidPedestrian.a_min) + 2.0 *
           Parameters->l_veh) && (l_veh > 1.0)) {
        /*  1ԭΪl_veh 01.21�޸� */
        dec_ped = 1;
      }
    } else {
      if ((l_veh <= 1.0) || (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1))
      {
        /*  1ԭΪl_veh 01.21�޸� */
        dec_ped = 0;
      }
    }

    /*  ͣ������ */
    if (dec_ped == 1) {
      b_ChassisInfo[0] = ChassisInfo->speed;
      b_ChassisInfo[1] = 1.0E-5;
      x_tmp = maximum(b_ChassisInfo);
      b_ChassisInfo[0] = (AvoPedInfo->w_cross + l_veh) / x_tmp * timeGap + 0.5 *
        Parameters->w_veh;
      b_ChassisInfo[1] = 0.0;
      if (D_safe <= maximum(b_ChassisInfo)) {
        wait_ped = 1;
      }
    }

    /*  �𲽾��� */
    if (wait_ped == 1) {
      b_ChassisInfo[0] = 0.0;
      b_ChassisInfo[1] = (D_safe - 0.5 * Parameters->w_veh) / timeGap;
      timeGap = maximum(b_ChassisInfo);
      if (l_veh < 10.0) {
        b_ChassisInfo[0] = ChassisInfo->speed +
          CalibrationVars->SpeedPlanAvoidPedestrian.a_max * timeGap;
        b_ChassisInfo[1] = CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int;
        if (0.5 * (minimum(b_ChassisInfo) + ChassisInfo->speed) * timeGap >
            AvoPedInfo->w_cross + l_veh) {
          wait_ped = 0;
        }
      }
    }

    /*  ACC�ٶȹ滮 */
    if (wait_ped == 0) {
      timeGap = b_ACC(CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int,
                      BasicsInfo->CurrentLaneFrontVel,
                      BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed, 0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    } else if (dec_ped == 1) {
      b_ChassisInfo[0] = b_ACC
        (CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int_emg,
         BasicsInfo->CurrentLaneFrontVel, BasicsInfo->CurrentLaneFrontDis,
         ChassisInfo->speed, wait_ped, CalibrationVars->ACC.a_max,
         CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc);
      b_ChassisInfo[1] = b_ACC
        (CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int_emg, 0.0, (l_veh +
          3.5) + Parameters->l_veh, ChassisInfo->speed, wait_ped,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
      timeGap = minimum(b_ChassisInfo);
    } else {
      b_ChassisInfo[0] = b_ACC
        (CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int,
         BasicsInfo->CurrentLaneFrontVel, BasicsInfo->CurrentLaneFrontDis,
         ChassisInfo->speed, wait_ped, CalibrationVars->ACC.a_max,
         CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc);
      b_ChassisInfo[1] = b_ACC
        (CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int, 0.0, (l_veh + 3.5)
         + Parameters->l_veh, ChassisInfo->speed, wait_ped,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.a_min_com, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
      timeGap = minimum(b_ChassisInfo);
    }

    if ((wait_ped == 0) && (dec_ped == 0) && (l_veh > 2.0 * Parameters->l_veh))
    {
      timeGap = b_ACC(BasicsInfo->v_max, BasicsInfo->CurrentLaneFrontVel,
                      BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed, 0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    }

    GlobVars->SpeedPlanAvoidPedestrian.dec_ped = dec_ped;
    GlobVars->SpeedPlanAvoidPedestrian.wait_ped = wait_ped;

    /*  d_veh2cross<=l_veh || wait_ped==1 ��<=l_veh���Ƿ���� */
    /*  �����к�� ���ˡ�ͣ�� */
    b_ChassisInfo[0] = timeGap;
    b_ChassisInfo[1] = a_soll;
    a_soll = minimum(b_ChassisInfo);
  } else {
    if (GlobVars->SpeedPlanAvoidPedestrian.dec_ped != 0) {
      GlobVars->SpeedPlanAvoidPedestrian.dec_ped = 0;
    }

    if (GlobVars->SpeedPlanAvoidPedestrian.wait_ped != 0) {
      GlobVars->SpeedPlanAvoidPedestrian.dec_ped = 0;
    }
  }

  if (TrafficLightActive != 0) {
    timeGap = SpeedPlanTrafficLight(ChassisInfo->speed,
      TrafficLightInfo->d_veh2stopline, BasicsInfo->CurrentLaneFrontDis,
      BasicsInfo->CurrentLaneFrontVel, TrafficLightInfo->greenLight,
      TrafficLightInfo->time2nextSwitch, GlobVars, Parameters->l_veh,
      CalibrationVars->SpeedPlanTrafficLight.a_min_com,
      CalibrationVars->SpeedPlanTrafficLight.a_max,
      CalibrationVars->SpeedPlanTrafficLight.a_min,
      CalibrationVars->SpeedPlanTrafficLight.v_max,
      CalibrationVars->SpeedPlanTrafficLight.v_max_int,
      CalibrationVars->SpeedPlanTrafficLight.t_acc, &CalibrationVars->ACC,
      CalibrationVars->ACCcust);
    b_ChassisInfo[0] = timeGap;
    b_ChassisInfo[1] = a_soll;
    a_soll = minimum(b_ChassisInfo);
  } else {
    if (GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight != 0) {
      GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight = 0;
    }

    if (GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight != 0) {
      GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight = 0;
    }

    if (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight != 0) {
      GlobVars->SpeedPlanTrafficLight.wait_TrafficLight = 0;
    }
  }

  if (VehicleCrossingActive != 0) {
    timeGap = SpeedPlanAvoidVehicle(ChassisInfo->speed,
      AvoMainRoVehInfo->d_veh2converge, AvoMainRoVehInfo->d_veh2stopline,
      AvoMainRoVehInfo->CurrentLaneFrontDisAvoidVehicle,
      AvoMainRoVehInfo->CurrentLaneFrontVelAvoidVehicle,
      AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle,
      AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle,
      AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle,
      AvoMainRoVehInfo->TargetLaneBehindVelAvoidVehicle, GlobVars,
      CalibrationVars->SpeedPlanAvoidVehicle.a_min_com,
      CalibrationVars->SpeedPlanAvoidVehicle.a_max,
      CalibrationVars->SpeedPlanAvoidVehicle.a_min,
      CalibrationVars->SpeedPlanAvoidVehicle.v_max,
      CalibrationVars->SpeedPlanAvoidVehicle.t_re,
      CalibrationVars->SpeedPlanAvoidVehicle.GapIndex, &CalibrationVars->ACC,
      Parameters->l_veh);
    b_ChassisInfo[0] = timeGap;
    b_ChassisInfo[1] = a_soll;
    a_soll = minimum(b_ChassisInfo);
  } else {
    if (GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle != 0) {
      GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle = 0;
    }

    if (GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle != 0) {
      GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle = 0;
    }

    if (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle != 0) {
      GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle = 0;
    }
  }

  if (VehicleOncomingActive != 0) {
    /* , */
    timeGap = ChassisInfo->speed;

    /* , */
    /* globalVariable---------------------------------------------------------------------------------------------------------------------- */
    dec_ped = GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle;
    wait_ped = GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;

    /*  ���ò��� */
    /*  a_max=2.5; */
    /* 1.5; */
    /* -3; */
    /* 30/3.6; */
    D_safe = CalibrationVars->SpeedPlanAvoidOncomingVehicle.D_safe;

    /* 2 */
    /*  v_max_overall=50/3.6; */
    l_veh = Parameters->l_veh;
    w_veh = Parameters->w_veh;

    /*  ����ģʽ�ж� */
    if (GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle == 0) {
      if ((AvoOncomingVehInfo->d_veh2waitingArea <= (0.0 - ChassisInfo->speed *
            ChassisInfo->speed) / (2.0 *
            CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_min) + 2.0 *
           Parameters->l_veh) && (AvoOncomingVehInfo->d_veh2waitingArea >
           Parameters->l_veh)) {
        dec_ped = 1;
      }
    } else {
      if ((AvoOncomingVehInfo->d_veh2waitingArea <= 0.5 * Parameters->l_veh) ||
          (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle ==
           1)) {
        dec_ped = 0;
      }
    }

    /*  ͣ������ */
    for (k = 0; k < 6; k++) {
      b_ChassisInfo[0] = timeGap;
      b_ChassisInfo[1] = 1.0E-5;
      x_tmp = maximum(b_ChassisInfo);
      b_ChassisInfo[0] = ((AvoOncomingVehInfo->d_veh2conflict[k] + l_veh) /
                          x_tmp * AvoOncomingVehInfo->v_veh[k] + 0.5 * w_veh) +
        D_safe;
      b_ChassisInfo[1] = 0.0;
      d_veh[k] = maximum(b_ChassisInfo);
    }

    if (dec_ped == 1) {
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k < 6)) {
        if ((AvoOncomingVehInfo->s_veh[k] <= d_veh[k]) ||
            (AvoOncomingVehInfo->s_veh1apostrophe[k] > -l_veh)) {
          wait_ped = 1;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    /*  �𲽾��� */
    if (wait_ped == 1) {
      /*      timeGap1=max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.5*(v_veh1+v_max_overall) 0.00001])]); */
      /*      if (v_max_overall-v_veh1)>a_max_com*timeGap1 */
      /*          fun_x=@(x)v_veh1*x+0.5*a_max_com*(x.^2); */
      /*          [timeGap1,~,~] = fzero(@(x)fun_x(x)-(s_veh1-0.5*w_veh-l_veh),[0 max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.00001])])]); */
      /*      end */
      for (k = 0; k < 6; k++) {
        b_ChassisInfo[0] = AvoOncomingVehInfo->v_veh[k];
        b_ChassisInfo[1] = 1.0E-5;
        x_tmp = maximum(b_ChassisInfo);
        b_ChassisInfo[0] = 0.0;
        b_ChassisInfo[1] = ((AvoOncomingVehInfo->s_veh[k] - 0.5 * w_veh) -
                            D_safe) / x_tmp;
        x_tmp = maximum(b_ChassisInfo);
        b_ChassisInfo[0] = timeGap +
          CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_max_com * x_tmp;
        b_ChassisInfo[1] =
          CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int;
        d_veh[k] = 0.5 * (minimum(b_ChassisInfo) + timeGap) * x_tmp;
      }

      wait_ped = 0;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k < 6)) {
        if ((AvoOncomingVehInfo->d_veh2waitingArea < 10.0) && (d_veh[k] >
             AvoOncomingVehInfo->d_veh2conflict[k] + l_veh) &&
            (AvoOncomingVehInfo->s_veh1apostrophe[k] < -l_veh)) {
          k++;
        } else {
          wait_ped = 1;
          exitg1 = true;
        }
      }
    }

    /*  ACC�ٶȹ滮 */
    if (wait_ped == 1) {
      /*  a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]); */
      b_ChassisInfo[0] = 0.0;
      b_ChassisInfo[1] = (AvoOncomingVehInfo->d_veh2waitingArea + 4.0) +
        Parameters->l_veh;
      x_tmp = maximum(b_ChassisInfo);
      b_ChassisInfo[0] = b_ACC
        (CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int,
         BasicsInfo->CurrentLaneFrontVel, BasicsInfo->CurrentLaneFrontDis,
         ChassisInfo->speed, 1, CalibrationVars->ACC.a_max,
         CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc);
      b_ChassisInfo[1] = b_ACC
        (CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int, 0.0, x_tmp,
         ChassisInfo->speed, 1, CalibrationVars->ACC.a_max,
         CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc);
      timeGap = minimum(b_ChassisInfo);
    } else if (dec_ped == 1) {
      timeGap = b_ACC(CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int,
                      BasicsInfo->CurrentLaneFrontVel,
                      BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed,
                      wait_ped, CalibrationVars->ACC.a_max,
                      CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    } else if (AvoOncomingVehInfo->d_veh2waitingArea > 2.0 * Parameters->l_veh)
    {
      timeGap = b_ACC(BasicsInfo->v_max, BasicsInfo->CurrentLaneFrontVel,
                      BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed,
                      wait_ped, CalibrationVars->ACC.a_max,
                      CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    } else {
      timeGap = b_ACC(CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int,
                      BasicsInfo->CurrentLaneFrontVel,
                      BasicsInfo->CurrentLaneFrontDis, ChassisInfo->speed,
                      wait_ped, CalibrationVars->ACC.a_max,
                      CalibrationVars->ACC.a_min, CalibrationVars->ACC.a_min_com,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc);
    }

    GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle = dec_ped;
    GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle = wait_ped;

    /*  v_veh1 */
    /*  a_soll */
    /*  if d_veh2waitingArea<5.4 */
    /*      s_veh1 */
    /*  end */
    /*  d_veh2waitingArea */
    /*  �������� */
    /*  speed=10; */
    /*  dec_avoidOncomingVehicle=0; */
    /*  d_veh2waitingArea=20; */
    /*  wait_avoidOncomingVehicle=0; */
    /*  s_b=100; */
    /*  v_b=10; */
    /*  s_veh1=40; */
    /*  v_veh1=5; */
    /*  d_veh2cross1=30; */
    /*  s_veh1apostrophe1=-10; */
    /*  s_veh2=200; */
    /*  v_veh2=0; */
    /*  d_veh2cross2=0; */
    /*  s_veh1apostrophe2=-200; */
    /*  s_veh3=200; */
    /*  v_veh3=0; */
    /*  d_veh2cross3=0; */
    /*  s_veh1apostrophe3=-200; */
    /*  v_max=50/3.6; */
    /*  [a_soll,dec_avoidOncomingVehicle,wait_avoidOncomingVehicle]=SpeedPlanAvoidOncomingVehicle(speed,dec_avoidOncomingVehicle,d_veh2waitingArea,wait_avoidOncomingVehicle,s_b,v_b,..., */
    /*      s_veh1,v_veh1,d_veh2cross1,s_veh1apostrophe1,s_veh2,v_veh2,d_veh2cross2,s_veh1apostrophe2,s_veh3,v_veh3,d_veh2cross3,s_veh1apostrophe3,v_max); */
    b_ChassisInfo[0] = timeGap;
    b_ChassisInfo[1] = a_soll;
    a_soll = minimum(b_ChassisInfo);
  } else {
    if (GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle != 0) {
      GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle = 0;
    }

    if (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle != 0)
    {
      GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle = 0;
    }
  }

  if (LaneChangeActive != 0) {
    if (CurrentTargetLaneIndex <= BasicsInfo->CurrentLaneIndex) {
      timeGap = LaneChangeInfo->LeftLaneBehindDis;
      D_safe = LaneChangeInfo->LeftLaneBehindVel;
      TargetLaneFrontDis = LaneChangeInfo->LeftLaneFrontDis;
      TargetLaneFrontVel = LaneChangeInfo->LeftLaneFrontVel;
    } else {
      timeGap = LaneChangeInfo->RightLaneBehindDis;
      D_safe = LaneChangeInfo->RightLaneBehindVel;
      TargetLaneFrontDis = LaneChangeInfo->RightLaneFrontDis;
      TargetLaneFrontVel = LaneChangeInfo->RightLaneFrontVel;
    }

    x_tmp = rt_roundd_snf(t_lc_traj / 0.05);
    k = GlobVars->TrajPlanLaneChange.DurationLaneChange - 1;
    if (k < -32768) {
      k = -32768;
    }

    b_ChassisInfo[0] = t_lc_traj - (double)k * 0.1;
    b_ChassisInfo[1] = 0.0;
    t_lc_traj = maximum(b_ChassisInfo);
    b_ChassisInfo[0] = 0.0;
    b_ChassisInfo[1] = TargetLaneFrontVel + -0.25 * t_lc_traj;
    V_c_end = maximum(b_ChassisInfo);
    b_ChassisInfo[0] = 0.0;
    b_ChassisInfo[1] = D_safe + 0.25 * t_lc_traj;
    w_veh = maximum(b_ChassisInfo);

    /*  if DurationLaneChange~=0  */
    /*      if (CurrentLaneIndex~=TargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) )   */
    /*          a=1; */
    /*      end */
    /*      a_soll_TrajPlanLaneChange=99; */
    /*  if (CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) ) || DurationLaneChange_RePlan~=0 */
    guard1 = false;
    guard2 = false;
    if ((BasicsInfo->CurrentLaneIndex != CurrentTargetLaneIndex) &&
        (GlobVars->TrajPlanLaneChange.DurationLaneChange != 0)) {
      timeGap = ((timeGap + BasicsInfo->pos_s) + 0.5 * (w_veh + D_safe) *
                 t_lc_traj) + w_veh * 0.5;
      b_ChassisInfo[0] = timeGap + 5.0;
      D_safe = LaneChangePath[(int)x_tmp + 359];
      l_veh = D_safe * D_safe;
      b_ChassisInfo[1] = (timeGap + (l_veh - w_veh * w_veh) / -7.0) + 5.0;
      x_tmp = LaneChangePath[(int)x_tmp - 1];
      if (x_tmp < maximum(b_ChassisInfo)) {
        guard1 = true;
      } else {
        timeGap = ((TargetLaneFrontDis + BasicsInfo->pos_s) + 0.5 * (V_c_end +
                    TargetLaneFrontVel) * t_lc_traj) - 0.5 * D_safe;
        b_ChassisInfo[0] = timeGap - 5.0;
        b_ChassisInfo[1] = (timeGap - (V_c_end * V_c_end - l_veh) / -7.0) - 5.0;
        if (x_tmp > minimum(b_ChassisInfo)) {
          guard1 = true;
        } else {
          guard2 = true;
        }
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if (GlobVars->TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan != 0) {
        guard1 = true;
      } else {
        /* , */
        /* , */
        /* , */
        TrajPlanLaneChange(BasicsInfo->CurrentLaneFrontDis,
                           BasicsInfo->CurrentLaneFrontVel,
                           LaneChangeInfo->LeftLaneBehindDis,
                           LaneChangeInfo->LeftLaneBehindVel,
                           LaneChangeInfo->LeftLaneFrontDis,
                           LaneChangeInfo->LeftLaneFrontVel,
                           LaneChangeInfo->RightLaneBehindDis,
                           LaneChangeInfo->RightLaneBehindVel,
                           LaneChangeInfo->RightLaneFrontDis,
                           LaneChangeInfo->RightLaneFrontVel, ChassisInfo->speed,
                           BasicsInfo->pos_s, BasicsInfo->pos_l_CurrentLane,
                           BasicsInfo->CurrentLaneIndex, TargetLaneIndex,
                           BackupTargetLaneIndex, LaneChangeInfo->d_veh2int,
                           BasicsInfo->WidthOfLanes, BasicsInfo->v_max,
                           AvoFailVehInfo->LanesWithFail, GlobVars,
                           CalibrationVars, *Parameters, &timeGap,
                           Trajectory->traj_s, Trajectory->traj_l,
                           Trajectory->traj_psi, Trajectory->traj_vs,
                           Trajectory->traj_vl, Trajectory->traj_omega);
      }
    }

    if (guard1) {
      /*   if abs(pos_l-pos_l_CurrentLane)>0.3 */
      /* , */
      TrajPlanLaneChange_RePlan(ChassisInfo->speed, BasicsInfo->pos_s,
        BasicsInfo->pos_l, BasicsInfo->pos_l_CurrentLane,
        BasicsInfo->WidthOfLanes, BasicsInfo->CurrentLaneIndex,
        BasicsInfo->CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel,
        GlobVars, CalibrationVars->TrajPlanLaneChange_RePlan.t_re,
        CalibrationVars->TrajPlanLaneChange_RePlan.index_accel,
        CalibrationVars->TrajPlanLaneChange_RePlan.a_min_comfort,
        CalibrationVars->TrajPlanLaneChange_RePlan.a_min, Parameters->l_veh,
        Trajectory->traj_s, Trajectory->traj_l, Trajectory->traj_psi,
        Trajectory->traj_vs, Trajectory->traj_vl, Trajectory->traj_omega);
      timeGap = 100.0;

      /*  end */
      GlobVars->TrajPlanLaneChange.CountLaneChange = 0;
      GlobVars->TrajPlanLaneChange.DurationLaneChange = 0;
    }

    if (timeGap != 100.0) {
      b_ChassisInfo[0] = timeGap;
      b_ChassisInfo[1] = a_soll;
      a_soll = minimum(b_ChassisInfo);
    } else {
      a_soll = 100.0;
    }
  } else {
    if (GlobVars->TrajPlanLaneChange.CountLaneChange != 0) {
      GlobVars->TrajPlanLaneChange.CountLaneChange = 0;
    }

    if (GlobVars->TrajPlanLaneChange.DurationLaneChange != 0) {
      GlobVars->TrajPlanLaneChange.DurationLaneChange = 0;
    }
  }

  if (TurnAroundActive != 0) {
    /* , */
    /* , */
    /* , */
    TrajPlanTurnAround(BasicsInfo->CurrentLaneFrontDis,
                       BasicsInfo->CurrentLaneFrontVel, ChassisInfo->speed,
                       BasicsInfo->pos_l_CurrentLane, BasicsInfo->pos_s,
                       BasicsInfo->pos_l, TurnAroundInfo->NumOfLanesOpposite,
                       TurnAroundInfo->WidthOfLanesOpposite,
                       TurnAroundInfo->WidthOfGap, BasicsInfo->WidthOfLanes,
                       TurnAroundInfo->s_turnaround_border,
                       TurnAroundInfo->IndexOfLaneOppositeCar,
                       TurnAroundInfo->SpeedOppositeCar,
                       TurnAroundInfo->PosSOppositeCar,
                       TurnAroundInfo->IndexOfLaneCodirectCar,
                       TurnAroundInfo->SpeedCodirectCar,
                       TurnAroundInfo->PosSCodirectCar,
                       BasicsInfo->CurrentLaneIndex, BasicsInfo->v_max, a_soll,
                       ChassisInfo->CurrentGear, &TurnAroundActive, &AEBActive,
                       GlobVars, CalibrationVars, *Parameters, &timeGap,
                       Trajectory->traj_s, Trajectory->traj_l,
                       Trajectory->traj_psi, Trajectory->traj_vs,
                       Trajectory->traj_vl, Trajectory->traj_omega,
                       &Decision->TargetGear);
    if (timeGap != 100.0) {
      b_ChassisInfo[0] = timeGap;
      b_ChassisInfo[1] = a_soll;
      a_soll = minimum(b_ChassisInfo);
    } else {
      a_soll = 100.0;
    }
  } else {
    if (GlobVars->TrajPlanTurnAround.dec_trunAround != 0) {
      GlobVars->TrajPlanTurnAround.dec_trunAround = 0;
    }

    if (GlobVars->TrajPlanTurnAround.TurnAroundState != 0) {
      GlobVars->TrajPlanTurnAround.TurnAroundState = 0;
    }

    if (GlobVars->TrajPlanTurnAround.TargetLaneIndexOpposite != 0) {
      GlobVars->TrajPlanTurnAround.TargetLaneIndexOpposite = 0;
    }

    if (GlobVars->TrajPlanTurnAround.wait_turnAround != 0) {
      GlobVars->TrajPlanTurnAround.wait_turnAround = 0;
    }

    if (GlobVars->TrajPlanTurnAround.TypeOfTurnAround != 0) {
      GlobVars->TrajPlanTurnAround.TypeOfTurnAround = 0;
    }
  }

  if (a_soll != 100.0) {
    b_ChassisInfo[0] = 0.0;
    for (k = 0; k < 80; k++) {
      timeGap = 0.05 * ((double)k + 1.0);
      b_ChassisInfo[1] = speed + a_soll * timeGap;
      x_tmp = maximum(b_ChassisInfo);
      Trajectory->traj_vs[k] = x_tmp;
      Trajectory->traj_vl[k] = 0.0;
      Trajectory->traj_omega[k] = 0.0;
      Trajectory->traj_l[k] = BasicsInfo->pos_l_CurrentLane;
      Trajectory->traj_psi[k] = 90.0;
      if (x_tmp == 0.0) {
        Trajectory->traj_s[k] = pos_s + (0.0 - speed * speed) / (2.0 * a_soll +
          2.2204460492503131E-16);
      } else {
        Trajectory->traj_s[k] = pos_s + (x_tmp + speed) * timeGap / 2.0;
      }
    }
  }

  Decision->Driverwait = 0.0;
  Decision->wait_avoidOncomingVehicle =
    GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;
  Decision->wait_ped = GlobVars->SpeedPlanAvoidPedestrian.wait_ped;
  Decision->wait_AvoidVehicle =
    GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle;
  Decision->wait_TrafficLight =
    GlobVars->SpeedPlanTrafficLight.wait_TrafficLight;
  Decision->wait_turnAround = GlobVars->TrajPlanTurnAround.wait_turnAround;
  Decision->a_soll = a_soll;
  Decision->AEBActive = AEBActive;
  GlobVars->AEBDecision.AEBActive = AEBActive;
  GlobVars->TrajPlanTurnAround.TurnAroundActive = TurnAroundActive;

  /*  ȫ�ֱ����������� */
  /*  �����ò�����t_re l_veh w_veh tau_v tau_d a_max a_min a_min_comfort */
  /*  if TrafficLightActive */
  /*  a_soll_TrafficLightActive */
  /*  end */
  /*  if VehicleCrossingActive */
  /*  a_soll_SpeedPlanAvoidVehicle */
  /*  end */
  /*  a_soll */
  /*  a_soll */
  /*  speed */
  /*  �������� */
  /*  [a_soll,traj_s,traj_l,traj_psi,traj_vs,traj_vl,traj_omega,CountLaneChange,DurationLaneChange,LaneChangePath,t_lc_traj,dec_ped,wait_ped,dec_fol,dec_bre,wait]=..., */
  /*      UrbanPlanner(CurrentLaneFrontDis,CurrentLaneFrontVel,TargetLaneBehindDis,TargetLaneBehindVel,TargetLaneFrontDis,TargetLaneFrontVel,speed,pos_s,pos_l,CurrentLaneIndex,..., */
  /*      CountLaneChange,DurationLaneChange,LaneChangePath,TargetLaneIndex,t_lc_traj,d_veh2int,w_lane,dec_ped,d_veh2cross,wait_ped,s_ped,dec_fol,dec_bre,wait,greenLight,time2nextSwitch,v_max,..., */
  /*      LaneChangeActive,PedestrianActive,TrafficLightActive,VehicleCrossingActive); */
  /*  if strcmp(current_road_ID,'12') && pos_s>210 */
  /*      if a_soll==100 */
  /*          traci.vehicle.moveToXY('S0','12', 2, traj_l(2), traj_s(2),90-traj_psi(2),2); */
  /*      else */
  /*          traci.vehicle.setSpeed('S0',traj_vs(2)); */
  /*          traci.vehicle.changeLane('S0',1-CurrentLaneIndex,2); */
  /*      end */
  /*  else */
  /*      traci.vehicle.setSpeed('S0',traj_vs(2)); */
  /*  end */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void UrbanPlanner_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_UrbanPlanner = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void UrbanPlanner_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_UrbanPlanner = false;
}

/*
 * File trailer for UrbanPlanner.c
 *
 * [EOF]
 */
