/*
 * File: UrbanPlanner_emxutil.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 12-Oct-2022 16:15:41
 */

#ifndef URBANPLANNER_EMXUTIL_H
#define URBANPLANNER_EMXUTIL_H

/* Include Files */
#include "UrbanPlanner_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
    oldNumel);
  extern void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel);
  extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
  extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
  extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
  extern void emxEnsureCapacity_uint16_T(emxArray_uint16_T *emxArray, int
    oldNumel);
  extern void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int
    oldNumel);
  extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
  extern void emxFree_int16_T(emxArray_int16_T **pEmxArray);
  extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
  extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
  extern void emxFree_real_T(emxArray_real_T **pEmxArray);
  extern void emxFree_uint16_T(emxArray_uint16_T **pEmxArray);
  extern void emxFree_uint32_T(emxArray_uint32_T **pEmxArray);
  extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int
    numDimensions);
  extern void emxInit_int16_T(emxArray_int16_T **pEmxArray, int numDimensions);
  extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
  extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
  extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
  extern void emxInit_uint16_T(emxArray_uint16_T **pEmxArray, int numDimensions);
  extern void emxInit_uint32_T(emxArray_uint32_T **pEmxArray, int numDimensions);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for UrbanPlanner_emxutil.h
 *
 * [EOF]
 */
