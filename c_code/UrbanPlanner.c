/*
 * File: UrbanPlanner.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
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

/* Type Definitions */
#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3

typedef struct {
  double f1[6];
} cell_wrap_3;

#endif                                 /*typedef_cell_wrap_3*/

#ifndef typedef_cell_wrap_5
#define typedef_cell_wrap_5

typedef struct {
  double f1[3];
} cell_wrap_5;

#endif                                 /*typedef_cell_wrap_5*/

#ifndef typedef_anonymous_function
#define typedef_anonymous_function

typedef struct {
  cell_wrap_3 tunableEnvironment[1];
} anonymous_function;

#endif                                 /*typedef_anonymous_function*/

#ifndef typedef_b_anonymous_function
#define typedef_b_anonymous_function

typedef struct {
  cell_wrap_5 tunableEnvironment[1];
} b_anonymous_function;

#endif                                 /*typedef_b_anonymous_function*/

// 附加c文件1
/*
 * File: rt_nonfinite.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
 */

 /*
  * Abstract:
  *      MATLAB for code generation function to initialize non-finites,
  *      (Inf, NaN and -Inf).
  */
  /* Include Files */
#include "rt_nonfinite.h"
#include <math.h>

/* Suppress overflow warning with Intel 17. */
#if defined(__ICL) && __ICL == 1700

#pragma warning(disable: 264)

#endif

real_T rtNaN = (real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = (real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

#if defined(__ICL) && __ICL == 1700

#pragma warning(default: 264)

#endif

/* Function: rtIsInf ==================================================
 * Abstract:
 * Test if value is infinite
 */
boolean_T rtIsInf(real_T value)
{
	return (isinf(value) != 0U);
}

/* Function: rtIsInfF =================================================
 * Abstract:
 * Test if single-precision value is infinite
 */
boolean_T rtIsInfF(real32_T value)
{
	return (isinf((real_T)value) != 0U);
}

/* Function: rtIsNaN ==================================================
 * Abstract:
 * Test if value is not a number
 */
boolean_T rtIsNaN(real_T value)
{
	return (isnan(value) != 0U);
}

/* Function: rtIsNaNF =================================================
 * Abstract:
 * Test if single-precision value is not a number
 */
boolean_T rtIsNaNF(real32_T value)
{
	return (isnan((real_T)value) != 0U);
}

/*
 * File trailer for rt_nonfinite.c
 *
 * [EOF]
 */

// 附加c文件2
/*
 * File: rtGetInf.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
 */

 /*
  * Abstract:
  *       MATLAB for code generation function to initialize non-finite, Inf and MinusInf
  */
  /* Include Files */
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Function: rtGetInf ==================================================================
 * Abstract:
 * Initialize rtInf needed by the generated code.
 */
real_T rtGetInf(void)
{
	return rtInf;
}

/* Function: rtGetInfF =================================================================
 * Abstract:
 * Initialize rtInfF needed by the generated code.
 */
real32_T rtGetInfF(void)
{
	return rtInfF;
}

/* Function: rtGetMinusInf =============================================================
 * Abstract:
 * Initialize rtMinusInf needed by the generated code.
 */
real_T rtGetMinusInf(void)
{
	return rtMinusInf;
}

/* Function: rtGetMinusInfF ============================================================
 * Abstract:
 * Initialize rtMinusInfF needed by the generated code.
 */
real32_T rtGetMinusInfF(void)
{
	return rtMinusInfF;
}

/*
 * File trailer for rtGetInf.c
 *
 * [EOF]
 */

// 附加c文件3
/*
 * File: rtGetNaN.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
 */

 /*
  * Abstract:
  *       MATLAB for code generation function to initialize non-finite, NaN
  */
  /* Include Files */
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/* Function: rtGetNaN ======================================================================
 * Abstract:
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
	return rtNaN;
}

/* Function: rtGetNaNF =====================================================================
 * Abstract:
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
real32_T rtGetNaNF(void)
{
	return rtNaNF;
}

/*
 * File trailer for rtGetNaN.c
 *
 * [EOF]
 */

// 附件c文件3
/*
 * File: UrbanPlanner_emxutil.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
 */

 /* Include Files */
#include "UrbanPlanner_emxutil.h"
#include "UrbanPlanner_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>
#include <string.h>

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
			}
			else {
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
			}
			else {
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
			}
			else {
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
			}
			else {
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
			}
			else {
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
			}
			else {
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

/* Function Declarations */
static double ACC(double v_max, double v_soll, double d_ist, double speed,
                  double b_wait, double CalibrationVars_ACC_a_max, double
                  CalibrationVars_ACC_a_min, double
                  CalibrationVars_ACC_tau_v_com, double
                  CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                  double CalibrationVars_ACC_tau_v_bre, double
                  CalibrationVars_ACC_tau_v_emg, double
                  CalibrationVars_ACC_tau_d_emg, double
                  CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait);
static double ACClowSpeed(double v_max, double v_soll, double d_ist, double
  speed, double c_CalibrationVars_ACClowSpeed_a, double
  d_CalibrationVars_ACClowSpeed_a, double e_CalibrationVars_ACClowSpeed_a,
  double c_CalibrationVars_ACClowSpeed_t, double d_CalibrationVars_ACClowSpeed_t,
  double e_CalibrationVars_ACClowSpeed_t, double f_CalibrationVars_ACClowSpeed_t,
  double g_CalibrationVars_ACClowSpeed_t, double h_CalibrationVars_ACClowSpeed_t,
  double i_CalibrationVars_ACClowSpeed_t, double j_CalibrationVars_ACClowSpeed_t,
  double c_CalibrationVars_ACClowSpeed_d);
static void AEBDecision(short *AEBActive, double speed, double
  d_veh2stopline_ped, double d_veh2stopline, double d_veh2waitingArea, double
  s_veh[6], double v_veh[6], const double d_veh2conflict[6], double
  s_vehapostrophe[6], double d_veh2int, double greenLight, TypeGlobVars
  *GlobVars, double c_CalibrationVars_SpeedPlanTraf, double
  c_CalibrationVars_SpeedPlanAvoi, double CalibrationVars_ACC_a_min, const
  TypeParameters Parameters);
static struct1_T Decider(short PlannerLevel, double
  BasicsInfo_CurrentLaneFrontDis, double BasicsInfo_CurrentLaneFrontVel, double
  BasicsInfo_CurrentLaneFrontLen, short BasicsInfo_CurrentLaneIndex, const
  double BasicsInfo_WidthOfLanes[6], double BasicsInfo_v_max, double
  BasicsInfo_d_veh2goal, double BasicsInfo_SampleTime, double ChassisInfo_speed,
  const TypeLaneChangeInfo *LaneChangeInfo, const TypeAvoMainRoVehInfo
  *AvoMainRoVehInfo, const TypeAvoPedInfo *AvoPedInfo, const
  TypeTrafficLightInfo TrafficLightInfo, const TypeAvoOncomingVehInfo
  *AvoOncomingVehInfo, const TypeStopSignInfo StopSignInfo, short
  LaneChangeActive, short PedestrianActive, short TrafficLightActive, short
  VehicleCrossingActive, short VehicleOncomingActive, short AEBActive, short
  TargetGear, double a_soll_ACC, double a_soll_SpeedPlanAvoidPedestrian, double
  a_soll_TrafficLightActive, double a_soll_SpeedPlanAvoidVehicle, double
  c_a_soll_SpeedPlanAvoidOncoming, short TargetLaneIndex, short
  BackupTargetLaneIndex, double d_veh2stopline_ped, TypeGlobVars *GlobVars,
  const TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters);
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
static double SpeedPlanAvoidOncomingVehicle(double speed, double
  d_veh2waitingArea, double s_b, double v_b, const double s_veh[6], const double
  v_veh[6], const double d_veh2conflict[6], const double s_vehapostrophe[6],
  double v_max, TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanAvoi,
  double d_CalibrationVars_SpeedPlanAvoi, double e_CalibrationVars_SpeedPlanAvoi,
  double f_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC,
  double Parameters_w_veh, double Parameters_l_veh);
static void SpeedPlanAvoidPedestrian(double pos_s, double speed, double
  d_veh2cross, double w_cross, const double s_ped[40], const double l_ped[40],
  const double v_ped[40], const double psi_ped[40], double s_b, double v_b,
  double v_max, TypeGlobVars *GlobVars, double Parameters_w_veh, double
  Parameters_l_veh, const CalibSpeedPlanAvoidPedestrian
  c_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC, double
  *a_soll, double *d_veh2stopline);
static double SpeedPlanAvoidVehicle(double speed, double d_veh2int, double
  d_veh2stopline, double s_a, double v_a, double l_a, double s_b, double v_b,
  double l_b, double s_c, double v_c, double l_c, TypeGlobVars *GlobVars, const
  CalibSpeedPlanAvoidVehicle c_CalibrationVars_SpeedPlanAvoi, const CalibACC
  *CalibrationVars_ACC, double Parameters_l_veh);
static double SpeedPlanTrafficLight(double speed, double d_veh2int, double s_b,
  double v_b, double greenLight, double time2nextSwitch, double v_max,
  TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanTraf, double
  d_CalibrationVars_SpeedPlanTraf, double e_CalibrationVars_SpeedPlanTraf,
  double f_CalibrationVars_SpeedPlanTraf, const CalibACC *CalibrationVars_ACC);
static void TrajPlanLaneChange(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double LeftLaneBehindDis, double LeftLaneBehindVel,
  double LeftLaneFrontDis, double LeftLaneFrontVel, double RightLaneBehindDis,
  double RightLaneBehindVel, double RightLaneFrontDis, double RightLaneFrontVel,
  double speed, double pos_s, double pos_l_CurrentLane, short CurrentLaneIndex,
  short TargetLaneIndex, short GoalLaneIndex, short BackupTargetLaneIndex,
  double d_veh2int, double d_veh2goal, double WidthOfLanes[6], double v_max,
  const short LanesWithFail[6], TypeGlobVars *GlobVars, const
  TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters, double *
  a_soll, double traj_s[80], double traj_l[80], double traj_psi[80], double
  traj_vs[80], double traj_vl[80], double traj_omega[80]);
static void TrajPlanLaneChange_RePlan(double speed, double pos_s, double pos_l,
  double pos_l_CurrentLane, const double WidthOfLanes[6], short CurrentLaneIndex,
  double CurrentLaneFrontDis, double CurrentLaneFrontVel, TypeGlobVars *GlobVars,
  double c_CalibrationVars_TrajPlanLaneC, double d_CalibrationVars_TrajPlanLaneC,
  double e_CalibrationVars_TrajPlanLaneC, double f_CalibrationVars_TrajPlanLaneC,
  double traj_s[80], double traj_l[80], double traj_psi[80], double traj_vs[80],
  double traj_vl[80], double traj_omega[80]);
static void TrajPlanTurnAround(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double speed, double pos_l_CurrentLane, double pos_s,
  double pos_l, short NumOfLanesOpposite, const double WidthOfLanesOpposite[6],
  double WidthOfGap, const double WidthOfLanes[6], double s_turnaround_border,
  const short IndexOfLaneOppositeCar[20], const double SpeedOppositeCar[20],
  const double PosSOppositeCar[20], const double LengthOppositeCar[20], const
  short IndexOfLaneCodirectCar[10], const double SpeedCodirectCar[10], const
  double PosSCodirectCar[10], const double LengthCodirectCar[20], short
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
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait);
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
static void b_minimum(const double x_data[], const int x_size[2], double *ex,
                      int *idx);
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
static double c_minimum(const double x[2]);
static void c_mldivide(const double A[16], double B[4]);
static double d_minimum(const double x[3]);
static void do_vectors(const emxArray_real_T *a, const emxArray_real_T *b,
  emxArray_real_T *c, emxArray_int32_T *ia, int ib_size[1]);
static double e_minimum(const double x_data[], const int x_size[2]);
static void eml_float_colon(double a, double d, double b, emxArray_real_T *y);
static void eml_integer_colon_dispatcher(double a, short b, emxArray_int16_T *y);
static short f_minimum(const short x[2]);
static void fzero(const double c_FunFcn_tunableEnvironment_f1_[1], double
                  FunFcn_tunableEnvironment_f2, double
                  FunFcn_tunableEnvironment_f3, double *b, double *fval, double *
                  exitflag);
static double g_minimum(const double x[7]);
static double h_minimum(const double x[8]);
static void linspace(double d2, double y[100]);
static boolean_T local_ismember(short a, const emxArray_real_T *s);
static double maximum(const double x[2]);
static double median(const double x[3]);
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void minimum(const double x[4], double *ex, int *idx);
static void mldivide(const double A[9], const double B[3], double Y[3]);
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_remd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static double skip_to_last_equal_value(int *k, const emxArray_real_T *x);
static void sort(emxArray_real_T *x);
static double sum(const double x_data[], const int x_size[2]);
static double trapz(const double x[100], const double y[100]);
static boolean_T vectorAny(const short x_data[], const int x_size[2]);

/* Function Definitions */
/*
 * Arguments    : double v_max
 *                double v_soll
 *                double d_ist
 *                double speed
 *                double b_wait
 *                double CalibrationVars_ACC_a_max
 *                double CalibrationVars_ACC_a_min
 *                double CalibrationVars_ACC_tau_v_com
 *                double CalibrationVars_ACC_tau_v
 *                double CalibrationVars_ACC_tau_d
 *                double CalibrationVars_ACC_tau_v_bre
 *                double CalibrationVars_ACC_tau_v_emg
 *                double CalibrationVars_ACC_tau_d_emg
 *                double CalibrationVars_ACC_t_acc
 *                double CalibrationVars_ACC_d_wait
 * Return Type  : double
 */
static double ACC(double v_max, double v_soll, double d_ist, double speed,
                  double b_wait, double CalibrationVars_ACC_a_max, double
                  CalibrationVars_ACC_a_min, double
                  CalibrationVars_ACC_tau_v_com, double
                  CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                  double CalibrationVars_ACC_tau_v_bre, double
                  CalibrationVars_ACC_tau_v_emg, double
                  CalibrationVars_ACC_tau_d_emg, double
                  CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait)
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
  /* 4 */
  accel = 100.0;
  if (d_ist < 100.0) {
    if (b_wait == -1.0) {
      /*  停车距离远一些，避免停在故障车后面停得过近，无法换道 */
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 17.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    } else {
      /*          d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]); */
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = CalibrationVars_ACC_d_wait;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    }

    if ((v_soll == 0.0) && (d_ist <= CalibrationVars_ACC_d_wait + 0.15)) {
      d_soll = speed;
      if (speed < 0.0) {
        d_soll = -1.0;
      } else if (speed > 0.0) {
        d_soll = 1.0;
      } else {
        if (speed == 0.0) {
          d_soll = 0.0;
        }
      }

      accel = -4.0 * d_soll;
    } else {
      if (v_soll == 0.0) {
        v_soll = 0.6;
      }

      if (d_ist < CalibrationVars_ACC_d_wait) {
        accel = ((v_soll - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
      } else if ((d_ist < CalibrationVars_ACC_d_wait + 3.0) && (b_wait == 0.0))
      {
        /*  */
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_emg;

        /*  elseif wait==1 || speed.^2/d_ist/2>-a_min_com */
        /*      accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_bre; */
      } else if (b_wait == 1.0) {
        if (d_ist > CalibrationVars_ACC_d_wait + 15.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_com;
        } else if (d_ist > CalibrationVars_ACC_d_wait + 10.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v;
        } else if (d_ist > CalibrationVars_ACC_d_wait + 5.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_bre;
        } else {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_emg;
        }
      } else if (fabs(speed - v_soll) < 2.0) {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_com;
      } else {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v;
      }
    }
  }

  b_accel[0] = -2.5;
  b_accel[1] = (v_max - speed) / CalibrationVars_ACC_tau_v;
  d_soll = maximum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_soll;
  accel = c_minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_max;
  accel = c_minimum(b_accel);
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
 *                double c_CalibrationVars_ACClowSpeed_d
 * Return Type  : double
 */
static double ACClowSpeed(double v_max, double v_soll, double d_ist, double
  speed, double c_CalibrationVars_ACClowSpeed_a, double
  d_CalibrationVars_ACClowSpeed_a, double e_CalibrationVars_ACClowSpeed_a,
  double c_CalibrationVars_ACClowSpeed_t, double d_CalibrationVars_ACClowSpeed_t,
  double e_CalibrationVars_ACClowSpeed_t, double f_CalibrationVars_ACClowSpeed_t,
  double g_CalibrationVars_ACClowSpeed_t, double h_CalibrationVars_ACClowSpeed_t,
  double i_CalibrationVars_ACClowSpeed_t, double j_CalibrationVars_ACClowSpeed_t,
  double c_CalibrationVars_ACClowSpeed_d)
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
  /* 4 */
  accel = 100.0;
  if (d_ist < 100.0) {
    /*      d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]); */
    b_speed[0] = speed * j_CalibrationVars_ACClowSpeed_t;
    b_speed[1] = c_CalibrationVars_ACClowSpeed_d;
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

    if (d_ist < c_CalibrationVars_ACClowSpeed_d) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               h_CalibrationVars_ACClowSpeed_t) /
        g_CalibrationVars_ACClowSpeed_t;
    } else {
      if (d_ist < c_CalibrationVars_ACClowSpeed_d + 2.0) {
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
  accel = c_minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = c_CalibrationVars_ACClowSpeed_a;
  accel = c_minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_CalibrationVars_ACClowSpeed_a;
  return maximum(b_accel);
}

/*
 * ,
 * Arguments    : short *AEBActive
 *                double speed
 *                double d_veh2stopline_ped
 *                double d_veh2stopline
 *                double d_veh2waitingArea
 *                double s_veh[6]
 *                double v_veh[6]
 *                const double d_veh2conflict[6]
 *                double s_vehapostrophe[6]
 *                double d_veh2int
 *                double greenLight
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_SpeedPlanTraf
 *                double c_CalibrationVars_SpeedPlanAvoi
 *                double CalibrationVars_ACC_a_min
 *                const TypeParameters Parameters
 * Return Type  : void
 */
static void AEBDecision(short *AEBActive, double speed, double
  d_veh2stopline_ped, double d_veh2stopline, double d_veh2waitingArea, double
  s_veh[6], double v_veh[6], const double d_veh2conflict[6], double
  s_vehapostrophe[6], double d_veh2int, double greenLight, TypeGlobVars
  *GlobVars, double c_CalibrationVars_SpeedPlanTraf, double
  c_CalibrationVars_SpeedPlanAvoi, double CalibrationVars_ACC_a_min, const
  TypeParameters Parameters)
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

  /*  紧急停车决策 */
  /* 30/3.6; */
  /* 2 */
  if (*AEBActive == 0) {
    /*  if PrePedestrianActive==1 && PedestrianActive==0 && wait_ped==1 && speed>0 */
    d = 0.0 - speed * speed;
    if ((d / (2.0 * CalibrationVars_ACC_a_min) >= d_veh2stopline_ped) &&
        (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1) && (speed > 0.0)) {
      /*  避让行人决策 → AEB */
      /*  判断是否碰撞行人 */
      *AEBActive = 1;
      wait_ped = 0;
    }

    if ((d_veh2stopline <= 0.0) &&
        (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1) && (speed > 0.0)
        && (*AEBActive == 0)) {
      /*  避让同向车辆决策 → AEB */
      *AEBActive = 2;
      wait_AvoidVehicle = 0;
    }

    /*  if d_veh2int<=0 && d_veh2int>=-0.5*l_veh && greenLight==0 && speed>0 && AEBActive==0 % 红绿灯通行决策 → AEB */
    if ((d_veh2int <= 0.0) && (d / -8.0 - d_veh2int <= 10.0) && (greenLight ==
         0.0) && (speed > 0.0) && (*AEBActive == 0)) {
      /*  红绿灯通行决策 → AEB */
      *AEBActive = 4;
      wait_TrafficLight = 0;
    }

    if ((d_veh2waitingArea <= 0.0) && (speed > 0.0) && (*AEBActive == 0)) {
      /*  避让对向车辆决策 → AEB */
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
          s_vehapostrophe[i] = -200.0;
        }

        b_v_veh[0] = v_veh[i];
        b_v_veh[1] = 1.0E-5;
        d = maximum(b_v_veh);
        b_v_veh[0] = 0.0;
        b_v_veh[1] = ((s_veh[i] - 0.5 * Parameters.w_veh) -
                      c_CalibrationVars_SpeedPlanAvoi) / d;
        d = maximum(b_v_veh);
        b_v_veh[0] = speed + 1.5 * d;
        b_v_veh[1] = c_CalibrationVars_SpeedPlanTraf;
        s_max[i] = 0.5 * (c_minimum(b_v_veh) + speed) * d;
      }

      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 6)) {
        /*              if ~(s_max(i)>d_veh2conflict(i)+l_veh && s_veh1apostrophe(i)<-l_veh) */
        if ((s_max[i] > d_veh2conflict[i] + Parameters.l_veh) &&
            (s_vehapostrophe[i] < 0.0)) {
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
 * globalVariable-------------------------------------------------------------------------------------------------------------------------------------
 * Arguments    : short PlannerLevel
 *                double BasicsInfo_CurrentLaneFrontDis
 *                double BasicsInfo_CurrentLaneFrontVel
 *                double BasicsInfo_CurrentLaneFrontLen
 *                short BasicsInfo_CurrentLaneIndex
 *                const double BasicsInfo_WidthOfLanes[6]
 *                double BasicsInfo_v_max
 *                double BasicsInfo_d_veh2goal
 *                double BasicsInfo_SampleTime
 *                double ChassisInfo_speed
 *                const TypeLaneChangeInfo *LaneChangeInfo
 *                const TypeAvoMainRoVehInfo *AvoMainRoVehInfo
 *                const TypeAvoPedInfo *AvoPedInfo
 *                const TypeTrafficLightInfo TrafficLightInfo
 *                const TypeAvoOncomingVehInfo *AvoOncomingVehInfo
 *                const TypeStopSignInfo StopSignInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short AEBActive
 *                short TargetGear
 *                double a_soll_ACC
 *                double a_soll_SpeedPlanAvoidPedestrian
 *                double a_soll_TrafficLightActive
 *                double a_soll_SpeedPlanAvoidVehicle
 *                double c_a_soll_SpeedPlanAvoidOncoming
 *                short TargetLaneIndex
 *                short BackupTargetLaneIndex
 *                double d_veh2stopline_ped
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 * Return Type  : struct1_T
 */
static struct1_T Decider(short PlannerLevel, double
  BasicsInfo_CurrentLaneFrontDis, double BasicsInfo_CurrentLaneFrontVel, double
  BasicsInfo_CurrentLaneFrontLen, short BasicsInfo_CurrentLaneIndex, const
  double BasicsInfo_WidthOfLanes[6], double BasicsInfo_v_max, double
  BasicsInfo_d_veh2goal, double BasicsInfo_SampleTime, double ChassisInfo_speed,
  const TypeLaneChangeInfo *LaneChangeInfo, const TypeAvoMainRoVehInfo
  *AvoMainRoVehInfo, const TypeAvoPedInfo *AvoPedInfo, const
  TypeTrafficLightInfo TrafficLightInfo, const TypeAvoOncomingVehInfo
  *AvoOncomingVehInfo, const TypeStopSignInfo StopSignInfo, short
  LaneChangeActive, short PedestrianActive, short TrafficLightActive, short
  VehicleCrossingActive, short VehicleOncomingActive, short AEBActive, short
  TargetGear, double a_soll_ACC, double a_soll_SpeedPlanAvoidPedestrian, double
  a_soll_TrafficLightActive, double a_soll_SpeedPlanAvoidVehicle, double
  c_a_soll_SpeedPlanAvoidOncoming, short TargetLaneIndex, short
  BackupTargetLaneIndex, double d_veh2stopline_ped, TypeGlobVars *GlobVars,
  const TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters)
{
  struct1_T Decision;
  double a_soll_matrix[8];
  double wait_matrix[7];
  double b_S_b_end[3];
  double b_ChassisInfo_speed[2];
  double c_this_tunableEnvironment_f1_tu[1];
  double d_this_tunableEnvironment_f1_tu[1];
  double ChassisInfo_speed_tmp;
  double CurrentLaneFrontDis;
  double RightLaneFrontDis;
  double S_b_end;
  double S_c_end;
  double S_end;
  double S_max;
  double S_min_dyn;
  double TargetLaneBehindDis;
  double TargetLaneBehindVel;
  double TargetLaneFrontDis;
  double TargetLaneFrontVel;
  double TargetSpeed;
  double TargetVelocity;
  double V_c_end;
  double V_end;
  double a;
  double a_soll_pullover;
  double b_ChassisInfo_speed_tmp;
  double distBehindGoal;
  double dist_wait;
  double s_d;
  double t_lc;
  double t_mid;
  double v_d;
  double v_e;
  double w_lane;
  int a_soll_index;
  int i;
  short b_BasicsInfo_CurrentLaneIndex[2];
  short CountLaneChange;
  short CurrentTargetLaneIndex;
  short SlowDown;
  short Wait;
  short b_a_soll_index;
  short dec_follow;
  short dec_start;
  short dir_start;
  short wait_pullover;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T prereq5;
  CountLaneChange = GlobVars->Decider.CountLaneChangeDecider;
  CurrentTargetLaneIndex = GlobVars->Decider.CurrentTargetLaneIndexDecider;
  dec_start = GlobVars->Decider.dec_start;
  wait_pullover = GlobVars->Decider.wait_pullover;
  distBehindGoal = GlobVars->Decider.distBehindGoal;
  dec_follow = GlobVars->Decider.dec_follow;

  /* CalibrationVars------------------------------------------------------------------------------------------------------------------------------------ */
  /* -3;%m/s^2 */
  /* -1.5;%m/s^2 */
  /* 7;%km/h */
  /* 10;%m */
  /* 15;%m */
  if (PlannerLevel == 2) {
    dist_wait = CalibrationVars->Decider.dist_wait2veh;
  } else {
    /* PlannerLevel==3 */
    dist_wait = CalibrationVars->Decider.dist_wait2pilot;
  }

  /* 入参----------------------------------------------------------------------------------------------------------------------------------------------- */
  /* 4 */
  /*  v_max_SpeedPlanAvoidVehicle=CalibrationVars.SpeedPlanAvoidVehicle.v_max;%40/3.6; */
  /*  TargetLaneIndex = BasicsInfo.TargetLaneIndex;%目标车道取自避让故障概车函数 */
  /*  Environmental car ------------------------------------------------------------------------------------------------------------------------------- */
  CurrentLaneFrontDis = BasicsInfo_CurrentLaneFrontDis -
    BasicsInfo_CurrentLaneFrontLen;
  RightLaneFrontDis = LaneChangeInfo->RightLaneFrontDis -
    LaneChangeInfo->RightLaneFrontLen;

  /* -------------------------------------------------------------------------------------------------------------------------------------------------- */
  i = BasicsInfo_CurrentLaneIndex - 1;
  if (BasicsInfo_CurrentLaneIndex - 1 < -32768) {
    i = -32768;
  }

  /* 初值 */
  TargetVelocity = -20.0;
  TargetSpeed = -20.0;
  SlowDown = 0;
  Wait = 0;

  /* 初始化 */
  /* LaneChangeDecision-------------------------------------------------------------------------------------------------------------------------------- */
  if (TargetLaneIndex <= BasicsInfo_CurrentLaneIndex) {
    TargetLaneBehindDis = LaneChangeInfo->LeftLaneBehindDis;
    TargetLaneBehindVel = LaneChangeInfo->LeftLaneBehindVel;
    TargetLaneFrontDis = LaneChangeInfo->LeftLaneFrontDis -
      LaneChangeInfo->LeftLaneFrontLen;
    TargetLaneFrontVel = LaneChangeInfo->LeftLaneFrontVel;
  } else {
    TargetLaneBehindDis = LaneChangeInfo->RightLaneBehindDis;
    TargetLaneBehindVel = LaneChangeInfo->RightLaneBehindVel;
    TargetLaneFrontDis = RightLaneFrontDis;
    TargetLaneFrontVel = LaneChangeInfo->RightLaneFrontVel;
  }

  if (BackupTargetLaneIndex != -1) {
    s_d = LaneChangeInfo->RightLaneBehindDis;
    v_d = LaneChangeInfo->RightLaneBehindVel;
    v_e = LaneChangeInfo->RightLaneFrontVel;
    b_BasicsInfo_CurrentLaneIndex[0] = (short)(BasicsInfo_CurrentLaneIndex + 1);
    b_BasicsInfo_CurrentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = f_minimum(b_BasicsInfo_CurrentLaneIndex);
    b_BasicsInfo_CurrentLaneIndex[0] = (short)(BasicsInfo_CurrentLaneIndex - 1);
    b_BasicsInfo_CurrentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = c_maximum(b_BasicsInfo_CurrentLaneIndex);
  } else {
    s_d = -200.0;
    v_d = -20.0;
    RightLaneFrontDis = 200.0;
    v_e = 20.0;
  }

  /*  TargetLaneIndex=TargetLaneIndex-1; */
  /*  CurrentLaneIndex=CurrentLaneIndex-1; */
  /*  v_max_int=CalibrationVars.TrajPlanLaneChange.v_max_int;%30/3.6; */
  /* 1; */
  /*  t_permit=CalibrationVars.TrajPlanLaneChange.t_permit;%3; */
  /* 0.5; */
  /* 0.5; */
  /* 1; */
  /* -3.5; */
  /* 2.5; */
  /* -1; */
  /*  a_soll=100; */
  /*  t_lc=max([2-0.05*(speed-10) 1.7]); */
  /*  t_lc=min([t_lc 2.3]); */
  b_ChassisInfo_speed[0] = 2.0 - 0.04 * (ChassisInfo_speed - 15.0);
  b_ChassisInfo_speed[1] = 2.0;
  S_max = maximum(b_ChassisInfo_speed);
  b_ChassisInfo_speed[0] = S_max;
  b_ChassisInfo_speed[1] = 2.5;
  t_lc = ceil(c_minimum(b_ChassisInfo_speed) / 0.1) * 0.1;

  /*  traj=zeros([6 120]); */
  /*  l_veh=Parameters.l_veh; */
  /*  w_veh=1.8; */
  b_BasicsInfo_CurrentLaneIndex[0] = (short)(BasicsInfo_CurrentLaneIndex + 1);
  b_BasicsInfo_CurrentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = f_minimum(b_BasicsInfo_CurrentLaneIndex);
  b_BasicsInfo_CurrentLaneIndex[0] = (short)(BasicsInfo_CurrentLaneIndex - 1);
  b_BasicsInfo_CurrentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = c_maximum(b_BasicsInfo_CurrentLaneIndex);

  /*  S_traj=zeros(1,4/0.05); */
  /*  X_traj=zeros(1,4/0.05); */
  /*  V_traj=zeros(1,4/0.05); */
  /*  wait=0; */
  /*  if isempty(LanesWithFail)==0 */
  /*      for i=LanesWithFail */
  /*          if CurrentLaneIndex==i */
  /*              wait=-1; */
  /*          end */
  /*      end */
  /*  end */
  /*  Lane change decision */
  /*  S_end=0;%2020324,编译c报错增加初始值 */
  /*  V_end=0; */
  if (GlobVars->Decider.CurrentTargetLaneIndexDecider ==
      BasicsInfo_CurrentLaneIndex) {
    CountLaneChange = 0;
  }

  if ((BasicsInfo_CurrentLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_CurrentLaneIndex != BackupTargetLaneIndex)) {
    /*      if TargetLaneIndex<=CurrentLaneIndex */
    /*          w_lane=w_lane_left; */
    /*      else */
    /*          w_lane=w_lane_right; */
    /*      end */
    w_lane = 0.5 * BasicsInfo_WidthOfLanes[BasicsInfo_CurrentLaneIndex - 1] +
      0.5 * BasicsInfo_WidthOfLanes[TargetLaneIndex - 1];

    /* 20220328 */
    if (ChassisInfo_speed < 5.0) {
      b_ChassisInfo_speed[0] = 10.0;
      b_ChassisInfo_speed[1] = t_lc * ChassisInfo_speed;
      S_end = maximum(b_ChassisInfo_speed);
      c_this_tunableEnvironment_f1_tu[0] = ChassisInfo_speed;
      fzero(c_this_tunableEnvironment_f1_tu, S_end, w_lane, &a_soll_pullover,
            &S_min_dyn, &S_max);
      b_ChassisInfo_speed[0] = t_lc;
      b_ChassisInfo_speed[1] = a_soll_pullover;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_ChassisInfo_speed) / 0.1);
      b_ChassisInfo_speed[0] = w_lane;
      ChassisInfo_speed_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_ChassisInfo_speed[1] = (0.0 - ChassisInfo_speed * ChassisInfo_speed) /
        ChassisInfo_speed_tmp;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = w_lane;
      b_ChassisInfo_speed_tmp = ChassisInfo_speed * t_lc;
      b_ChassisInfo_speed[1] = b_ChassisInfo_speed_tmp + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_min_dyn = maximum(b_ChassisInfo_speed);
      a_soll_pullover = w_lane * w_lane;
      b_ChassisInfo_speed[0] = sqrt(a * a - a_soll_pullover);
      b_ChassisInfo_speed[1] = sqrt(S_min_dyn * S_min_dyn - a_soll_pullover);
      S_min_dyn = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, TargetLaneFrontVel,
        TargetLaneFrontDis - TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      S_max = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneBehindVel + S_max *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_ChassisInfo_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (TargetLaneFrontVel +
        TargetLaneBehindVel) * t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel_strich*a_max_comfort)*t_lc; */
      /* , */
      S_max = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_max) && (TargetLaneFrontDis <=
           b_ChassisInfo_speed_tmp)) {
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + a_soll_pullover) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) /
                        ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - a_soll_pullover);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= S_max) && (TargetLaneFrontDis >
                  b_ChassisInfo_speed_tmp)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + a_soll_pullover) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - a_soll_pullover);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis > S_max) && (TargetLaneFrontDis >
                  b_ChassisInfo_speed_tmp)) {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - a_soll_pullover);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) /
                        ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - a_soll_pullover);
        S_max = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((S_max > S_end) && (S_end > w_lane)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > TargetLaneFrontVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min * t_lc)
          && (V_end < ChassisInfo_speed +
              CalibrationVars->TrajPlanLaneChange.a_max * t_lc) && prereq5 &&
          (LaneChangeInfo->d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (TargetLaneBehindDis <= -Parameters.l_veh) &&
          (TargetLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_CurrentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          a_soll_index = CountLaneChange + 1;
          if (CountLaneChange + 1 > 32767) {
            a_soll_index = 32767;
          }

          CountLaneChange = (short)a_soll_index;
          CurrentTargetLaneIndex = TargetLaneIndex;
        }
      }
    } else {
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, TargetLaneFrontVel,
        TargetLaneFrontDis - TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      S_max = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneBehindVel + S_max *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_ChassisInfo_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (TargetLaneFrontVel +
        TargetLaneBehindVel) * t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel*index_accel_strich)*t_lc; */
      /* , */
      S_max = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_max) && (TargetLaneFrontDis <=
           ChassisInfo_speed * (t_lc + CalibrationVars->TrajPlanLaneChange.t_re)))
      {
        /*                  b车不存在 c车存在 */
        b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + V_c_end);
        b_ChassisInfo_speed[1] = ChassisInfo_speed;
        V_end = c_minimum(b_ChassisInfo_speed);
        S_max = S_b_end + TargetLaneFrontVel *
          CalibrationVars->TrajPlanLaneChange.t_re;
        b_S_b_end[0] = S_max + Parameters.l_veh;
        a_soll_pullover = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = (S_max + (V_end * V_end - TargetLaneFrontVel *
          TargetLaneFrontVel) / a_soll_pullover) + Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
          a_soll_pullover;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= S_max) && (TargetLaneFrontDis >
                  ChassisInfo_speed * (t_lc +
                   CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车存在 c车不存在 */
        b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + TargetLaneFrontVel);
        b_ChassisInfo_speed[1] = ChassisInfo_speed;
        V_end = c_minimum(b_ChassisInfo_speed);
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_max = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / S_max) +
          Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        a_soll_pullover = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[0] = a_soll_pullover;
        b_S_b_end[1] = a_soll_pullover - (V_c_end * V_c_end - V_end * V_end) /
          S_max;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis > S_max) && (TargetLaneFrontDis >
                  ChassisInfo_speed * (t_lc +
                   CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车不存在 */
        S_end = ChassisInfo_speed * t_lc;
        V_end = ChassisInfo_speed;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_max = ChassisInfo_speed * ChassisInfo_speed;
        a_soll_pullover = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (S_max -
          TargetLaneFrontVel * TargetLaneFrontVel) / a_soll_pullover) +
          Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          ChassisInfo_speed;
        b_S_b_end[0] = S_min_dyn;
        b_S_b_end[1] = S_min_dyn - (V_c_end * V_c_end - S_max) / a_soll_pullover;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        b_ChassisInfo_speed[0] = ChassisInfo_speed;
        b_ChassisInfo_speed[1] = 0.5 * (TargetLaneFrontVel + V_c_end);
        V_end = c_minimum(b_ChassisInfo_speed);
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      }

      /*  参见TrajPlanLaneChange_S_max_withAccel.bmp */
      b_ChassisInfo_speed[0] = V_end;
      b_ChassisInfo_speed[1] = ChassisInfo_speed;
      a_soll_pullover = (CalibrationVars->TrajPlanLaneChange.a_max_comfort *
                         t_lc - fabs(V_end - ChassisInfo_speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移  */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= w_lane) {
        S_max = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        TargetLaneBehindVel = a_soll_pullover * a_soll_pullover /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_ChassisInfo_speed) + S_max) -
             TargetLaneBehindVel) && (S_end >= (t_lc * maximum
              (b_ChassisInfo_speed) - S_max) + TargetLaneBehindVel)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > TargetLaneFrontVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >=
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min_comfort
           * t_lc) && (V_end <= ChassisInfo_speed +
                       CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc)
          && prereq5 && (LaneChangeInfo->d_veh2int >= S_end +
                         CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int
                         * Parameters.l_veh) && (TargetLaneBehindDis <=
           -Parameters.l_veh) && (TargetLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_CurrentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          a_soll_index = CountLaneChange + 1;
          if (CountLaneChange + 1 > 32767) {
            a_soll_index = 32767;
          }

          CountLaneChange = (short)a_soll_index;
          CurrentTargetLaneIndex = TargetLaneIndex;
        }
      }
    }
  }

  if ((CurrentTargetLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_CurrentLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_CurrentLaneIndex != BackupTargetLaneIndex) && (-1 !=
       BackupTargetLaneIndex)) {
    if (BackupTargetLaneIndex <= BasicsInfo_CurrentLaneIndex) {
      if ((short)i < 1) {
        i = 1;
      } else {
        i = (short)i;
      }

      w_lane = 0.5 * BasicsInfo_WidthOfLanes[i - 1] + 0.5 *
        BasicsInfo_WidthOfLanes[BasicsInfo_CurrentLaneIndex - 1];
    } else {
      if (BasicsInfo_CurrentLaneIndex + 1 > 6) {
        a_soll_index = 6;
      } else {
        a_soll_index = BasicsInfo_CurrentLaneIndex + 1;
      }

      w_lane = 0.5 * BasicsInfo_WidthOfLanes[a_soll_index - 1] + 0.5 *
        BasicsInfo_WidthOfLanes[BasicsInfo_CurrentLaneIndex - 1];
    }

    if (ChassisInfo_speed < 5.0) {
      b_ChassisInfo_speed[0] = 10.0;
      b_ChassisInfo_speed[1] = t_lc * ChassisInfo_speed;
      S_end = maximum(b_ChassisInfo_speed);
      d_this_tunableEnvironment_f1_tu[0] = ChassisInfo_speed;
      fzero(d_this_tunableEnvironment_f1_tu, S_end, w_lane, &a_soll_pullover,
            &S_min_dyn, &S_max);
      b_ChassisInfo_speed[0] = t_lc;
      b_ChassisInfo_speed[1] = a_soll_pullover;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_ChassisInfo_speed) / 0.1);
      b_ChassisInfo_speed[0] = w_lane;
      ChassisInfo_speed_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_ChassisInfo_speed[1] = (0.0 - ChassisInfo_speed * ChassisInfo_speed) /
        ChassisInfo_speed_tmp;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = w_lane;
      b_ChassisInfo_speed[1] = ChassisInfo_speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_min_dyn = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed_tmp = w_lane * w_lane;
      b_ChassisInfo_speed[0] = sqrt(a * a - b_ChassisInfo_speed_tmp);
      b_ChassisInfo_speed[1] = sqrt(S_min_dyn * S_min_dyn -
        b_ChassisInfo_speed_tmp);
      S_min_dyn = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_e +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = RightLaneFrontDis + 0.5 * (V_c_end + v_e) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, v_e, RightLaneFrontDis -
        s_d, v_d, 0.0, CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      S_max = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_d + S_max *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_ChassisInfo_speed);
      S_b_end = s_d + 0.5 * (TargetLaneFrontVel + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      S_max = v_d * t_lc;
      if ((-s_d > S_max) && (RightLaneFrontDis <= ChassisInfo_speed * t_lc)) {
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + b_ChassisInfo_speed_tmp) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) /
                        ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - b_ChassisInfo_speed_tmp);
        S_max = d_minimum(b_S_b_end);
      } else if ((-s_d <= S_max) && (RightLaneFrontDis > ChassisInfo_speed *
                  t_lc)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + b_ChassisInfo_speed_tmp) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) /
                        ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_max;
        b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - b_ChassisInfo_speed_tmp);
        S_max = d_minimum(b_S_b_end);
      } else if ((-s_d > S_max) && (RightLaneFrontDis > ChassisInfo_speed * t_lc))
      {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - b_ChassisInfo_speed_tmp);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + b_ChassisInfo_speed_tmp) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = TargetLaneFrontVel - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) /
                        ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - b_ChassisInfo_speed_tmp);
        S_max = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((S_max > S_end) && (S_end > w_lane)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > TargetLaneFrontVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min * t_lc)
          && (V_end < ChassisInfo_speed +
              CalibrationVars->TrajPlanLaneChange.a_max * t_lc) && prereq5 &&
          (LaneChangeInfo->d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (RightLaneFrontDis
           >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_CurrentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          i = CountLaneChange + 1;
          if (CountLaneChange + 1 > 32767) {
            i = 32767;
          }

          CountLaneChange = (short)i;
          TargetLaneIndex = BackupTargetLaneIndex;
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    } else {
      b_ChassisInfo_speed[0] = 0.0;
      ChassisInfo_speed_tmp = CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort;
      b_ChassisInfo_speed[1] = v_e + ChassisInfo_speed_tmp * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = RightLaneFrontDis + 0.5 * (V_c_end + v_e) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, v_e, RightLaneFrontDis -
        s_d, v_d, 0.0, CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      S_max = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_d + S_max *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = maximum(b_ChassisInfo_speed);
      S_b_end = s_d + 0.5 * (TargetLaneFrontVel + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      S_max = v_d * t_lc;
      guard1 = false;
      guard2 = false;
      guard3 = false;
      if (-s_d > S_max) {
        TargetLaneBehindVel = ChassisInfo_speed * t_lc;
        if (RightLaneFrontDis <= TargetLaneBehindVel) {
          /*                  b车不存在 c车存在 */
          b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + V_c_end);
          b_ChassisInfo_speed[1] = ChassisInfo_speed;
          V_end = c_minimum(b_ChassisInfo_speed);
          S_max = S_b_end + TargetLaneFrontVel *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = S_max + Parameters.l_veh;
          a_soll_pullover = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = (S_max + (V_end * V_end - TargetLaneFrontVel *
            TargetLaneFrontVel) / a_soll_pullover) + Parameters.l_veh;
          b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          w_lane = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          S_max = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = S_max;
          b_S_b_end[1] = S_max - (V_c_end * V_c_end - V_end * V_end) /
            a_soll_pullover;
          b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          S_max = d_minimum(b_S_b_end);
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = S_max;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
          S_end = median(b_S_b_end);
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }

      if (guard3) {
        if (-s_d <= S_max) {
          TargetLaneBehindVel = ChassisInfo_speed * t_lc;
          if (RightLaneFrontDis > TargetLaneBehindVel) {
            /* , */
            /*                  b车存在 c车不存在 */
            b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed +
              TargetLaneFrontVel);
            b_ChassisInfo_speed[1] = ChassisInfo_speed;
            V_end = c_minimum(b_ChassisInfo_speed);
            b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            S_max = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                             CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
              V_end - TargetLaneFrontVel * TargetLaneFrontVel) / S_max) +
              Parameters.l_veh;
            b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            w_lane = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            a_soll_pullover = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re
              * V_end;
            b_S_b_end[0] = a_soll_pullover;
            b_S_b_end[1] = a_soll_pullover - (V_c_end * V_c_end - V_end * V_end)
              / S_max;
            b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            S_max = d_minimum(b_S_b_end);
            b_S_b_end[0] = w_lane;
            b_S_b_end[1] = S_max;
            b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
            S_end = median(b_S_b_end);
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        if (-s_d > S_max) {
          S_end = ChassisInfo_speed * t_lc;
          if (RightLaneFrontDis > S_end) {
            /* , */
            /*                  b车不存在 c车不存在 */
            V_end = ChassisInfo_speed;
            b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            S_max = ChassisInfo_speed * ChassisInfo_speed;
            a_soll_pullover = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                             CalibrationVars->TrajPlanLaneChange.t_re) + (S_max
              - TargetLaneFrontVel * TargetLaneFrontVel) / a_soll_pullover) +
              Parameters.l_veh;
            b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            w_lane = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
              ChassisInfo_speed;
            b_S_b_end[0] = S_min_dyn;
            b_S_b_end[1] = S_min_dyn - (V_c_end * V_c_end - S_max) /
              a_soll_pullover;
            b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            S_max = d_minimum(b_S_b_end);
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /*                  b车存在 c车存在 */
        b_ChassisInfo_speed[0] = ChassisInfo_speed;
        b_ChassisInfo_speed[1] = 0.5 * (TargetLaneFrontVel + V_c_end);
        V_end = c_minimum(b_ChassisInfo_speed);
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        w_lane = b_maximum(b_S_b_end);

        /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      }

      b_ChassisInfo_speed[0] = V_end;
      b_ChassisInfo_speed[1] = ChassisInfo_speed;
      S_min_dyn = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      a_soll_pullover = (S_min_dyn - fabs(V_end - ChassisInfo_speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= w_lane) {
        S_max = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        TargetLaneBehindVel = a_soll_pullover * a_soll_pullover /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_ChassisInfo_speed) + S_max) -
             TargetLaneBehindVel) && (S_end >= (t_lc * maximum
              (b_ChassisInfo_speed) - S_max) + TargetLaneBehindVel)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > TargetLaneFrontVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            TargetLaneFrontVel * TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >=
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min_comfort
           * t_lc) && (V_end <= ChassisInfo_speed + S_min_dyn) && prereq5 &&
          (LaneChangeInfo->d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (RightLaneFrontDis
           >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_CurrentLaneFrontVel +
          ChassisInfo_speed_tmp * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_CurrentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          i = CountLaneChange + 1;
          if (CountLaneChange + 1 > 32767) {
            i = 32767;
          }

          CountLaneChange = (short)i;
          TargetLaneIndex = BackupTargetLaneIndex;
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    }
  }

  if (LaneChangeActive != 0) {
    i = CurrentTargetLaneIndex - BasicsInfo_CurrentLaneIndex;
    if (i > 32767) {
      i = 32767;
    } else {
      if (i < -32768) {
        i = -32768;
      }
    }

    if ((CountLaneChange > 0) && (i == -1)) {
      /* 左换道 */
      Decision.LaneChange = 1;
    } else if ((CountLaneChange > 0) && (i == 1)) {
      /* 右换道 */
      Decision.LaneChange = 2;
    } else {
      Decision.LaneChange = 0;

      /*          GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0); */
    }
  } else {
    Decision.LaneChange = 0;

    /*      GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0); */
    CountLaneChange = 0;
  }

  /* ------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /*  status */
  Decision.PedestrianState = (short)(PedestrianActive != 0);
  Decision.TrafficLightState = (short)(TrafficLightActive != 0);
  Decision.VehicleCrossingState = (short)(VehicleCrossingActive != 0);
  Decision.VehicleOncomingState = (short)(VehicleOncomingActive != 0);
  if ((CurrentLaneFrontDis < 195.0) && (BasicsInfo_CurrentLaneFrontVel < 20.0))
  {
    Decision.FollowState = 1;
  } else {
    Decision.FollowState = 0;
  }

  Decision.StopSignState = (short)(GlobVars->SpeedPlanStopSign.wait_stopsign ==
    1);
  if (BasicsInfo_d_veh2goal <= 60.0) {
    /* 靠边停车 */
    Decision.PullOverState = 1;
  } else {
    Decision.PullOverState = 0;
  }

  /*  wait */
  for (i = 0; i < 7; i++) {
    wait_matrix[i] = 200.0;
  }

  /*  if CurrentLaneFrontVel<=0.1 && CurrentLaneFrontDis<=dist_wait+l_veh+5 */
  /*      wait_matrix(6)=CurrentLaneFrontDis-l_veh-5; */
  /*  end */
  if ((BasicsInfo_CurrentLaneFrontVel <= 0.5) && (CurrentLaneFrontDis <=
       dist_wait + CalibrationVars->ACC.d_wait)) {
    /* CurrentLaneFrontDis为前车车尾距离 */
    wait_matrix[5] = CurrentLaneFrontDis - CalibrationVars->ACC.d_wait;
  }

  if ((GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1) &&
      (AvoMainRoVehInfo->d_veh2stopline <= dist_wait)) {
    wait_matrix[1] = AvoMainRoVehInfo->d_veh2stopline;
  }

  if ((GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle == 1) &&
      (AvoOncomingVehInfo->d_veh2waitingArea <= dist_wait)) {
    wait_matrix[2] = AvoOncomingVehInfo->d_veh2waitingArea;
  }

  /*  if wait_ped==1 && d_veh2cross<=dist_wait */
  /*      wait_matrix(1)=d_veh2cross; */
  /*  end */
  if ((GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1) && (d_veh2stopline_ped <=
       dist_wait)) {
    wait_matrix[0] = d_veh2stopline_ped;
  }

  if ((GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1) &&
      (TrafficLightInfo.d_veh2stopline <= dist_wait)) {
    wait_matrix[3] = TrafficLightInfo.d_veh2stopline;
  }

  if ((GlobVars->SpeedPlanStopSign.wait_stopsign == 1) &&
      (StopSignInfo.d_veh2stopline <= dist_wait) && (StopSignInfo.d_veh2stopline
       >= 0.0)) {
    wait_matrix[4] = StopSignInfo.d_veh2stopline;
  }

  /*  if d_veh2goal<=dist_wait%靠边停车 */
  /*      wait_matrix(7)=d_veh2goal; */
  /*  end */
  /*  靠边停车子功能wait状态的切换及靠边停车位置的计算 */
  if ((BasicsInfo_CurrentLaneIndex == TargetLaneIndex) && (BasicsInfo_d_veh2goal
       < 60.0) && (GlobVars->Decider.wait_pullover == 0)) {
    S_max = ChassisInfo_speed * ChassisInfo_speed / 4.0;
    if (S_max < 15.0) {
      wait_pullover = 1;
      distBehindGoal = fmax(0.0, S_max - BasicsInfo_d_veh2goal);

      /*  distBehindGoal为全局变量，初值为0 */
    }
  }

  /*  靠边停车距离的计算 */
  if (wait_pullover == 1) {
    wait_matrix[6] = BasicsInfo_d_veh2goal + distBehindGoal;
  }

  TargetLaneBehindVel = g_minimum(wait_matrix);
  if (TargetLaneBehindVel <= dist_wait) {
    a_soll_index = 0;
    exitg1 = false;
    while ((!exitg1) && (a_soll_index < 7)) {
      if (wait_matrix[a_soll_index] == TargetLaneBehindVel) {
        Wait = (short)(a_soll_index + 1);
        exitg1 = true;
      } else {
        a_soll_index++;
      }
    }
  } else {
    TargetLaneBehindVel = 200.0;
  }

  if ((Wait == 2) && (StopSignInfo.d_veh2stopline <= dist_wait) &&
      (StopSignInfo.d_veh2stopline >= 0.0)) {
    /* 有停止让行标志 */
    Wait = 5;
  }

  /*  slowdown */
  S_max = BasicsInfo_v_max - ChassisInfo_speed;
  if (S_max < 0.0) {
    /* 限速加速度 */
    b_ChassisInfo_speed[0] = -2.5;
    b_ChassisInfo_speed[1] = S_max / CalibrationVars->ACC.tau_v;
    S_min_dyn = maximum(b_ChassisInfo_speed);
  } else {
    S_min_dyn = 100.0;
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 1) {
    /* 停车让行加速度 */
    b_ChassisInfo_speed[0] = 0.0;
    b_ChassisInfo_speed[1] = StopSignInfo.d_veh2stopline +
      CalibrationVars->ACC.d_wait;
    S_max = ACC(BasicsInfo_v_max, 0.0, maximum(b_ChassisInfo_speed),
                ChassisInfo_speed, 0.0, CalibrationVars->ACC.a_max,
                CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
                CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
                CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                CalibrationVars->ACC.d_wait);
  } else {
    S_max = 100.0;
  }

  if (BasicsInfo_d_veh2goal < 60.0) {
    /* 靠边停车 */
    if (BasicsInfo_CurrentLaneIndex != TargetLaneIndex) {
      /* 未在目标车道较晚减速 */
      if (BasicsInfo_d_veh2goal < (CalibrationVars->TrajPlanLaneChange.v_max_int
           * CalibrationVars->TrajPlanLaneChange.v_max_int - BasicsInfo_v_max *
           BasicsInfo_v_max) / -3.0 +
          CalibrationVars->TrajPlanLaneChange.v_max_int
          * CalibrationVars->TrajPlanLaneChange.t_permit) {
        a_soll_pullover = ACC(8.3333333333333339, 20.0, 200.0, ChassisInfo_speed,
                              0.0, CalibrationVars->ACC.a_max,
                              CalibrationVars->ACC.a_min,
                              CalibrationVars->ACC.tau_v_com,
                              CalibrationVars->ACC.tau_v,
                              CalibrationVars->ACC.tau_d,
                              CalibrationVars->ACC.tau_v_bre,
                              CalibrationVars->ACC.tau_v_emg,
                              CalibrationVars->ACC.tau_d_emg,
                              CalibrationVars->ACC.t_acc,
                              CalibrationVars->ACC.d_wait);
      } else {
        a_soll_pullover = 100.0;
      }
    } else {
      a_soll_pullover = ACC(BasicsInfo_v_max, 0.0, BasicsInfo_d_veh2goal +
                            CalibrationVars->ACC.d_wait, ChassisInfo_speed, 1.0,
                            CalibrationVars->ACC.a_max,
                            CalibrationVars->ACC.a_min,
                            CalibrationVars->ACC.tau_v_com,
                            CalibrationVars->ACC.tau_v,
                            CalibrationVars->ACC.tau_d,
                            CalibrationVars->ACC.tau_v_bre,
                            CalibrationVars->ACC.tau_v_emg,
                            CalibrationVars->ACC.tau_d_emg,
                            CalibrationVars->ACC.t_acc,
                            CalibrationVars->ACC.d_wait);
    }
  } else {
    a_soll_pullover = 100.0;
  }

  /*  a_soll_matrix=[a_soll_SpeedPlanAvoidPedestrian,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_soll_TrafficLightActive,a_soll_StopSign,a_soll_ACC,a_soll_veh2goal,accel_speedlimit]; */
  a_soll_matrix[0] = a_soll_SpeedPlanAvoidPedestrian;
  a_soll_matrix[1] = a_soll_SpeedPlanAvoidVehicle;
  a_soll_matrix[2] = c_a_soll_SpeedPlanAvoidOncoming;
  a_soll_matrix[3] = a_soll_TrafficLightActive;
  a_soll_matrix[4] = S_max;
  a_soll_matrix[5] = a_soll_ACC;
  a_soll_matrix[6] = a_soll_pullover;
  a_soll_matrix[7] = S_min_dyn;
  TargetLaneFrontVel = h_minimum(a_soll_matrix);
  if (ChassisInfo_speed <= 0.0) {
    TargetLaneFrontVel = fmax(0.0, TargetLaneFrontVel);
  }

  /* 跟车安全距离 */
  if (GlobVars->Decider.dec_follow == 0) {
    /* 跟车减速指示的决策 */
    if ((BasicsInfo_CurrentLaneFrontVel * BasicsInfo_CurrentLaneFrontVel -
         ChassisInfo_speed * ChassisInfo_speed) * 0.5 /
        CalibrationVars->Decider.a_bre + CalibrationVars->ACC.d_wait >
        CurrentLaneFrontDis) {
      dec_follow = 1;
    }
  } else {
    if ((BasicsInfo_CurrentLaneFrontVel * BasicsInfo_CurrentLaneFrontVel -
         ChassisInfo_speed * ChassisInfo_speed) * 0.5 /
        CalibrationVars->Decider.a_bre_com + CalibrationVars->ACC.d_wait <
        CurrentLaneFrontDis) {
      dec_follow = 0;
    }
  }

  if (PlannerLevel == 2) {
    dir_start = 8;
    b_a_soll_index = 8;
    exitg1 = false;
    while ((!exitg1) && (b_a_soll_index > 0)) {
      dir_start = b_a_soll_index;

      /* 从后往前查 */
      if (a_soll_matrix[b_a_soll_index - 1] == TargetLaneFrontVel) {
        exitg1 = true;
      } else {
        b_a_soll_index--;
      }
    }

    if (dir_start == 1) {
      if ((TargetLaneFrontVel <= -0.2) ||
          (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1)) {
        SlowDown = 1;
      }
    } else if (dir_start == 2) {
      if ((TargetLaneFrontVel <= -0.2) ||
          (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1)) {
        SlowDown = 2;
      }
    } else if (dir_start == 3) {
      if ((TargetLaneFrontVel <= -0.2) ||
          (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle ==
           1)) {
        SlowDown = 3;
      }
    } else if (dir_start == 4) {
      if ((TargetLaneFrontVel <= -0.2) ||
          (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
        SlowDown = 4;
      }
    } else if (dir_start == 5) {
      if (TargetLaneFrontVel <= -0.2) {
        SlowDown = 5;
      }
    } else if (dir_start == 6) {
      if (dec_follow == 1) {
        SlowDown = 6;
      }
    } else if (dir_start == 7) {
      if (TargetLaneFrontVel <= -0.2) {
        SlowDown = 7;
      }
    } else {
      if (TargetLaneFrontVel <= -0.2) {
        SlowDown = 8;
      }
    }

    /* TargetSpeed */
    if (Wait > 0) {
      SlowDown = 0;
      GlobVars->Decider.a_soll_pre = 100.0;

      /*  GlobVars.Decider.a_soll_pre的初始值为100 */
    } else if ((PedestrianActive != 0) || (TrafficLightActive != 0) ||
               (VehicleCrossingActive != 0) || (VehicleOncomingActive != 0) ||
               (SlowDown == 6) || (Decision.StopSignState != 0) ||
               (Decision.PullOverState != 0)) {
      /* 跟车减速提示时，停车让行时，靠边停车时 */
      /*          TargetSpeed=(speed+a_soll*SampleTime); */
      if (GlobVars->Decider.a_soll_pre != 100.0) {
        if (TargetLaneFrontVel > -2.0) {
          b_S_b_end[0] = GlobVars->Decider.a_soll_pre + 2.0 *
            BasicsInfo_SampleTime;
          b_S_b_end[1] = TargetLaneFrontVel;
          b_S_b_end[2] = GlobVars->Decider.a_soll_pre - 2.0 *
            BasicsInfo_SampleTime;
          S_min_dyn = median(b_S_b_end);
        } else {
          b_S_b_end[0] = GlobVars->Decider.a_soll_pre + 2.0 *
            BasicsInfo_SampleTime;
          b_S_b_end[1] = TargetLaneFrontVel;
          b_S_b_end[2] = GlobVars->Decider.a_soll_pre - 5.0 *
            BasicsInfo_SampleTime;
          S_min_dyn = median(b_S_b_end);
        }
      } else {
        S_min_dyn = TargetLaneFrontVel;
      }

      TargetSpeed = fmax(0.0, ChassisInfo_speed + S_min_dyn *
                         BasicsInfo_SampleTime);
      GlobVars->Decider.a_soll_pre = S_min_dyn;

      /*  GlobVars.Decider.a_soll_pre只有在下发目标速度时才更新 */
    } else {
      TargetSpeed = BasicsInfo_v_max;
      GlobVars->Decider.a_soll_pre = 100.0;
    }
  } else {
    /* 驾驶员 */
    TargetVelocity = sqrt(-dist_wait * 2.0 * CalibrationVars->Decider.a_bre_com);
    S_min_dyn = (TargetVelocity * TargetVelocity - 69.444444444444457) * 0.5 /
      CalibrationVars->Decider.a_bre_com;
    S_max = -27.006172839506164 / CalibrationVars->Decider.a_bre_com;
    a_soll_pullover = -34.722222222222229 / CalibrationVars->Decider.a_bre_com;
    if ((TargetLaneFrontVel < -0.2) && (Wait == 0)) {
      /* wait时速度默认值 */
      dir_start = 8;
      exitg1 = false;
      while ((!exitg1) && (dir_start > 0)) {
        /* 从后往前查 */
        if (a_soll_matrix[dir_start - 1] == TargetLaneFrontVel) {
          SlowDown = dir_start;
          exitg1 = true;
        } else {
          dir_start--;
        }
      }

      if (SlowDown == 6) {
        /* followcar */
        if ((BasicsInfo_CurrentLaneFrontVel <= 0.1) && (CurrentLaneFrontDis >=
             dist_wait)) {
          if (CurrentLaneFrontDis <= S_min_dyn + dist_wait) {
            TargetVelocity *= 3.6;
            TargetVelocity = rt_roundd_snf(TargetVelocity);

            /* km/h */
          } else if (CurrentLaneFrontDis <= (S_min_dyn + S_max) + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if (CurrentLaneFrontDis <= ((S_min_dyn + S_max) +
                      a_soll_pullover) + dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else {
          TargetVelocity = BasicsInfo_CurrentLaneFrontVel * 3.6;

          /* km/h */
        }
      } else if (SlowDown == 8) {
        /* Speedlimit */
        if (ChassisInfo_speed >= BasicsInfo_v_max * 1.1) {
          /* 超过限速%10 */
          TargetVelocity = BasicsInfo_v_max * 3.6;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 5) {
        if (StopSignInfo.d_veh2stopline <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (StopSignInfo.d_veh2stopline <= (S_min_dyn + S_max) +
                   dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (StopSignInfo.d_veh2stopline <= ((S_min_dyn + S_max) +
                    a_soll_pullover) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 2) {
        /* Ramp */
        /*          if wait_AvoidVehicle==1 */
        if (AvoMainRoVehInfo->d_veh2stopline <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (AvoMainRoVehInfo->d_veh2stopline <= (S_min_dyn + S_max) +
                   dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (AvoMainRoVehInfo->d_veh2stopline <= ((S_min_dyn + S_max) +
                    a_soll_pullover) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }

        /*          elseif speed>=min(v_max_SpeedPlanAvoidVehicle,v_soll_SpeedPlanAvoidVehicle) */
        /*              TargetSpeed=min(v_max_SpeedPlanAvoidVehicle,v_soll_SpeedPlanAvoidVehicle)*3.6;%km/h */
        /*          else */
        /*              TargetSpeed=-20; */
        /*          end */
      } else if (SlowDown == 3) {
        /* oncommingcar */
        if (AvoOncomingVehInfo->d_veh2waitingArea <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (AvoOncomingVehInfo->d_veh2waitingArea <= (S_min_dyn + S_max)
                   + dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (AvoOncomingVehInfo->d_veh2waitingArea <= ((S_min_dyn + S_max)
                    + a_soll_pullover) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 1) {
        /* ped */
        if (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1) {
          if (d_veh2stopline_ped <= S_min_dyn + dist_wait) {
            TargetVelocity *= 3.6;
            TargetVelocity = rt_roundd_snf(TargetVelocity);

            /* km/h */
          } else if (d_veh2stopline_ped <= (S_min_dyn + S_max) + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if (d_veh2stopline_ped <= ((S_min_dyn + S_max) +
                      a_soll_pullover) + dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else if (AvoPedInfo->d_veh2cross <= (S_min_dyn + S_max) + dist_wait) {
          /* 人行道限速 */
          TargetVelocity = 30.0;

          /* km/h */
        } else if (AvoPedInfo->d_veh2cross <= ((S_min_dyn + S_max) +
                    a_soll_pullover) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 4) {
        /* trafficLight */
        if (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1) {
          if (TrafficLightInfo.d_veh2stopline <= S_min_dyn + dist_wait) {
            TargetVelocity *= 3.6;
            TargetVelocity = rt_roundd_snf(TargetVelocity);

            /* km/h */
          } else if ((TrafficLightInfo.d_veh2stopline <= (S_min_dyn + S_max) +
                      dist_wait) && (ChassisInfo_speed >= 8.88888888888889)) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if ((TrafficLightInfo.d_veh2stopline <= ((S_min_dyn + S_max) +
                       a_soll_pullover) + dist_wait) && (ChassisInfo_speed >=
                      11.666666666666666)) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else if (ChassisInfo_speed >= 8.88888888888889) {
          if (TrafficLightInfo.d_veh2stopline <= (S_min_dyn + S_max) + dist_wait)
          {
            TargetVelocity = 30.0;

            /* km/h */
          } else if ((TrafficLightInfo.d_veh2stopline <= ((S_min_dyn + S_max) +
                       a_soll_pullover) + dist_wait) && (ChassisInfo_speed >=
                      11.666666666666666)) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 7) {
        /* 靠边停车 */
        if (BasicsInfo_d_veh2goal <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (BasicsInfo_d_veh2goal <= (S_min_dyn + S_max) + dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (BasicsInfo_d_veh2goal <= ((S_min_dyn + S_max) +
                    a_soll_pullover) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else {
        SlowDown = 0;
        TargetVelocity = -20.0;
      }

      if (TargetVelocity == -20.0) {
        SlowDown = 0;
      }
    } else if ((CurrentLaneFrontDis < (BasicsInfo_CurrentLaneFrontVel *
                 BasicsInfo_CurrentLaneFrontVel - ChassisInfo_speed *
                 ChassisInfo_speed) * 0.5 / CalibrationVars->Decider.a_bre) &&
               (Wait == 0)) {
      /* wait时速度默认值 */
      SlowDown = 6;
      if ((BasicsInfo_CurrentLaneFrontVel <= 0.1) && (CurrentLaneFrontDis >=
           dist_wait)) {
        if (CurrentLaneFrontDis <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else {
          S_max += S_min_dyn;
          if (CurrentLaneFrontDis <= S_max + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if (CurrentLaneFrontDis <= (S_max + a_soll_pullover) +
                     dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        }
      } else {
        TargetVelocity = BasicsInfo_CurrentLaneFrontVel * 3.6;
      }
    } else {
      /* wait时速度默认值 */
      TargetVelocity = -20.0;
    }
  }

  /*  start */
  if (GlobVars->Decider.dec_start == 0) {
    if ((Wait >= 1) && (ChassisInfo_speed <= 0.41666666666666663)) {
      dec_start = 1;
    }
  } else {
    if ((GlobVars->Decider.dir_start == 1) && (ChassisInfo_speed >=
         CalibrationVars->Decider.idle_speed / 3.6)) {
      dec_start = 0;
    }
  }

  if (dec_start == 1) {
    if ((TargetLaneFrontVel > 0.0) && (Wait == 0)) {
      dir_start = 1;
    } else {
      dir_start = 0;
    }
  } else {
    dir_start = 0;
  }

  Decision.Start = dir_start;

  /* AEBActive */
  if (AEBActive > 0) {
    Decision.LaneChange = 0;
    SlowDown = 0;
    TargetVelocity = -20.0;
    TargetSpeed = -20.0;
    Wait = 0;
    TargetLaneBehindVel = 200.0;
    Decision.Start = 0;
  }

  /* 精度 */
  TargetVelocity = rt_roundd_snf(TargetVelocity);

  /* km/h */
  /* m/s */
  Decision.WaitDistance = rt_roundd_snf(10.0 * TargetLaneBehindVel) / 10.0;

  /* m */
  /* output */
  if (PlannerLevel == 2) {
    Decision.TargetSpeed = rt_roundd_snf(100.0 * TargetSpeed) / 100.0;

    /* m/s */
  } else {
    Decision.TargetSpeed = TargetVelocity / 3.6;

    /* m/s */
  }

  Decision.Wait = Wait;

  /* m */
  Decision.SlowDown = SlowDown;
  Decision.AEBactive = AEBActive;
  Decision.TargetGear = TargetGear;
  Decision.a_soll = TargetLaneFrontVel;

  /* 全局变量 */
  GlobVars->Decider.dec_start = dec_start;
  GlobVars->Decider.dir_start = dir_start;
  GlobVars->Decider.wait_pullover = wait_pullover;
  GlobVars->Decider.distBehindGoal = distBehindGoal;
  GlobVars->Decider.dec_follow = dec_follow;
  GlobVars->Decider.CountLaneChangeDecider = CountLaneChange;
  GlobVars->Decider.CurrentTargetLaneIndexDecider = CurrentTargetLaneIndex;
  return Decision;
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
 * NumOfLanes=2; % 车道数量
 *  LanesWithFail=[1 2]; % 故障车所在车道序号，车道序号自小到大排列，最左侧车道序号为1
 *  TargetLaneIndex=2; % 原目标车道
 *  CurrentLaneIndex=1; 本车所在车道
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

  /*  TargetLaneIndex 现目标车道 */
  /*  BackupTargetLaneIndex % 现备用目标车道，仅当车辆左右换道都可以避开故障车时存在（此时TargetLaneIndex为左侧车道，BackupTargetLaneIndex为右侧车道），否则为-1 */
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
  /*  r = R;%半径 */
  /*  a1 = sr1;%圆心横坐标 */
  /*  b1 = lr1;%圆心纵坐标 */
  /*  theta = 0:pi/20:2*pi; %角度[0,2*pi]  */
  /*  theta1=-pi/2:pi/20:atan((l_p1-lr1)/(s_p1-sr1)); */
  /*  x1 = a1+r*cos(theta); */
  /*  y1= b1+r*sin(theta); */
  /*  x11 = a1+r*cos(theta1); */
  /*  y11= b1+r*sin(theta1); */
  /*  plot(x1,y1,'g--',x11,y11,'r-') */
  /*  %plot(x1,y1,'g--',x1(s_start:s_p1),y1(l_start:l_p1),'b-') */
  /*  hold on */
  /*  a2 = sr2;%圆心横坐标 */
  /*  b2 = lr2;%圆心纵坐标 */
  /*  theta2=(atan((l_p1-lr2)/(s_p1-sr2))+pi):pi/20:(atan((l_p2-lr2)/(s_p2-sr2))+pi); */
  /*  x2 = a2+r*cos(theta); */
  /*  y2= b2+r*sin(theta); */
  /*  x22 = a2+r*cos(theta2); */
  /*  y22= b2+r*sin(theta2); */
  /*  plot(x2,y2,'g--',x22,y22,'r-') */
  /*  hold on  */
  /*  a3 = sr3;%圆心横坐标 */
  /*  b3 = lr3;%圆心纵坐标 */
  /*  theta3=atan((l_p2-lr3)/(s_p2-sr3)):pi/20:pi/2; */
  /*  x3 = a3+r*cos(theta); */
  /*  y3= b3+r*sin(theta); */
  /*  x33 = a3+r*cos(theta3); */
  /*  y33= b3+r*sin(theta3); */
  /*  plot(x3,y3,'g--',x33,y33,'r-') */
  /*  %车道线 */
  /*  hold on */
  /*  %目标车道线 */
  /*  plot([0,s_turnround_border],[l_targetlane,l_targetlane],'r--') */
  /*  %当前车道线 */
  /*  hold on  */
  /*  plot([0,s_turnround_border],[l_current,l_current],'r--') */
  /*  %车道边界 */
  /*  hold on */
  /*  plot([0,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_current-0.5*l_targetlane],'r-') */
  /*  hold on  */
  /*  plot([0,s_turnround_border],[1.5*l_targetlane-0.5*l_current,1.5*l_targetlane-0.5*l_current],'r-') */
  /*  hold on */
  /*  plot([0,s_turnround_border-6],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r-',[s_turnround_border-6,s_turnround_border],[0.5*l_targetlane+0.5*l_current,0.5*l_targetlane+0.5*l_current],'r--') */
  /*  %掉头边界 */
  /*  plot([s_turnround_border,s_turnround_border],[1.5*l_current-0.5*l_targetlane,1.5*l_targetlane-0.5*l_current],'r-') */
  /*  hold on */
  /*  %点坐标 */
  /*  plot(s_start,l_start,'r*',s_end,l_end,'r*',s_p1,l_p1,'r*',s_p2,l_p2,'r*',sr1,lr1,'r.',sr2,lr2,'r.',sr3,lr3,'r.') */
  /*  axis equal */
  /*   */
}

/*
 * ,
 * Arguments    : double speed
 *                double d_veh2waitingArea
 *                double s_b
 *                double v_b
 *                const double s_veh[6]
 *                const double v_veh[6]
 *                const double d_veh2conflict[6]
 *                const double s_vehapostrophe[6]
 *                double v_max
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_SpeedPlanAvoi
 *                double d_CalibrationVars_SpeedPlanAvoi
 *                double e_CalibrationVars_SpeedPlanAvoi
 *                double f_CalibrationVars_SpeedPlanAvoi
 *                const CalibACC *CalibrationVars_ACC
 *                double Parameters_w_veh
 *                double Parameters_l_veh
 * Return Type  : double
 */
static double SpeedPlanAvoidOncomingVehicle(double speed, double
  d_veh2waitingArea, double s_b, double v_b, const double s_veh[6], const double
  v_veh[6], const double d_veh2conflict[6], const double s_vehapostrophe[6],
  double v_max, TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanAvoi,
  double d_CalibrationVars_SpeedPlanAvoi, double e_CalibrationVars_SpeedPlanAvoi,
  double f_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC,
  double Parameters_w_veh, double Parameters_l_veh)
{
  double d_veh[6];
  double b_speed[2];
  double a_soll;
  double d;
  int i;
  short dec_avoidOncomingVehicle;
  short wait_avoidOncomingVehicle;
  boolean_T exitg1;

  /* globalVariable---------------------------------------------------------------------------------------------------------------------- */
  dec_avoidOncomingVehicle =
    GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle;
  wait_avoidOncomingVehicle =
    GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle;

  /*  设置参数 */
  /*  a_max=2.5; */
  /* 1.5; */
  /* -3; */
  /* 30/3.6; */
  /* 2 */
  /*  v_max_overall=50/3.6; */
  /*  策略模式判断 */
  if (GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle == 0) {
    if ((d_veh2waitingArea <= (0.0 - speed * speed) / (2.0 *
          d_CalibrationVars_SpeedPlanAvoi) + 2.0 * Parameters_l_veh) &&
        (d_veh2waitingArea > Parameters_l_veh)) {
      dec_avoidOncomingVehicle = 1;
    }
  } else {
    if ((d_veh2waitingArea <= 0.5 * Parameters_l_veh) ||
        (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle == 1))
    {
      dec_avoidOncomingVehicle = 0;
    }
  }

  /*  停车决策 */
  for (i = 0; i < 6; i++) {
    b_speed[0] = speed;
    b_speed[1] = 1.0E-5;
    d = maximum(b_speed);
    b_speed[0] = ((d_veh2conflict[i] + Parameters_l_veh) / d * v_veh[i] + 0.5 *
                  Parameters_w_veh) + f_CalibrationVars_SpeedPlanAvoi;
    b_speed[1] = 0.0;
    d_veh[i] = maximum(b_speed);
  }

  if (dec_avoidOncomingVehicle == 1) {
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 6)) {
      /*          if s_veh(i)<=d_veh(i) || s_vehapostrophe(i)>-l_veh */
      if ((s_veh[i] <= d_veh[i]) || (s_vehapostrophe[i] > 0.0)) {
        /* 20220704 */
        wait_avoidOncomingVehicle = 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  /*  起步决策 */
  if (wait_avoidOncomingVehicle == 1) {
    /*      timeGap1=max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.5*(v_veh1+v_max_overall) 0.00001])]); */
    /*      if (v_max_overall-v_veh1)>a_max_com*timeGap1 */
    /*          fun_x=@(x)v_veh1*x+0.5*a_max_com*(x.^2); */
    /*          [timeGap1,~,~] = fzero(@(x)fun_x(x)-(s_veh1-0.5*w_veh-l_veh),[0 max([0 (s_veh1-0.5*w_veh-l_veh)/max([v_veh1 0.00001])])]); */
    /*      end */
    for (i = 0; i < 6; i++) {
      b_speed[0] = v_veh[i];
      b_speed[1] = 1.0E-5;
      d = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = ((s_veh[i] - 0.5 * Parameters_w_veh) -
                    f_CalibrationVars_SpeedPlanAvoi) / d;
      d = maximum(b_speed);
      b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi * d;
      b_speed[1] = e_CalibrationVars_SpeedPlanAvoi;
      d_veh[i] = 0.5 * (c_minimum(b_speed) + speed) * d;
    }

    wait_avoidOncomingVehicle = 0;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 6)) {
      /*          if ~(d_veh2waitingArea<10 && s_max(i)>d_veh2conflict(i)+l_veh && s_vehapostrophe(i)<-l_veh) */
      if ((d_veh2waitingArea < 10.0) && (d_veh[i] > d_veh2conflict[i] +
           Parameters_l_veh) && (s_vehapostrophe[i] < 0.0)) {
        i++;
      } else {
        /* 20220704 */
        wait_avoidOncomingVehicle = 1;
        exitg1 = true;
      }
    }
  }

  /*  ACC速度规划 */
  if (wait_avoidOncomingVehicle == 1) {
    /*  a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]); */
    b_speed[0] = 0.0;
    b_speed[1] = d_veh2waitingArea + CalibrationVars_ACC->d_wait;
    d = maximum(b_speed);
    b_speed[0] = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed, 1,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    b_speed[1] = b_ACC(e_CalibrationVars_SpeedPlanAvoi, 0.0, d, speed, 1,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    a_soll = c_minimum(b_speed);
  } else if (dec_avoidOncomingVehicle == 1) {
    a_soll = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed,
                   wait_avoidOncomingVehicle, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                   CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                   CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else if (d_veh2waitingArea > 2.0 * Parameters_l_veh) {
    a_soll = b_ACC(v_max, v_b, s_b, speed, wait_avoidOncomingVehicle,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else {
    a_soll = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed,
                   wait_avoidOncomingVehicle, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                   CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                   CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  }

  GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle =
    dec_avoidOncomingVehicle;
  GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle =
    wait_avoidOncomingVehicle;

  /*  v_veh1 */
  /*  a_soll */
  /*  if d_veh2waitingArea<5.4 */
  /*      s_veh1 */
  /*  end */
  /*  d_veh2waitingArea */
  /*  函数调用 */
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
  return a_soll;
}

/*
 * pos_s为网联车车头s坐标
 *  s_ped,l_ped,v_ped,psi_ped皆为大小40的数组，默认值为0,0,-1,0，s_ped按从小到大排列，psi_ped范围为0到360（沿l轴为0度，沿s轴为90度）
 *  d_veh2cross=max([0 d_veh2cross]);
 * Arguments    : double pos_s
 *                double speed
 *                double d_veh2cross
 *                double w_cross
 *                const double s_ped[40]
 *                const double l_ped[40]
 *                const double v_ped[40]
 *                const double psi_ped[40]
 *                double s_b
 *                double v_b
 *                double v_max
 *                TypeGlobVars *GlobVars
 *                double Parameters_w_veh
 *                double Parameters_l_veh
 *                const CalibSpeedPlanAvoidPedestrian c_CalibrationVars_SpeedPlanAvoi
 *                const CalibACC *CalibrationVars_ACC
 *                double *a_soll
 *                double *d_veh2stopline
 * Return Type  : void
 */
static void SpeedPlanAvoidPedestrian(double pos_s, double speed, double
  d_veh2cross, double w_cross, const double s_ped[40], const double l_ped[40],
  const double v_ped[40], const double psi_ped[40], double s_b, double v_b,
  double v_max, TypeGlobVars *GlobVars, double Parameters_w_veh, double
  Parameters_l_veh, const CalibSpeedPlanAvoidPedestrian
  c_CalibrationVars_SpeedPlanAvoi, const CalibACC *CalibrationVars_ACC, double
  *a_soll, double *d_veh2stopline)
{
  double d_veh2ped[40];
  double d_veh2ped_data[40];
  double b_speed[2];
  double d_bre;
  double timeGap;
  int d_veh2ped_size[2];
  int i;
  int partialTrueCount;
  int trueCount;
  short dec_ped;
  short wait_ped;
  signed char tmp_data[40];
  boolean_T exitg1;
  for (i = 0; i < 40; i++) {
    d_veh2ped[i] = s_ped[i] - pos_s;
    if (v_ped[i] < 0.0) {
      d_veh2ped[i] = 200.0;
    }
  }

  /* globalVariable--------------------------------------------------------------------------------------------------------------------------------------- */
  dec_ped = GlobVars->SpeedPlanAvoidPedestrian.dec_ped;
  wait_ped = GlobVars->SpeedPlanAvoidPedestrian.wait_ped;

  /*  CalibrationVariable--------------------------------------------------------------------------------------------------------------------------------- */
  /* 2.5; */
  /* -2; */
  /* v_max_in30/3.6; */
  /* 0.5m; */
  /*  v_max_int_emg=CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg;%v_max_int_emg=20/3.6 */
  /* Parameters------------------------------------------------------------------------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------------------------------------------------------------------------------- */
  /*  v_ped=2; */
  /*  v_ped=max([0.00001 2*v_ped]); */
  /*  s_ped=abs(s_ped); */
  /*  for i=1:1:50 */
  /*      v_ped(i)=max([0.00001 v_ped(i)]); */
  /*  end */
  /*  策略模式判断 */
  timeGap = 2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min;
  d_bre = (0.0 - speed * speed) / timeGap;
  if (GlobVars->SpeedPlanAvoidPedestrian.dec_ped == 0) {
    /*  if min(d_veh2ped(d_veh2ped>=0))<=d_bre+2*l_veh || (d_veh2cross<=d_bre+2*l_veh && d_veh2cross>0) % 1原为l_veh 01.21修改 */
    trueCount = 0;
    partialTrueCount = 0;
    for (i = 0; i < 40; i++) {
      if (d_veh2ped[i] >= 0.0) {
        trueCount++;
        tmp_data[partialTrueCount] = (signed char)(i + 1);
        partialTrueCount++;
      }
    }

    d_veh2ped_size[0] = 1;
    d_veh2ped_size[1] = trueCount;
    for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++)
    {
      d_veh2ped_data[partialTrueCount] = d_veh2ped[tmp_data[partialTrueCount] -
        1];
    }

    if ((e_minimum(d_veh2ped_data, d_veh2ped_size) <= d_bre) || ((d_veh2cross <=
          d_bre) && (d_veh2cross > 0.0))) {
      /*  1原为l_veh 01.21修改 */
      dec_ped = 1;
    }
  } else {
    /*  if min(d_veh2ped(d_veh2ped>=0))>max(d_bre,(0-v_max_int.^2)/(2*a_min))+3*l_veh && (d_veh2cross==0 || d_veh2cross>max(d_bre,(0-v_max_int.^2)/(2*a_min))+3*l_veh) % 1原为l_veh 01.21修改 */
    trueCount = 0;
    partialTrueCount = 0;
    for (i = 0; i < 40; i++) {
      if (d_veh2ped[i] >= 0.0) {
        trueCount++;
        tmp_data[partialTrueCount] = (signed char)(i + 1);
        partialTrueCount++;
      }
    }

    d_veh2ped_size[0] = 1;
    d_veh2ped_size[1] = trueCount;
    for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++)
    {
      d_veh2ped_data[partialTrueCount] = d_veh2ped[tmp_data[partialTrueCount] -
        1];
    }

    timeGap = fmax(d_bre, (0.0 - c_CalibrationVars_SpeedPlanAvoi.v_max_int
      * c_CalibrationVars_SpeedPlanAvoi.v_max_int) / timeGap) + 10.0;
    if ((e_minimum(d_veh2ped_data, d_veh2ped_size) > timeGap) && ((d_veh2cross ==
          0.0) || (d_veh2cross > timeGap))) {
      /*  1原为l_veh 01.21修改 */
      dec_ped = 0;
    }
  }

  /*  停车决策 */
  *d_veh2stopline = 200.0;
  if (dec_ped == 1) {
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 40)) {
      if ((v_ped[i] >= 0.0) && (d_veh2ped[i] >= 0.0)) {
        /*  v_ped(i)默认值为-1 */
        if (psi_ped[i] <= 90.0) {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = psi_ped[i] / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        } else if ((psi_ped[i] > 90.0) && (psi_ped[i] <= 270.0)) {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = (psi_ped[i] + 180.0) / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        } else {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = (psi_ped[i] + 360.0) / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        }

        b_speed[0] = speed;
        b_speed[1] = 1.0E-5;
        timeGap = maximum(b_speed);
        b_speed[0] = d_veh2ped[i] / timeGap * d_bre + 0.5 * Parameters_w_veh *
          1.2;
        b_speed[1] = 0.0;
        if (fabs(l_ped[i]) <= maximum(b_speed)) {
          wait_ped = 1;
          *d_veh2stopline = d_veh2ped[i];
          exitg1 = true;
        } else {
          i++;
        }
      } else {
        i++;
      }
    }
  }

  /*  起步决策 */
  if (wait_ped == 1) {
    wait_ped = 0;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 40)) {
      if ((v_ped[i] >= 0.0) && (d_veh2ped[i] >= 0.0)) {
        /*  v_ped(i)默认值为-1 */
        if (psi_ped[i] <= 90.0) {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = psi_ped[i] / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        } else if ((psi_ped[i] > 90.0) && (psi_ped[i] <= 270.0)) {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = (psi_ped[i] + 180.0) / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        } else {
          d_bre = l_ped[i];
          if (l_ped[i] < 0.0) {
            d_bre = -1.0;
          } else if (l_ped[i] > 0.0) {
            d_bre = 1.0;
          } else {
            if (l_ped[i] == 0.0) {
              d_bre = 0.0;
            }
          }

          timeGap = (psi_ped[i] + 360.0) / 2.0;
          b_cosd(&timeGap);
          d_bre = fmax(0.0, -v_ped[i] * d_bre * timeGap);
        }

        b_speed[0] = 0.0;
        b_speed[1] = (fabs(l_ped[i]) - 0.5 * Parameters_w_veh * 1.2) / (d_bre +
          2.2204460492503131E-16);
        timeGap = maximum(b_speed);

        /*  if ~(d_veh2ped(i)<10 && s_max>d_veh2ped(i)) % 是否考虑车长？ */
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * timeGap;
        b_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max_int;
        if (!(0.5 * (c_minimum(b_speed) + speed) * timeGap > d_veh2ped[i])) {
          /*  是否考虑车长？ */
          wait_ped = 1;
          exitg1 = true;
        } else {
          i++;
        }
      } else {
        i++;
      }
    }
  }

  *d_veh2stopline -= c_CalibrationVars_SpeedPlanAvoi.d_gap2ped;

  /* 停车位置距行人的距离 */
  if ((*d_veh2stopline >= d_veh2cross) && (*d_veh2stopline <= w_cross +
       d_veh2cross)) {
    *d_veh2stopline = d_veh2cross;
  }

  /*  ACC速度规划 */
  if (wait_ped == 0) {
    if ((dec_ped == 0) && (d_veh2cross > 2.0 * Parameters_l_veh)) {
      *a_soll = b_ACC(v_max, v_b, s_b, speed, 0, CalibrationVars_ACC->a_max,
                      CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                      CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                      CalibrationVars_ACC->tau_v_bre,
                      CalibrationVars_ACC->tau_v_emg,
                      CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                      CalibrationVars_ACC->d_wait);
    } else {
      *a_soll = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, v_b, s_b, speed,
                      0, CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                      CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                      CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                      CalibrationVars_ACC->tau_v_emg,
                      CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                      CalibrationVars_ACC->d_wait);
    }
  } else {
    b_speed[0] = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, v_b, s_b,
                       speed, wait_ped, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    b_speed[1] = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, 0.0,
                       (*d_veh2stopline + CalibrationVars_ACC->d_wait) - 0.5,
                       speed, wait_ped, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    *a_soll = c_minimum(b_speed);
  }

  GlobVars->SpeedPlanAvoidPedestrian.dec_ped = dec_ped;
  GlobVars->SpeedPlanAvoidPedestrian.wait_ped = wait_ped;

  /*  d_veh2cross<=l_veh || wait_ped==1 “<=l_veh”是否合理 */
  /*  过人行横道 有人→停车 */
}

/*
 * CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 * Arguments    : double speed
 *                double d_veh2int
 *                double d_veh2stopline
 *                double s_a
 *                double v_a
 *                double l_a
 *                double s_b
 *                double v_b
 *                double l_b
 *                double s_c
 *                double v_c
 *                double l_c
 *                TypeGlobVars *GlobVars
 *                const CalibSpeedPlanAvoidVehicle c_CalibrationVars_SpeedPlanAvoi
 *                const CalibACC *CalibrationVars_ACC
 *                double Parameters_l_veh
 * Return Type  : double
 */
static double SpeedPlanAvoidVehicle(double speed, double d_veh2int, double
  d_veh2stopline, double s_a, double v_a, double l_a, double s_b, double v_b,
  double l_b, double s_c, double v_c, double l_c, TypeGlobVars *GlobVars, const
  CalibSpeedPlanAvoidVehicle c_CalibrationVars_SpeedPlanAvoi, const CalibACC
  *CalibrationVars_ACC, double Parameters_l_veh)
{
  double dv[3];
  double b_speed[2];
  double c_speed[2];
  double a_soll;
  double d;
  double d1;
  double d2;
  double d_ist;
  double d_ist_tmp;
  double s_b_end;
  double t_a2int;
  double t_b2int;
  double t_c2int;
  double v_soll;
  short b_wait;
  short dec_bre;
  short dec_fol;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
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
  d_ist_tmp = s_a - l_a;
  d_ist = d_ist_tmp;
  v_soll = v_a;

  /*  策略模式判断 */
  if ((GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle == 0) &&
      (GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle == 0) &&
      (d_veh2stopline > 0.0) && (d_veh2stopline <= (0.0 - speed * speed) / (2.0 *
        c_CalibrationVars_SpeedPlanAvoi.a_min_com))) {
    dec_fol = 1;
  }

  if (GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle == 0) {
    /*  if d_veh2stopline<=min([d_bre+15 20]) && d_veh2stopline>0 */
    b_speed[0] = (0.0 - speed * speed) / (2.0 *
      c_CalibrationVars_SpeedPlanAvoi.a_min) + 10.0;
    b_speed[1] = 15.0;
    if ((d_veh2stopline <= c_minimum(b_speed)) && (d_veh2stopline > 0.0)) {
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

  /*  跟车决策 */
  if (dec_fol == 1) {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    guard5 = false;
    if (t_a2int < t_b2int) {
      /*  a在b先 */
      s_b_end = s_a + t_b2int * v_a;

      /*  s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2); */
      /*  s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2); */
      dv[0] = 0.0;
      d = v_b * c_CalibrationVars_SpeedPlanAvoi.t_re;
      dv[1] = d;
      d1 = v_b * v_b;
      d2 = 2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min;
      dv[2] = (v_a * v_a - d1) / d2;
      if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_b2int;
        b_speed[1] = 0.0;
        c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_b2int;
        c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
        d /= c_CalibrationVars_SpeedPlanAvoi.GapIndex;
        if (!(fmax(0.5 * (maximum(b_speed) + speed) * t_b2int, (d_veh2int
               + Parameters_l_veh) + d) < fmin(0.5 * (c_minimum(c_speed) + speed)
              * t_b2int, (s_b_end - l_a) - d))) {
          guard5 = true;
        } else {
          /*  前车=a */
        }
      } else {
        guard5 = true;
      }
    } else {
      /*  b在a先 */
      s_b_end = s_b + t_a2int * v_b;

      /*  s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2); */
      /*  s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2); */
      dv[0] = 0.0;
      d = v_a * c_CalibrationVars_SpeedPlanAvoi.t_re;
      dv[1] = d;
      d1 = v_a * v_a;
      d2 = 2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min;
      dv[2] = (v_b * v_b - d1) / d2;
      if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_a2int;
        b_speed[1] = 0.0;
        c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_a2int;
        c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
        d /= c_CalibrationVars_SpeedPlanAvoi.GapIndex;
        if (fmax(0.5 * (maximum(b_speed) + speed) * t_a2int, (d_veh2int
              + Parameters_l_veh) + d) < fmin(0.5 * (c_minimum(c_speed) + speed)
             * t_a2int, (s_b_end - l_b) - d)) {
          /*  前车=b */
          d_ist = s_b - l_b;
          v_soll = v_b;
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }
    }

    if (guard5) {
      s_b_end = s_b + t_c2int * v_b;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      /*  b在c先 */
      if (t_b2int < t_c2int) {
        dv[0] = 0.0;
        d = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[1] = d;
        dv[2] = (d1 - v_c * v_c) / d2;
        if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
          b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          b_speed[1] = 0.0;
          c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
          d /= c_CalibrationVars_SpeedPlanAvoi.GapIndex;
          if (fmax(0.5 * (maximum(b_speed) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + d) < fmin(0.5 * (c_minimum(c_speed) +
                speed) * t_c2int, (s_b_end - l_b) - d)) {
            guard1 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    }

    if (guard4) {
      s_b_end = s_a + t_c2int * v_a;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      /*  a在c先 */
      if (t_a2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[2] = (d1 - v_c * v_c) / d2;
        if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
          b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          b_speed[1] = 0.0;
          c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
          if (!(fmax(0.5 * (maximum(b_speed) + speed) * t_c2int, (d_veh2int
                 + Parameters_l_veh) + v_c *
                     c_CalibrationVars_SpeedPlanAvoi.t_re /
                     c_CalibrationVars_SpeedPlanAvoi.GapIndex) < fmin(0.5 *
                (c_minimum(c_speed) + speed) * t_c2int, (s_b_end - l_a) - v_c *
                c_CalibrationVars_SpeedPlanAvoi.t_re /
                c_CalibrationVars_SpeedPlanAvoi.GapIndex))) {
            guard2 = true;
          } else {
            /*  前车=a */
          }
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    }

    if (guard3) {
      if (s_c <= -200.0) {
        guard1 = true;
      } else {
        /*  前车=c */
        d_ist = s_c - l_c;
        v_soll = v_c;
      }
    }

    if (guard2 && (!(s_c <= -200.0))) {
      /*  前车=c */
      d_ist = s_c - l_c;
      v_soll = v_c;
    } else {
      /*  前车=a */
    }

    if (guard1) {
      /*  前车=b */
      d_ist = s_b - l_b;
      v_soll = v_b;
    }

    /*      s_a,v_a,s_b,v_b,s_c,v_c */
    /*                      d_ist */
    /*                      v_soll */
    /*                  v_soll */
  }

  /*  停车决策 */
  if (dec_bre == 1) {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    guard5 = false;
    guard6 = false;
    if (t_a2int < t_b2int) {
      /*  a在b先 */
      s_b_end = s_a + t_b2int * v_a;

      /*  s_max=speed*t_b2int+0.5*a_max*(t_b2int.^2); */
      /*  s_min=speed*t_b2int+0.5*a_min*(t_b2int.^2); */
      dv[0] = 0.0;
      dv[1] = v_b * c_CalibrationVars_SpeedPlanAvoi.t_re;
      dv[2] = (v_a * v_a - v_b * v_b) / (2.0 *
        c_CalibrationVars_SpeedPlanAvoi.a_min);
      if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_b2int;
        b_speed[1] = 0.0;
        c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_b2int;
        c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
        if (fmax(0.5 * (maximum(b_speed) + speed) * t_b2int, (d_veh2int
              + Parameters_l_veh) + v_b * c_CalibrationVars_SpeedPlanAvoi.t_re /
                 c_CalibrationVars_SpeedPlanAvoi.GapIndex) < fmin(0.5 *
             (c_minimum(c_speed) + speed) * t_b2int, (s_b_end - l_a) - v_b *
             c_CalibrationVars_SpeedPlanAvoi.t_re /
             c_CalibrationVars_SpeedPlanAvoi.GapIndex)) {
          /*              prereq1 */
          /*              prereq2 */
          /*  前车=a */
          d_ist = d_ist_tmp;
          v_soll = v_a;
        } else {
          guard6 = true;
        }
      } else {
        guard6 = true;
      }
    } else {
      /*  b在a先 */
      s_b_end = s_b + t_a2int * v_b;

      /*  s_max=speed*t_a2int+0.5*a_max*(t_a2int.^2); */
      /*  s_min=speed*t_a2int+0.5*a_min*(t_a2int.^2); */
      dv[0] = 0.0;
      dv[1] = v_a * c_CalibrationVars_SpeedPlanAvoi.t_re;
      dv[2] = (v_b * v_b - v_a * v_a) / (2.0 *
        c_CalibrationVars_SpeedPlanAvoi.a_min);
      if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_a2int;
        b_speed[1] = 0.0;
        c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_a2int;
        c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
        d = v_a * c_CalibrationVars_SpeedPlanAvoi.t_re /
          c_CalibrationVars_SpeedPlanAvoi.GapIndex;
        if (fmax(0.5 * (maximum(b_speed) + speed) * t_a2int, (d_veh2int
              + Parameters_l_veh) + d) < fmin(0.5 * (c_minimum(c_speed) + speed)
             * t_a2int, (s_b_end - l_b) - d)) {
          /*  前车=b */
          d_ist = s_b - l_b;
          v_soll = v_b;
        } else {
          guard5 = true;
        }
      } else {
        guard5 = true;
      }
    }

    if (guard6) {
      s_b_end = s_b + t_c2int * v_b;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      /*  b在c先 */
      if (t_b2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[2] = (v_b * v_b - v_c * v_c) / (2.0 *
          c_CalibrationVars_SpeedPlanAvoi.a_min);
        if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
          b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          b_speed[1] = 0.0;
          c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
          if (fmax(0.5 * (maximum(b_speed) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + v_c * c_CalibrationVars_SpeedPlanAvoi.t_re
                   / c_CalibrationVars_SpeedPlanAvoi.GapIndex) < fmin(0.5 *
               (c_minimum(c_speed) + speed) * t_c2int, (s_b_end - l_b) - v_c *
               c_CalibrationVars_SpeedPlanAvoi.t_re /
               c_CalibrationVars_SpeedPlanAvoi.GapIndex)) {
            guard2 = true;
          } else {
            guard4 = true;
          }
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }
    }

    if (guard5) {
      s_b_end = s_a + t_c2int * v_a;

      /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
      /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
      /*  a在c先 */
      if (t_a2int < t_c2int) {
        dv[0] = 0.0;
        dv[1] = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[2] = (v_a * v_a - v_c * v_c) / (2.0 *
          c_CalibrationVars_SpeedPlanAvoi.a_min);
        if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
          b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          b_speed[1] = 0.0;
          c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
          if (fmax(0.5 * (maximum(b_speed) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + v_c * c_CalibrationVars_SpeedPlanAvoi.t_re
                   / c_CalibrationVars_SpeedPlanAvoi.GapIndex) < fmin(0.5 *
               (c_minimum(c_speed) + speed) * t_c2int, (s_b_end - l_a) - v_c *
               c_CalibrationVars_SpeedPlanAvoi.t_re /
               c_CalibrationVars_SpeedPlanAvoi.GapIndex)) {
            guard1 = true;
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    }

    if (guard4) {
      if (s_c <= -200.0) {
        guard2 = true;
      } else {
        /*  无解 */
        b_wait = 1;
      }
    }

    if (guard3) {
      if (s_c <= -200.0) {
        guard1 = true;
      } else {
        /*  无解 */
        b_wait = 1;
      }
    }

    if (guard2) {
      /*  前车=b */
      d_ist = s_b - l_b;
      v_soll = v_b;
    }

    if (guard1) {
      /*  前车=a */
      d_ist = d_ist_tmp;
      v_soll = v_a;
    }
  }

  /*  起步决策 */
  if (b_wait == 1) {
    s_b_end = s_b + t_c2int * v_b;

    /*  s_max=speed*t_c2int+0.5*a_max*(t_c2int.^2); */
    /*  s_min=speed*t_c2int+0.5*a_min*(t_c2int.^2); */
    /*  b在c先 */
    if ((s_a > 10.0) && (d_veh2stopline < 10.0)) {
      prereq4 = true;
    } else {
      prereq4 = false;
    }

    /*  prereq4=(s_a>10)&&(d_veh2stopline<15); */
    if (t_b2int < t_c2int) {
      dv[0] = 0.0;
      d = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
      dv[1] = d;
      dv[2] = (v_b * v_b - v_c * v_c) / (2.0 *
        c_CalibrationVars_SpeedPlanAvoi.a_min);
      if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
        b_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
        b_speed[1] = 0.0;
        c_speed[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
        c_speed[1] = c_CalibrationVars_SpeedPlanAvoi.v_max;
        d /= c_CalibrationVars_SpeedPlanAvoi.GapIndex;
        if ((fmax(0.5 * (maximum(b_speed) + speed) * t_c2int, (d_veh2int
               + Parameters_l_veh) + d) < fmin(0.5 * (c_minimum(c_speed) + speed)
              * t_c2int, (s_b_end - l_b) - d)) && prereq4) {
          /*  前车=b */
          d_ist = s_b - l_b;
          v_soll = v_b;
          b_wait = 0;
        }
      }
    }
  }

  /*  ACC速度规划 */
  /*  a_acc=min([ACC(v_max,v_soll,d_ist,speed) ACC(v_max,v_a,s_a,speed)]); */
  s_b_end = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, v_a, d_ist_tmp, speed,
                  b_wait, CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                  CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                  CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                  CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
                  CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
  b_speed[0] = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, v_soll, d_ist, speed,
                     b_wait, CalibrationVars_ACC->a_max,
                     CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                     CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                     CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                     CalibrationVars_ACC->d_wait);
  b_speed[1] = s_b_end;
  a_soll = c_minimum(b_speed);
  if (b_wait == 0) {
    if (d_veh2stopline <= 0.0) {
      b_speed[0] = a_soll;
      b_speed[1] = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, v_b, s_b - l_b,
                         speed, 0, CalibrationVars_ACC->a_max,
                         CalibrationVars_ACC->a_min,
                         CalibrationVars_ACC->tau_v_com,
                         CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                         CalibrationVars_ACC->tau_v_bre,
                         CalibrationVars_ACC->tau_v_emg,
                         CalibrationVars_ACC->tau_d_emg,
                         CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
      a_soll = c_minimum(b_speed);
    }
  } else {
    /*  停车待通行状态下速度规划 */
    b_speed[0] = 0.0;
    b_speed[1] = d_veh2stopline + CalibrationVars_ACC->d_wait;
    d = maximum(b_speed);
    b_speed[0] = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, 0.0, d, speed,
                       b_wait, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    b_speed[1] = s_b_end;
    a_soll = c_minimum(b_speed);
  }

  if (dec_fol == 1) {
    /* 跟车状态下，跟主路车加速度和停车加速度取最大且与匝道前车加速度取最小 */
    b_speed[0] = 0.0;
    b_speed[1] = d_veh2stopline + CalibrationVars_ACC->d_wait;
    d = maximum(b_speed);
    b_speed[0] = a_soll;
    b_speed[1] = ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, 0.0, d, speed, 1.0,
                     CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                     CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                     CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                     CalibrationVars_ACC->d_wait);
    a_soll = fmin(s_b_end, maximum(b_speed));
  }

  GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle = dec_fol;
  GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle = dec_bre;
  GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle = b_wait;
  return a_soll;
}

/*
 * greenLight：0 红灯
 *  greenLight：1 绿灯
 *  greenLight：2 黄灯
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 * Arguments    : double speed
 *                double d_veh2int
 *                double s_b
 *                double v_b
 *                double greenLight
 *                double time2nextSwitch
 *                double v_max
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_SpeedPlanTraf
 *                double d_CalibrationVars_SpeedPlanTraf
 *                double e_CalibrationVars_SpeedPlanTraf
 *                double f_CalibrationVars_SpeedPlanTraf
 *                const CalibACC *CalibrationVars_ACC
 * Return Type  : double
 */
static double SpeedPlanTrafficLight(double speed, double d_veh2int, double s_b,
  double v_b, double greenLight, double time2nextSwitch, double v_max,
  TypeGlobVars *GlobVars, double c_CalibrationVars_SpeedPlanTraf, double
  d_CalibrationVars_SpeedPlanTraf, double e_CalibrationVars_SpeedPlanTraf,
  double f_CalibrationVars_SpeedPlanTraf, const CalibACC *CalibrationVars_ACC)
{
  double b_speed[2];
  double a_soll;
  double d;
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
  /* 30/3.6; */
  /* 1.5; */
  /* Parameters-------------------------------------------------------------------------------------------------------------------------- */
  /*  w_veh=Parameters.w_veh; */
  decel = 0;
  b_speed[0] = 0.0;
  b_speed[1] = d_veh2int;
  d_veh2int = maximum(b_speed);

  /*  策略模式判断 */
  if ((GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight == 0) &&
      (GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight == 0) && (d_veh2int >
       10.0) && (d_veh2int < (0.0 - speed * speed) / (2.0 *
        c_CalibrationVars_SpeedPlanTraf))) {
    dec_fol = 1;
  }

  if (GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight == 0) {
    if ((d_veh2int <= (0.0 - speed * speed) / (2.0 *
          e_CalibrationVars_SpeedPlanTraf) + 10.0) && (d_veh2int > 1.0)) {
      /*  1原为10 12.31修改 */
      dec_bre = 1;
    }
  } else {
    if ((d_veh2int <= 1.0) || (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight
         == 1)) {
      /*  1原为10 12.31修改 */
      dec_bre = 0;
    }
  }

  if ((dec_fol == 1) && (dec_bre == 1)) {
    dec_fol = 0;
  }

  /*  减速决策 */
  if (dec_fol == 1) {
    if (greenLight == 1.0) {
      if (!(d_veh2int / speed < time2nextSwitch)) {
        decel = 1;
      }
    } else {
      decel = 1;
    }
  }

  /*  停车决策 */
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

  /*  起步决策 */
  if ((b_wait == 1) && (greenLight == 1.0) && (d_veh2int < 30.0)) {
    /*  12.31修改 */
    b_speed[0] = ACC(v_max, v_b, s_b, speed, 0.0, CalibrationVars_ACC->a_max,
                     CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                     CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                     CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                     CalibrationVars_ACC->d_wait);
    b_speed[1] = d_CalibrationVars_SpeedPlanTraf;
    d = c_minimum(b_speed);
    b_speed[0] = speed + d * time2nextSwitch;
    b_speed[1] = f_CalibrationVars_SpeedPlanTraf;
    if (0.5 * (c_minimum(b_speed) + speed) * time2nextSwitch > d_veh2int) {
      b_wait = 0;

      /*  12.31修改 */
      /*  decel=0; */
      /*  dec_fol=0; */
      /*  dec_bre=0; */
    }
  }

  /*  ACC速度规划 */
  if ((dec_fol == 0) && (dec_bre == 0) && (d_veh2int > 10.0) && (b_wait == 0)) {
    a_soll = b_ACC(v_max, v_b, s_b, speed, 0, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min, CalibrationVars_ACC->tau_v_com,
                   CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                   CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else if (decel == 1) {
    /*          a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars) ACCcust(v_max_int,0,d_veh2int+CalibrationVars.ACC.d_wait-0.5,speed,a_max,a_min_com,t_acc,CalibrationVars)]);            */
    b_speed[0] = ACC(f_CalibrationVars_SpeedPlanTraf, 0.0, (d_veh2int
      + CalibrationVars_ACC->d_wait) - 0.5, speed, 1.0,
                     CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                     CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                     CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                     CalibrationVars_ACC->tau_v_emg,
                     CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                     CalibrationVars_ACC->d_wait);
    b_speed[1] = c_CalibrationVars_SpeedPlanTraf;
    d = maximum(b_speed);
    b_speed[0] = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    b_speed[1] = d;
    a_soll = c_minimum(b_speed);

    /* 20220712,去掉ACCcust */
  } else if (b_wait == 1) {
    b_speed[0] = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, 1,
                       CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    b_speed[1] = b_ACC(f_CalibrationVars_SpeedPlanTraf, 0.0, (d_veh2int
      + CalibrationVars_ACC->d_wait) - 0.5, speed, 1, CalibrationVars_ACC->a_max,
                       CalibrationVars_ACC->a_min,
                       CalibrationVars_ACC->tau_v_com,
                       CalibrationVars_ACC->tau_v, CalibrationVars_ACC->tau_d,
                       CalibrationVars_ACC->tau_v_bre,
                       CalibrationVars_ACC->tau_v_emg,
                       CalibrationVars_ACC->tau_d_emg,
                       CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    a_soll = c_minimum(b_speed);

    /*  a_soll */
    /*  greenLight */
  } else {
    a_soll = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
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
 *                short GoalLaneIndex
 *                short BackupTargetLaneIndex
 *                double d_veh2int
 *                double d_veh2goal
 *                double WidthOfLanes[6]
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
  short TargetLaneIndex, short GoalLaneIndex, short BackupTargetLaneIndex,
  double d_veh2int, double d_veh2goal, double WidthOfLanes[6], double v_max,
  const short LanesWithFail[6], TypeGlobVars *GlobVars, const
  TypeCalibrationVars *CalibrationVars, const TypeParameters Parameters, double *
  a_soll, double traj_s[80], double traj_l[80], double traj_psi[80], double
  traj_vs[80], double traj_vl[80], double traj_omega[80])
{
  b_anonymous_function e_this_tunableEnvironment_f1_tu[1];
  cell_wrap_5 fun_S_tunableEnvironment[1];
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
  double S_max;
  double S_min;
  double S_tlc;
  double TargetLaneBehindDis;
  double TargetLaneBehindVel;
  double TargetLaneFrontDis;
  double TargetLaneFrontVel;
  double V_b_end;
  double V_c_end;
  double V_end;
  double a;
  double b_a;
  double s_d;
  double s_e;
  double speed_tmp_tmp;
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

  /* 决策之后目标车道，用于换道重归划判断 */
  if ((d_veh2goal < 40.0) && (GoalLaneIndex != CurrentLaneIndex)) {
    S_tlc = WidthOfLanes[GoalLaneIndex - 1];
    WidthOfLanes[GoalLaneIndex - 1] = S_tlc + 2.0 * ((0.5 * S_tlc - 0.5 *
      Parameters.w_veh) - 0.2);
  }

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
    BackupTargetLaneIndex = f_minimum(b_CurrentLaneIndex);
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
  S_min = maximum(b_speed);
  b_speed[0] = S_min;
  b_speed[1] = 2.5;
  t_lc = ceil(c_minimum(b_speed) / 0.1) * 0.1;
  memset(&traj[0], 0, 720U * sizeof(double));

  /*  w_veh=1.8; */
  b_CurrentLaneIndex[0] = (short)(CurrentLaneIndex + 1);
  b_CurrentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = f_minimum(b_CurrentLaneIndex);
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

  /* 2020324,编译c报错增加初始值 */
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
      fzero(d_this_tunableEnvironment_f1_tu, S_end, w_lane, &S_max, &S_tlc, &a);
      b_speed[0] = t_lc;
      b_speed[1] = S_max;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      b_speed[1] = (0.0 - speed * speed) / (2.0 *
        CalibrationVars->TrajPlanLaneChange.a_min);
      b_a = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      a = maximum(b_speed);
      speed_tmp_tmp = w_lane * w_lane;
      b_speed[0] = sqrt(b_a * b_a - speed_tmp_tmp);
      b_speed[1] = sqrt(a * a - speed_tmp_tmp);
      S_max = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_speed[0] = ACC(v_max, TargetLaneFrontVel, TargetLaneFrontDis -
                       TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_min = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + S_min *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      V_b_end = maximum(b_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (V_b_end + TargetLaneBehindVel) *
        t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel_strich*a_max_comfort)*t_lc; */
      /* , */
      S_min = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_min) && (TargetLaneFrontDis <= speed * t_lc))
      {
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + speed_tmp_tmp) / t_lc * 2.0 - speed;
        b_speed[0] = V_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(b_a * b_a - speed_tmp_tmp);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= TargetLaneBehindVel * t_lc) &&
                 (TargetLaneFrontDis > speed * t_lc)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + speed_tmp_tmp) / t_lc * 2.0 - speed;
        b_speed[0] = V_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        w_lane = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / w_lane) + Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        TargetLaneFrontVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[0] = TargetLaneFrontVel;
        b_S_b_end[1] = TargetLaneFrontVel - (V_c_end * V_c_end - V_end * V_end) /
          w_lane;
        b_S_b_end[2] = sqrt(b_a * b_a - speed_tmp_tmp);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis > S_min) && (TargetLaneFrontDis > speed *
                  t_lc)) {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
        b_speed[0] = V_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(b_a * b_a - speed_tmp_tmp);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + speed_tmp_tmp) / t_lc * 2.0 - speed;
        b_speed[0] = V_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(b_a * b_a - speed_tmp_tmp);
        S_max = d_minimum(b_S_b_end);
      }

      /* 目标车道前车减速，后车加速极限条件下仍可避免碰撞 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end));%被条件9覆盖，暂时注释 */
      if ((S_max > S_end) && (S_end > S_min)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /* 目标车道后车车头位于自车后方时才换道 */
      /* 目标车道前车车尾位于自车前方时才换到 */
      /* 换道时刻前车车尾与自车保证一定距离 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > V_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            V_b_end * V_b_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end > speed +
           CalibrationVars->TrajPlanLaneChange.a_min * t_lc) && (V_end < speed +
           CalibrationVars->TrajPlanLaneChange.a_max * t_lc) && prereq5 &&
          (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (TargetLaneBehindDis <= -Parameters.l_veh) &&
          (TargetLaneFrontDis >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex = TargetLaneIndex;
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
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_min = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + S_min *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      V_b_end = maximum(b_speed);
      S_b_end = TargetLaneBehindDis + 0.5 * (V_b_end + TargetLaneBehindVel) *
        t_lc;
      t_mid = 0.5 * t_lc;

      /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  S_b_end=s_b+v_b*t_lc+0.5*(index_accel_strich*a_max_comfort)*t_lc*t_lc; */
      /*  S_c_end=s_c+v_c*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
      /*  V_c_end=v_c+(index_accel*a_min_comfort)*t_lc; */
      /*  V_b_end=v_b+(index_accel*index_accel_strich)*t_lc; */
      /* , */
      S_min = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_min) && (TargetLaneFrontDis <= speed * (t_lc
            + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /*                  b车不存在 c车存在 */
        b_speed[0] = 0.5 * (speed + V_c_end);
        b_speed[1] = speed;
        V_end = c_minimum(b_speed);
        w_lane = S_b_end + V_b_end * CalibrationVars->TrajPlanLaneChange.t_re;
        b_S_b_end[0] = w_lane + Parameters.l_veh;
        TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = (w_lane + (V_end * V_end - V_b_end * V_b_end) /
                        TargetLaneFrontVel) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
          TargetLaneFrontVel;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= S_min) && (TargetLaneFrontDis > speed *
                  (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车存在 c车不存在 */
        b_speed[0] = 0.5 * (speed + V_b_end);
        b_speed[1] = speed;
        V_end = c_minimum(b_speed);
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + V_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis > S_min) && (TargetLaneFrontDis > speed *
                  (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车不存在 */
        S_end = speed * t_lc;
        V_end = speed;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        w_lane = speed * speed;
        TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + V_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (w_lane -
          V_b_end * V_b_end) / TargetLaneFrontVel) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_tlc = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
        b_S_b_end[0] = S_tlc;
        b_S_b_end[1] = S_tlc - (V_c_end * V_c_end - w_lane) / TargetLaneFrontVel;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        b_speed[0] = speed;
        b_speed[1] = 0.5 * (V_b_end + V_c_end);
        V_end = c_minimum(b_speed);
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + V_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      }

      /*  参见TrajPlanLaneChange_S_max_withAccel.bmp */
      b_speed[0] = V_end;
      b_speed[1] = speed;
      TargetLaneFrontVel = fabs(V_end - speed);
      b_a = (CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc -
             TargetLaneFrontVel) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移  */
      S_tlc = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      a = (S_tlc - TargetLaneFrontVel) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= S_min) {
        S_min = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        if ((S_end <= (t_lc * c_minimum(b_speed) + S_min) - b_a * b_a /
             CalibrationVars->TrajPlanLaneChange.a_max_comfort) && (S_end >=
             (t_lc * maximum(b_speed) - S_min) + a * a /
             CalibrationVars->TrajPlanLaneChange.a_max_comfort)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((S_c_end - S_b_end) - Parameters.l_veh > V_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            V_b_end * V_b_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >= speed +
           CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc) && (V_end <=
           speed + S_tlc) && prereq5 && (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (TargetLaneBehindDis <= -Parameters.l_veh) &&
          (TargetLaneFrontDis >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex = TargetLaneIndex;
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
      fzero(c_this_tunableEnvironment_f1_tu, S_end, w_lane, &S_max, &S_tlc, &a);
      b_speed[0] = t_lc;
      b_speed[1] = S_max;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      speed_tmp_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_speed[1] = (0.0 - speed * speed) / speed_tmp_tmp;
      b_a = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      a = maximum(b_speed);
      S_tlc = w_lane * w_lane;
      b_speed[0] = sqrt(b_a * b_a - S_tlc);
      b_speed[1] = sqrt(a * a - S_tlc);
      S_max = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_e + CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      S_b_end = maximum(b_speed);
      V_c_end = s_e + 0.5 * (S_b_end + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_min = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + S_min *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneBehindVel = maximum(b_speed);
      V_b_end = s_d + 0.5 * (TargetLaneBehindVel + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      S_min = v_d * t_lc;
      if ((-s_d > S_min) && (s_e <= speed * t_lc)) {
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = TargetLaneBehindVel - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((V_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneBehindVel * TargetLaneBehindVel) / speed_tmp_tmp) +
          Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        w_lane = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (S_b_end * S_b_end - V_end * V_end) /
          speed_tmp_tmp;
        b_S_b_end[2] = sqrt(b_a * b_a - S_tlc);
        S_max = d_minimum(b_S_b_end);
      } else if ((-s_d <= S_min) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = TargetLaneBehindVel - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((V_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneBehindVel * TargetLaneBehindVel) / speed_tmp_tmp) +
          Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        w_lane = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (S_b_end * S_b_end - V_end * V_end) /
          speed_tmp_tmp;
        b_S_b_end[2] = sqrt(b_a * b_a - S_tlc);
        S_max = d_minimum(b_S_b_end);
      } else if ((-s_d > v_d * t_lc) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
        b_speed[0] = TargetLaneBehindVel - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((V_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneBehindVel * TargetLaneBehindVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        b_S_b_end[0] = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (V_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (S_b_end *
          S_b_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(b_a * b_a - S_tlc);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = TargetLaneBehindVel - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((V_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneBehindVel * TargetLaneBehindVel) / speed_tmp_tmp) +
          Parameters.l_veh;
        b_S_b_end[2] = S_max;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        b_a = speed * t_lc + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
          t_lc * t_lc;
        w_lane = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (S_b_end * S_b_end - V_end * V_end) /
          speed_tmp_tmp;
        b_S_b_end[2] = sqrt(b_a * b_a - S_tlc);
        S_max = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((S_max > S_end) && (S_end > S_min)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      if (((V_c_end - V_b_end) - Parameters.l_veh > TargetLaneBehindVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (S_b_end * S_b_end -
            TargetLaneBehindVel * TargetLaneBehindVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end > speed +
           CalibrationVars->TrajPlanLaneChange.a_min * t_lc) && (V_end < speed +
           CalibrationVars->TrajPlanLaneChange.a_max * t_lc) && prereq5 &&
          (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (s_e >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          TargetLaneIndex = BackupTargetLaneIndex;
          GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
            BackupTargetLaneIndex;
        }
      }
    } else {
      b_speed[0] = 0.0;
      a = CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort;
      b_speed[1] = v_e + a * t_lc;
      S_b_end = maximum(b_speed);
      V_c_end = s_e + 0.5 * (S_b_end + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_min = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + S_min *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneBehindVel = maximum(b_speed);
      V_b_end = s_d + 0.5 * (TargetLaneBehindVel + v_d) * t_lc;
      t_mid = 0.5 * t_lc;

      /* , */
      S_min = v_d * t_lc;
      guard1 = false;
      guard2 = false;
      if (-s_d > S_min) {
        S_max = speed * t_lc;
        if (s_e <= S_max) {
          /*                  b车不存在 c车存在 */
          b_speed[0] = 0.5 * (speed + S_b_end);
          b_speed[1] = speed;
          V_end = c_minimum(b_speed);
          w_lane = V_b_end + TargetLaneBehindVel *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = w_lane + Parameters.l_veh;
          TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = (w_lane + (V_end * V_end - TargetLaneBehindVel *
            TargetLaneBehindVel) / TargetLaneFrontVel) + Parameters.l_veh;
          b_S_b_end[2] = S_max + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          S_min = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          w_lane = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = w_lane - (S_b_end * S_b_end - V_end * V_end) /
            TargetLaneFrontVel;
          b_S_b_end[2] = S_max + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          S_max = d_minimum(b_S_b_end);
          b_S_b_end[0] = S_min;
          b_S_b_end[1] = S_max;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        if ((-s_d <= S_min) && (s_e > speed * t_lc)) {
          /* , */
          /*                  b车存在 c车不存在 */
          b_speed[0] = 0.5 * (speed + TargetLaneBehindVel);
          b_speed[1] = speed;
          V_end = c_minimum(b_speed);
          b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                          CalibrationVars->TrajPlanLaneChange.t_re) +
            Parameters.l_veh;
          w_lane = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = ((V_b_end + TargetLaneBehindVel *
                           CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
            V_end - TargetLaneBehindVel * TargetLaneBehindVel) / w_lane) +
            Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          S_min = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          TargetLaneFrontVel = V_c_end -
            CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = TargetLaneFrontVel;
          b_S_b_end[1] = TargetLaneFrontVel - (S_b_end * S_b_end - V_end * V_end)
            / w_lane;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          S_max = d_minimum(b_S_b_end);
          b_S_b_end[0] = S_min;
          b_S_b_end[1] = S_max;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else if (-s_d > S_min) {
          S_end = speed * t_lc;
          if (s_e > S_end) {
            /* , */
            /*                  b车不存在 c车不存在 */
            V_end = speed;
            b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            w_lane = speed * speed;
            TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((V_b_end + TargetLaneBehindVel *
                             CalibrationVars->TrajPlanLaneChange.t_re) + (w_lane
              - TargetLaneBehindVel * TargetLaneBehindVel) / TargetLaneFrontVel)
              + Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            S_tlc = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
            b_S_b_end[0] = S_tlc;
            b_S_b_end[1] = S_tlc - (S_b_end * S_b_end - w_lane) /
              TargetLaneFrontVel;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            S_max = d_minimum(b_S_b_end);
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /*                  b车存在 c车存在 */
        b_speed[0] = speed;
        b_speed[1] = 0.5 * (TargetLaneBehindVel + S_b_end);
        V_end = c_minimum(b_speed);
        b_S_b_end[0] = (V_b_end + TargetLaneBehindVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((V_b_end + TargetLaneBehindVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - TargetLaneBehindVel * TargetLaneBehindVel) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        w_lane = V_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (S_b_end * S_b_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      }

      b_speed[0] = V_end;
      b_speed[1] = speed;
      S_tlc = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      TargetLaneFrontVel = (S_tlc - fabs(V_end - speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= S_min) {
        S_min = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        S_max = TargetLaneFrontVel * TargetLaneFrontVel /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_speed) + S_min) - S_max) && (S_end >=
             (t_lc * maximum(b_speed) - S_min) + S_max)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((V_c_end - V_b_end) - Parameters.l_veh > TargetLaneBehindVel *
           CalibrationVars->TrajPlanLaneChange.t_re + (S_b_end * S_b_end -
            TargetLaneBehindVel * TargetLaneBehindVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >= speed +
           CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc) && (V_end <=
           speed + S_tlc) && prereq5 && (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (s_e >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel + a * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          TargetLaneIndex = BackupTargetLaneIndex;
          GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex =
            BackupTargetLaneIndex;
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
    b_a = 3.0 * fun_S_tunableEnvironment[0].f1[0];
    a = 4.0 * fun_S_tunableEnvironment[0].f1[1];
    S_tlc = 5.0 * fun_S_tunableEnvironment[0].f1[2];
    for (SwitchACC = 0; SwitchACC < 100; SwitchACC++) {
      S_min = x1[SwitchACC];
      S_min = ((b_a * (S_min * S_min) + a * rt_powd_snf(S_min, 3.0)) + S_tlc *
               rt_powd_snf(S_min, 4.0)) / 1.0E+6;
      y[SwitchACC] = sqrt(S_min * S_min + 1.0);
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
    S_min = t_lc * t_lc;
    dv1[10] = S_min;
    dv1[14] = rt_powd_snf(t_lc, 3.0);
    dv1[3] = 0.0;
    dv1[7] = 1.0;
    dv1[11] = 2.0 * t_lc;
    dv1[15] = 3.0 * S_min;
    c_mldivide(dv1, para_ST);

    /*  ST参数到轨迹 */
    S_min = rt_roundd_snf(t_lc / 0.05);
    i = (int)(S_min + 1.0);
    if (0 <= (int)(S_min + 1.0) - 1) {
      e_this_tunableEnvironment_f1_tu[0].tunableEnvironment[0] =
        fun_S_tunableEnvironment[0];
      b_speed[0] = 0.0;
      b_speed[1] = S_tlc;
    }

    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      S_tlc = 0.05 * (((double)SwitchACC + 1.0) - 1.0);
      a = S_tlc * S_tlc;
      S_traj[SwitchACC] = ((para_ST[0] + para_ST[1] * S_tlc) + para_ST[2] * a) +
        para_ST[3] * rt_powd_snf(S_tlc, 3.0);
      V_traj[SwitchACC] = (para_ST[1] + para_ST[2] * 2.0 * S_tlc) + para_ST[3] *
        3.0 * a;
      b_fzero(e_this_tunableEnvironment_f1_tu, S_traj, (double)SwitchACC + 1.0,
              b_speed, &X_traj[SwitchACC], &S_tlc, &a);
    }

    i = (int)(t_lc / 0.05);
    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      S_max = X_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC] = pos_s + S_max;
      i3 = CurrentLaneIndex - TargetLaneIndex;
      S_tlc = rt_powd_snf(S_max, 3.0);
      a = rt_powd_snf(S_max, 4.0);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 120] =
        pos_l_CurrentLane + (double)i3 * ((fun_S_tunableEnvironment[0].f1[0] *
        S_tlc + fun_S_tunableEnvironment[0].f1[1] * a) +
        fun_S_tunableEnvironment[0].f1[2] * rt_powd_snf(S_max, 5.0)) * 1.0E-6;
      S_max = atan(((fun_S_tunableEnvironment[0].f1[0] * 3.0 * (S_max * S_max) +
                     fun_S_tunableEnvironment[0].f1[1] * 4.0 * S_tlc) +
                    fun_S_tunableEnvironment[0].f1[2] * 5.0 * a) * 1.0E-6);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 240] = 90.0 -
        (double)i3 * 180.0 / 3.1415926535897931 * S_max;
      S_max *= 57.295779513082323;
      S_tlc = S_max;
      b_cosd(&S_tlc);
      a = V_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 360] = a * S_tlc;
      b_sind(&S_max);
      GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 480] = (double)i3 *
        a * S_max;
      if (SwitchACC + 1U == 1U) {
        GlobVars->TrajPlanLaneChange.LaneChangePath[600] = 0.0;
      } else {
        GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 600] =
          (GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 240] -
           GlobVars->TrajPlanLaneChange.LaneChangePath[SwitchACC + 239]) / 0.05;
      }
    }

    if (GlobVars->TrajPlanLaneChange.LaneChangePath[(int)S_min + 359] == 0.0) {
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
    S_min = rt_roundd_snf(t_lc_traj / 0.1);
    S_max = (S_min + 1.0) - (double)DurationLaneChange;
    if (S_max < 32768.0) {
      if (S_max >= -32768.0) {
        i1 = (short)S_max;
      } else {
        i1 = MIN_int16_T;
      }
    } else if (S_max >= 32768.0) {
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
      S_max = (rt_roundd_snf(t_lc_traj / 0.1) + 1.0) - (double)
        DurationLaneChange;
      if (S_max < 32768.0) {
        if (S_max >= -32768.0) {
          count_2 = (short)S_max;
        } else {
          count_2 = MIN_int16_T;
        }
      } else if (S_max >= 32768.0) {
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
        S_min) - 1] + traj[6 * (i1 - 1) + 3] * 0.05 * (double)i;
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
      *a_soll = ACC(8.3333333333333339, CurrentLaneFrontVel, CurrentLaneFrontDis,
                    speed, b_wait, CalibrationVars->ACC.a_max,
                    CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
                    CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                    CalibrationVars->ACC.tau_v_bre,
                    CalibrationVars->ACC.tau_v_emg,
                    CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                    CalibrationVars->ACC.d_wait);
    } else {
      *a_soll = ACC(v_max, CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                    b_wait, CalibrationVars->ACC.a_max,
                    CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
                    CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                    CalibrationVars->ACC.tau_v_bre,
                    CalibrationVars->ACC.tau_v_emg,
                    CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                    CalibrationVars->ACC.d_wait);
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

  /* 决策之后目标车道 */
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
  /*  函数调用 */
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
  /*  验证横向加速度在范围内 */
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
  double traj_s[80], double traj_l[80], double traj_psi[80], double traj_vs[80],
  double traj_vl[80], double traj_omega[80])
{
  static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };

  static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };

  static const signed char iv2[6] = { 0, 0, 2, 0, 0, 0 };

  anonymous_function c_this_tunableEnvironment_f1_tu[1];
  cell_wrap_3 fun_S_tunableEnvironment[1];
  double traj[480];
  double c[100];
  double x1[100];
  double y[100];
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
  double b;
  double b_c;
  double d;
  double e;
  double fb;
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
    fb = maximum(x);
    x[0] = fb;
    x[1] = 1.2;
    t_lc = rt_roundd_snf(c_minimum(x) / 0.1) * 0.1;

    /*  S_a_end=s_a+v_a*t_lc+0.5*(index_accel*a_min_comfort)*t_lc*t_lc; */
    x[0] = 0.0;
    x[1] = CurrentLaneFrontVel + d_CalibrationVars_TrajPlanLaneC *
      e_CalibrationVars_TrajPlanLaneC * t_lc;
    V_a_end = maximum(x);
    x[0] = 0.5 * (speed + V_a_end);
    x[1] = speed;
    fb = c_minimum(x);
    x[0] = fb;
    x[1] = speed + t_lc * e_CalibrationVars_TrajPlanLaneC;
    V_end = maximum(x);
    L_end = pos_l_CurrentLane - pos_l;

    /*  L_end=-L_end; */
    /*      S_end=min([S_a_end-t_re*V_end-l_veh S_a_end-V_end*t_re-(V_a_end.^2-V_end.^2)/(2*a_min)-l_veh sqrt((0.5*(V_0+V_end)*t_lc).^2-L_end.^2)]); */
    a = 0.5 * (speed + V_end) * t_lc;
    S_end = (CurrentLaneFrontDis + 0.5 * (V_a_end + CurrentLaneFrontVel) * t_lc)
      - c_CalibrationVars_TrajPlanLaneC * V_end;
    S_a_end[0] = S_end;
    S_a_end[1] = S_end - (V_a_end * V_a_end - V_end * V_end) / (2.0 *
      f_CalibrationVars_TrajPlanLaneC);
    S_a_end[2] = sqrt(a * a - L_end * L_end);
    S_end = d_minimum(S_a_end);

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
    fb = S_end * S_end;
    dv[15] = fb;
    L_end = rt_powd_snf(S_end, 3.0);
    dv[21] = L_end;
    b = rt_powd_snf(S_end, 4.0);
    dv[27] = b;
    dv[33] = rt_powd_snf(S_end, 5.0);
    dv[4] = 0.0;
    dv[10] = 1.0;
    dv[16] = 2.0 * S_end;
    dv[22] = 3.0 * fb;
    dv[28] = 4.0 * L_end;
    dv[34] = 5.0 * b;
    dv[5] = 0.0;
    dv[11] = 0.0;
    dv[17] = 2.0;
    dv[23] = 6.0 * S_end;
    dv[29] = 12.0 * fb;
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
      fb = x1[ix];
      fb = (((fun_S_tunableEnvironment[0].f1[1] + a * fb) + V_a_end * (fb * fb))
            + L_end * rt_powd_snf(fb, 3.0)) + S_end * rt_powd_snf(fb, 4.0);
      y[ix] = sqrt(fb * fb + 1.0);
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
        S_end += y[ia - 1] * c[ix];
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
    fb = t_lc * t_lc;
    dv1[10] = fb;
    dv1[14] = rt_powd_snf(t_lc, 3.0);
    dv1[3] = 0.0;
    dv1[7] = 1.0;
    dv1[11] = 2.0 * t_lc;
    dv1[15] = 3.0 * fb;
    c_mldivide(dv1, para_ST);
    ia = (int)(t_lc / 0.05 + 1.0);
    if (0 <= ia - 1) {
      c_this_tunableEnvironment_f1_tu[0].tunableEnvironment[0] =
        fun_S_tunableEnvironment[0];
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
      b = x[1];
      V_a_end = anon(c_this_tunableEnvironment_f1_tu, S_traj, (double)ix + 1.0,
                     -1.0);
      fb = anon(c_this_tunableEnvironment_f1_tu, S_traj, (double)ix + 1.0, x[1]);
      if (V_a_end == 0.0) {
        b = -1.0;
      } else {
        if (!(fb == 0.0)) {
          V_end = fb;
          b_c = x[1];
          e = 0.0;
          d = 0.0;
          exitg1 = false;
          while ((!exitg1) && ((fb != 0.0) && (a != b))) {
            if ((fb > 0.0) == (V_end > 0.0)) {
              b_c = a;
              V_end = V_a_end;
              d = b - a;
              e = d;
            }

            if (fabs(V_end) < fabs(fb)) {
              a = b;
              b = b_c;
              b_c = a;
              V_a_end = fb;
              fb = V_end;
              V_end = V_a_end;
            }

            m = 0.5 * (b_c - b);
            toler = 4.4408920985006262E-16 * fmax(fabs(b), 1.0);
            if ((fabs(m) <= toler) || (fb == 0.0)) {
              exitg1 = true;
            } else {
              if ((fabs(e) < toler) || (fabs(V_a_end) <= fabs(fb))) {
                d = m;
                e = m;
              } else {
                s = fb / V_a_end;
                if (a == b_c) {
                  V_a_end = 2.0 * m * s;
                  L_end = 1.0 - s;
                } else {
                  L_end = V_a_end / V_end;
                  S_end = fb / V_end;
                  V_a_end = s * (2.0 * m * L_end * (L_end - S_end) - (b - a) *
                                 (S_end - 1.0));
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

              a = b;
              V_a_end = fb;
              if (fabs(d) > toler) {
                b += d;
              } else if (b > b_c) {
                b -= toler;
              } else {
                b += toler;
              }

              fb = anon(c_this_tunableEnvironment_f1_tu, S_traj, (double)ix +
                        1.0, b);
            }
          }
        }
      }

      X_traj[ix] = b;
    }

    /*      plot(S_traj); */
    fb = rt_roundd_snf(t_lc / 0.05);
    ia = (int)fb;
    for (ix = 0; ix < ia; ix++) {
      L_end = X_traj[ix + 1];
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix] = pos_s +
        L_end;
      b = L_end * L_end;
      S_end = rt_powd_snf(L_end, 3.0);
      V_a_end = rt_powd_snf(L_end, 4.0);
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 120] =
        pos_l + (((((fun_S_tunableEnvironment[0].f1[0] +
                     fun_S_tunableEnvironment[0].f1[1] * L_end) +
                    fun_S_tunableEnvironment[0].f1[2] * b) +
                   fun_S_tunableEnvironment[0].f1[3] * S_end) +
                  fun_S_tunableEnvironment[0].f1[4] * V_a_end) +
                 fun_S_tunableEnvironment[0].f1[5] * rt_powd_snf(L_end, 5.0));
      L_end = 57.295779513082323 * atan((((fun_S_tunableEnvironment[0].f1[1] +
        2.0 * fun_S_tunableEnvironment[0].f1[2] * L_end) + 3.0 *
        fun_S_tunableEnvironment[0].f1[3] * b) + 4.0 * fun_S_tunableEnvironment
        [0].f1[4] * S_end) + 5.0 * fun_S_tunableEnvironment[0].f1[5] * V_a_end);
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 240] = 90.0
        - L_end;
      b = L_end;
      b_cosd(&b);
      S_end = V_traj[ix + 1];
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[ix + 360] =
        S_end * b;
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

    if (GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[(int)fb + 359]
        == 0.0) {
      GlobVars->TrajPlanLaneChange_RePlan.LaneChangePath_RePlan[(int)
        rt_roundd_snf(t_lc / 0.05) + 359] = V_traj[(int)fb];
    }

    /*  plot(LaneChangePath_RePlan(1:round(t_lc/0.05),1),LaneChangePath_RePlan(1:round(t_lc/0.05),2)); */
    DurationLaneChange_RePlan = 1;
  }

  if ((DurationLaneChange_RePlan > 0) && (DurationLaneChange_RePlan <= 10.0 *
       t_lc)) {
    fb = rt_roundd_snf(t_lc / 0.1);
    L_end = (fb + 1.0) - (double)DurationLaneChange_RePlan;
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
        [(int)(2.0 * fb) - 1] + traj[6 * (i - 1) + 3] * 0.05 * (double)ia;
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
 *                const double LengthOppositeCar[20]
 *                const short IndexOfLaneCodirectCar[10]
 *                const double SpeedCodirectCar[10]
 *                const double PosSCodirectCar[10]
 *                const double LengthCodirectCar[20]
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
  const double PosSOppositeCar[20], const double LengthOppositeCar[20], const
  short IndexOfLaneCodirectCar[10], const double SpeedCodirectCar[10], const
  double PosSCodirectCar[10], const double LengthCodirectCar[20], short
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
  double LengthOppositeCarRear[6];
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
  double targetSpeed;
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
    LengthOppositeCarRear[i] = 5.0;
  }

  /*  目标车道选择 ----(判断掉头一次二次，选择掉头目标车道，只计算一次) */
  pos_l_TargetLane = 0.0;

  /* 20220324因为只用一次，所以可以设初值 */
  WidthOfLaneCurrent_tmp = WidthOfLanes[CurrentLane - 1];
  if (GlobVars->TrajPlanTurnAround.TypeOfTurnAround == 0) {
    /*  d_cur2tar=0.5*(WidthOfLanesOpposite(1)-w_veh)+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(1); */
    TargetLaneIndexOpposite = 1;
    TypeOfTurnAround = 2;

    /*  1为一次顺车掉头，2为二次顺车掉头 */
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

    /*  一次顺车掉头路径生成 */
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

      /*  车道中心线位置 全局变量 */
    }

    /*  二次顺车掉头路径生成 */
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
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1(2)); % 例如[2 1] 全局变量 对向车道序号为正，最左侧为1 */
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]);
      GlobVars->TrajPlanTurnAround.pos_mid1[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, WidthOfLaneCurrent_tmp, WidthOfGap,
        WidthOfLanesOpposite, NumOfLanesOpposite,
        GlobVars->TrajPlanTurnAround.pos_mid1[1]);

      /*  OccupiedLanesPosMid2=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2_rear(2)):1:..., */
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2(2)); % 例如[1 -1] 全局变量 掉头前道路车道序号为负，最左侧为-1 */
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

      /*  车道中心线位置 全局变量 */
    }
  }

  /* 筛选二次/一次顺车掉头前车后车到路径圆心距离 */
  for (i = 0; i < 6; i++) {
    posOfLaneCenterline[i] = 0.0;
  }

  /* 20220324 */
  if (TypeOfTurnAround == 1) {
    /*      posOfLaneCenterline=zeros([6,1]);%车道中心线在掉头路径上交点s坐标 */
    for (i = 0; i < 7; i++) {
      if ((GlobVars->TrajPlanTurnAround.LaneCenterline[i] != 0.0) && (i + 1 <=
           TargetLaneIndexOpposite)) {
        if (GlobVars->TrajPlanTurnAround.LaneCenterline[i] <
            GlobVars->TrajPlanTurnAround.PosCircle[1]) {
          targetSpeed = GlobVars->TrajPlanTurnAround.PosCircle[1] -
            GlobVars->TrajPlanTurnAround.LaneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            targetSpeed * targetSpeed) + GlobVars->TrajPlanTurnAround.PosCircle
            [0];
        } else if ((GlobVars->TrajPlanTurnAround.LaneCenterline[i] >=
                    GlobVars->TrajPlanTurnAround.PosCircle[1]) &&
                   (GlobVars->TrajPlanTurnAround.LaneCenterline[i] <=
                    GlobVars->TrajPlanTurnAround.PosCircle2[1])) {
          posOfLaneCenterline[i] = TurningRadius +
            GlobVars->TrajPlanTurnAround.PosCircle[0];
        } else {
          if (GlobVars->TrajPlanTurnAround.LaneCenterline[i] >
              GlobVars->TrajPlanTurnAround.PosCircle2[1]) {
            targetSpeed = GlobVars->TrajPlanTurnAround.PosCircle2[1] -
              GlobVars->TrajPlanTurnAround.LaneCenterline[i];
            posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
              targetSpeed * targetSpeed) +
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
          targetSpeed = GlobVars->TrajPlanTurnAround.PosCircle[1] -
            GlobVars->TrajPlanTurnAround.LaneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            targetSpeed * targetSpeed) + GlobVars->TrajPlanTurnAround.PosCircle
            [0];
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
          LengthOppositeCarRear[i1 - 1] = LengthOppositeCar[i];
        }
      }
    }
  }

  /*  一次顺车掉头决策 */
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
      b = (0.5 * WidthOfLaneCurrent_tmp + WidthOfGap) + d;
      if (b + sum(posOfLaneCenterline, WidthOfLanesOpposite_size) <
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
          TurningRadius * acos((TurningRadius - (b + sum
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

        if (b + sum(WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size) >
            (TurningRadius + GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
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
            (((TurningRadius * asin(((((b + sum(WidthOfLanesOpposite_data,
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

      /*          d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确 */
    }

    /*  策略模式判断 */
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

    /*  停车决策 */
    if (dec_trunAround == 1) {
      /*          for i=1:TargetLaneIndexOpposite */
      /*              d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确 */
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
        i1 = f_minimum(iv);
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j <= i1 - 1)) {
          /*                  if PosSOppositeCarRear(j)>-l_veh */
          if (PosSOppositeCarRear[j] > -LengthOppositeCarRear[j]) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        }
      }
    }

    /*  起步决策 */
    if ((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] -
         pos_s < 10.0)) {
      wait_turnAround = 0;
      a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                      CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                      CalibrationVars->ACC.d_wait);

      /* 自车预估加速度 */
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
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
          b_speed[1] = 0.0;
          passedPerimeter = maximum(b_speed);
          if (SpeedOppositeCarFront[j] <= 0.01) {
            passedPerimeter = 0.0;
          }

          /*  [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-(max(SpeedOppositeCarFront(j)+a_OppositeCarFront*t-v_max,0))^2/(2*a_OppositeCarFront+eps)..., */
          /*      -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
          /*      [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
          /* , */
          /* , */
          b_speed[0] = 0.0;
          b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
          dv[0] = 1.0E-5;
          dv[1] = fmin(SpeedOppositeCarFront[j], v_max);
          d = maximum(b_speed);
          b_speed[0] = -0.01;
          b_speed[1] = d / maximum(dv) + 0.01;
          c_fzero(SpeedOppositeCarFront, (double)j + 1.0, fmin(passedPerimeter,
                   1.5), v_max, PosSOppositeCarFront, w_veh, D_safe2, b_speed,
                  &pos_l_TargetLane, &s_max, &d_cur2pot_tar);

          /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
          b_speed[0] = a_predict;
          b_speed[1] = a_max_com;
          a_predict = c_minimum(b_speed);
          if (a_predict > 0.0) {
            s_max = speed + a_predict * pos_l_TargetLane;
            targetSpeed = fmax(s_max - v_max_turnAround, 0.0);
            s_max = 0.5 * (s_max + speed) * pos_l_TargetLane - targetSpeed *
              targetSpeed / (2.0 * a_predict + 2.2204460492503131E-16);
          } else {
            dv1[0] = 0.0;
            dv1[1] = speed + a_predict * pos_l_TargetLane;
            dv1[2] = v_max_turnAround;
            s_max = 0.5 * (median(dv1) + speed) * pos_l_TargetLane;
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
        i1 = f_minimum(iv);
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j <= i1 - 1)) {
          /*                  if PosSOppositeCarRear(j)>-l_veh */
          if (PosSOppositeCarRear[j] > -LengthOppositeCarRear[j]) {
            wait_turnAround = 1;
            exitg1 = true;
          } else {
            j++;
          }
        }
      }
    }
  }

  /*  二次顺车掉头决策 */
  if (TypeOfTurnAround == 2) {
    /*  二次顺车掉头状态判断 */
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
      targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
      s_max = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
      emxInit_real_T(&OccupiedLanesPosMid1, 2);
      emxInit_real_T(&OccupiedLanes, 2);
      emxInit_int32_T(&b_ia, 1);
      if ((sqrt(targetSpeed * targetSpeed + s_max * s_max) < 0.15) && (speed <=
           0.05)) {
        *TargetGear = 2;
        if (CurrentGear == 2) {
          /*  环境车允许倒车 % 倒车前决策 */
          /*  当前位置mid1车头车尾所在车道 → 确定已占据车道 */
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

          /*  例如[1 -1] 掉头前道路车道序号为负，最左侧为-1 */
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

          /*  例如[2 1] 对向车道序号为正，最左侧为1 */
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

          /*  目标位置mid2车尾所在车道 → 确定将侵入的车道 */
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

          /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道） */
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

          /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将倒车路径简化为pos_mid1_rear到pos_mid2_rear的线段→d_veh2cross,timegap” */
          TurnAroundState = 2;
          a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                          CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                          CalibrationVars->ACC.tau_v_com,
                          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                          CalibrationVars->ACC.tau_v_bre,
                          CalibrationVars->ACC.tau_v_emg,
                          CalibrationVars->ACC.tau_d_emg,
                          CalibrationVars->ACC.t_acc,
                          CalibrationVars->ACC.d_wait);
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
                pos_l_TargetLane = maximum(b_speed);
                targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                  (d_cur2pot_tar - b) / k;
                s_max = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                  d_cur2pot_tar;
                b_speed[0] = a_predict;
                b_speed[1] = a_max_com;
                d = c_minimum(b_speed);
                b_speed[0] = speed + d * pos_l_TargetLane;
                b_speed[1] = v_max_turnAround;
                if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                    sqrt(targetSpeed * targetSpeed + s_max * s_max) + l_veh) {
                  TurnAroundState = 1;
                  exitg1 = true;
                } else {
                  i++;
                }
              } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                          -LengthOppositeCar[i])) {
                /*                              if disOppositeCar2circle2>-l_veh */
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
            /*  SpeedCodirectCar默认值为-1, LaneCenterline最后一列为掉头前所在车道 */
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
                  pos_l_TargetLane = maximum(b_speed);
                  if (IndexOfLaneCodirectCar[j] == -1) {
                    targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]
                      - (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k;
                    s_max = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                      GlobVars->TrajPlanTurnAround.LaneCenterline[6];
                    s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max);
                  } else {
                    targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]
                      - ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2)
                         - b) / k;
                    s_max = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                             GlobVars->TrajPlanTurnAround.LaneCenterline[6]) +
                      3.2;
                    s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max);
                  }

                  b_speed[0] = a_predict;
                  b_speed[1] = a_max_com;
                  d = c_minimum(b_speed);
                  b_speed[0] = speed + d * pos_l_TargetLane;
                  b_speed[1] = v_max_turnAround;
                  if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                      s_max + l_veh) {
                    TurnAroundState = 1;
                    exitg1 = true;
                  } else {
                    j++;
                  }
                } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                            -LengthCodirectCar[j])) {
                  /*                                  if disOppositeCar2circle2>-l_veh */
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
      targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid2[0] - pos_s;
      s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
      s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max);
      if ((s_max < 0.15) && (speed <= 0.05)) {
        *TargetGear = 4;
      }

      emxInit_int16_T(&b_OccupiedLanes, 2);
      emxInit_int16_T(&Lanes2Search, 2);
      emxInit_int32_T(&c_ia, 1);
      if ((s_max < 0.15) && (CurrentGear == 4) && (speed <= 0.05)) {
        /*  环境车允许前进 % 前进前决策 */
        /*  当前位置mid2车头车尾所在车道 → 确定已占据车道 */
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

        /*  例如[1 -1] 掉头前道路车道序号为负，最左侧为-1 */
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

        /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道） */
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

        /*  （非已占据车道的目标车道+已占据车道到目标车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将前进路径简化为pos_mid2到pos_end的线段→d_veh2cross,timegap” */
        TurnAroundState = 3;
        a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                        CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                        CalibrationVars->ACC.tau_v_com,
                        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                        CalibrationVars->ACC.tau_v_bre,
                        CalibrationVars->ACC.tau_v_emg,
                        CalibrationVars->ACC.tau_d_emg,
                        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
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
              pos_l_TargetLane = maximum(b_speed);
              targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid2[0] - s_max / k;
              s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - d_cur2pot_tar;
              b_speed[0] = a_predict;
              b_speed[1] = a_max_com;
              d = c_minimum(b_speed);
              b_speed[0] = speed + d * pos_l_TargetLane;
              b_speed[1] = v_max_turnAround;
              if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <= sqrt
                  (targetSpeed * targetSpeed + s_max * s_max) + l_veh) {
                TurnAroundState = 2;
                exitg1 = true;
              } else {
                i++;
              }
            } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                        -LengthOppositeCar[i])) {
              /*                          if disOppositeCar2circle2>-l_veh */
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

    /*  进入掉头路径前决策 */
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

      /*  策略模式判断 */
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

      /*  停车决策 */
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
          i1 = f_minimum(iv);
          j = 0;
          exitg1 = false;
          while ((!exitg1) && (j <= i1 - 1)) {
            /*                      if PosSOppositeCarRear(j)>-l_veh */
            if (PosSOppositeCarRear[j] > -LengthOppositeCarRear[j]) {
              wait_turnAround = 1;
              exitg1 = true;
            } else {
              j++;
            }
          }
        }
      }

      /*  起步决策 */
      if ((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.pos_start[0] -
           pos_s < 10.0)) {
        wait_turnAround = 0;

        /*  a_predict=ACC(v_max_turnAround,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0); */
        b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
        b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           0.0, fabs(rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
           GlobVars->TrajPlanTurnAround.PosCircle[1],
           GlobVars->TrajPlanTurnAround.pos_mid1[0] -
           GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf(pos_l -
          GlobVars->TrajPlanTurnAround.PosCircle[1], pos_s -
          GlobVars->TrajPlanTurnAround.PosCircle[0])) * Parameters.TurningRadius
                           + CalibrationVars->ACC.d_wait, speed, 0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
        a_predict = c_minimum(b_speed);
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
                             CalibrationVars->ACC.tau_v_com,
                             CalibrationVars->ACC.tau_v,
                             CalibrationVars->ACC.tau_d,
                             CalibrationVars->ACC.tau_v_bre,
                             CalibrationVars->ACC.tau_v_emg,
                             CalibrationVars->ACC.tau_d_emg,
                             CalibrationVars->ACC.t_acc,
                             CalibrationVars->ACC.d_wait);
            b_speed[1] = 0.0;
            passedPerimeter = maximum(b_speed);
            if (SpeedOppositeCarFront[j] <= 0.01) {
              passedPerimeter = 0.0;
            }

            /*                      [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
            /*                          [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
            /*                      % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)/max([SpeedOppositeCarFront(j) 0.00001])]); */
            /*                      s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            /* , */
            /* , */
            b_speed[0] = 0.0;
            b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
            dv[0] = 1.0E-5;
            dv[1] = fmin(SpeedOppositeCarFront[j], v_max);
            d = maximum(b_speed);
            b_speed[0] = -0.01;
            b_speed[1] = d / maximum(dv) + 0.01;
            c_fzero(SpeedOppositeCarFront, (double)j + 1.0, fmin(passedPerimeter,
                     1.5), v_max, PosSOppositeCarFront, w_veh, D_safe2, b_speed,
                    &pos_l_TargetLane, &s_max, &d_cur2pot_tar);

            /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            b_speed[0] = a_predict;
            b_speed[1] = a_max_com;
            a_predict = c_minimum(b_speed);
            if (a_predict > 0.0) {
              s_max = speed + a_predict * pos_l_TargetLane;
              targetSpeed = fmax(s_max - v_max_turnAround, 0.0);
              s_max = 0.5 * (s_max + speed) * pos_l_TargetLane - targetSpeed *
                targetSpeed / (2.0 * a_predict + 2.2204460492503131E-16);
            } else {
              dv1[0] = 0.0;
              dv1[1] = speed + a_predict * pos_l_TargetLane;
              dv1[2] = v_max_turnAround;
              s_max = 0.5 * (median(dv1) + speed) * pos_l_TargetLane;
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
          i1 = f_minimum(iv);
          j = 0;
          exitg1 = false;
          while ((!exitg1) && (j <= i1 - 1)) {
            /*                      if PosSOppositeCarRear(j)>-l_veh */
            if (PosSOppositeCarRear[j] > -LengthOppositeCarRear[j]) {
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

  /*  ACC速度规划 */
  *a_soll_TrajPlanTurnAround = 0.0;

  /* 20220322 */
  if (TypeOfTurnAround == 1) {
    /*  一次顺车掉头速度规划 */
    if (((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] >
          pos_s)) || (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
      /*  a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]); */
      b_speed[0] = 0.0;
      b_speed[1] = (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s) +
        CalibrationVars->ACC.d_wait;
      d = maximum(b_speed);
      b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                         wait_turnAround, CalibrationVars->ACC.a_max,
                         CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.tau_v_com,
                         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                         CalibrationVars->ACC.tau_v_bre,
                         CalibrationVars->ACC.tau_v_emg,
                         CalibrationVars->ACC.tau_d_emg,
                         CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
      b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         0.0, d, speed, wait_turnAround,
                         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.tau_v_com,
                         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                         CalibrationVars->ACC.tau_v_bre,
                         CalibrationVars->ACC.tau_v_emg,
                         CalibrationVars->ACC.tau_d_emg,
                         CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
      *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
    } else if (dec_trunAround == 1) {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
    } else if (GlobVars->TrajPlanTurnAround.PosCircle[0] - pos_s > 2.0 *
               Parameters.l_veh) {
      *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
        CurrentLaneFrontDis, speed, wait_turnAround, CalibrationVars->ACC.a_max,
        CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
        CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
        CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
        CalibrationVars->ACC.d_wait);
    } else {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
         CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
         CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
         CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
    }

    b_speed[0] = *a_soll_TrajPlanTurnAround;
    b_speed[1] = a_soll;
    *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
  } else {
    if (TypeOfTurnAround == 2) {
      /*  二次顺车掉头速度规划 */
      /*      if TurnAroundState==0 */
      if ((TurnAroundState == 0) || ((TurnAroundState == 1) && (pos_s <=
            GlobVars->TrajPlanTurnAround.pos_start[0]))) {
        if ((wait_turnAround == 1) ||
            (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
          b_speed[0] = 0.0;
          b_speed[1] = (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s) +
            CalibrationVars->ACC.d_wait;
          d = maximum(b_speed);
          b_speed[0] = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
             CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
             CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
             CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
          b_speed[1] = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0, d, speed,
             wait_turnAround, CalibrationVars->ACC.a_max,
             CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
             CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
             CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
             CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
             CalibrationVars->ACC.d_wait);
          *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
        } else if (dec_trunAround == 1) {
          *a_soll_TrajPlanTurnAround = fmin(ACC(v_max, 0.0, (((rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_mid1[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_start[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_start[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0])) *
            Parameters.TurningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
            - pos_s) + CalibrationVars->ACC.d_wait, speed, 0.0,
            CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
            CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
            CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
            CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
            CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait), b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
             CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
             CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
             CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait));
        } else if (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s > 2.0 *
                   Parameters.l_veh) {
          *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
            CurrentLaneFrontDis, speed, wait_turnAround,
            CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
            CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
            CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
            CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
            CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
        } else {
          *a_soll_TrajPlanTurnAround = fmin(ACC(v_max, 0.0, (((rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_mid1[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0]) - rt_atan2d_snf
            (GlobVars->TrajPlanTurnAround.pos_start[1] -
             GlobVars->TrajPlanTurnAround.PosCircle[1],
             GlobVars->TrajPlanTurnAround.pos_start[0] -
             GlobVars->TrajPlanTurnAround.PosCircle[0])) *
            Parameters.TurningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
            - pos_s) + CalibrationVars->ACC.d_wait, speed, 0.0,
            CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
            CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
            CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
            CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
            CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait), b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
             CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
             CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
             CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait));
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
           CalibrationVars->ACClowSpeed.t_acc,
           CalibrationVars->ACClowSpeed.d_wait);
        b_speed[1] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0,
           (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid1[1] -
                          GlobVars->TrajPlanTurnAround.PosCircle[1],
                          GlobVars->TrajPlanTurnAround.pos_mid1[0] -
                          GlobVars->TrajPlanTurnAround.PosCircle[0]) -
            rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.PosCircle[1],
                          pos_s - GlobVars->TrajPlanTurnAround.PosCircle[0])) *
           Parameters.TurningRadius + CalibrationVars->ACClowSpeed.d_wait, speed,
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
           CalibrationVars->ACClowSpeed.t_acc,
           CalibrationVars->ACClowSpeed.d_wait);
        *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
        targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
        s_max = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
        if (sqrt(targetSpeed * targetSpeed + s_max * s_max) < 0.15) {
          passedPerimeter = speed;
          if (speed < 0.0) {
            passedPerimeter = -1.0;
          } else if (speed > 0.0) {
            passedPerimeter = 1.0;
          } else {
            if (speed == 0.0) {
              passedPerimeter = 0.0;
            }
          }

          *a_soll_TrajPlanTurnAround = -4.0 * passedPerimeter;
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
           CalibrationVars->ACClowSpeed.t_acc,
           CalibrationVars->ACClowSpeed.d_wait);
        b_speed[1] = ACClowSpeed
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0, fabs
           (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                          GlobVars->TrajPlanTurnAround.PosCircle2[1],
                          GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                          GlobVars->TrajPlanTurnAround.PosCircle2[0]) -
            rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.PosCircle2[1],
                          pos_s - GlobVars->TrajPlanTurnAround.PosCircle2[0])) *
           Parameters.TurningRadius + CalibrationVars->ACClowSpeed.d_wait, speed,
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
           CalibrationVars->ACClowSpeed.t_acc,
           CalibrationVars->ACClowSpeed.d_wait);
        *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
        targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid2[0] - pos_s;
        s_max = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
        if (sqrt(targetSpeed * targetSpeed + s_max * s_max) < 0.15) {
          passedPerimeter = speed;
          if (speed < 0.0) {
            passedPerimeter = -1.0;
          } else if (speed > 0.0) {
            passedPerimeter = 1.0;
          } else {
            if (speed == 0.0) {
              passedPerimeter = 0.0;
            }
          }

          *a_soll_TrajPlanTurnAround = -4.0 * passedPerimeter;
        }
      } else {
        if (TurnAroundState == 3) {
          *a_soll_TrajPlanTurnAround = b_ACC
            (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
             CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
             CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
             CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
             CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
             CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
             CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
        }
      }

      b_speed[0] = *a_soll_TrajPlanTurnAround;
      b_speed[1] = a_soll;
      *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
    }
  }

  /*  AEB决策 */
  if (*AEBactive == 0) {
    if ((TypeOfTurnAround == 1) && (GlobVars->TrajPlanTurnAround.PosCircle[0] -
         pos_s <= 0.0)) {
      j = 0;
      exitg4 = false;
      while ((!exitg4) && (j < 20)) {
        /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离) */
        if (IndexOfLaneOppositeCar[j] != 0) {
          d = GlobVars->
            TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j] - 1];
          b = d - (GlobVars->TrajPlanTurnAround.PosCircle[1] - TurningRadius);
          if (b < TurningRadius) {
            s_max = GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius *
              acos((TurningRadius - b) / TurningRadius);
          } else if (b > (TurningRadius +
                          GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                     GlobVars->TrajPlanTurnAround.PosCircle[1]) {
            /* , */
            s_max = GlobVars->TrajPlanTurnAround.PosCircle[0] + (((TurningRadius
              * asin((((b - TurningRadius) -
                       GlobVars->TrajPlanTurnAround.PosCircle2[1]) +
                      GlobVars->TrajPlanTurnAround.PosCircle[1]) / TurningRadius)
              + GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
              GlobVars->TrajPlanTurnAround.PosCircle[1]) + TurningRadius *
              3.1415926535897931 / 2.0);
          } else {
            s_max = ((GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius *
                      3.1415926535897931 / 2.0) + b) - TurningRadius;
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

          /*  用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2 */
          if (d < GlobVars->TrajPlanTurnAround.PosCircle[1]) {
            targetSpeed = d - GlobVars->TrajPlanTurnAround.PosCircle[1];
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle[0] + sqrt(fabs
                (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
          } else if ((d >= GlobVars->TrajPlanTurnAround.PosCircle[1]) && (d <=
                      GlobVars->TrajPlanTurnAround.PosCircle2[1])) {
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle[0] + TurningRadius);
          } else {
            targetSpeed = d - GlobVars->TrajPlanTurnAround.PosCircle2[1];
            pos_l_TargetLane = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.PosCircle2[0] + sqrt(fabs
                (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
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
              pos_l_TargetLane = maximum(b_speed);
              b_speed[0] = speed + a_max_com * pos_l_TargetLane;
              b_speed[1] = v_max_turnAround;
              if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <= s_max
                  + l_veh) {
                *AEBactive = 5;
                exitg4 = true;
              } else {
                j++;
              }
            } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                        -LengthOppositeCar[j])) {
              /*                          if disOppositeCar2circle2>-l_veh */
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
                /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离) */
                if (pos_s < GlobVars->TrajPlanTurnAround.PosCircle[0]) {
                  b = GlobVars->
                    TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j]
                    - 1] - (GlobVars->TrajPlanTurnAround.PosCircle[1] -
                            TurningRadius);
                  if (b < TurningRadius) {
                    s_max = d + TurningRadius * acos((TurningRadius - b) /
                      TurningRadius);
                  } else {
                    s_max = d + (TurningRadius * asin((b - TurningRadius) /
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

                /*  用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2 */
                targetSpeed = GlobVars->
                  TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[j] -
                  1] - GlobVars->TrajPlanTurnAround.PosCircle[1];
                pos_l_TargetLane = PosSOppositeCar[j] -
                  (GlobVars->TrajPlanTurnAround.PosCircle[0] + sqrt(fabs
                    (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
                if ((IndexOfLaneOppositeCar[j] <= TargetLaneIndexOpposite) &&
                    (IndexOfLaneOppositeCar[j] > 0) && (s_max > 0.0)) {
                  if (pos_l_TargetLane > 0.0) {
                    b_speed[0] = SpeedOppositeCarFront[j];
                    b_speed[1] = 1.0E-5;
                    b = maximum(b_speed);
                    b_speed[0] = 0.0;
                    b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) / b;
                    pos_l_TargetLane = maximum(b_speed);
                    b_speed[0] = speed + a_max_com * pos_l_TargetLane;
                    b_speed[1] = v_max_turnAround;
                    if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                        s_max + l_veh * (double)(IndexOfLaneOppositeCar[j] !=
                         TargetLaneIndexOpposite)) {
                      *AEBactive = 5;
                      exitg3 = true;
                    } else {
                      j++;
                    }
                  } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                              -LengthOppositeCar[j])) {
                    /*                              if disOppositeCar2circle2>-l_veh */
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
                /*  用pos_s、Pos_circle2、pos_l求pos_s_rear、pos_l_rear */
                d_cur2pot_tar = atan((pos_l -
                                      GlobVars->TrajPlanTurnAround.PosCircle2[1])
                                     / (pos_s -
                  GlobVars->TrajPlanTurnAround.PosCircle2[0]));

                /*  用IndexOfLaneOppositeCar(i)、pos_s_rear、pos_l_rear、LaneCenterline、k、b、求d_veh2cross_strich(注意是车尾到车道中心线的距离) */
                s_max = GlobVars->
                  TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i] -
                  1];
                pos_l_TargetLane = (s_max - b) / k;
                targetSpeed = (pos_s + sin(d_cur2pot_tar) * l_veh) -
                  pos_l_TargetLane;
                s_max = (pos_l - cos(d_cur2pot_tar) * l_veh) - s_max;
                passedPerimeter = s_max;
                if (s_max < 0.0) {
                  passedPerimeter = -1.0;
                } else if (s_max > 0.0) {
                  passedPerimeter = 1.0;
                } else {
                  if (s_max == 0.0) {
                    passedPerimeter = 0.0;
                  }
                }

                s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max) *
                  passedPerimeter;
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
                      pos_l_TargetLane = maximum(b_speed);
                      b_speed[0] = speed + a_max_com * pos_l_TargetLane;
                      b_speed[1] = v_max_turnAround;
                      if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                          s_max + l_veh * (double)(IndexOfLaneOppositeCar[i] !=
                           TargetLaneIndexOpposite)) {
                        *AEBactive = 5;
                        exitg5 = true;
                      } else {
                        i++;
                      }
                    } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                                -LengthOppositeCar[i])) {
                      /*                              if disOppositeCar2circle2>-l_veh */
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
              /*  SpeedCodirectCar默认值为-1, LaneCenterline最后一列为掉头前所在车道 */
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
                /*  用pos_s、Pos_circle2、pos_l求pos_s_rear、pos_l_rear */
                d_cur2pot_tar = atan((pos_l -
                                      GlobVars->TrajPlanTurnAround.PosCircle2[1])
                                     / (pos_s -
                  GlobVars->TrajPlanTurnAround.PosCircle2[0]));
                s_max = pos_s + sin(d_cur2pot_tar) * l_veh;
                d_cur2pot_tar = pos_l - cos(d_cur2pot_tar) * l_veh;

                /*  IndexOfLaneCodirectCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(默认同向第二条车道的宽度为3.2)(注意是车尾到车道中心线的距离) */
                if (IndexOfLaneCodirectCar[j] == -1) {
                  targetSpeed = s_max -
                    (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - b) / k;
                  s_max = d_cur2pot_tar -
                    GlobVars->TrajPlanTurnAround.LaneCenterline[6];
                  passedPerimeter = s_max;
                  if (s_max < 0.0) {
                    passedPerimeter = -1.0;
                  } else if (s_max > 0.0) {
                    passedPerimeter = 1.0;
                  } else {
                    if (s_max == 0.0) {
                      passedPerimeter = 0.0;
                    }
                  }

                  s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max) *
                    passedPerimeter;
                } else {
                  targetSpeed = s_max -
                    ((GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2) - b)
                    / k;
                  s_max = d_cur2pot_tar -
                    (GlobVars->TrajPlanTurnAround.LaneCenterline[6] - 3.2);
                  passedPerimeter = s_max;
                  if (s_max < 0.0) {
                    passedPerimeter = -1.0;
                  } else if (s_max > 0.0) {
                    passedPerimeter = 1.0;
                  } else {
                    if (s_max == 0.0) {
                      passedPerimeter = 0.0;
                    }
                  }

                  s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max) *
                    passedPerimeter;
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
                    pos_l_TargetLane = maximum(b_speed);
                    b_speed[0] = speed + a_max_com * pos_l_TargetLane;
                    b_speed[1] = v_max_turnAround;
                    if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                        s_max + l_veh * (double)(IndexOfLaneCodirectCar[j] !=
                         GlobVars->TrajPlanTurnAround.pos_mid2_rear[3])) {
                      *AEBactive = 5;
                      exitg2 = true;
                    } else {
                      j++;
                    }
                  } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                              -LengthCodirectCar[j])) {
                    /*                              if disOppositeCar2circle2>-l_veh */
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
                  /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(注意是车头到车道中心线的距离) */
                  s_max = GlobVars->
                    TrajPlanTurnAround.LaneCenterline[IndexOfLaneOppositeCar[i]
                    - 1];
                  pos_l_TargetLane = (s_max - b) / k;
                  targetSpeed = pos_l_TargetLane - pos_s;
                  s_max -= pos_l;
                  passedPerimeter = s_max;
                  if (s_max < 0.0) {
                    passedPerimeter = -1.0;
                  } else if (s_max > 0.0) {
                    passedPerimeter = 1.0;
                  } else {
                    if (s_max == 0.0) {
                      passedPerimeter = 0.0;
                    }
                  }

                  s_max = sqrt(targetSpeed * targetSpeed + s_max * s_max) *
                    passedPerimeter;
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
                      pos_l_TargetLane = maximum(b_speed);

                      /* 20220214 */
                      b_speed[0] = speed + a_max_com * pos_l_TargetLane;
                      b_speed[1] = v_max_turnAround;
                      if (0.5 * (c_minimum(b_speed) + speed) * pos_l_TargetLane <=
                          s_max + l_veh) {
                        *AEBactive = 5;
                        exitg1 = true;
                      } else {
                        i++;
                      }
                    } else if ((pos_l_TargetLane <= 0.0) && (pos_l_TargetLane >
                                -LengthOppositeCar[i])) {
                      /*                              if disOppositeCar2circle2>-l_veh */
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

  /*  输出a_soll_TrajPlanTurnAround（-4或者不变）和AEBactive=5 */
  if (*AEBactive == 5) {
    passedPerimeter = speed;
    if (speed < 0.0) {
      passedPerimeter = -1.0;
    } else if (speed > 0.0) {
      passedPerimeter = 1.0;
    } else {
      if (speed == 0.0) {
        passedPerimeter = 0.0;
      }
    }

    b_speed[0] = -4.0 * passedPerimeter;
    b_speed[1] = *a_soll_TrajPlanTurnAround;
    *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
  }

  /*  一次顺车掉头轨迹生成 */
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
      b = 3.1415926535897931 * TurningRadius / 2.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        s_max = 0.05 * ((double)loop_ub + 1.0);
        b_speed[1] = speed + *a_soll_TrajPlanTurnAround * s_max;
        targetSpeed = maximum(b_speed);
        if (targetSpeed == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (targetSpeed + speed) * s_max / 2.0;
        }

        d_cur2pot_tar = s_max + passedPerimeter;
        if (d_cur2pot_tar < b) {
          pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
        } else if ((d_cur2pot_tar <= (b +
                     GlobVars->TrajPlanTurnAround.PosCircle2[1]) -
                    GlobVars->TrajPlanTurnAround.PosCircle[1]) && (d_cur2pot_tar
                    >= b)) {
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
          traj_vs[loop_ub] = targetSpeed;
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
          traj_vs[loop_ub] = -targetSpeed * sin(pos_l_TargetLane);
          traj_vl[loop_ub] = targetSpeed * s_max;
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else if (pos_l_TargetLane == 0.0) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] +
            TurningRadius;
          traj_l[loop_ub] = (GlobVars->TrajPlanTurnAround.PosCircle[1] +
                             d_cur2pot_tar) - b;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = 0.0;
          traj_vl[loop_ub] = targetSpeed;
          traj_omega[loop_ub] = targetSpeed;
        } else if (pos_l_TargetLane <= 1.5707963267948966) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] + cos
            (pos_l_TargetLane) * TurningRadius;
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] + sin
            (pos_l_TargetLane) * TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = -targetSpeed * sin(pos_l_TargetLane);
          traj_vl[loop_ub] = targetSpeed * cos(pos_l_TargetLane);
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] -
            (((d_cur2pot_tar - TurningRadius * 3.1415926535897931) -
              GlobVars->TrajPlanTurnAround.PosCircle2[1]) +
             GlobVars->TrajPlanTurnAround.PosCircle[1]);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] +
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -targetSpeed;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    }

    /* ---------------------------------------------- */
    /* 一次掉头结束判断 */
    if ((traj_psi[0] == -90.0) && (pos_s <
         GlobVars->TrajPlanTurnAround.PosCircle[0] - Parameters.TurningRadius /
         2.0)) {
      *TurnAroundActive = 0;
    }

    if ((traj_psi[79] != 90.0) && (pos_s > d)) {
      *a_soll_TrajPlanTurnAround = 100.0;
    }
  }

  /*  二次顺车掉头轨迹生成 */
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
        targetSpeed = maximum(b_speed);
        if (targetSpeed == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (targetSpeed + speed) * s_max / 2.0;
        }

        d_cur2pot_tar = s_max + passedPerimeter;
        pos_l_TargetLane = d_cur2pot_tar / TurningRadius - 1.5707963267948966;
        if (pos_l_TargetLane <= -1.5707963267948966) {
          traj_s[loop_ub] = pos_s + s_max;
          traj_l[loop_ub] = pos_l;
          traj_psi[loop_ub] = 90.0;
          traj_vs[loop_ub] = targetSpeed;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        } else if (pos_l_TargetLane < 1.5707963267948966) {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] + cos
            (pos_l_TargetLane) * TurningRadius;
          s_max = sin(pos_l_TargetLane);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[1] + s_max *
            TurningRadius;
          traj_psi[loop_ub] = -pos_l_TargetLane * 180.0 / 3.1415926535897931;
          traj_vs[loop_ub] = -targetSpeed * s_max;
          traj_vl[loop_ub] = targetSpeed * cos(pos_l_TargetLane);
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[0] -
            (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle[1] +
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -targetSpeed;
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
        targetSpeed = maximum(b_speed);
        if (targetSpeed == 0.0) {
          s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
            2.2204460492503131E-16);
        } else {
          s_max = (targetSpeed + speed) * s_max / 2.0;
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
          traj_vs[loop_ub] = targetSpeed * d;
          traj_vl[loop_ub] = -targetSpeed * s_max;
          traj_omega[loop_ub] = targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[0] +
            (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle2[1] -
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = targetSpeed;
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
          targetSpeed = maximum(b_speed);
          if (targetSpeed == 0.0) {
            s_max = (0.0 - speed * speed) / (2.0 * *a_soll_TrajPlanTurnAround +
              2.2204460492503131E-16);
          } else {
            s_max = (targetSpeed + speed) * s_max / 2.0;
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
            traj_vs[loop_ub] = -targetSpeed * s_max;
            traj_vl[loop_ub] = targetSpeed * cos(pos_l_TargetLane);
            traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
              3.1415926535897931;
          } else {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[0] -
              (d_cur2pot_tar - TurningRadius * 3.1415926535897931);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.PosCircle3[1] +
              TurningRadius;
            traj_psi[loop_ub] = -90.0;
            traj_vs[loop_ub] = -targetSpeed;
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
 *                double CalibrationVars_ACC_tau_v_com
 *                double CalibrationVars_ACC_tau_v
 *                double CalibrationVars_ACC_tau_d
 *                double CalibrationVars_ACC_tau_v_bre
 *                double CalibrationVars_ACC_tau_v_emg
 *                double CalibrationVars_ACC_tau_d_emg
 *                double CalibrationVars_ACC_t_acc
 *                double CalibrationVars_ACC_d_wait
 * Return Type  : double
 */
static double b_ACC(double v_max, double v_soll, double d_ist, double speed,
                    short b_wait, double CalibrationVars_ACC_a_max, double
                    CalibrationVars_ACC_a_min, double
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait)
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
  /* 4 */
  accel = 100.0;
  if (d_ist < 100.0) {
    if (b_wait == -1) {
      /*  停车距离远一些，避免停在故障车后面停得过近，无法换道 */
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = 17.0;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    } else {
      /*          d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]); */
      b_speed[0] = speed * CalibrationVars_ACC_t_acc;
      b_speed[1] = CalibrationVars_ACC_d_wait;
      b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
        CalibrationVars_ACC_a_min);
      d_soll = b_maximum(b_speed);
    }

    if ((v_soll == 0.0) && (d_ist <= CalibrationVars_ACC_d_wait + 0.15)) {
      d_soll = speed;
      if (speed < 0.0) {
        d_soll = -1.0;
      } else if (speed > 0.0) {
        d_soll = 1.0;
      } else {
        if (speed == 0.0) {
          d_soll = 0.0;
        }
      }

      accel = -4.0 * d_soll;
    } else {
      if (v_soll == 0.0) {
        v_soll = 0.6;
      }

      if (d_ist < CalibrationVars_ACC_d_wait) {
        accel = ((v_soll - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
      } else if ((d_ist < CalibrationVars_ACC_d_wait + 3.0) && (b_wait == 0)) {
        /*  */
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_emg;

        /*  elseif wait==1 || speed.^2/d_ist/2>-a_min_com */
        /*      accel=(v_soll-speed+(d_ist-d_soll)/tau_d)/tau_v_bre; */
      } else if (b_wait == 1) {
        if (d_ist > CalibrationVars_ACC_d_wait + 15.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_com;
        } else if (d_ist > CalibrationVars_ACC_d_wait + 10.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v;
        } else if (d_ist > CalibrationVars_ACC_d_wait + 5.0) {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_bre;
        } else {
          accel = ((v_soll - speed) + (d_ist - d_soll) /
                   CalibrationVars_ACC_tau_d) / CalibrationVars_ACC_tau_v_emg;
        }
      } else if (fabs(speed - v_soll) < 2.0) {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v_com;
      } else {
        accel = ((v_soll - speed) + (d_ist - d_soll) / CalibrationVars_ACC_tau_d)
          / CalibrationVars_ACC_tau_v;
      }
    }
  }

  b_accel[0] = -2.5;
  b_accel[1] = (v_max - speed) / CalibrationVars_ACC_tau_v;
  d_soll = maximum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = d_soll;
  accel = c_minimum(b_accel);
  b_accel[0] = accel;
  b_accel[1] = CalibrationVars_ACC_a_max;
  accel = c_minimum(b_accel);
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
  int k;
  int n;
  int nm1d2;
  ndbl = floor((b - a) + 0.5);
  apnd = a + ndbl;
  cdiff = apnd - b;
  if (fabs(cdiff) < 4.4408920985006262E-16 * fmax(fabs(a), fabs(b))) {
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
  double q;
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
        toler = 4.4408920985006262E-16 * fmax(fabs(*b), 1.0);
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              fa = 2.0 * m * s;
              q = 1.0 - s;
            } else {
              q = fa / fc;
              r = *fval / fc;
              fa = s * (2.0 * m * q * (q - r) - (*b - a) * (r - 1.0));
              q = (q - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (fa > 0.0) {
              q = -q;
            } else {
              fa = -fa;
            }

            if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e
                  * q))) {
              e = d;
              d = fa / q;
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

      if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
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
    while ((!exitg1) && (k <= 3)) {
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
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 *                double *ex
 *                int *idx
 * Return Type  : void
 */
static void b_minimum(const double x_data[], const int x_size[2], double *ex,
                      int *idx)
{
  double d;
  int i;
  int k;
  int n;
  boolean_T exitg1;
  n = x_size[1];
  if (x_size[1] <= 2) {
    if (x_size[1] == 1) {
      *ex = x_data[0];
      *idx = 1;
    } else if ((x_data[0] > x_data[1]) || (rtIsNaN(x_data[0]) && (!rtIsNaN
                 (x_data[1])))) {
      *ex = x_data[1];
      *idx = 2;
    } else {
      *ex = x_data[0];
      *idx = 1;
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      *idx = 1;
    } else {
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x_size[1])) {
        if (!rtIsNaN(x_data[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (*idx == 0) {
      *ex = x_data[0];
      *idx = 1;
    } else {
      *ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= n; k++) {
        d = x_data[k - 1];
        if (*ex > d) {
          *ex = d;
          *idx = k;
        }
      }
    }
  }
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
  double fa_tmp;
  double fc;
  double m;
  double q;
  double r;
  double s;
  double savefa;
  double savefb;
  double toler;
  boolean_T exitg1;
  *exitflag = 1.0;
  a = -0.01;
  *b = x[1];
  dv[0] = 0.0;
  d = (FunFcn_tunableEnvironment_f5[(int)FunFcn_tunableEnvironment_f2 - 1] - 0.5
       * FunFcn_tunableEnvironment_f6) - FunFcn_tunableEnvironment_f7;
  dv[1] = d;
  fa_tmp = FunFcn_tunableEnvironment_f1[(int)FunFcn_tunableEnvironment_f2 - 1];
  fa = fmax((fa_tmp + fmin(fa_tmp + FunFcn_tunableEnvironment_f3 * -0.01,
              FunFcn_tunableEnvironment_f4)) / 2.0, 0.0001) * -0.01 - maximum(dv);
  dv[0] = 0.0;
  dv[1] = d;
  *fval = fmax((fa_tmp + fmin(fa_tmp + FunFcn_tunableEnvironment_f3 * x[1],
    FunFcn_tunableEnvironment_f4)) / 2.0, 0.0001) * x[1] - maximum(dv);
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
        toler = 4.4408920985006262E-16 * fmax(fabs(*b), 1.0);
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            b_d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              fa = 2.0 * m * s;
              q = 1.0 - s;
            } else {
              q = fa / fc;
              r = *fval / fc;
              fa = s * (2.0 * m * q * (q - r) - (*b - a) * (r - 1.0));
              q = (q - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (fa > 0.0) {
              q = -q;
            } else {
              fa = -fa;
            }

            if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e
                  * q))) {
              e = b_d;
              b_d = fa / q;
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

          dv[0] = 0.0;
          dv[1] = d;
          *fval = fmax((fa_tmp + fmin(fa_tmp + FunFcn_tunableEnvironment_f3 * *b,
            FunFcn_tunableEnvironment_f4)) / 2.0, 0.0001) * *b - maximum(dv);
        }
      }

      if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
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
 * Arguments    : const double x[2]
 * Return Type  : double
 */
static double c_minimum(const double x[2])
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
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double d_minimum(const double x[3])
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
    while ((!exitg1) && (k <= 3)) {
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
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
static double e_minimum(const double x_data[], const int x_size[2])
{
  double d;
  double ex;
  int idx;
  int k;
  int n;
  boolean_T exitg1;
  n = x_size[1];
  if (x_size[1] <= 2) {
    if (x_size[1] == 1) {
      ex = x_data[0];
    } else if ((x_data[0] > x_data[1]) || (rtIsNaN(x_data[0]) && (!rtIsNaN
                 (x_data[1])))) {
      ex = x_data[1];
    } else {
      ex = x_data[0];
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x_size[1])) {
        if (!rtIsNaN(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = x_data[0];
    } else {
      ex = x_data[idx - 1];
      idx++;
      for (k = idx; k <= n; k++) {
        d = x_data[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
  }

  return ex;
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

  if (fabs(cdiff) < 4.4408920985006262E-16 * fmax(fabs(a), fabs(b))) {
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
 * Arguments    : const short x[2]
 * Return Type  : short
 */
static short f_minimum(const short x[2])
{
  short ex;
  ex = x[0];
  if (x[0] > x[1]) {
    ex = x[1];
  }

  return ex;
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
  double q;
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
        toler = 4.4408920985006262E-16 * fmax(fabs(*b), 1.0);
        if ((fabs(m) <= toler) || (*fval == 0.0)) {
          exitg1 = true;
        } else {
          if ((fabs(e) < toler) || (fabs(fa) <= fabs(*fval))) {
            d = m;
            e = m;
          } else {
            s = *fval / fa;
            if (a == c) {
              fa = 2.0 * m * s;
              q = 1.0 - s;
            } else {
              q = fa / fc;
              r = *fval / fc;
              fa = s * (2.0 * m * q * (q - r) - (*b - a) * (r - 1.0));
              q = (q - 1.0) * (r - 1.0) * (s - 1.0);
            }

            if (fa > 0.0) {
              q = -q;
            } else {
              fa = -fa;
            }

            if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e
                  * q))) {
              e = d;
              d = fa / q;
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

      if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
        *exitflag = -5.0;
      }
    }
  }
}

/*
 * Arguments    : const double x[7]
 * Return Type  : double
 */
static double g_minimum(const double x[7])
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
    while ((!exitg1) && (k <= 7)) {
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
    for (k = idx; k < 8; k++) {
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double x[8]
 * Return Type  : double
 */
static double h_minimum(const double x[8])
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
    while ((!exitg1) && (k <= 8)) {
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
    for (k = idx; k < 9; k++) {
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
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
 * Arguments    : const double x[4]
 *                double *ex
 *                int *idx
 * Return Type  : void
 */
static void minimum(const double x[4], double *ex, int *idx)
{
  double d;
  int i;
  int k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    *idx = 1;
  } else {
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaN(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 5; k++) {
      d = x[k - 1];
      if (*ex > d) {
        *ex = d;
        *idx = k;
      }
    }
  }
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
  double q;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = rtNaN;
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    q = fabs(u0 / u1);
    if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
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
 * Arguments    : const short x_data[]
 *                const int x_size[2]
 * Return Type  : boolean_T
 */
static boolean_T vectorAny(const short x_data[], const int x_size[2])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= x_size[1] - 1)) {
    if (x_data[k] == 0) {
      k++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
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
 *                const TypeStopSignInfo *StopSignInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short TurnAroundActive
 *                short PlannerLevel
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
                  *TurnAroundInfo, const TypeStopSignInfo *StopSignInfo, short
                  LaneChangeActive, short PedestrianActive, short
                  TrafficLightActive, short VehicleCrossingActive, short
                  VehicleOncomingActive, short TurnAroundActive, short
                  PlannerLevel, TypeGlobVars *GlobVars, const
                  TypeCalibrationVars *CalibrationVars, const TypeParameters
                  *Parameters, struct0_T *Trajectory, struct1_T *Decision)
{
  static const int iv[2] = { 1, 6 };

  static const int iv1[2] = { 1, 5 };

  double LaneChangePath[720];
  double b_AvoOncomingVehInfo[6];
  double b_s_vehapostrophe[6];
  double c_AvoOncomingVehInfo[6];
  double s_vehapostrophe[6];
  double AvoMainRoVehInfo_data[4];
  double b_a_soll_Fail[3];
  double b_a_soll_SpeedPlanAvoidPedestri[2];
  double CurrentLaneFrontDis;
  double LeftLaneFrontDis;
  double RightLaneFrontDis;
  double TargetLaneBehindLenAvoidVehicle;
  double TargetLaneFrontDisAvoidVehicle;
  double TargetLaneFrontLenAvoidVehicle;
  double TargetLaneFrontVelAvoidVehicle;
  double a_soll;
  double a_soll_ACC;
  double a_soll_Fail;
  double a_soll_SpeedPlanAvoidPedestrian;
  double a_soll_SpeedPlanAvoidVehicle;
  double a_soll_TrafficLightActive;
  double c_a_soll_SpeedPlanAvoidOncoming;
  double d;
  double d_veh2stopline_ped;
  double pos_l_CurrentLane;
  double pos_s;
  double speed;
  double t_TargetLaneFront2int_idx_0;
  double t_TargetLaneFront2int_idx_1;
  double t_TargetLaneFront2int_idx_2;
  double t_lc_traj;
  int AvoMainRoVehInfo_size[2];
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int partialTrueCount;
  int trueCount;
  short AEBActive;
  short BackupTargetLaneIndex;
  short CurrentTargetLaneIndex;
  short DurationLaneChange;
  short DurationLaneChange_RePlan;
  short TargetGear;
  short TargetLaneIndex;
  short b_i;
  signed char b_tmp_data[4];
  signed char c_tmp_data[4];
  signed char d_tmp_data[4];
  signed char e_tmp_data[4];
  signed char f_tmp_data[4];
  signed char tmp_data[4];
  boolean_T b_unnamed_idx_0;
  boolean_T b_unnamed_idx_1;
  boolean_T b_unnamed_idx_2;
  boolean_T b_unnamed_idx_3;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T unnamed_idx_0;
  boolean_T unnamed_idx_1;
  boolean_T unnamed_idx_2;
  boolean_T unnamed_idx_3;

  /* 入参 */
  pos_s = BasicsInfo->pos_s;
  if ((BasicsInfo->d_veh2goal < 60.0) && (BasicsInfo->GoalLaneIndex ==
       BasicsInfo->CurrentLaneIndex)) {
    pos_l_CurrentLane = BasicsInfo->pos_l_CurrentLane - ((0.5 *
      BasicsInfo->WidthOfLanes[BasicsInfo->GoalLaneIndex - 1] - 0.5 *
      Parameters->w_veh) - 0.2);
  } else {
    pos_l_CurrentLane = BasicsInfo->pos_l_CurrentLane;
  }

  if (BasicsInfo->d_veh2goal < 60.0) {
    trueCount = 0;
    for (i = 0; i < 6; i++) {
      if (BasicsInfo->WidthOfLanes[i] > 0.0) {
        trueCount++;
      }
    }

    if (trueCount > BasicsInfo->GoalLaneIndex) {
      TargetLaneIndex = BasicsInfo->GoalLaneIndex;
    } else {
      TargetLaneIndex = (short)trueCount;
    }
  } else {
    TargetLaneIndex = BasicsInfo->TargetLaneIndex;
  }

  /*  d_veh2int = BasicsInform.d_veh2int; */
  speed = ChassisInfo->speed;

  /*  故障车所在车道序号,数组大小5 */
  DurationLaneChange = GlobVars->TrajPlanLaneChange.DurationLaneChange;
  DurationLaneChange_RePlan =
    GlobVars->TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan;

  /*  TargetLaneBehindDisAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle; */
  /*  TargetLaneBehindVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle; */
  /*  TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle; */
  /*  TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle; */
  /*  AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的前车数据放入数组，默认值为200 */
  /*  AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle“大小为4的数组”，AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle(i)对应AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle(i)，默认值为20 */
  minimum(AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle,
          &TargetLaneBehindLenAvoidVehicle, &i);
  TargetLaneFrontDisAvoidVehicle =
    AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[i - 1];
  TargetLaneFrontVelAvoidVehicle =
    AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[i - 1];
  TargetLaneFrontLenAvoidVehicle =
    AvoMainRoVehInfo->TargetLaneFrontLenAvoidVehicle[i - 1];

  /* , */
  d = 2.0 * AvoMainRoVehInfo->d_veh2converge / (ChassisInfo->speed +
    2.2204460492503131E-16);
  trueCount = 0;
  a_soll_Fail = (AvoMainRoVehInfo->d_veh2converge -
                 AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[0]) /
    (AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[0] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_0 = a_soll_Fail;
  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[0] < 200.0);
  unnamed_idx_0 = unnamed_idx_3;
  b_unnamed_idx_3 = (a_soll_Fail < d);
  b_unnamed_idx_0 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount = 1;
  }

  a_soll_Fail = (AvoMainRoVehInfo->d_veh2converge -
                 AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[1]) /
    (AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[1] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_1 = a_soll_Fail;
  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[1] < 200.0);
  unnamed_idx_1 = unnamed_idx_3;
  b_unnamed_idx_3 = (a_soll_Fail < d);
  b_unnamed_idx_1 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  a_soll_Fail = (AvoMainRoVehInfo->d_veh2converge -
                 AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[2]) /
    (AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[2] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_2 = a_soll_Fail;
  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[2] < 200.0);
  unnamed_idx_2 = unnamed_idx_3;
  b_unnamed_idx_3 = (a_soll_Fail < d);
  b_unnamed_idx_2 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  a_soll_Fail = (AvoMainRoVehInfo->d_veh2converge -
                 AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[3]) /
    (AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[3] +
     2.2204460492503131E-16);
  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[3] < 200.0);
  b_unnamed_idx_3 = (a_soll_Fail < d);
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  partialTrueCount = 0;

  /* , */
  /* , */
  if (unnamed_idx_0 && b_unnamed_idx_0) {
    tmp_data[0] = 1;
    partialTrueCount = 1;
  }

  if (unnamed_idx_1 && b_unnamed_idx_1) {
    tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }

  if (unnamed_idx_2 && b_unnamed_idx_2) {
    tmp_data[partialTrueCount] = 3;
    partialTrueCount++;
  }

  if (unnamed_idx_3 && b_unnamed_idx_3) {
    tmp_data[partialTrueCount] = 4;
  }

  if (trueCount != 0) {
    AvoMainRoVehInfo_size[0] = 1;
    AvoMainRoVehInfo_size[1] = trueCount;
    for (i = 0; i < trueCount; i++) {
      AvoMainRoVehInfo_data[i] =
        AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[tmp_data[i] - 1];
    }

    b_minimum(AvoMainRoVehInfo_data, AvoMainRoVehInfo_size,
              &TargetLaneBehindLenAvoidVehicle, &i);
    TargetLaneFrontDisAvoidVehicle =
      AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[tmp_data[i - 1] - 1];
    partialTrueCount = 0;
    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[0] < 200.0) &&
        (t_TargetLaneFront2int_idx_0 < d)) {
      b_tmp_data[0] = 1;
      partialTrueCount = 1;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[1] < 200.0) &&
        (t_TargetLaneFront2int_idx_1 < d)) {
      b_tmp_data[partialTrueCount] = 2;
      partialTrueCount++;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[2] < 200.0) &&
        (t_TargetLaneFront2int_idx_2 < d)) {
      b_tmp_data[partialTrueCount] = 3;
      partialTrueCount++;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[3] < 200.0) &&
        (a_soll_Fail < d)) {
      b_tmp_data[partialTrueCount] = 4;
    }

    TargetLaneFrontVelAvoidVehicle =
      AvoMainRoVehInfo->TargetLaneFrontVelAvoidVehicle[b_tmp_data[i - 1] - 1];
    partialTrueCount = 0;
    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[0] < 200.0) &&
        (t_TargetLaneFront2int_idx_0 < d)) {
      c_tmp_data[0] = 1;
      partialTrueCount = 1;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[1] < 200.0) &&
        (t_TargetLaneFront2int_idx_1 < d)) {
      c_tmp_data[partialTrueCount] = 2;
      partialTrueCount++;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[2] < 200.0) &&
        (t_TargetLaneFront2int_idx_2 < d)) {
      c_tmp_data[partialTrueCount] = 3;
      partialTrueCount++;
    }

    if ((AvoMainRoVehInfo->TargetLaneFrontDisAvoidVehicle[3] < 200.0) &&
        (a_soll_Fail < d)) {
      c_tmp_data[partialTrueCount] = 4;
    }

    TargetLaneFrontLenAvoidVehicle =
      AvoMainRoVehInfo->TargetLaneFrontLenAvoidVehicle[c_tmp_data[i - 1] - 1];
  }

  /*  AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的后车数据放入数组，默认值为-200 */
  /*  AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle为“大小为4的数组”，AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(i)对应AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle(i)，默认值为20 */
  t_TargetLaneFront2int_idx_1 = -200.0;
  t_TargetLaneFront2int_idx_2 = 20.0;
  TargetLaneBehindLenAvoidVehicle = 5.0;
  trueCount = 0;
  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[0] > -200.0);
  unnamed_idx_0 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount = 1;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[1] > -200.0);
  unnamed_idx_1 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount++;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[2] > -200.0);
  unnamed_idx_2 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount++;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[3] > -200.0);
  if (unnamed_idx_3) {
    trueCount++;
  }

  partialTrueCount = 0;
  if (unnamed_idx_0) {
    d_tmp_data[0] = 1;
    partialTrueCount = 1;
  }

  if (unnamed_idx_1) {
    d_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }

  if (unnamed_idx_2) {
    d_tmp_data[partialTrueCount] = 3;
    partialTrueCount++;
  }

  if (unnamed_idx_3) {
    d_tmp_data[partialTrueCount] = 4;
  }

  partialTrueCount = 0;
  if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[0] > -200.0) {
    e_tmp_data[0] = 1;
    partialTrueCount = 1;
  }

  if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[1] > -200.0) {
    e_tmp_data[partialTrueCount] = 2;
    partialTrueCount++;
  }

  if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[2] > -200.0) {
    e_tmp_data[partialTrueCount] = 3;
    partialTrueCount++;
  }

  if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[3] > -200.0) {
    e_tmp_data[partialTrueCount] = 4;
  }

  if (trueCount != 0) {
    AvoMainRoVehInfo_size[0] = 1;
    AvoMainRoVehInfo_size[1] = trueCount;
    for (i = 0; i < trueCount; i++) {
      AvoMainRoVehInfo_data[i] = (AvoMainRoVehInfo->d_veh2converge -
        AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[d_tmp_data[i] - 1]) /
        (AvoMainRoVehInfo->TargetLaneBehindVelAvoidVehicle[e_tmp_data[i] - 1] +
         2.2204460492503131E-16);
    }

    b_minimum(AvoMainRoVehInfo_data, AvoMainRoVehInfo_size,
              &TargetLaneBehindLenAvoidVehicle, &i);
    t_TargetLaneFront2int_idx_1 =
      AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[d_tmp_data[i - 1] - 1];
    t_TargetLaneFront2int_idx_2 =
      AvoMainRoVehInfo->TargetLaneBehindVelAvoidVehicle[e_tmp_data[i - 1] - 1];
    partialTrueCount = 0;
    if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[0] > -200.0) {
      f_tmp_data[0] = 1;
      partialTrueCount = 1;
    }

    if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[1] > -200.0) {
      f_tmp_data[partialTrueCount] = 2;
      partialTrueCount++;
    }

    if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[2] > -200.0) {
      f_tmp_data[partialTrueCount] = 3;
      partialTrueCount++;
    }

    if (AvoMainRoVehInfo->TargetLaneBehindDisAvoidVehicle[3] > -200.0) {
      f_tmp_data[partialTrueCount] = 4;
    }

    TargetLaneBehindLenAvoidVehicle =
      AvoMainRoVehInfo->TargetLaneBehindLenAvoidVehicle[f_tmp_data[i - 1] - 1];
  }

  /*  d_veh2stopline = AvoMainRoVehInform.d_veh2stopline; */
  /*  d_veh2stopline = TrafficLightInform.d_veh2stopline; */
  AEBActive = GlobVars->AEBDecision.AEBActive;

  /* --------------------------------------------------------- */
  CurrentTargetLaneIndex = GlobVars->TrajPlanLaneChange.CurrentTargetLaneIndex;
  t_lc_traj = GlobVars->TrajPlanLaneChange.t_lc_traj;
  memcpy(&LaneChangePath[0], &GlobVars->TrajPlanLaneChange.LaneChangePath[0],
         720U * sizeof(double));
  TargetGear = ChassisInfo->CurrentGear;

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

  /*  Environmental car */
  CurrentLaneFrontDis = BasicsInfo->CurrentLaneFrontDis -
    BasicsInfo->CurrentLaneFrontLen;
  for (i = 0; i < 6; i++) {
    s_vehapostrophe[i] = AvoOncomingVehInfo->s_vehapostrophe[i] +
      AvoOncomingVehInfo->l_vehapostrophe[i];
  }

  RightLaneFrontDis = LaneChangeInfo->RightLaneFrontDis -
    LaneChangeInfo->RightLaneFrontLen;
  LeftLaneFrontDis = LaneChangeInfo->LeftLaneFrontDis -
    LaneChangeInfo->LeftLaneFrontLen;

  /*  避让故障车功能（搜寻本车所在link前方故障车） */
  BackupTargetLaneIndex = -1;
  a_soll_Fail = 100.0;
  if (vectorAny(AvoFailVehInfo->LanesWithFail, iv)) {
    /* 避让故障车 */
    BackupTargetLaneIndex = LaneSelectionWithBlockedLanes
      (BasicsInfo->WidthOfLanes, AvoFailVehInfo->LanesWithFail, &TargetLaneIndex,
       BasicsInfo->CurrentLaneIndex);
  }

  if (BasicsInfo->d_veh2goal < 60.0) {
    /* 靠边停车 */
    if (BasicsInfo->CurrentLaneIndex != TargetLaneIndex) {
      /* 未在目标车道较晚减速 */
      if (BasicsInfo->d_veh2goal <
          (CalibrationVars->TrajPlanLaneChange.v_max_int
           * CalibrationVars->TrajPlanLaneChange.v_max_int - BasicsInfo->v_max *
           BasicsInfo->v_max) / -3.0 +
          CalibrationVars->TrajPlanLaneChange.v_max_int
          * CalibrationVars->TrajPlanLaneChange.t_permit) {
        t_TargetLaneFront2int_idx_0 = ACC(8.3333333333333339, 0.0,
          BasicsInfo->d_veh2goal + CalibrationVars->ACC.d_wait,
          ChassisInfo->speed, 1.0, CalibrationVars->ACC.a_max,
          CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
          CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
          CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
          CalibrationVars->ACC.d_wait);
      } else {
        t_TargetLaneFront2int_idx_0 = 100.0;
      }
    } else {
      t_TargetLaneFront2int_idx_0 = ACC(BasicsInfo->v_max, 0.0,
        BasicsInfo->d_veh2goal + CalibrationVars->ACC.d_wait, ChassisInfo->speed,
        1.0, CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
    }
  } else {
    t_TargetLaneFront2int_idx_0 = 100.0;
  }

  a_soll_ACC = ACC(BasicsInfo->v_max, BasicsInfo->CurrentLaneFrontVel,
                   CurrentLaneFrontDis, ChassisInfo->speed, 0.0,
                   CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                   CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                   CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                   CalibrationVars->ACC.tau_v_emg,
                   CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                   CalibrationVars->ACC.d_wait);
  if (vectorAny(AvoFailVehInfo->FailLaneindex, iv1)) {
    /* 异常车避免碰撞 */
    for (i = 0; i < 5; i++) {
      b_i = AvoFailVehInfo->FailLaneindex[i];
      if ((b_i != 0) && (b_i == BasicsInfo->CurrentLaneIndex)) {
        a_soll_Fail = fmin(a_soll_Fail, ACC(BasicsInfo->v_max,
          AvoFailVehInfo->FailLaneFrontVel[i], AvoFailVehInfo->
          FailLaneFrontDis[i] - AvoFailVehInfo->FailLaneFrontLen[i], speed, 0.0,
          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
          CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
          CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
          CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
          CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait));
      }
    }
  }

  b_a_soll_Fail[0] = a_soll_Fail;
  b_a_soll_Fail[1] = a_soll_ACC;
  b_a_soll_Fail[2] = t_TargetLaneFront2int_idx_0;
  a_soll = d_minimum(b_a_soll_Fail);
  if (PedestrianActive != 0) {
    SpeedPlanAvoidPedestrian(BasicsInfo->pos_s, ChassisInfo->speed,
      AvoPedInfo->d_veh2cross, AvoPedInfo->w_cross, AvoPedInfo->s_ped,
      AvoPedInfo->l_ped, AvoPedInfo->v_ped, AvoPedInfo->psi_ped,
      CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel, BasicsInfo->v_max,
      GlobVars, Parameters->w_veh, Parameters->l_veh,
      CalibrationVars->SpeedPlanAvoidPedestrian, &CalibrationVars->ACC,
      &a_soll_SpeedPlanAvoidPedestrian, &d_veh2stopline_ped);
    b_a_soll_SpeedPlanAvoidPedestri[0] = a_soll_SpeedPlanAvoidPedestrian;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  } else {
    a_soll_SpeedPlanAvoidPedestrian = 100.0;
    d_veh2stopline_ped = 200.0;
    if (GlobVars->SpeedPlanAvoidPedestrian.dec_ped != 0) {
      GlobVars->SpeedPlanAvoidPedestrian.dec_ped = 0;
    }

    if (GlobVars->SpeedPlanAvoidPedestrian.wait_ped != 0) {
      GlobVars->SpeedPlanAvoidPedestrian.dec_ped = 0;
    }
  }

  /* , */
  for (i1 = 0; i1 < 6; i1++) {
    b_AvoOncomingVehInfo[i1] = AvoOncomingVehInfo->s_veh[i1];
  }

  for (i2 = 0; i2 < 6; i2++) {
    c_AvoOncomingVehInfo[i2] = AvoOncomingVehInfo->v_veh[i2];
  }

  for (i3 = 0; i3 < 6; i3++) {
    b_s_vehapostrophe[i3] = s_vehapostrophe[i3];
  }

  AEBDecision(&AEBActive, ChassisInfo->speed, d_veh2stopline_ped,
              AvoMainRoVehInfo->d_veh2stopline,
              AvoOncomingVehInfo->d_veh2waitingArea, b_AvoOncomingVehInfo,
              c_AvoOncomingVehInfo, AvoOncomingVehInfo->d_veh2conflict,
              b_s_vehapostrophe, TrafficLightInfo->d_veh2stopline,
              TrafficLightInfo->greenLight, GlobVars,
              CalibrationVars->SpeedPlanTrafficLight.v_max_int,
              CalibrationVars->SpeedPlanAvoidOncomingVehicle.D_safe,
              CalibrationVars->ACC.a_min, *Parameters);

  /*  PrePedestrianActive=PedestrianActive; */
  if (AEBActive != 0) {
    /*  a_soll=min([ACC(0,CurrentLaneFrontVel,CurrentLaneFrontDis,speed,0),a_soll]); */
    t_TargetLaneFront2int_idx_0 = ChassisInfo->speed;
    if (ChassisInfo->speed < 0.0) {
      t_TargetLaneFront2int_idx_0 = -1.0;
    } else if (ChassisInfo->speed > 0.0) {
      t_TargetLaneFront2int_idx_0 = 1.0;
    } else {
      if (ChassisInfo->speed == 0.0) {
        t_TargetLaneFront2int_idx_0 = 0.0;
      }
    }

    b_a_soll_SpeedPlanAvoidPedestri[0] = -4.0 * t_TargetLaneFront2int_idx_0;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 0) {
    /* 停车让行 */
    if ((StopSignInfo->d_veh2stopline < 60.0) && (StopSignInfo->d_veh2stopline >=
         1.0)) {
      GlobVars->SpeedPlanStopSign.wait_stopsign = 1;
    }
  } else {
    if (((ChassisInfo->speed <= 0.05) && (StopSignInfo->d_veh2stopline < 1.0)) ||
        (StopSignInfo->d_veh2stopline < 0.0) || (StopSignInfo->d_veh2stopline >=
         200.0)) {
      GlobVars->SpeedPlanStopSign.wait_stopsign = 0;
    }
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 1) {
    b_a_soll_SpeedPlanAvoidPedestri[0] = 0.0;
    b_a_soll_SpeedPlanAvoidPedestri[1] = StopSignInfo->d_veh2stopline +
      CalibrationVars->ACC.d_wait;
    d = maximum(b_a_soll_SpeedPlanAvoidPedestri);
    b_a_soll_SpeedPlanAvoidPedestri[0] = ACC(BasicsInfo->v_max, 0.0, d,
      ChassisInfo->speed, 0.0, CalibrationVars->ACC.a_max,
      CalibrationVars->ACC.a_min, CalibrationVars->ACC.tau_v_com,
      CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
      CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
      CalibrationVars->ACC.d_wait);
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
    VehicleCrossingActive = 0;
  }

  if (TrafficLightActive != 0) {
    a_soll_TrafficLightActive = SpeedPlanTrafficLight(ChassisInfo->speed,
      TrafficLightInfo->d_veh2stopline, CurrentLaneFrontDis,
      BasicsInfo->CurrentLaneFrontVel, TrafficLightInfo->greenLight,
      TrafficLightInfo->time2nextSwitch, BasicsInfo->v_max, GlobVars,
      CalibrationVars->SpeedPlanTrafficLight.a_min_com,
      CalibrationVars->SpeedPlanTrafficLight.a_max,
      CalibrationVars->SpeedPlanTrafficLight.a_min,
      CalibrationVars->SpeedPlanTrafficLight.v_max_int, &CalibrationVars->ACC);
    b_a_soll_SpeedPlanAvoidPedestri[0] = a_soll_TrafficLightActive;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  } else {
    a_soll_TrafficLightActive = 100.0;
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
    a_soll_SpeedPlanAvoidVehicle = SpeedPlanAvoidVehicle(ChassisInfo->speed,
      AvoMainRoVehInfo->d_veh2converge, AvoMainRoVehInfo->d_veh2stopline,
      BasicsInfo->CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel,
      BasicsInfo->CurrentLaneFrontLen, TargetLaneFrontDisAvoidVehicle,
      TargetLaneFrontVelAvoidVehicle, TargetLaneFrontLenAvoidVehicle,
      t_TargetLaneFront2int_idx_1, t_TargetLaneFront2int_idx_2,
      TargetLaneBehindLenAvoidVehicle, GlobVars,
      CalibrationVars->SpeedPlanAvoidVehicle, &CalibrationVars->ACC,
      Parameters->l_veh);
    b_a_soll_SpeedPlanAvoidPedestri[0] = a_soll_SpeedPlanAvoidVehicle;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  } else {
    a_soll_SpeedPlanAvoidVehicle = 100.0;
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
    c_a_soll_SpeedPlanAvoidOncoming = SpeedPlanAvoidOncomingVehicle
      (ChassisInfo->speed, AvoOncomingVehInfo->d_veh2waitingArea,
       CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel,
       AvoOncomingVehInfo->s_veh, AvoOncomingVehInfo->v_veh,
       AvoOncomingVehInfo->d_veh2conflict, s_vehapostrophe, BasicsInfo->v_max,
       GlobVars, CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_max_com,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_min,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.D_safe,
       &CalibrationVars->ACC, Parameters->w_veh, Parameters->l_veh);
    b_a_soll_SpeedPlanAvoidPedestri[0] = c_a_soll_SpeedPlanAvoidOncoming;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  } else {
    c_a_soll_SpeedPlanAvoidOncoming = 100.0;
    if (GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle != 0) {
      GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle = 0;
    }

    if (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle != 0)
    {
      GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle = 0;
    }
  }

  if (LaneChangeActive != 0) {
    /* 0.5; */
    /* 5; */
    /* 0.25; */
    /* 1; */
    /* -1; */
    /* -3.5; */
    if (CurrentTargetLaneIndex <= BasicsInfo->CurrentLaneIndex) {
      a_soll_Fail = LaneChangeInfo->LeftLaneBehindDis;
      TargetLaneBehindLenAvoidVehicle = LaneChangeInfo->LeftLaneBehindVel;
      t_TargetLaneFront2int_idx_0 = LeftLaneFrontDis;
      t_TargetLaneFront2int_idx_1 = LaneChangeInfo->LeftLaneFrontVel;
    } else {
      a_soll_Fail = LaneChangeInfo->RightLaneBehindDis;
      TargetLaneBehindLenAvoidVehicle = LaneChangeInfo->RightLaneBehindVel;
      t_TargetLaneFront2int_idx_0 = RightLaneFrontDis;
      t_TargetLaneFront2int_idx_1 = LaneChangeInfo->RightLaneFrontVel;
    }

    TargetLaneFrontLenAvoidVehicle = rt_roundd_snf(t_lc_traj / 0.05);
    i = DurationLaneChange - 1;
    if (DurationLaneChange - 1 < -32768) {
      i = -32768;
    }

    b_a_soll_SpeedPlanAvoidPedestri[0] = t_lc_traj - (double)(short)i * 0.1;
    b_a_soll_SpeedPlanAvoidPedestri[1] = 0.0;
    t_TargetLaneFront2int_idx_2 = maximum(b_a_soll_SpeedPlanAvoidPedestri);
    b_a_soll_SpeedPlanAvoidPedestri[0] = 0.0;
    b_a_soll_SpeedPlanAvoidPedestri[1] = t_TargetLaneFront2int_idx_1 +
      CalibrationVars->TrajPlanLaneChange.index_accel *
      CalibrationVars->TrajPlanLaneChange.a_min_comfort *
      t_TargetLaneFront2int_idx_2;
    t_lc_traj = maximum(b_a_soll_SpeedPlanAvoidPedestri);
    TargetLaneFrontVelAvoidVehicle = (t_TargetLaneFront2int_idx_0 +
      BasicsInfo->pos_s) + 0.5 * (t_lc_traj + t_TargetLaneFront2int_idx_1) *
      t_TargetLaneFront2int_idx_2;
    b_a_soll_SpeedPlanAvoidPedestri[0] = 0.0;
    b_a_soll_SpeedPlanAvoidPedestri[1] = TargetLaneBehindLenAvoidVehicle +
      CalibrationVars->TrajPlanLaneChange.index_accel *
      CalibrationVars->TrajPlanLaneChange.a_max_comfort *
      t_TargetLaneFront2int_idx_2;
    TargetLaneFrontDisAvoidVehicle = maximum(b_a_soll_SpeedPlanAvoidPedestri);

    /*      S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh]); */
    /* 20220706 */
    /*  S_max=min([S_c_end-t_re*V_end S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)]);%20220706 */
    /*  if DurationLaneChange~=0  */
    /*      if (CurrentLaneIndex~=TargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) )   */
    /*          a=1; */
    /*      end */
    /*      a_soll_TrajPlanLaneChange=99; */
    /*  if (CurrentLaneIndex~=TargetLaneIndex && CurrentLaneIndex~=BackupTargetLaneIndex && DurationLaneChange~=0 && (S_end<S_min||S_end>S_max) ) || DurationLaneChange_RePlan~=0 */
    /* , */
    guard1 = false;
    guard2 = false;
    if ((BasicsInfo->CurrentLaneIndex != CurrentTargetLaneIndex) &&
        (DurationLaneChange != 0)) {
      t_TargetLaneFront2int_idx_0 = ((a_soll_Fail + BasicsInfo->pos_s) + 0.5 *
        (TargetLaneFrontDisAvoidVehicle + TargetLaneBehindLenAvoidVehicle) *
        t_TargetLaneFront2int_idx_2) + TargetLaneFrontDisAvoidVehicle *
        CalibrationVars->TrajPlanLaneChange.t_re;
      b_a_soll_SpeedPlanAvoidPedestri[0] = t_TargetLaneFront2int_idx_0 +
        Parameters->l_veh;
      t_TargetLaneFront2int_idx_1 = LaneChangePath[(int)
        TargetLaneFrontLenAvoidVehicle + 359];
      t_TargetLaneFront2int_idx_2 = t_TargetLaneFront2int_idx_1 *
        t_TargetLaneFront2int_idx_1;
      a_soll_Fail = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_a_soll_SpeedPlanAvoidPedestri[1] = (t_TargetLaneFront2int_idx_0 +
        (t_TargetLaneFront2int_idx_2 - TargetLaneFrontDisAvoidVehicle *
         TargetLaneFrontDisAvoidVehicle) / a_soll_Fail) + Parameters->l_veh;
      d = LaneChangePath[(int)TargetLaneFrontLenAvoidVehicle - 1];
      if (d < maximum(b_a_soll_SpeedPlanAvoidPedestri)) {
        guard1 = true;
      } else {
        b_a_soll_SpeedPlanAvoidPedestri[0] = TargetLaneFrontVelAvoidVehicle -
          CalibrationVars->TrajPlanLaneChange.t_re * t_TargetLaneFront2int_idx_1;
        b_a_soll_SpeedPlanAvoidPedestri[1] = TargetLaneFrontVelAvoidVehicle -
          (t_lc_traj * t_lc_traj - t_TargetLaneFront2int_idx_2) / a_soll_Fail;
        if (d > c_minimum(b_a_soll_SpeedPlanAvoidPedestri)) {
          guard1 = true;
        } else {
          guard2 = true;
        }
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if ((DurationLaneChange_RePlan != 0) && ((!(BasicsInfo->d_veh2goal < 40.0))
           || (BasicsInfo->CurrentLaneIndex != BasicsInfo->GoalLaneIndex))) {
        guard1 = true;
      } else {
        /* , */
        /* , */
        /* , */
        for (i4 = 0; i4 < 6; i4++) {
          s_vehapostrophe[i4] = BasicsInfo->WidthOfLanes[i4];
        }

        TrajPlanLaneChange(CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel,
                           LaneChangeInfo->LeftLaneBehindDis,
                           LaneChangeInfo->LeftLaneBehindVel, LeftLaneFrontDis,
                           LaneChangeInfo->LeftLaneFrontVel,
                           LaneChangeInfo->RightLaneBehindDis,
                           LaneChangeInfo->RightLaneBehindVel, RightLaneFrontDis,
                           LaneChangeInfo->RightLaneFrontVel, ChassisInfo->speed,
                           BasicsInfo->pos_s, pos_l_CurrentLane,
                           BasicsInfo->CurrentLaneIndex, TargetLaneIndex,
                           BasicsInfo->GoalLaneIndex, BackupTargetLaneIndex,
                           LaneChangeInfo->d_veh2int, BasicsInfo->d_veh2goal,
                           s_vehapostrophe, BasicsInfo->v_max,
                           AvoFailVehInfo->LanesWithFail, GlobVars,
                           CalibrationVars, *Parameters,
                           &TargetLaneBehindLenAvoidVehicle, Trajectory->traj_s,
                           Trajectory->traj_l, Trajectory->traj_psi,
                           Trajectory->traj_vs, Trajectory->traj_vl,
                           Trajectory->traj_omega);
      }
    }

    if (guard1) {
      /*   if abs(pos_l-pos_l_CurrentLane)>0.3 */
      /* , */
      TrajPlanLaneChange_RePlan(ChassisInfo->speed, BasicsInfo->pos_s,
        BasicsInfo->pos_l, pos_l_CurrentLane, BasicsInfo->WidthOfLanes,
        BasicsInfo->CurrentLaneIndex, CurrentLaneFrontDis,
        BasicsInfo->CurrentLaneFrontVel, GlobVars,
        CalibrationVars->TrajPlanLaneChange_RePlan.t_re,
        CalibrationVars->TrajPlanLaneChange_RePlan.index_accel,
        CalibrationVars->TrajPlanLaneChange_RePlan.a_min_comfort,
        CalibrationVars->TrajPlanLaneChange_RePlan.a_min, Trajectory->traj_s,
        Trajectory->traj_l, Trajectory->traj_psi, Trajectory->traj_vs,
        Trajectory->traj_vl, Trajectory->traj_omega);
      TargetLaneBehindLenAvoidVehicle = 100.0;

      /*  end */
      GlobVars->TrajPlanLaneChange.CountLaneChange = 0;
      GlobVars->TrajPlanLaneChange.DurationLaneChange = 0;
    }

    if (TargetLaneBehindLenAvoidVehicle != 100.0) {
      b_a_soll_SpeedPlanAvoidPedestri[0] = TargetLaneBehindLenAvoidVehicle;
      b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
      a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
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

  if ((TurnAroundActive != 0) && (DurationLaneChange == 0) &&
      (DurationLaneChange_RePlan == 0)) {
    /* , */
    /* , */
    /* , */
    TrajPlanTurnAround(CurrentLaneFrontDis, BasicsInfo->CurrentLaneFrontVel,
                       ChassisInfo->speed, pos_l_CurrentLane, BasicsInfo->pos_s,
                       BasicsInfo->pos_l, TurnAroundInfo->NumOfLanesOpposite,
                       TurnAroundInfo->WidthOfLanesOpposite,
                       TurnAroundInfo->WidthOfGap, BasicsInfo->WidthOfLanes,
                       TurnAroundInfo->s_turnaround_border,
                       TurnAroundInfo->IndexOfLaneOppositeCar,
                       TurnAroundInfo->SpeedOppositeCar,
                       TurnAroundInfo->PosSOppositeCar,
                       TurnAroundInfo->LengthOppositeCar,
                       TurnAroundInfo->IndexOfLaneCodirectCar,
                       TurnAroundInfo->SpeedCodirectCar,
                       TurnAroundInfo->PosSCodirectCar,
                       TurnAroundInfo->LengthCodirectCar,
                       BasicsInfo->CurrentLaneIndex, BasicsInfo->v_max, a_soll,
                       ChassisInfo->CurrentGear, &TurnAroundActive, &AEBActive,
                       GlobVars, CalibrationVars, *Parameters,
                       &TargetLaneBehindLenAvoidVehicle, Trajectory->traj_s,
                       Trajectory->traj_l, Trajectory->traj_psi,
                       Trajectory->traj_vs, Trajectory->traj_vl,
                       Trajectory->traj_omega, &TargetGear);
    if (TargetLaneBehindLenAvoidVehicle != 100.0) {
      b_a_soll_SpeedPlanAvoidPedestri[0] = TargetLaneBehindLenAvoidVehicle;
      b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
      a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
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

  /*  靠边停车的右偏轨迹规划 */
  /* , */
  if ((PlannerLevel == 1) && (BasicsInfo->d_veh2goal < 40.0) &&
      (BasicsInfo->CurrentLaneIndex == BasicsInfo->GoalLaneIndex) &&
      (((DurationLaneChange == 0) && (DurationLaneChange_RePlan == 0) && (fabs
         (BasicsInfo->pos_l - pos_l_CurrentLane) > 0.3)) ||
       (DurationLaneChange_RePlan != 0))) {
    /* , */
    TrajPlanLaneChange_RePlan(ChassisInfo->speed, BasicsInfo->pos_s,
      BasicsInfo->pos_l, pos_l_CurrentLane, BasicsInfo->WidthOfLanes,
      BasicsInfo->CurrentLaneIndex, CurrentLaneFrontDis,
      BasicsInfo->CurrentLaneFrontVel, GlobVars,
      CalibrationVars->TrajPlanLaneChange_RePlan.t_re,
      CalibrationVars->TrajPlanLaneChange_RePlan.index_accel,
      CalibrationVars->TrajPlanLaneChange_RePlan.a_min_comfort,
      CalibrationVars->TrajPlanLaneChange_RePlan.a_min, Trajectory->traj_s,
      Trajectory->traj_l, Trajectory->traj_psi, Trajectory->traj_vs,
      Trajectory->traj_vl, Trajectory->traj_omega);
    a_soll = 100.0;
  }

  /* Decider */
  if ((PlannerLevel == 2) || (PlannerLevel == 3)) {
    *Decision = Decider(PlannerLevel, BasicsInfo->CurrentLaneFrontDis,
                        BasicsInfo->CurrentLaneFrontVel,
                        BasicsInfo->CurrentLaneFrontLen,
                        BasicsInfo->CurrentLaneIndex, BasicsInfo->WidthOfLanes,
                        BasicsInfo->v_max, BasicsInfo->d_veh2goal,
                        BasicsInfo->SampleTime, ChassisInfo->speed,
                        LaneChangeInfo, AvoMainRoVehInfo, AvoPedInfo,
                        *TrafficLightInfo, AvoOncomingVehInfo, *StopSignInfo,
                        LaneChangeActive, PedestrianActive, TrafficLightActive,
                        VehicleCrossingActive, VehicleOncomingActive, AEBActive,
                        TargetGear, a_soll_ACC, a_soll_SpeedPlanAvoidPedestrian,
                        a_soll_TrafficLightActive, a_soll_SpeedPlanAvoidVehicle,
                        c_a_soll_SpeedPlanAvoidOncoming, TargetLaneIndex,
                        BackupTargetLaneIndex, d_veh2stopline_ped, GlobVars,
                        CalibrationVars, *Parameters);
  } else {
    Decision->AEBactive = AEBActive;
    Decision->TargetGear = TargetGear;
    Decision->SlowDown = 0;
    Decision->TargetSpeed = 0.0;
    Decision->Wait = 0;
    Decision->WaitDistance = 200.0;
    Decision->Start = 0;
    Decision->a_soll = a_soll;
    Decision->LaneChange = 0;
    Decision->PedestrianState = 0;
    Decision->TrafficLightState = 0;
    Decision->VehicleCrossingState = 0;
    Decision->VehicleOncomingState = 0;
    Decision->StopSignState = 0;
    Decision->FollowState = 0;
    Decision->PullOverState = 0;
  }

  if (a_soll != 100.0) {
    b_a_soll_SpeedPlanAvoidPedestri[0] = 0.0;
    for (i = 0; i < 80; i++) {
      TargetLaneBehindLenAvoidVehicle = 0.05 * ((double)i + 1.0);
      b_a_soll_SpeedPlanAvoidPedestri[1] = speed + a_soll *
        TargetLaneBehindLenAvoidVehicle;
      d = maximum(b_a_soll_SpeedPlanAvoidPedestri);
      Trajectory->traj_vs[i] = d;
      Trajectory->traj_vl[i] = 0.0;
      Trajectory->traj_omega[i] = 0.0;
      Trajectory->traj_l[i] = pos_l_CurrentLane;
      Trajectory->traj_psi[i] = 90.0;
      if (d == 0.0) {
        Trajectory->traj_s[i] = pos_s + (0.0 - speed * speed) / (2.0 * a_soll +
          2.2204460492503131E-16);
      } else {
        Trajectory->traj_s[i] = pos_s + (d + speed) *
          TargetLaneBehindLenAvoidVehicle / 2.0;
      }
    }
  }

  GlobVars->AEBDecision.AEBActive = AEBActive;
  GlobVars->TrajPlanTurnAround.TurnAroundActive = TurnAroundActive;

  /*  全局变量类型设置 */
  /*  可配置参数：t_re l_veh w_veh tau_v tau_d a_max a_min a_min_comfort */
  /*  if TrafficLightActive */
  /*  a_soll_TrafficLightActive */
  /*  end */
  /*  if VehicleCrossingActive */
  /*  a_soll_SpeedPlanAvoidVehicle */
  /*  end */
  /*  a_soll */
  /*  a_soll */
  /*  speed */
  /*  函数调用 */
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
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void UrbanPlanner_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for UrbanPlanner.c
 *
 * [EOF]
 */
