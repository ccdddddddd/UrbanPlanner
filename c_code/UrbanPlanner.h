/*
 * File: UrbanPlanner.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 16-Mar-2023 11:35:33
 */

#ifndef URBANPLANNER_H
#define URBANPLANNER_H

/* Include Files */
#include "UrbanPlanner_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void UrbanPlanner(TypeBasicsInfo *BasicsInfo, const TypeChassisInfo
    *ChassisInfo, const TypeLaneChangeInfo *LaneChangeInfo, const
    TypeAvoMainRoVehInfo *AvoMainRoVehInfo, const TypeAvoPedInfo *AvoPedInfo,
    const TypeTrafficLightInfo *TrafficLightInfo, const TypeAvoOncomingVehInfo
    *AvoOncomingVehInfo, const TypeAvoFailVehInfo *AvoFailVehInfo, const
    TypeTurnAroundInfo *TurnAroundInfo, TypeStopSignInfo *StopSignInfo, short
    LaneChangeActive, short PedestrianActive, short TrafficLightActive, short
    VehicleCrossingActive, short VehicleOncomingActive, short TurnAroundActive,
    short GlosaActive, short PlannerLevel, TypeGlobVars *GlobVars, const
    TypeCalibrationVars *CalibrationVars, const TypeParameters *Parameters,
    struct0_T *Trajectory, struct1_T *Decision, struct2_T *Refline);
  extern void UrbanPlanner_initialize(void);
  extern void UrbanPlanner_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for UrbanPlanner.h
 *
 * [EOF]
 */
