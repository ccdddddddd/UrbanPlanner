/*
 * File: UrbanPlanner_types.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 09-Aug-2022 09:37:15
 */

#ifndef URBANPLANNER_TYPES_H
#define URBANPLANNER_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_TypeChassisInfo
#define typedef_TypeChassisInfo

typedef struct {
  double speed;
  short CurrentGear;
} TypeChassisInfo;

#endif                                 /*typedef_TypeChassisInfo*/

#ifndef typedef_TypeLaneChangeInfo
#define typedef_TypeLaneChangeInfo

typedef struct {
  double d_veh2int;
  double LeftLaneBehindDis;
  double LeftLaneBehindVel;
  double LeftLaneFrontDis;
  double LeftLaneFrontVel;
  double RightLaneBehindDis;
  double RightLaneBehindVel;
  double RightLaneFrontDis;
  double RightLaneFrontVel;
  double LeftLaneBehindLen;
  double LeftLaneFrontLen;
  double RightLaneBehindLen;
  double RightLaneFrontLen;
} TypeLaneChangeInfo;

#endif                                 /*typedef_TypeLaneChangeInfo*/

#ifndef typedef_TypeTrafficLightInfo
#define typedef_TypeTrafficLightInfo

typedef struct {
  double greenLight;
  double time2nextSwitch;
  double d_veh2stopline;
} TypeTrafficLightInfo;

#endif                                 /*typedef_TypeTrafficLightInfo*/

#ifndef typedef_TypeStopSignInfo
#define typedef_TypeStopSignInfo

typedef struct {
  double d_veh2stopline;
} TypeStopSignInfo;

#endif                                 /*typedef_TypeStopSignInfo*/

#ifndef typedef_GlobAEBDecision
#define typedef_GlobAEBDecision

typedef struct {
  short AEBActive;
} GlobAEBDecision;

#endif                                 /*typedef_GlobAEBDecision*/

#ifndef typedef_GlobSpeedPlanAvoidPedestrian
#define typedef_GlobSpeedPlanAvoidPedestrian

typedef struct {
  short dec_ped;
  short wait_ped;
} GlobSpeedPlanAvoidPedestrian;

#endif                                 /*typedef_GlobSpeedPlanAvoidPedestrian*/

#ifndef typedef_GlobSpeedPlanTrafficLight
#define typedef_GlobSpeedPlanTrafficLight

typedef struct {
  short dec_fol_TrafficLight;
  short dec_bre_TrafficLight;
  short wait_TrafficLight;
} GlobSpeedPlanTrafficLight;

#endif                                 /*typedef_GlobSpeedPlanTrafficLight*/

#ifndef typedef_GlobSpeedPlanAvoidVehicle
#define typedef_GlobSpeedPlanAvoidVehicle

typedef struct {
  short dec_fol_AvoidVehicle;
  short dec_bre_AvoidVehicle;
  short wait_AvoidVehicle;
} GlobSpeedPlanAvoidVehicle;

#endif                                 /*typedef_GlobSpeedPlanAvoidVehicle*/

#ifndef typedef_c_GlobSpeedPlanAvoidOncomingVeh
#define typedef_c_GlobSpeedPlanAvoidOncomingVeh

typedef struct {
  short dec_avoidOncomingVehicle;
  short wait_avoidOncomingVehicle;
} c_GlobSpeedPlanAvoidOncomingVeh;

#endif                                 /*typedef_c_GlobSpeedPlanAvoidOncomingVeh*/

#ifndef typedef_GlobDecider
#define typedef_GlobDecider

typedef struct {
  short dec_start;
  short dir_start;
  short CountLaneChangeDecider;
  short CurrentTargetLaneIndexDecider;
  double a_soll_pre;
  short wait_pullover;
  double distBehindGoal;
  short dec_follow;
} GlobDecider;

#endif                                 /*typedef_GlobDecider*/

#ifndef typedef_GlobSpeedPlanStopSign
#define typedef_GlobSpeedPlanStopSign

typedef struct {
  short wait_stopsign;
} GlobSpeedPlanStopSign;

#endif                                 /*typedef_GlobSpeedPlanStopSign*/

#ifndef typedef_CalibTrajPlanTurnAround
#define typedef_CalibTrajPlanTurnAround

typedef struct {
  double D_safe1;
  double D_safe2;
  double dec2line;
  double a_min;
  double a_max_com;
  double v_max_turnAround;
} CalibTrajPlanTurnAround;

#endif                                 /*typedef_CalibTrajPlanTurnAround*/

#ifndef typedef_CalibSpeedPlanAvoidPedestrian
#define typedef_CalibSpeedPlanAvoidPedestrian

typedef struct {
  double a_max;
  double a_min;
  double v_max_int;
  double v_max_int_emg;
  double d_gap2ped;
} CalibSpeedPlanAvoidPedestrian;

#endif                                 /*typedef_CalibSpeedPlanAvoidPedestrian*/

#ifndef typedef_CalibSpeedPlanTrafficLight
#define typedef_CalibSpeedPlanTrafficLight

typedef struct {
  double a_min_com;
  double a_max;
  double a_min;
  double v_max_int;
  double t_acc;
} CalibSpeedPlanTrafficLight;

#endif                                 /*typedef_CalibSpeedPlanTrafficLight*/

#ifndef typedef_CalibSpeedPlanAvoidVehicle
#define typedef_CalibSpeedPlanAvoidVehicle

typedef struct {
  double a_min_com;
  double a_max;
  double a_min;
  double v_max;
  double t_re;
  double GapIndex;
} CalibSpeedPlanAvoidVehicle;

#endif                                 /*typedef_CalibSpeedPlanAvoidVehicle*/

#ifndef typedef_c_CalibSpeedPlanAvoidOncomingVe
#define typedef_c_CalibSpeedPlanAvoidOncomingVe

typedef struct {
  double a_max_com;
  double a_min;
  double v_max_int;
  double D_safe;
} c_CalibSpeedPlanAvoidOncomingVe;

#endif                                 /*typedef_c_CalibSpeedPlanAvoidOncomingVe*/

#ifndef typedef_CalibTrajPlanLaneChange
#define typedef_CalibTrajPlanLaneChange

typedef struct {
  double v_max_int;
  double indexAfterLaneChangeDis2Int;
  double t_permit;
  double t_re;
  double index_accel;
  double a_max_comfort;
  double a_min;
  double a_max;
  double a_min_comfort;
} CalibTrajPlanLaneChange;

#endif                                 /*typedef_CalibTrajPlanLaneChange*/

#ifndef typedef_CalibTrajPlanLaneChange_RePlan
#define typedef_CalibTrajPlanLaneChange_RePlan

typedef struct {
  double t_re;
  double index_accel;
  double a_min_comfort;
  double a_min;
} CalibTrajPlanLaneChange_RePlan;

#endif                                 /*typedef_CalibTrajPlanLaneChange_RePlan*/

#ifndef typedef_CalibACC
#define typedef_CalibACC

typedef struct {
  double a_max;
  double a_min;
  double a_min_com;
  double tau_v_com;
  double tau_v;
  double tau_d;
  double tau_v_bre;
  double tau_v_emg;
  double tau_d_emg;
  double t_acc;
  double d_wait;
} CalibACC;

#endif                                 /*typedef_CalibACC*/

#ifndef typedef_CalibACCcust
#define typedef_CalibACCcust

typedef struct {
  double tau_v_com;
  double tau_v;
  double tau_d;
} CalibACCcust;

#endif                                 /*typedef_CalibACCcust*/

#ifndef typedef_CalibACClowSpeed
#define typedef_CalibACClowSpeed

typedef struct {
  double a_max;
  double a_min;
  double a_min_com;
  double tau_v_com;
  double tau_v;
  double tau_d;
  double tau_v_bre;
  double tau_v_emg;
  double tau_d_emg;
  double tau_d_lowspeed;
  double t_acc;
  double d_wait;
} CalibACClowSpeed;

#endif                                 /*typedef_CalibACClowSpeed*/

#ifndef typedef_CalibDecider
#define typedef_CalibDecider

typedef struct {
  double a_bre;
  double a_bre_com;
  double idle_speed;
  double dist_wait2pilot;
  double dist_wait2veh;
} CalibDecider;

#endif                                 /*typedef_CalibDecider*/

#ifndef typedef_TypeCalibrationVars
#define typedef_TypeCalibrationVars

typedef struct {
  CalibTrajPlanTurnAround TrajPlanTurnAround;
  CalibSpeedPlanAvoidPedestrian SpeedPlanAvoidPedestrian;
  CalibSpeedPlanTrafficLight SpeedPlanTrafficLight;
  CalibSpeedPlanAvoidVehicle SpeedPlanAvoidVehicle;
  c_CalibSpeedPlanAvoidOncomingVe SpeedPlanAvoidOncomingVehicle;
  CalibTrajPlanLaneChange TrajPlanLaneChange;
  CalibTrajPlanLaneChange_RePlan TrajPlanLaneChange_RePlan;
  CalibACC ACC;
  CalibACCcust ACCcust;
  CalibACClowSpeed ACClowSpeed;
  CalibDecider Decider;
} TypeCalibrationVars;

#endif                                 /*typedef_TypeCalibrationVars*/

#ifndef typedef_TypeParameters
#define typedef_TypeParameters

typedef struct {
  double TurningRadius;
  double w_veh;
  double l_veh;
} TypeParameters;

#endif                                 /*typedef_TypeParameters*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  short AEBactive;
  short TargetGear;
  short LaneChange;
  short SlowDown;
  double TargetSpeed;
  short Wait;
  double WaitDistance;
  short Start;
  double a_soll;
  short PedestrianState;
  short TrafficLightState;
  short VehicleCrossingState;
  short VehicleOncomingState;
  short StopSignState;
  short FollowState;
  short PullOverState;
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_TypeBasicsInfo
#define typedef_TypeBasicsInfo

typedef struct {
  double CurrentLaneFrontDis;
  double CurrentLaneFrontVel;
  double CurrentLaneFrontLen;
  double pos_s;
  double pos_l;
  double pos_l_CurrentLane;
  short CurrentLaneIndex;
  double WidthOfLanes[6];
  short TargetLaneIndex;
  double v_max;
  short GoalLaneIndex;
  double d_veh2goal;
  double SampleTime;
} TypeBasicsInfo;

#endif                                 /*typedef_TypeBasicsInfo*/

#ifndef typedef_TypeAvoMainRoVehInfo
#define typedef_TypeAvoMainRoVehInfo

typedef struct {
  double TargetLaneBehindDisAvoidVehicle[4];
  double TargetLaneBehindVelAvoidVehicle[4];
  double TargetLaneFrontDisAvoidVehicle[4];
  double TargetLaneFrontVelAvoidVehicle[4];
  double TargetLaneBehindLenAvoidVehicle[4];
  double TargetLaneFrontLenAvoidVehicle[4];
  double d_veh2converge;
  double d_veh2stopline;
} TypeAvoMainRoVehInfo;

#endif                                 /*typedef_TypeAvoMainRoVehInfo*/

#ifndef typedef_TypeAvoPedInfo
#define typedef_TypeAvoPedInfo

typedef struct {
  double d_veh2cross;
  double w_cross;
  double s_ped[40];
  double v_ped[40];
  double l_ped[40];
  double psi_ped[40];
} TypeAvoPedInfo;

#endif                                 /*typedef_TypeAvoPedInfo*/

#ifndef typedef_TypeAvoOncomingVehInfo
#define typedef_TypeAvoOncomingVehInfo

typedef struct {
  double d_veh2waitingArea;
  double s_veh[6];
  double v_veh[6];
  double l_veh[6];
  double d_veh2conflict[6];
  double s_vehapostrophe[6];
  double l_vehapostrophe[6];
} TypeAvoOncomingVehInfo;

#endif                                 /*typedef_TypeAvoOncomingVehInfo*/

#ifndef typedef_TypeAvoFailVehInfo
#define typedef_TypeAvoFailVehInfo

typedef struct {
  short LanesWithFail[6];
  short FailLaneindex[5];
  double FailLaneFrontDis[5];
  double FailLaneFrontVel[5];
  double FailLaneFrontLen[5];
} TypeAvoFailVehInfo;

#endif                                 /*typedef_TypeAvoFailVehInfo*/

#ifndef typedef_TypeTurnAroundInfo
#define typedef_TypeTurnAroundInfo

typedef struct {
  short NumOfLanesOpposite;
  double WidthOfLanesOpposite[6];
  double WidthOfGap;
  double s_turnaround_border;
  short IndexOfLaneOppositeCar[20];
  double SpeedOppositeCar[20];
  double PosSOppositeCar[20];
  short IndexOfLaneCodirectCar[10];
  double SpeedCodirectCar[10];
  double PosSCodirectCar[10];
  double LengthOppositeCar[20];
  double LengthCodirectCar[20];
} TypeTurnAroundInfo;

#endif                                 /*typedef_TypeTurnAroundInfo*/

#ifndef typedef_GlobTrajPlanTurnAround
#define typedef_GlobTrajPlanTurnAround

typedef struct {
  double PosCircle[2];
  double PosCircle2[2];
  double PosCircle3[2];
  double pos_start[2];
  double pos_mid1[4];
  double pos_mid2[4];
  double pos_mid1_rear[4];
  double pos_mid2_rear[4];
  double pos_end[2];
  double LaneCenterline[7];
  short dec_trunAround;
  short wait_turnAround;
  short TypeOfTurnAround;
  short TurnAroundState;
  short TargetLaneIndexOpposite;
  short TurnAroundActive;
} GlobTrajPlanTurnAround;

#endif                                 /*typedef_GlobTrajPlanTurnAround*/

#ifndef typedef_GlobTrajPlanLaneChange
#define typedef_GlobTrajPlanLaneChange

typedef struct {
  short CountLaneChange;
  short DurationLaneChange;
  double LaneChangePath[720];
  double t_lc_traj;
  short CurrentTargetLaneIndex;
} GlobTrajPlanLaneChange;

#endif                                 /*typedef_GlobTrajPlanLaneChange*/

#ifndef typedef_GlobTrajPlanLaneChange_RePlan
#define typedef_GlobTrajPlanLaneChange_RePlan

typedef struct {
  short DurationLaneChange_RePlan;
  double LaneChangePath_RePlan[720];
  double t_lc_RePlan;
} GlobTrajPlanLaneChange_RePlan;

#endif                                 /*typedef_GlobTrajPlanLaneChange_RePlan*/

#ifndef typedef_TypeGlobVars
#define typedef_TypeGlobVars

typedef struct {
  GlobAEBDecision AEBDecision;
  GlobTrajPlanTurnAround TrajPlanTurnAround;
  GlobSpeedPlanAvoidPedestrian SpeedPlanAvoidPedestrian;
  GlobSpeedPlanTrafficLight SpeedPlanTrafficLight;
  GlobSpeedPlanAvoidVehicle SpeedPlanAvoidVehicle;
  c_GlobSpeedPlanAvoidOncomingVeh SpeedPlanAvoidOncomingVehicle;
  GlobTrajPlanLaneChange TrajPlanLaneChange;
  GlobTrajPlanLaneChange_RePlan TrajPlanLaneChange_RePlan;
  GlobDecider Decider;
  GlobSpeedPlanStopSign SpeedPlanStopSign;
} TypeGlobVars;

#endif                                 /*typedef_TypeGlobVars*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  double traj_s[80];
  double traj_l[80];
  double traj_psi[80];
  double traj_vs[80];
  double traj_vl[80];
  double traj_omega[80];
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int32_T*/

#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T

typedef struct emxArray_int32_T emxArray_int32_T;

#endif                                 /*typedef_emxArray_int32_T*/

#ifndef struct_emxArray_int16_T
#define struct_emxArray_int16_T

struct emxArray_int16_T
{
  short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int16_T*/

#ifndef typedef_emxArray_int16_T
#define typedef_emxArray_int16_T

typedef struct emxArray_int16_T emxArray_int16_T;

#endif                                 /*typedef_emxArray_int16_T*/

#ifndef struct_emxArray_uint16_T
#define struct_emxArray_uint16_T

struct emxArray_uint16_T
{
  unsigned short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint16_T*/

#ifndef typedef_emxArray_uint16_T
#define typedef_emxArray_uint16_T

typedef struct emxArray_uint16_T emxArray_uint16_T;

#endif                                 /*typedef_emxArray_uint16_T*/

#ifndef struct_emxArray_uint32_T
#define struct_emxArray_uint32_T

struct emxArray_uint32_T
{
  unsigned int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint32_T*/

#ifndef typedef_emxArray_uint32_T
#define typedef_emxArray_uint32_T

typedef struct emxArray_uint32_T emxArray_uint32_T;

#endif                                 /*typedef_emxArray_uint32_T*/

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_boolean_T*/

#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T

typedef struct emxArray_boolean_T emxArray_boolean_T;

#endif                                 /*typedef_emxArray_boolean_T*/
#endif

/*
 * File trailer for UrbanPlanner_types.h
 *
 * [EOF]
 */
