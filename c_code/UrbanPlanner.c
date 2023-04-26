/*
 * File: UrbanPlanner.c
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 26-Apr-2023 09:43:53
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
#include <stdio.h>
#include <string.h>

/* Function Declarations */
static double ACC(double v_max, double v_soll, double d_ist, double speed,
                  double b_wait, double CalibrationVars_ACC_a_max, double
                  CalibrationVars_ACC_a_min, double
                  c_CalibrationVars_ACC_d_wait2fa, double
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
  s_vehapostrophe[6], double d_veh2int, double greenLight, double
  CurrentLaneFrontDis, double CurrentLaneFrontVel, short CurrentLaneIndex,
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters);
static void Decider(short PlannerLevel, double BasicsInfo_currentLaneFrontDis,
                    double BasicsInfo_currentLaneFrontVel, double
                    BasicsInfo_currentLaneFrontLen, double BasicsInfo_pos_s,
                    short BasicsInfo_currentLaneIndex, const double
                    BasicsInfo_widthOfLanes[6], double BasicsInfo_v_max, double
                    BasicsInfo_d_veh2goal, double BasicsInfo_sampleTime, double
                    ChassisInfo_speed, const TypeLaneChangeInfo *LaneChangeInfo,
                    const TypeAvoMainRoVehInfo *AvoMainRoVehInfo, const
                    TypeAvoPedInfo *AvoPedInfo, const TypeTrafficLightInfo
                    *TrafficLightInfo, const TypeAvoOncomingVehInfo
                    *AvoOncomingVehInfo, const TypeStopSignInfo StopSignInfo,
                    short LaneChangeActive, short PedestrianActive, short
                    TrafficLightActive, short VehicleCrossingActive, short
                    VehicleOncomingActive, short GlosaActive, short AEBActive,
                    short TargetGear, double a_soll_ACC, double
                    a_soll_SpeedPlanAvoidPedestrian, double
                    a_soll_TrafficLightActive, double
                    a_soll_SpeedPlanAvoidVehicle, double
                    c_a_soll_SpeedPlanAvoidOncoming, double
                    a_sollTurnAround2Decider, double a_soll_Fail, short
                    TargetLaneIndex, short BackupTargetLaneIndex, double
                    d_veh2stopline_ped, TypeGlobVars *GlobVars, const
                    TypeCalibrationVars *CalibrationVars, const TypeParameters
                    Parameters, struct1_T *Decision);
static void JerkLimit(double a_sollpre2traj, double SampleTime, double *a_soll,
                      double c_CalibrationVars_UrbanPlanner_);
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
  dec2line, double l_veh, double R1[2], double R2[2], double R3[2], double
  pos_start[2], double p1[4], double p2[4], double pos_end[2]);
static void PathPlanTurnAroundDecider(double LaneCenterlineTargetLane, const
  double PosCircle1[2], const double PosCircle2[2], double TurningRadius, double
  pos_s, double *NumRefLaneTurnAround, emxArray_real_T *SRefLaneTurnAround,
  emxArray_real_T *LRefLaneTurnAround, double *SEnd);
static void ReplanCenter(double Rreplan, double pos_s, double pos_l, double
  pos_l_CurrentLane, double pos_psi, double *CenterS, double *CenterL);
static double ReplanTrajPosCalc3_anonFcn3(double fun_x_workspace_s0, const
  double c_fun_x_workspace_fun_a_workspa[4], const double
  d_fun_x_workspace_fun_a_workspa[9], double length, double x);
static double ReplanTrajPosCalc4_anonFcn3(double fun_x_workspace_s0, const
  double c_fun_x_workspace_fun_a_workspa[5], const double
  d_fun_x_workspace_fun_a_workspa[12], double length, double x);
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
  double speed, double pos_s, double pos_l_CurrentLane, double pos_l, short
  CurrentLaneIndex, short TargetLaneIndex, short GoalLaneIndex, short
  BackupTargetLaneIndex, double d_veh2int, double d_veh2goal, double
  WidthOfLanes[6], double v_max, const short LanesWithFail[6], short AEBActive,
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters, double *a_soll, double traj_s[80], double traj_l[80],
  double traj_psi[80], double traj_vs[80], double traj_vl[80], double
  traj_omega[80]);
static void TrajPlanLaneChange_RePlan(double a_soll, double speed, double pos_s,
  double pos_l, double pos_psi, double pos_l_CurrentLane, double stopdistance,
  double SampleTime, double a_soll_ACC, double CurrentLaneFrontVel, TypeGlobVars
  *GlobVars, double c_CalibrationVars_TrajPlanLaneC, double
  d_CalibrationVars_TrajPlanLaneC, double c_CalibrationVars_UrbanPlanner_,
  double Parameter_l_veh, double traj_s[80], double traj_l[80], double traj_psi
  [80], double traj_vs[80], double traj_vl[80], double traj_omega[80]);
static double TrajPlanLaneChange_anonFcn8(const double
  c_fun_x_workspace_fun_a_workspa[3], const double S_traj[80], double i_traj,
  double x);
static void TrajPlanTurnAround(double CurrentLaneFrontDis, double
  CurrentLaneFrontVel, double speed, double pos_l_CurrentLane, double pos_s,
  double pos_l, short NumOfLanesOpposite, const double WidthOfLanesOpposite[6],
  double WidthOfGap, const double WidthOfLanes[6], double s_turnaround_border,
  const short IndexOfLaneOppositeCar[20], const double SpeedOppositeCar[20],
  const double PosSOppositeCar[20], const double LengthOppositeCar[20], const
  short IndexOfLaneCodirectCar[10], const double SpeedCodirectCar[10], const
  double PosSCodirectCar[10], const double LengthCodirectCar[10], short
  CurrentLane, double v_max, double a_soll, short CurrentGear, short
  *TurnAroundActive, short *AEBactive, double stopdistance, double a_soll_ACC,
  double SampleTime, TypeGlobVars *GlobVars, const TypeCalibrationVars
  *CalibrationVars, const TypeParameters Parameters, double
  *a_soll_TrajPlanTurnAround, double *a_sollTurnAround2Decider, struct2_T
  *Refline, double traj_s[80], double traj_l[80], double traj_psi[80], double
  traj_vs[80], double traj_vl[80], double traj_omega[80], short *TargetGear);
static double b_ACC(double v_max, double v_soll, double d_ist, double speed,
                    short b_wait, double CalibrationVars_ACC_a_max, double
                    CalibrationVars_ACC_a_min, double
                    c_CalibrationVars_ACC_d_wait2fa, double
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait);
static void b_cosd(double *x);
static void b_cotd(double *x);
static void b_do_vectors(const emxArray_int16_T *a, const emxArray_real_T *b,
  emxArray_int16_T *c, emxArray_int32_T *ia, int *ib_size);
static void b_eml_float_colon(double a, double b, emxArray_real_T *y);
static void b_fzero(const double c_FunFcn_workspace_fun_x_worksp[3], const
                    double FunFcn_workspace_S_traj[80], double
                    FunFcn_workspace_i_traj, const double x[2], double *b,
                    double *fval, double *exitflag);
static void b_linspace(double d1, double d2, double n, emxArray_real_T *y);
static double b_maximum(const double x[3]);
static void b_merge(int idx[132], double x[132], int offset, int np, int nq, int
                    iwork[132], double xwork[132]);
static void b_minimum(const double x_data[], const int x_size[2], double *ex,
                      int *idx);
static void b_mldivide(const double A[16], double B[4]);
static double b_mod(double x);
static void b_pchip(const double x[4], const double y[4], double v_breaks[4],
                    double v_coefs[12]);
static double b_ppval(const double pp_breaks[5], const double pp_coefs[16],
                      double x);
static void b_sind(double *x);
static void b_sort(double x[132], int idx[132]);
static double b_trapz(const double x[50], const double y[50]);
static void binary_expand_op(const TypeAvoMainRoVehInfo *in1, const signed char
  in2_data[], const int in2_size[2], const signed char in3_data[], const int
  in3_size[2], double *out1, int *out2);
static void c_fzero(double c_FunFcn_workspace_fun_x_worksp, const double
                    d_FunFcn_workspace_fun_x_worksp[5], const double
                    e_FunFcn_workspace_fun_x_worksp[12], double
                    FunFcn_workspace_length, const double x[2], double *b,
                    double *fval, double *exitflag);
static void c_linspace(double d1, double d2, double y[5]);
static short c_maximum(const short x[2]);
static double c_minimum(const double x[2]);
static double c_ppval(const double pp_breaks[5], const double pp_coefs[12],
                      double x);
static void d_fzero(double c_FunFcn_workspace_fun_x_worksp, const double
                    d_FunFcn_workspace_fun_x_worksp[4], const double
                    e_FunFcn_workspace_fun_x_worksp[9], double
                    FunFcn_workspace_length, const double x[2], double *b,
                    double *fval, double *exitflag);
static void d_linspace(double d2, double y[100]);
static double d_minimum(const double x[3]);
static void d_ppval(const double pp_breaks[4], const double pp_coefs[9], const
                    double x[50], double v[50]);
static void do_vectors(const emxArray_real_T *a, const emxArray_real_T *b,
  emxArray_real_T *c, emxArray_int32_T *ia, int *ib_size);
static void e_fzero(const double c_FunFcn_workspace_SpeedOpposit[6], double
                    FunFcn_workspace_j, double c_FunFcn_workspace_a_OppositeCa,
                    double FunFcn_workspace_v_max, const double
                    c_FunFcn_workspace_PosSOpposite[6], double
                    FunFcn_workspace_w_veh, double FunFcn_workspace_D_safe2,
                    const double x[2], double *b, double *fval, double *exitflag);
static double e_minimum(const double x_data[], const int x_size[2]);
static double e_ppval(const double pp_breaks[4], const double pp_coefs[12],
                      double x);
static void eml_float_colon(double a, double d, double b, emxArray_real_T *y);
static void eml_integer_colon_dispatcher(double a, short b, emxArray_int16_T *y);
static short f_minimum(const short x[2]);
static double f_ppval(const double pp_breaks[4], const double pp_coefs[9],
                      double x);
static void fzero(double c_FunFcn_workspace_fun_t_worksp, double
                  FunFcn_workspace_S_end, double FunFcn_workspace_w_lane, double
                  *b, double *fval, double *exitflag);
static double g_minimum(const double x[8]);
static double h_minimum(const double x[9]);
static int intnnz(const double s[6]);
static boolean_T isMember(short a, const emxArray_int16_T *s);
static void linspace(double d1, double d2, double y[50]);
static void logInput(double BasicsInfo_currentLaneFrontDis, double
                     BasicsInfo_currentLaneFrontVel, double
                     BasicsInfo_currentLaneFrontLen, double BasicsInfo_pos_s,
                     double BasicsInfo_pos_l, double BasicsInfo_pos_psi, double
                     BasicsInfo_pos_l_CurrentLane, short
                     BasicsInfo_currentLaneIndex, const double
                     BasicsInfo_widthOfLanes[6], short
                     BasicsInfo_targetLaneIndex, double BasicsInfo_v_max, short
                     BasicsInfo_goalLaneIndex, double BasicsInfo_d_veh2goal,
                     double BasicsInfo_sampleTime, double ChassisInfo_speed,
                     short ChassisInfo_currentGear, const TypeLaneChangeInfo
                     *LaneChangeInfo, const TypeAvoMainRoVehInfo
                     *AvoMainRoVehInfo, const TypeAvoPedInfo *AvoPedInfo, const
                     TypeTrafficLightInfo *TrafficLightInfo, const
                     TypeAvoOncomingVehInfo *AvoOncomingVehInfo, const
                     TypeAvoFailVehInfo *AvoFailVehInfo, const
                     TypeTurnAroundInfo *TurnAroundInfo, const TypeStopSignInfo
                     StopSignInfo, short LaneChangeActive, short
                     PedestrianActive, short TrafficLightActive, short
                     VehicleCrossingActive, short VehicleOncomingActive, short
                     TurnAroundActive, short GlosaActive, short PlannerLevel,
                     const TypeGlobVars *GlobVars, const TypeCalibrationVars
                     *CalibrationVars, const TypeParameters Parameters);
static void logOutput(const double Trajectory_traj_s[80], const double
                      Trajectory_traj_l[80], const double Trajectory_traj_psi[80],
                      const double Trajectory_traj_vs[80], const double
                      Trajectory_traj_vl[80], const double
                      Trajectory_traj_omega[80], short
                      Trajectory_planning_states, const struct1_T *Decision,
                      const struct2_T *Refline, const TypeCalibrationVars
                      *CalibrationVars);
static double maximum(const double x[2]);
static double median(const double x[3]);
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
  int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real_T *xwork);
static void minimum(const double x[4], double *ex, int *idx);
static void mldivide(const double A[9], const double B[3], double Y[3]);
static void pchip(const double x[5], const double y[5], double v_breaks[5],
                  double v_coefs[16]);
static void ppval(const double pp_breaks[5], const double pp_coefs[12], const
                  double x[50], double v[50]);
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_remd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static void scen_glosa(double d, double v0, const double tl[10], double vMax,
  double vMin, double a0, double dec, double mrg, double desRate, double dMin,
  double *vOpt, double *vgMin, double *vgMax);
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
 *                double c_CalibrationVars_ACC_d_wait2fa
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
                  c_CalibrationVars_ACC_d_wait2fa, double
                  CalibrationVars_ACC_tau_v_com, double
                  CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                  double CalibrationVars_ACC_tau_v_bre, double
                  CalibrationVars_ACC_tau_v_emg, double
                  CalibrationVars_ACC_tau_d_emg, double
                  CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait)
{
  double accel;
  double d_soll;

  /* 2.5; */
  /* -4; */
  /* 13; */
  /* 4; */
  /* 2; */
  /* 5; */
  /* 1; */
  /* 0.5; */
  /* 2; */
  /* 4 */
  accel = 100.0;
  if (d_ist < 100.0) {
    double b_speed[3];

    /*      if wait==-1 % 停车距离远一些，避免停在故障车后面停得过近，无法换道 */
    /*          d_soll=max([speed*t_acc 17 (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*      else */
    /*  %         d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*          d_soll=max([speed*t_acc d_wait (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*      end */
    if (b_wait == -1.0) {
      /*  停车距离远一些，避免停在故障车后面停得过近，无法换道 */
      d_ist = fmax(d_ist - c_CalibrationVars_ACC_d_wait2fa, 0.0);
    }

    b_speed[0] = speed * CalibrationVars_ACC_t_acc;
    b_speed[1] = CalibrationVars_ACC_d_wait;
    b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
      CalibrationVars_ACC_a_min);
    d_soll = b_maximum(b_speed);
    if ((v_soll == 0.0) && (d_ist <= CalibrationVars_ACC_d_wait + 0.15)) {
      if (rtIsNaN(speed)) {
        d_soll = rtNaN;
      } else if (speed < 0.0) {
        d_soll = -1.0;
      } else {
        d_soll = (speed > 0.0);
      }

      accel = -4.0 * d_soll;
    } else {
      if (v_soll == 0.0) {
        v_soll = 0.6;
      }

      if (d_ist < CalibrationVars_ACC_d_wait) {
        accel = ((v_soll - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
      } else if ((d_ist < CalibrationVars_ACC_d_wait + 3.0) && (b_wait != 1.0))
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

  d_soll = (v_max - speed) / CalibrationVars_ACC_tau_v;
  if (!(d_soll > -2.5)) {
    d_soll = -2.5;
  }

  if ((accel > d_soll) || rtIsNaN(accel)) {
    accel = d_soll;
  }

  if (accel > CalibrationVars_ACC_a_max) {
    accel = CalibrationVars_ACC_a_max;
  }

  if ((accel < CalibrationVars_ACC_a_min) || (rtIsNaN(accel) && (!rtIsNaN
        (CalibrationVars_ACC_a_min)))) {
    accel = CalibrationVars_ACC_a_min;
  }

  return accel;
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
  double accel;
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
    double b_speed[3];
    double d_soll;

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
    } else if (d_ist < c_CalibrationVars_ACClowSpeed_d + 2.0) {
      accel = ((v_soll - speed) + (d_ist - d_soll) /
               e_CalibrationVars_ACClowSpeed_t) /
        g_CalibrationVars_ACClowSpeed_t;
    }
  }

  speed_tmp = (v_max - speed) / d_CalibrationVars_ACClowSpeed_t;
  if (!(speed_tmp > -2.5)) {
    speed_tmp = -2.5;
  }

  if ((accel > speed_tmp) || rtIsNaN(accel)) {
    accel = speed_tmp;
  }

  if (accel > c_CalibrationVars_ACClowSpeed_a) {
    accel = c_CalibrationVars_ACClowSpeed_a;
  }

  if ((accel < d_CalibrationVars_ACClowSpeed_a) || (rtIsNaN(accel) && (!rtIsNaN
        (d_CalibrationVars_ACClowSpeed_a)))) {
    accel = d_CalibrationVars_ACClowSpeed_a;
  }

  return accel;
}

/*
 * ,
 *
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
 *                double CurrentLaneFrontDis
 *                double CurrentLaneFrontVel
 *                short CurrentLaneIndex
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 * Return Type  : void
 */
static void AEBDecision(short *AEBActive, double speed, double
  d_veh2stopline_ped, double d_veh2stopline, double d_veh2waitingArea, double
  s_veh[6], double v_veh[6], const double d_veh2conflict[6], double
  s_vehapostrophe[6], double d_veh2int, double greenLight, double
  CurrentLaneFrontDis, double CurrentLaneFrontVel, short CurrentLaneIndex,
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters)
{
  int i;
  short wait_AvoidVehicle;
  short wait_TrafficLight;
  short wait_ped;

  /* -------------------------------------------------------------------------------------------------------------------------------------------------------- */
  wait_TrafficLight = GlobVars->SpeedPlanTrafficLight.wait_TrafficLight;
  wait_ped = GlobVars->SpeedPlanAvoidPedestrian.wait_ped;
  wait_AvoidVehicle = GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle;

  /*  紧急停车决策 */
  /* 30/3.6; */
  /* 2 */
  if (*AEBActive == 0) {
    double d;
    double ex;
    double x_idx_0;

    /*  if PrePedestrianActive==1 && PedestrianActive==0 && wait_ped==1 && speed>0 */
    d = 0.0 - speed * speed;
    if ((d / (2.0 * CalibrationVars->ACC.a_min) >= d_veh2stopline_ped) &&
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
    if ((d_veh2int >= -Parameters.l_veh) && (d_veh2int <= 0.0) && (d / -8.0 -
         d_veh2int <= 10.0) && (greenLight == 0.0) && (speed > 0.0) &&
        (*AEBActive == 0)) {
      /*  红绿灯通行决策 → AEB%  */
      /* 条件：车头超过停止线且车尾未超过停止线时，信号的红灯，速度大于0且以最大减速度制动在超过停止线10米之内 */
      *AEBActive = 4;
      wait_TrafficLight = 0;
    }

    if ((d_veh2waitingArea <= 0.0) && (speed > 0.0) && (*AEBActive == 0)) {
      double s_max[6];
      boolean_T exitg1;

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
      d = CalibrationVars->SpeedPlanTrafficLight.v_max_int;
      for (i = 0; i < 6; i++) {
        if (d_veh2conflict[i] == 0.0) {
          s_veh[i] = 200.0;
          v_veh[i] = 0.0;
          s_vehapostrophe[i] = -200.0;
        }

        ex = v_veh[i];
        if ((ex < 1.0E-5) || rtIsNaN(ex)) {
          ex = 1.0E-5;
        }

        ex = ((s_veh[i] - 0.5 * Parameters.w_veh) -
              CalibrationVars->SpeedPlanAvoidOncomingVehicle.d_safe) / ex;
        if (!(ex > 0.0)) {
          ex = 0.0;
        }

        x_idx_0 = speed + 1.5 * ex;
        if (x_idx_0 > d) {
          x_idx_0 = d;
        }

        s_max[i] = 0.5 * (x_idx_0 + speed) * ex;
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

    if (*AEBActive == 0) {
      if (CurrentLaneFrontDis <= CalibrationVars->ACC.d_wait) {
        if (((GlobVars->TrajPlanLaneChange.durationLaneChange != 0) ||
             (GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan != 0))
            && (GlobVars->TrajPlanLaneChange.currentTargetLaneIndex !=
                CurrentLaneIndex)) {
          /* , */
          if (CurrentLaneFrontDis <=
              CalibrationVars->AEBDecision.minGapIsTolerated) {
            *AEBActive = 7;

            /* 条件：主车与前车已经小于停车距离，且车处在换道过程中的原车道时，与前车的距离小于最小容忍间距 */
          }
        } else {
          ex = CurrentLaneFrontVel - speed;
          x_idx_0 = speed - CurrentLaneFrontVel;
          if (!rtIsNaN(x_idx_0)) {
            if (x_idx_0 < 0.0) {
              x_idx_0 = -1.0;
            } else {
              x_idx_0 = (x_idx_0 > 0.0);
            }
          }

          if (ex * ex / 8.0 * x_idx_0 >= CurrentLaneFrontDis -
              CalibrationVars->AEBDecision.minGapIsTolerated) {
            *AEBActive = 7;

            /* 条件：主车与前车已经小于停车距离，且以最大减速度（4米每二次方秒）制动仍会侵入与前车的最小容忍间距 */
          }
        }
      } else {
        ex = CurrentLaneFrontVel - speed;
        x_idx_0 = speed - CurrentLaneFrontVel;
        if (!rtIsNaN(x_idx_0)) {
          if (x_idx_0 < 0.0) {
            x_idx_0 = -1.0;
          } else {
            x_idx_0 = (x_idx_0 > 0.0);
          }
        }

        if ((ex * ex / 8.0 * x_idx_0 >= CurrentLaneFrontDis) &&
            (((GlobVars->TrajPlanLaneChange.durationLaneChange == 0) &&
              (GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan ==
               0)) || (GlobVars->TrajPlanLaneChange.currentTargetLaneIndex ==
                       CurrentLaneIndex))) {
          /* , */
          *AEBActive = 7;

          /* 条件：主车以最大减速度（4米每二次方秒）制动仍会撞击前车 */
        }
      }
    }
  }

  GlobVars->SpeedPlanTrafficLight.wait_TrafficLight = wait_TrafficLight;
  GlobVars->SpeedPlanAvoidPedestrian.wait_ped = wait_ped;
  GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle = wait_AvoidVehicle;
}

/*
 * globalVariable-------------------------------------------------------------------------------------------------------------------------------------
 *
 * Arguments    : short PlannerLevel
 *                double BasicsInfo_currentLaneFrontDis
 *                double BasicsInfo_currentLaneFrontVel
 *                double BasicsInfo_currentLaneFrontLen
 *                double BasicsInfo_pos_s
 *                short BasicsInfo_currentLaneIndex
 *                const double BasicsInfo_widthOfLanes[6]
 *                double BasicsInfo_v_max
 *                double BasicsInfo_d_veh2goal
 *                double BasicsInfo_sampleTime
 *                double ChassisInfo_speed
 *                const TypeLaneChangeInfo *LaneChangeInfo
 *                const TypeAvoMainRoVehInfo *AvoMainRoVehInfo
 *                const TypeAvoPedInfo *AvoPedInfo
 *                const TypeTrafficLightInfo *TrafficLightInfo
 *                const TypeAvoOncomingVehInfo *AvoOncomingVehInfo
 *                const TypeStopSignInfo StopSignInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short GlosaActive
 *                short AEBActive
 *                short TargetGear
 *                double a_soll_ACC
 *                double a_soll_SpeedPlanAvoidPedestrian
 *                double a_soll_TrafficLightActive
 *                double a_soll_SpeedPlanAvoidVehicle
 *                double c_a_soll_SpeedPlanAvoidOncoming
 *                double a_sollTurnAround2Decider
 *                double a_soll_Fail
 *                short TargetLaneIndex
 *                short BackupTargetLaneIndex
 *                double d_veh2stopline_ped
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 *                struct1_T *Decision
 * Return Type  : void
 */
static void Decider(short PlannerLevel, double BasicsInfo_currentLaneFrontDis,
                    double BasicsInfo_currentLaneFrontVel, double
                    BasicsInfo_currentLaneFrontLen, double BasicsInfo_pos_s,
                    short BasicsInfo_currentLaneIndex, const double
                    BasicsInfo_widthOfLanes[6], double BasicsInfo_v_max, double
                    BasicsInfo_d_veh2goal, double BasicsInfo_sampleTime, double
                    ChassisInfo_speed, const TypeLaneChangeInfo *LaneChangeInfo,
                    const TypeAvoMainRoVehInfo *AvoMainRoVehInfo, const
                    TypeAvoPedInfo *AvoPedInfo, const TypeTrafficLightInfo
                    *TrafficLightInfo, const TypeAvoOncomingVehInfo
                    *AvoOncomingVehInfo, const TypeStopSignInfo StopSignInfo,
                    short LaneChangeActive, short PedestrianActive, short
                    TrafficLightActive, short VehicleCrossingActive, short
                    VehicleOncomingActive, short GlosaActive, short AEBActive,
                    short TargetGear, double a_soll_ACC, double
                    a_soll_SpeedPlanAvoidPedestrian, double
                    a_soll_TrafficLightActive, double
                    a_soll_SpeedPlanAvoidVehicle, double
                    c_a_soll_SpeedPlanAvoidOncoming, double
                    a_sollTurnAround2Decider, double a_soll_Fail, short
                    TargetLaneIndex, short BackupTargetLaneIndex, double
                    d_veh2stopline_ped, TypeGlobVars *GlobVars, const
                    TypeCalibrationVars *CalibrationVars, const TypeParameters
                    Parameters, struct1_T *Decision)
{
  double a_soll_matrix[9];
  double wait_matrix[8];
  double b_S_b_end[3];
  double b_ChassisInfo_speed[2];
  double ChassisInfo_speed_tmp;
  double CurrentLaneFrontDis;
  double RightLaneBehindDis;
  double RightLaneFrontDis;
  double S_b_end;
  double S_c_end;
  double S_end;
  double S_max;
  double S_min;
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
  double a_soll_StopSign;
  double d_veh2Intstopline;
  double d_veh2Rampstopline;
  double d_veh2cross;
  double d_veh2goal;
  double d_veh2goal_tmp;
  double d_veh2int;
  double d_veh2waitingArea;
  double distBehindGoal;
  double dist_wait;
  double t_lc;
  double t_mid;
  double v_d;
  double v_e;
  double w_lane;
  int a_soll_index;
  short b_BasicsInfo_currentLaneIndex[2];
  short CountLaneChange;
  short CurrentTargetLaneIndex;
  short SlowDown;
  short Wait;
  short dec_follow;
  short dec_start;
  short decision_states;
  short dir_start;
  short wait_pullover;
  boolean_T exitg1;
  boolean_T prereq5;
  CurrentTargetLaneIndex = GlobVars->Decider.currentTargetLaneIndexDecider;
  dec_start = GlobVars->Decider.dec_start;
  wait_pullover = GlobVars->Decider.wait_pullover;
  distBehindGoal = GlobVars->Decider.distBehindGoal;
  dec_follow = GlobVars->Decider.dec_follow;

  /* CalibrationVars------------------------------------------------------------------------------------------------------------------------------------ */
  /* -3;%m/s^2 */
  /* -1.5;%m/s^2 */
  /* 7;%km/h */
  /* 20;%km/h */
  /* 10;%m */
  /* 15;%m */
  /* 0.8 */
  /* 2 */
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
  /*  CurrentLaneFrontDis = CurrentLaneFrontDis-CurrentLaneFrontLen; */
  /*  RightLaneFrontDis=RightLaneFrontDis-RightLaneFrontLen; */
  /*  LeftLaneFrontDis=LeftLaneFrontDis-LeftLaneFrontLen; */
  d_veh2goal_tmp = 0.5 * Parameters.l_veh;
  d_veh2goal = BasicsInfo_d_veh2goal - d_veh2goal_tmp;

  /* 车中心转车头 */
  CurrentLaneFrontDis = BasicsInfo_currentLaneFrontDis - 0.5 *
    (BasicsInfo_currentLaneFrontLen + Parameters.l_veh);

  /* 车头到前车车尾距离 */
  /* 避让对向车 */
  d_veh2waitingArea = AvoOncomingVehInfo->d_veh2waitingArea - d_veh2goal_tmp;

  /* 车中心距离转为车头距离 */
  /* 换道入参： */
  d_veh2int = LaneChangeInfo->d_veh2int - d_veh2goal_tmp;

  /* 车中心距离转为车头距离 */
  RightLaneFrontDis = LaneChangeInfo->rightLaneFrontDis - 0.5 *
    (LaneChangeInfo->rightLaneFrontLen + Parameters.l_veh);

  /* 车头到车尾距离 */
  RightLaneBehindDis = LaneChangeInfo->rightLaneBehindDis + 0.5 *
    (Parameters.l_veh - LaneChangeInfo->rightLaneBehindLen);

  /* 车头到车头距离 */
  /* 避让同向车入参：车中心距离转为车头距离 */
  d_veh2Rampstopline = AvoMainRoVehInfo->d_veh2stopline - d_veh2goal_tmp;

  /* 停车让行停止线距离 */
  /*  d_veh2Signstopline=d_veh2Signstopline-0.5*Parameters.l_veh;%%车中心距离转为车头距离 */
  /* 信号灯通行 */
  d_veh2Intstopline = TrafficLightInfo->d_veh2stopline - d_veh2goal_tmp;

  /* 车中心距离转为车头距离 */
  /* 行人 */
  d_veh2cross = AvoPedInfo->d_veh2cross - d_veh2goal_tmp;

  /* 车中心距离转为车头距离 */
  /* -------------------------------------------------------------------------------------------------------------------------------------------------- */
  a_soll_index = BasicsInfo_currentLaneIndex - 1;
  if (BasicsInfo_currentLaneIndex - 1 < -32768) {
    a_soll_index = -32768;
  }

  /* 初值 */
  TargetVelocity = -20.0;
  TargetSpeed = -20.0;
  SlowDown = 0;
  Wait = 0;
  decision_states = 0;

  /* 初始化 */
  /*  */
  if (GlobVars->TrajPlanTurnAround.typeOfTurnAround == 2) {
    decision_states = 3;
  }

  if (AEBActive != 0) {
    decision_states = 2;
  }

  /* LaneChangeDecision-------------------------------------------------------------------------------------------------------------------------------- */
  if (TargetLaneIndex <= BasicsInfo_currentLaneIndex) {
    TargetLaneBehindDis = LaneChangeInfo->leftLaneBehindDis + 0.5 *
      (Parameters.l_veh - LaneChangeInfo->leftLaneBehindLen);
    TargetLaneBehindVel = LaneChangeInfo->leftLaneBehindVel;
    TargetLaneFrontDis = LaneChangeInfo->leftLaneFrontDis - 0.5 *
      (LaneChangeInfo->leftLaneFrontLen + Parameters.l_veh);
    TargetLaneFrontVel = LaneChangeInfo->leftLaneFrontVel;
  } else {
    TargetLaneBehindDis = RightLaneBehindDis;
    TargetLaneBehindVel = LaneChangeInfo->rightLaneBehindVel;
    TargetLaneFrontDis = RightLaneFrontDis;
    TargetLaneFrontVel = LaneChangeInfo->rightLaneFrontVel;
  }

  if (BackupTargetLaneIndex != -1) {
    v_d = LaneChangeInfo->rightLaneBehindVel;
    v_e = LaneChangeInfo->rightLaneFrontVel;
    b_BasicsInfo_currentLaneIndex[0] = (short)(BasicsInfo_currentLaneIndex + 1);
    b_BasicsInfo_currentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = f_minimum(b_BasicsInfo_currentLaneIndex);
    b_BasicsInfo_currentLaneIndex[0] = (short)(BasicsInfo_currentLaneIndex - 1);
    b_BasicsInfo_currentLaneIndex[1] = BackupTargetLaneIndex;
    BackupTargetLaneIndex = c_maximum(b_BasicsInfo_currentLaneIndex);
  } else {
    RightLaneBehindDis = -200.0;
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
  /* 1; */
  /* -3.5; */
  /* 2.5; */
  /* -1; */
  /*  a_soll=100; */
  /*  t_lc=max([2-0.05*(speed-10) 1.7]); */
  /*  t_lc=min([t_lc 2.3]); */
  b_ChassisInfo_speed[0] = 2.0 - 0.04 * (ChassisInfo_speed - 15.0);
  b_ChassisInfo_speed[1] = 2.0;
  a = maximum(b_ChassisInfo_speed);
  b_ChassisInfo_speed[0] = a;
  b_ChassisInfo_speed[1] = 2.5;
  t_lc = ceil(c_minimum(b_ChassisInfo_speed) / 0.1) * 0.1;

  /*  traj=zeros([6 120]); */
  /*  l_veh=Parameters.l_veh; */
  /*  w_veh=1.8; */
  b_BasicsInfo_currentLaneIndex[0] = (short)(BasicsInfo_currentLaneIndex + 1);
  b_BasicsInfo_currentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = f_minimum(b_BasicsInfo_currentLaneIndex);
  b_BasicsInfo_currentLaneIndex[0] = (short)(BasicsInfo_currentLaneIndex - 1);
  b_BasicsInfo_currentLaneIndex[1] = TargetLaneIndex;
  TargetLaneIndex = c_maximum(b_BasicsInfo_currentLaneIndex);

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
  CountLaneChange = 0;
  if ((BasicsInfo_currentLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_currentLaneIndex != BackupTargetLaneIndex)) {
    /*      if TargetLaneIndex<=CurrentLaneIndex */
    /*          w_lane=w_lane_left; */
    /*      else */
    /*          w_lane=w_lane_right; */
    /*      end */
    w_lane = 0.5 * BasicsInfo_widthOfLanes[BasicsInfo_currentLaneIndex - 1] +
      0.5 * BasicsInfo_widthOfLanes[TargetLaneIndex - 1];

    /* 20220328 */
    if (ChassisInfo_speed < 5.0) {
      b_ChassisInfo_speed[0] = 10.0;
      b_ChassisInfo_speed[1] = t_lc * ChassisInfo_speed;
      S_end = maximum(b_ChassisInfo_speed);
      fzero(ChassisInfo_speed, S_end, w_lane, &a_soll_StopSign, &S_min_dyn,
            &S_max);
      b_ChassisInfo_speed[0] = t_lc;
      b_ChassisInfo_speed[1] = a_soll_StopSign;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_ChassisInfo_speed) / 0.1);
      b_ChassisInfo_speed[0] = w_lane;
      ChassisInfo_speed_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_ChassisInfo_speed[1] = (0.0 - ChassisInfo_speed * ChassisInfo_speed) /
        ChassisInfo_speed_tmp;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = w_lane;
      a_soll_StopSign = ChassisInfo_speed * t_lc;
      b_ChassisInfo_speed[1] = a_soll_StopSign + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_min_dyn = maximum(b_ChassisInfo_speed);
      S_max = w_lane * w_lane;
      b_ChassisInfo_speed[0] = sqrt(a * a - S_max);
      b_ChassisInfo_speed[1] = sqrt(S_min_dyn * S_min_dyn - S_max);
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
        CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
        CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
        CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
        CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneBehindVel + a *
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
      a = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > a) && (TargetLaneFrontDis <= a_soll_StopSign))
      {
        /* , */
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_max) / t_lc * 2.0 - ChassisInfo_speed;
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
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= a) && (TargetLaneFrontDis >
                  a_soll_StopSign)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
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
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else if ((-TargetLaneBehindDis > a) && (TargetLaneFrontDis >
                  a_soll_StopSign)) {
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
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + S_max) / t_lc * 2.0 - ChassisInfo_speed;
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
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((S_max > S_end) && (S_end > S_min)) {
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
          (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (TargetLaneBehindDis <= -Parameters.l_veh) &&
          (TargetLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_currentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_currentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          CountLaneChange = 1;
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
        CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
        CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
        CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
        CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = TargetLaneBehindVel + a *
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
      a = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > a) && (TargetLaneFrontDis <= ChassisInfo_speed
           * (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车存在 */
        b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + V_c_end);
        b_ChassisInfo_speed[1] = ChassisInfo_speed;
        V_end = c_minimum(b_ChassisInfo_speed);
        w_lane = S_b_end + TargetLaneFrontVel *
          CalibrationVars->TrajPlanLaneChange.t_re;
        b_S_b_end[0] = w_lane + Parameters.l_veh;
        a_soll_StopSign = V_end * V_end;
        S_min_dyn = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = (w_lane + (a_soll_StopSign - TargetLaneFrontVel *
          TargetLaneFrontVel) / S_min_dyn) + Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - a_soll_StopSign) /
          S_min_dyn;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= a) && (TargetLaneFrontDis >
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
        w_lane = V_end * V_end;
        a_soll_StopSign = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (w_lane -
          TargetLaneFrontVel * TargetLaneFrontVel) / a_soll_StopSign) +
          Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_min_dyn;
        b_S_b_end[1] = S_min_dyn - (V_c_end * V_c_end - w_lane) /
          a_soll_StopSign;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        S_max = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis > a) && (TargetLaneFrontDis >
                  ChassisInfo_speed * (t_lc +
                   CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车不存在 */
        S_end = ChassisInfo_speed * t_lc;
        V_end = ChassisInfo_speed;
        b_S_b_end[0] = (S_b_end + TargetLaneFrontVel *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        w_lane = ChassisInfo_speed * ChassisInfo_speed;
        a_soll_StopSign = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + TargetLaneFrontVel *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (w_lane -
          TargetLaneFrontVel * TargetLaneFrontVel) / a_soll_StopSign) +
          Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          ChassisInfo_speed;
        b_S_b_end[0] = S_min_dyn;
        b_S_b_end[1] = S_min_dyn - (V_c_end * V_c_end - w_lane) /
          a_soll_StopSign;
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
        S_min = b_maximum(b_S_b_end);

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
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      }

      /*  参见TrajPlanLaneChange_S_max_withAccel.bmp */
      b_ChassisInfo_speed[0] = V_end;
      b_ChassisInfo_speed[1] = ChassisInfo_speed;
      a_soll_StopSign = (CalibrationVars->TrajPlanLaneChange.a_max_comfort *
                         t_lc - fabs(V_end - ChassisInfo_speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移  */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= S_min) {
        a = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        TargetLaneBehindVel = a_soll_StopSign * a_soll_StopSign /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_ChassisInfo_speed) + a) -
             TargetLaneBehindVel) && (S_end >= (t_lc * maximum
              (b_ChassisInfo_speed) - a) + TargetLaneBehindVel)) {
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
          && prereq5 && (d_veh2int >= S_end +
                         CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int
                         * Parameters.l_veh) && (TargetLaneBehindDis <=
           -Parameters.l_veh) && (TargetLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_currentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_currentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          CountLaneChange = 1;
          CurrentTargetLaneIndex = TargetLaneIndex;
        }
      }
    }
  }

  if ((CurrentTargetLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_currentLaneIndex != TargetLaneIndex) &&
      (BasicsInfo_currentLaneIndex != BackupTargetLaneIndex) &&
      (BackupTargetLaneIndex != -1)) {
    if (BackupTargetLaneIndex <= BasicsInfo_currentLaneIndex) {
      if ((short)a_soll_index < 1) {
        a_soll_index = 1;
      } else {
        a_soll_index = (short)a_soll_index;
      }

      w_lane = 0.5 * BasicsInfo_widthOfLanes[a_soll_index - 1] + 0.5 *
        BasicsInfo_widthOfLanes[BasicsInfo_currentLaneIndex - 1];
    } else {
      if (BasicsInfo_currentLaneIndex + 1 > 6) {
        a_soll_index = 6;
      } else {
        a_soll_index = BasicsInfo_currentLaneIndex + 1;
      }

      w_lane = 0.5 * BasicsInfo_widthOfLanes[a_soll_index - 1] + 0.5 *
        BasicsInfo_widthOfLanes[BasicsInfo_currentLaneIndex - 1];
    }

    if (ChassisInfo_speed < 5.0) {
      b_ChassisInfo_speed[0] = 10.0;
      b_ChassisInfo_speed[1] = t_lc * ChassisInfo_speed;
      S_end = maximum(b_ChassisInfo_speed);
      fzero(ChassisInfo_speed, S_end, w_lane, &a_soll_StopSign, &S_min_dyn,
            &S_max);
      b_ChassisInfo_speed[0] = t_lc;
      b_ChassisInfo_speed[1] = a_soll_StopSign;
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
      S_max = w_lane * w_lane;
      b_ChassisInfo_speed[0] = sqrt(a * a - S_max);
      b_ChassisInfo_speed[1] = sqrt(S_min_dyn * S_min_dyn - S_max);
      S_min_dyn = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_e +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = RightLaneFrontDis + 0.5 * (V_c_end + v_e) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, v_e, RightLaneFrontDis -
        RightLaneBehindDis, v_d, 0.0, CalibrationVars->ACC.a_max,
        CalibrationVars->ACC.a_min, CalibrationVars->ACC.d_wait2faultyCar,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_d + a *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_b_end = maximum(b_ChassisInfo_speed);
      TargetLaneFrontVel = RightLaneBehindDis + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;
      a = v_d * t_lc;
      if ((-RightLaneBehindDis > a) && (RightLaneFrontDis <= ChassisInfo_speed *
           t_lc)) {
        /* , */
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_max) / t_lc * 2.0 - ChassisInfo_speed;
        b_ChassisInfo_speed[0] = S_b_end - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else if ((-RightLaneBehindDis <= a) && (RightLaneFrontDis >
                  ChassisInfo_speed * t_lc)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        V_end = sqrt(S_end * S_end + S_max) / t_lc * 2.0 - ChassisInfo_speed;
        b_ChassisInfo_speed[0] = S_b_end - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = w_lane;
        b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
          ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else if ((-RightLaneBehindDis > a) && (RightLaneFrontDis >
                  ChassisInfo_speed * t_lc)) {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 -
          ChassisInfo_speed;
        b_ChassisInfo_speed[0] = S_b_end - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + S_max) / t_lc * 2.0 - ChassisInfo_speed;
        b_ChassisInfo_speed[0] = S_b_end - V_end;
        b_ChassisInfo_speed[1] = 0.0;
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + maximum(b_ChassisInfo_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / ChassisInfo_speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        a = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (S_c_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end *
          V_c_end - V_end * V_end) / ChassisInfo_speed_tmp;
        b_S_b_end[2] = sqrt(a * a - S_max);
        S_max = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((S_max > S_end) && (S_end > S_min)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      if (((S_c_end - TargetLaneFrontVel) - Parameters.l_veh > S_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            S_b_end * S_b_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min * t_lc)
          && (V_end < ChassisInfo_speed +
              CalibrationVars->TrajPlanLaneChange.a_max * t_lc) && prereq5 &&
          (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (RightLaneBehindDis <= -Parameters.l_veh) &&
          (RightLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_currentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_currentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          CountLaneChange++;
          TargetLaneIndex = BackupTargetLaneIndex;
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    } else {
      boolean_T guard1 = false;
      boolean_T guard2 = false;
      boolean_T guard3 = false;
      b_ChassisInfo_speed[0] = 0.0;
      ChassisInfo_speed_tmp = CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort;
      b_ChassisInfo_speed[1] = v_e + ChassisInfo_speed_tmp * t_lc;
      V_c_end = maximum(b_ChassisInfo_speed);
      S_c_end = RightLaneFrontDis + 0.5 * (V_c_end + v_e) * t_lc;
      b_ChassisInfo_speed[0] = ACC(BasicsInfo_v_max, v_e, RightLaneFrontDis -
        RightLaneBehindDis, v_d, 0.0, CalibrationVars->ACC.a_max,
        CalibrationVars->ACC.a_min, CalibrationVars->ACC.d_wait2faultyCar,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_ChassisInfo_speed[1] = 0.5;
      a = maximum(b_ChassisInfo_speed);
      b_ChassisInfo_speed[0] = 0.0;
      b_ChassisInfo_speed[1] = v_d + a *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_b_end = maximum(b_ChassisInfo_speed);
      TargetLaneFrontVel = RightLaneBehindDis + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;
      a = v_d * t_lc;
      guard1 = false;
      guard2 = false;
      guard3 = false;
      if (-RightLaneBehindDis > a) {
        TargetLaneBehindVel = ChassisInfo_speed * t_lc;
        if (RightLaneFrontDis <= TargetLaneBehindVel) {
          /* , */
          /*                  b车不存在 c车存在 */
          b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + V_c_end);
          b_ChassisInfo_speed[1] = ChassisInfo_speed;
          V_end = c_minimum(b_ChassisInfo_speed);
          w_lane = TargetLaneFrontVel + S_b_end *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = w_lane + Parameters.l_veh;
          a_soll_StopSign = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = (w_lane + (V_end * V_end - S_b_end * S_b_end) /
                          a_soll_StopSign) + Parameters.l_veh;
          b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          S_min = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
          b_S_b_end[0] = w_lane;
          b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
            a_soll_StopSign;
          b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          S_max = d_minimum(b_S_b_end);
          b_S_b_end[0] = S_min;
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
        if (-RightLaneBehindDis <= a) {
          TargetLaneBehindVel = ChassisInfo_speed * t_lc;
          if (RightLaneFrontDis > TargetLaneBehindVel) {
            /* , */
            /*                  b车存在 c车不存在 */
            b_ChassisInfo_speed[0] = 0.5 * (ChassisInfo_speed + S_b_end);
            b_ChassisInfo_speed[1] = ChassisInfo_speed;
            V_end = c_minimum(b_ChassisInfo_speed);
            w_lane = TargetLaneFrontVel + S_b_end *
              CalibrationVars->TrajPlanLaneChange.t_re;
            b_S_b_end[0] = w_lane + Parameters.l_veh;
            a_soll_StopSign = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = (w_lane + (V_end * V_end - S_b_end * S_b_end) /
                            a_soll_StopSign) + Parameters.l_veh;
            b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            w_lane = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
            b_S_b_end[0] = w_lane;
            b_S_b_end[1] = w_lane - (V_c_end * V_c_end - V_end * V_end) /
              a_soll_StopSign;
            b_S_b_end[2] = TargetLaneBehindVel + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            S_max = d_minimum(b_S_b_end);
            b_S_b_end[0] = S_min;
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
        if (-RightLaneBehindDis > a) {
          S_end = ChassisInfo_speed * t_lc;
          if (RightLaneFrontDis > S_end) {
            /* , */
            /*                  b车不存在 c车不存在 */
            V_end = ChassisInfo_speed;
            b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            w_lane = ChassisInfo_speed * ChassisInfo_speed;
            a_soll_StopSign = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((TargetLaneFrontVel + S_b_end *
                             CalibrationVars->TrajPlanLaneChange.t_re) + (w_lane
              - S_b_end * S_b_end) / a_soll_StopSign) + Parameters.l_veh;
            b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            S_min_dyn = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
              ChassisInfo_speed;
            b_S_b_end[0] = S_min_dyn;
            b_S_b_end[1] = S_min_dyn - (V_c_end * V_c_end - w_lane) /
              a_soll_StopSign;
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
        b_ChassisInfo_speed[1] = 0.5 * (S_b_end + V_c_end);
        V_end = c_minimum(b_ChassisInfo_speed);
        b_S_b_end[0] = (TargetLaneFrontVel + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((TargetLaneFrontVel + S_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = ChassisInfo_speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

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
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = S_max;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + ChassisInfo_speed);
        S_end = median(b_S_b_end);
      }

      b_ChassisInfo_speed[0] = V_end;
      b_ChassisInfo_speed[1] = ChassisInfo_speed;
      S_min_dyn = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      a_soll_StopSign = (S_min_dyn - fabs(V_end - ChassisInfo_speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (S_max >= S_min) {
        a = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc *
          t_lc;
        TargetLaneBehindVel = a_soll_StopSign * a_soll_StopSign /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_ChassisInfo_speed) + a) -
             TargetLaneBehindVel) && (S_end >= (t_lc * maximum
              (b_ChassisInfo_speed) - a) + TargetLaneBehindVel)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((S_c_end - TargetLaneFrontVel) - Parameters.l_veh > S_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (V_c_end * V_c_end -
            S_b_end * S_b_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >=
           ChassisInfo_speed + CalibrationVars->TrajPlanLaneChange.a_min_comfort
           * t_lc) && (V_end <= ChassisInfo_speed + S_min_dyn) && prereq5 &&
          (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (RightLaneBehindDis <= -Parameters.l_veh) &&
          (RightLaneFrontDis >= 0.0)) {
        b_ChassisInfo_speed[0] = 0.0;
        b_ChassisInfo_speed[1] = BasicsInfo_currentLaneFrontVel +
          ChassisInfo_speed_tmp * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_ChassisInfo_speed) +
              BasicsInfo_currentLaneFrontVel) * t_mid > 0.5 * S_end) &&
            (CurrentLaneFrontDis >= 0.25 * t_lc * (0.75 * ChassisInfo_speed +
              0.25 * V_end))) {
          CountLaneChange++;
          TargetLaneIndex = BackupTargetLaneIndex;
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    }
  }

  if (LaneChangeActive != 0) {
    a_soll_index = CurrentTargetLaneIndex - BasicsInfo_currentLaneIndex;
    if (a_soll_index > 32767) {
      a_soll_index = 32767;
    } else if (a_soll_index < -32768) {
      a_soll_index = -32768;
    }

    if ((CountLaneChange > 0) && (a_soll_index == -1)) {
      /* 左换道 */
      Decision->LaneChange = 1;
    } else if ((CountLaneChange > 0) && (a_soll_index == 1)) {
      /* 右换道 */
      Decision->LaneChange = 2;
    } else {
      Decision->LaneChange = 0;

      /*          GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0); */
    }
  } else {
    Decision->LaneChange = 0;

    /*      GlobVars.TrajPlanLaneChange.CountLaneChange=int16(0); */
    CountLaneChange = 0;
  }

  /* ------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /*  status */
  Decision->PedestrianState = (short)(PedestrianActive != 0);
  Decision->TrafficLightState = (short)(TrafficLightActive != 0);
  Decision->VehicleCrossingState = (short)(VehicleCrossingActive != 0);
  Decision->VehicleOncomingState = (short)(VehicleOncomingActive != 0);
  if ((CurrentLaneFrontDis < 195.0) && (BasicsInfo_currentLaneFrontVel < 20.0))
  {
    Decision->FollowState = 1;
  } else {
    Decision->FollowState = 0;
  }

  Decision->StopSignState = (short)(GlobVars->SpeedPlanStopSign.wait_stopsign ==
    1);
  if (d_veh2goal <= 60.0) {
    /* 靠边停车 */
    Decision->PullOverState = 1;
  } else {
    Decision->PullOverState = 0;
  }

  Decision->TurnAroundState = (short)
    (GlobVars->TrajPlanTurnAround.turnAroundActive == 1);

  /*  wait */
  /*  CalibrationVars.Decider.GlosaAdp=2; */
  /*  CalibrationVars.Decider.mrg=4; */
  /*  CalibrationVars.Decider.desRate=0.75; */
  /*  CalibrationVars.Decider.dIntxn=10; */
  /*  CalibrationVars.Decider.dMin=2; */
  /*  TrafficLightInfo.Phase=zeros(1,10); */
  if ((GlosaActive == 1) && (TrafficLightActive == 1) && (d_veh2Intstopline >
       0.0)) {
    scen_glosa(d_veh2Intstopline, ChassisInfo_speed, TrafficLightInfo->phase,
               BasicsInfo_v_max, CalibrationVars->Decider.glosaVMin / 3.6,
               CalibrationVars->Decider.glosaAdp, CalibrationVars->Decider.dec,
               CalibrationVars->Decider.mrg, CalibrationVars->Decider.desRate,
               CalibrationVars->Decider.dMin, &S_max, &S_min_dyn,
               &a_soll_StopSign);

    /* , */
    if (S_min_dyn == -1.0) {
      a_soll_TrafficLightActive = ACC(BasicsInfo_v_max, 0.0, (d_veh2Intstopline
        + CalibrationVars->ACC.d_wait) - 0.5, ChassisInfo_speed, 1.0,
        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
        CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
        CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
        CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
        CalibrationVars->ACC.d_wait);
    } else if (ChassisInfo_speed > a_soll_StopSign *
               CalibrationVars->Decider.glosaAverageIndex + S_min_dyn * (1.0 -
                CalibrationVars->Decider.glosaAverageIndex)) {
      a_soll_TrafficLightActive = -CalibrationVars->Decider.glosaAdp;
    } else if (ChassisInfo_speed < S_min_dyn *
               CalibrationVars->Decider.glosaAverageIndex + a_soll_StopSign *
               (1.0 - CalibrationVars->Decider.glosaAverageIndex)) {
      a_soll_TrafficLightActive = CalibrationVars->Decider.glosaAdp;
    } else {
      a_soll_TrafficLightActive = 0.0;
    }
  } else {
    S_min_dyn = 0.0;

    /* Variable 'vgMin' is not fully defined on some execution paths. */
  }

  for (a_soll_index = 0; a_soll_index < 8; a_soll_index++) {
    wait_matrix[a_soll_index] = 200.0;
  }

  /*  if CurrentLaneFrontVel<=0.1 && CurrentLaneFrontDis<=dist_wait+l_veh+5 */
  /*      wait_matrix(6)=CurrentLaneFrontDis-l_veh-5; */
  /*  end */
  if ((BasicsInfo_currentLaneFrontVel <= 0.5) && (CurrentLaneFrontDis <=
       dist_wait + CalibrationVars->ACC.d_wait)) {
    /* CurrentLaneFrontDis为前车车尾距离 */
    wait_matrix[6] = CurrentLaneFrontDis - CalibrationVars->ACC.d_wait;
  }

  if ((Decision->TurnAroundState == 1) &&
      ((GlobVars->TrajPlanTurnAround.wait_turnAround == 1) ||
       (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) &&
      (GlobVars->TrajPlanTurnAround.posCircle[0] - BasicsInfo_pos_s <= dist_wait))
  {
    wait_matrix[4] = GlobVars->TrajPlanTurnAround.posCircle[0] -
      BasicsInfo_pos_s;

    /* 掉头激活且掉头停止线小于停车距离,掉头wait和信号灯wait任意=1 */
  }

  if ((GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1) &&
      (d_veh2Rampstopline <= dist_wait)) {
    wait_matrix[1] = d_veh2Rampstopline;
  }

  if ((GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle == 1) &&
      (d_veh2waitingArea <= dist_wait)) {
    wait_matrix[2] = d_veh2waitingArea;
  }

  /*  if wait_ped==1 && d_veh2cross<=dist_wait */
  /*      wait_matrix(1)=d_veh2cross; */
  /*  end */
  if ((GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1) && (d_veh2stopline_ped <=
       dist_wait)) {
    wait_matrix[0] = d_veh2stopline_ped;
  }

  if ((GlosaActive == 1) && (TrafficLightActive == 1) && (d_veh2Intstopline >
       0.0)) {
    if ((S_min_dyn == -1.0) && (d_veh2Intstopline <= dist_wait)) {
      wait_matrix[3] = d_veh2Intstopline;
    }
  } else if ((GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1) &&
             (d_veh2Intstopline <= dist_wait)) {
    wait_matrix[3] = d_veh2Intstopline;
  }

  if ((GlobVars->SpeedPlanStopSign.wait_stopsign == 1) &&
      (StopSignInfo.d_veh2stopline <= dist_wait) && (StopSignInfo.d_veh2stopline
       >= 0.0)) {
    wait_matrix[5] = StopSignInfo.d_veh2stopline;
  }

  /*  if d_veh2goal<=dist_wait%靠边停车 */
  /*      wait_matrix(7)=d_veh2goal; */
  /*  end */
  /*  靠边停车子功能wait状态的切换及靠边停车位置的计算 */
  if ((BasicsInfo_currentLaneIndex == TargetLaneIndex) && (BasicsInfo_d_veh2goal
       < 60.0) && (GlobVars->Decider.wait_pullover == 0)) {
    a = ChassisInfo_speed * ChassisInfo_speed / 4.0;
    if (a < 15.0) {
      wait_pullover = 1;
      distBehindGoal = fmax(0.0, a - BasicsInfo_d_veh2goal);

      /*  distBehindGoal为全局变量，初值为0 */
    }
  }

  /*  靠边停车距离的计算 */
  if (wait_pullover == 1) {
    wait_matrix[7] = BasicsInfo_d_veh2goal + distBehindGoal;
  }

  S_b_end = g_minimum(wait_matrix);
  if (S_b_end <= dist_wait) {
    a_soll_index = 0;
    exitg1 = false;
    while ((!exitg1) && (a_soll_index < 8)) {
      if (wait_matrix[a_soll_index] == S_b_end) {
        Wait = (short)(a_soll_index + 1);
        exitg1 = true;
      } else {
        a_soll_index++;
      }
    }
  } else {
    S_b_end = 200.0;
  }

  if ((Wait == 2) && (StopSignInfo.d_veh2stopline <= dist_wait) &&
      (StopSignInfo.d_veh2stopline >= 0.0)) {
    /* 有停止让行标志 */
    Wait = 6;
  }

  /*  slowdown */
  a = BasicsInfo_v_max - ChassisInfo_speed;
  if (a < 0.0) {
    /* 限速加速度 */
    b_ChassisInfo_speed[0] = -2.5;
    b_ChassisInfo_speed[1] = a / CalibrationVars->ACC.tau_v;
    S_max = maximum(b_ChassisInfo_speed);
  } else {
    S_max = 100.0;
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 1) {
    /* 停车让行加速度 */
    b_ChassisInfo_speed[0] = 0.0;
    b_ChassisInfo_speed[1] = StopSignInfo.d_veh2stopline +
      CalibrationVars->ACC.d_wait;
    a_soll_StopSign = ACC(BasicsInfo_v_max, 0.0, maximum(b_ChassisInfo_speed),
                          ChassisInfo_speed, 0.0, CalibrationVars->ACC.a_max,
                          CalibrationVars->ACC.a_min,
                          CalibrationVars->ACC.d_wait2faultyCar,
                          CalibrationVars->ACC.tau_v_com,
                          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                          CalibrationVars->ACC.tau_v_bre,
                          CalibrationVars->ACC.tau_v_emg,
                          CalibrationVars->ACC.tau_d_emg,
                          CalibrationVars->ACC.t_acc,
                          CalibrationVars->ACC.d_wait);
  } else {
    a_soll_StopSign = 100.0;
  }

  if (BasicsInfo_d_veh2goal < 60.0) {
    /* 靠边停车 */
    if (BasicsInfo_currentLaneIndex != TargetLaneIndex) {
      /* 未在目标车道较晚减速 */
      if (BasicsInfo_d_veh2goal < (CalibrationVars->TrajPlanLaneChange.v_max_int
           * CalibrationVars->TrajPlanLaneChange.v_max_int - BasicsInfo_v_max *
           BasicsInfo_v_max) / -3.0 +
          CalibrationVars->TrajPlanLaneChange.v_max_int
          * CalibrationVars->TrajPlanLaneChange.t_permit) {
        S_min_dyn = ACC(8.3333333333333339, 20.0, 200.0, ChassisInfo_speed, 0.0,
                        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                        CalibrationVars->ACC.d_wait2faultyCar,
                        CalibrationVars->ACC.tau_v_com,
                        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                        CalibrationVars->ACC.tau_v_bre,
                        CalibrationVars->ACC.tau_v_emg,
                        CalibrationVars->ACC.tau_d_emg,
                        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
      } else {
        S_min_dyn = 100.0;
      }
    } else {
      S_min_dyn = ACC(BasicsInfo_v_max, 0.0, BasicsInfo_d_veh2goal +
                      CalibrationVars->ACC.d_wait, ChassisInfo_speed, 1.0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.d_wait2faultyCar,
                      CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                      CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                      CalibrationVars->ACC.tau_v_emg,
                      CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                      CalibrationVars->ACC.d_wait);
      if ((BasicsInfo_d_veh2goal <= CalibrationVars->Decider.d_veh2endpoint) &&
          (ChassisInfo_speed < 0.2)) {
        decision_states = 1;
      }
    }
  } else {
    S_min_dyn = 100.0;
  }

  a_soll_ACC = fmin(a_soll_Fail, a_soll_ACC);

  /*  a_soll_matrix=[a_soll_SpeedPlanAvoidPedestrian,a_soll_SpeedPlanAvoidVehicle,a_soll_SpeedPlanAvoidOncomingVehicle,a_soll_TrafficLightActive,a_soll_StopSign,a_soll_ACC,a_soll_veh2goal,accel_speedlimit]; */
  a_soll_matrix[0] = a_soll_SpeedPlanAvoidPedestrian;
  a_soll_matrix[1] = a_soll_SpeedPlanAvoidVehicle;
  a_soll_matrix[2] = c_a_soll_SpeedPlanAvoidOncoming;
  a_soll_matrix[3] = a_soll_TrafficLightActive;
  a_soll_matrix[4] = a_sollTurnAround2Decider;
  a_soll_matrix[5] = a_soll_StopSign;
  a_soll_matrix[6] = a_soll_ACC;
  a_soll_matrix[7] = S_min_dyn;
  a_soll_matrix[8] = S_max;
  a_soll_StopSign = h_minimum(a_soll_matrix);
  TargetLaneFrontVel = a_soll_StopSign;
  if (ChassisInfo_speed <= 0.0) {
    a_soll_StopSign = fmax(0.0, a_soll_StopSign);
  }

  /* 跟车安全距离 */
  if (GlobVars->Decider.dec_follow == 0) {
    /* 跟车减速指示的决策 */
    if ((BasicsInfo_currentLaneFrontVel * BasicsInfo_currentLaneFrontVel -
         ChassisInfo_speed * ChassisInfo_speed) * 0.5 /
        CalibrationVars->Decider.a_bre + CalibrationVars->ACC.d_wait >
        CurrentLaneFrontDis) {
      dec_follow = 1;
    }
  } else if ((BasicsInfo_currentLaneFrontVel * BasicsInfo_currentLaneFrontVel -
              ChassisInfo_speed * ChassisInfo_speed) * 0.5 /
             CalibrationVars->Decider.a_bre_com + CalibrationVars->ACC.d_wait <
             CurrentLaneFrontDis) {
    dec_follow = 0;
  }

  if (PlannerLevel == 2) {
    short b_a_soll_index;
    dir_start = 9;
    b_a_soll_index = 9;
    exitg1 = false;
    while ((!exitg1) && (b_a_soll_index > 0)) {
      dir_start = b_a_soll_index;

      /* 从后往前查 */
      if (a_soll_matrix[b_a_soll_index - 1] == a_soll_StopSign) {
        exitg1 = true;
      } else {
        b_a_soll_index--;
      }
    }

    if (dir_start == 1) {
      if ((a_soll_StopSign <= -0.2) ||
          (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1)) {
        SlowDown = 1;
      }
    } else if (dir_start == 2) {
      if ((a_soll_StopSign <= -0.2) ||
          (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1)) {
        SlowDown = 2;
      }
    } else if (dir_start == 3) {
      if ((a_soll_StopSign <= -0.2) ||
          (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle ==
           1)) {
        SlowDown = 3;
      }
    } else if (dir_start == 4) {
      if ((a_soll_StopSign <= -0.2) &&
          (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
        SlowDown = 4;
      }
    } else if (dir_start == 5) {
      if (a_soll_StopSign <= -0.2) {
        SlowDown = 5;
      }
    } else if (dir_start == 6) {
      if (a_soll_StopSign <= -0.2) {
        SlowDown = 6;
      }
    } else if (dir_start == 7) {
      if (dec_follow == 1) {
        SlowDown = 7;
      }
    } else if (dir_start == 8) {
      if (a_soll_StopSign <= -0.2) {
        SlowDown = 8;
      }
    } else if (a_soll_StopSign <= -0.2) {
      SlowDown = 9;
    }

    /* TargetSpeed */
    if (Wait > 0) {
      SlowDown = 0;
      GlobVars->Decider.a_soll_pre = 100.0;

      /*  GlobVars.Decider.a_soll_pre的初始值为100 */
    } else if ((PedestrianActive != 0) || (TrafficLightActive != 0) ||
               (VehicleCrossingActive != 0) || (VehicleOncomingActive != 0) ||
               (Decision->TurnAroundState != 0) || (SlowDown == 7) ||
               (Decision->StopSignState != 0) || (Decision->PullOverState != 0))
    {
      /* 跟车减速提示时，停车让行时，靠边停车时 */
      /*          TargetSpeed=(speed+a_soll*SampleTime); */
      if (GlobVars->Decider.a_soll_pre != 100.0) {
        if (a_soll_StopSign > -2.0) {
          b_S_b_end[0] = GlobVars->Decider.a_soll_pre +
            CalibrationVars->UrbanPlanner.jerkLimit * BasicsInfo_sampleTime;
          b_S_b_end[1] = a_soll_StopSign;
          b_S_b_end[2] = GlobVars->Decider.a_soll_pre -
            CalibrationVars->UrbanPlanner.jerkLimit * BasicsInfo_sampleTime;
          TargetLaneFrontVel = median(b_S_b_end);
        } else {
          b_S_b_end[0] = GlobVars->Decider.a_soll_pre +
            CalibrationVars->UrbanPlanner.jerkLimit * BasicsInfo_sampleTime;
          b_S_b_end[1] = a_soll_StopSign;
          b_S_b_end[2] = GlobVars->Decider.a_soll_pre - 2.5 *
            CalibrationVars->UrbanPlanner.jerkLimit * BasicsInfo_sampleTime;
          TargetLaneFrontVel = median(b_S_b_end);
        }
      } else {
        TargetLaneFrontVel = a_soll_StopSign;
      }

      TargetSpeed = fmax(0.0, ChassisInfo_speed + TargetLaneFrontVel *
                         BasicsInfo_sampleTime);
      GlobVars->Decider.a_soll_pre = TargetLaneFrontVel;

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
    TargetLaneBehindVel = -34.722222222222229 /
      CalibrationVars->Decider.a_bre_com;
    if ((a_soll_StopSign < -0.2) && (Wait == 0)) {
      /* wait时速度默认值 */
      dir_start = 9;
      exitg1 = false;
      while ((!exitg1) && (dir_start > 0)) {
        /* 从后往前查 */
        if (a_soll_matrix[dir_start - 1] == a_soll_StopSign) {
          SlowDown = dir_start;
          exitg1 = true;
        } else {
          dir_start--;
        }
      }

      if (SlowDown == 7) {
        /* followcar */
        if ((BasicsInfo_currentLaneFrontVel <= 0.1) && (CurrentLaneFrontDis >=
             dist_wait)) {
          if (CurrentLaneFrontDis <= S_min_dyn + dist_wait) {
            TargetVelocity *= 3.6;
            TargetVelocity = rt_roundd_snf(TargetVelocity);

            /* km/h */
          } else if (CurrentLaneFrontDis <= (S_min_dyn + S_max) + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if (CurrentLaneFrontDis <= ((S_min_dyn + S_max) +
                      TargetLaneBehindVel) + dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else {
          TargetVelocity = BasicsInfo_currentLaneFrontVel * 3.6;

          /* km/h */
        }
      } else if (SlowDown == 9) {
        /* Speedlimit */
        if (ChassisInfo_speed >= BasicsInfo_v_max * 1.1) {
          /* 超过限速%10 */
          TargetVelocity = BasicsInfo_v_max * 3.6;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 6) {
        if (StopSignInfo.d_veh2stopline <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (StopSignInfo.d_veh2stopline <= (S_min_dyn + S_max) +
                   dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (StopSignInfo.d_veh2stopline <= ((S_min_dyn + S_max) +
                    TargetLaneBehindVel) + dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 2) {
        /* Ramp */
        /*          if wait_AvoidVehicle==1 */
        if (d_veh2Rampstopline <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (d_veh2Rampstopline <= (S_min_dyn + S_max) + dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (d_veh2Rampstopline <= ((S_min_dyn + S_max) +
                    TargetLaneBehindVel) + dist_wait) {
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
        if (d_veh2waitingArea <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (d_veh2waitingArea <= (S_min_dyn + S_max) + dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (d_veh2waitingArea <= ((S_min_dyn + S_max) +
                    TargetLaneBehindVel) + dist_wait) {
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
                      TargetLaneBehindVel) + dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else if (d_veh2cross <= (S_min_dyn + S_max) + dist_wait) {
          /* 人行道限速 */
          TargetVelocity = 30.0;

          /* km/h */
        } else if (d_veh2cross <= ((S_min_dyn + S_max) + TargetLaneBehindVel) +
                   dist_wait) {
          TargetVelocity = 40.0;

          /* km/h */
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 4) {
        /* trafficLight */
        if (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1) {
          if (d_veh2Intstopline <= S_min_dyn + dist_wait) {
            TargetVelocity *= 3.6;
            TargetVelocity = rt_roundd_snf(TargetVelocity);

            /* km/h */
          } else if ((d_veh2Intstopline <= (S_min_dyn + S_max) + dist_wait) &&
                     (ChassisInfo_speed >= 8.88888888888889)) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if ((d_veh2Intstopline <= ((S_min_dyn + S_max) +
                       TargetLaneBehindVel) + dist_wait) && (ChassisInfo_speed >=
                      11.666666666666666)) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else if (ChassisInfo_speed >= 8.88888888888889) {
          if (d_veh2Intstopline <= (S_min_dyn + S_max) + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if ((d_veh2Intstopline <= ((S_min_dyn + S_max) +
                       TargetLaneBehindVel) + dist_wait) && (ChassisInfo_speed >=
                      11.666666666666666)) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        } else {
          TargetVelocity = -20.0;
        }
      } else if (SlowDown == 8) {
        /* 靠边停车 */
        if (d_veh2goal <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else if (d_veh2goal <= (S_min_dyn + S_max) + dist_wait) {
          TargetVelocity = 30.0;

          /* km/h */
        } else if (d_veh2goal <= ((S_min_dyn + S_max) + TargetLaneBehindVel) +
                   dist_wait) {
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
    } else if ((CurrentLaneFrontDis < (BasicsInfo_currentLaneFrontVel *
                 BasicsInfo_currentLaneFrontVel - ChassisInfo_speed *
                 ChassisInfo_speed) * 0.5 / CalibrationVars->Decider.a_bre) &&
               (Wait == 0)) {
      /* wait时速度默认值 */
      SlowDown = 7;
      if ((BasicsInfo_currentLaneFrontVel <= 0.1) && (CurrentLaneFrontDis >=
           dist_wait)) {
        if (CurrentLaneFrontDis <= S_min_dyn + dist_wait) {
          TargetVelocity *= 3.6;
          TargetVelocity = rt_roundd_snf(TargetVelocity);

          /* km/h */
        } else {
          a = S_min_dyn + S_max;
          if (CurrentLaneFrontDis <= a + dist_wait) {
            TargetVelocity = 30.0;

            /* km/h */
          } else if (CurrentLaneFrontDis <= (a + TargetLaneBehindVel) +
                     dist_wait) {
            TargetVelocity = 40.0;

            /* km/h */
          } else {
            TargetVelocity = -20.0;
          }
        }
      } else {
        TargetVelocity = BasicsInfo_currentLaneFrontVel * 3.6;
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
  } else if ((GlobVars->Decider.dir_start == 1) && (ChassisInfo_speed >=
              CalibrationVars->Decider.idle_speed / 3.6)) {
    dec_start = 0;
  }

  if (dec_start == 1) {
    if ((a_soll_StopSign > 0.0) && (Wait == 0)) {
      dir_start = 1;
    } else {
      dir_start = 0;
    }
  } else {
    dir_start = 0;
  }

  Decision->Start = dir_start;

  /* AEBActive */
  if (AEBActive > 0) {
    Decision->LaneChange = 0;
    SlowDown = 0;
    TargetVelocity = -20.0;
    TargetSpeed = -20.0;
    Wait = 0;
    S_b_end = 200.0;
    Decision->Start = 0;
  }

  /* 精度 */
  TargetVelocity = rt_roundd_snf(TargetVelocity);

  /* km/h */
  /* m/s */
  /* m */
  /* output */
  if (PlannerLevel == 2) {
    Decision->TargetSpeed = rt_roundd_snf(100.0 * TargetSpeed) / 100.0;

    /* m/s */
  } else {
    Decision->TargetSpeed = TargetVelocity / 3.6;

    /* m/s */
  }

  Decision->states = decision_states;
  Decision->Wait = Wait;
  Decision->WaitDistance = rt_roundd_snf(10.0 * S_b_end) / 10.0 + d_veh2goal_tmp;

  /* m 车中心到停止线距离 */
  Decision->SlowDown = SlowDown;
  Decision->AEBactive = AEBActive;
  Decision->TargetGear = TargetGear;
  Decision->a_soll = TargetLaneFrontVel;

  /* 全局变量 */
  GlobVars->Decider.dec_start = dec_start;
  GlobVars->Decider.dir_start = dir_start;
  GlobVars->Decider.wait_pullover = wait_pullover;
  GlobVars->Decider.distBehindGoal = distBehindGoal;
  GlobVars->Decider.dec_follow = dec_follow;
  GlobVars->Decider.countLaneChangeDecider = CountLaneChange;
  GlobVars->Decider.currentTargetLaneIndexDecider = CurrentTargetLaneIndex;
}

/*
 * Arguments    : double a_sollpre2traj
 *                double SampleTime
 *                double *a_soll
 *                double c_CalibrationVars_UrbanPlanner_
 * Return Type  : void
 */
static void JerkLimit(double a_sollpre2traj, double SampleTime, double *a_soll,
                      double c_CalibrationVars_UrbanPlanner_)
{
  if (a_sollpre2traj != 100.0) {
    if (*a_soll > -2.0) {
      double b_a_sollpre2traj[3];
      b_a_sollpre2traj[0] = a_sollpre2traj + c_CalibrationVars_UrbanPlanner_ *
        SampleTime;
      b_a_sollpre2traj[1] = *a_soll;
      b_a_sollpre2traj[2] = a_sollpre2traj - c_CalibrationVars_UrbanPlanner_ *
        SampleTime;
      *a_soll = median(b_a_sollpre2traj);
    } else {
      double b_a_sollpre2traj[3];
      b_a_sollpre2traj[0] = a_sollpre2traj + c_CalibrationVars_UrbanPlanner_ *
        SampleTime;
      b_a_sollpre2traj[1] = *a_soll;
      b_a_sollpre2traj[2] = a_sollpre2traj - 2.5 *
        c_CalibrationVars_UrbanPlanner_ * SampleTime;
      *a_soll = median(b_a_sollpre2traj);
    }
  }
}

/*
 * 输出车道中心线[对向1车道中心L坐标，对向2车道中心L坐标，对向3车道中心L坐标，对向4车道中心L坐标，对向5车道中心L坐标，对向6车道中心L坐标，当前车道中心线L坐标]
 *
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
  double y;
  int b_i;
  int i;
  int k;
  i = -CurrentLane;
  if (-CurrentLane > 32767) {
    i = 32767;
  }

  WidthOfLanes[0] = WidthOfLaneCurrent;
  WidthOfLanes[1] = WidthOfGap;
  for (b_i = 0; b_i < 6; b_i++) {
    WidthOfLanes[b_i + 2] = WidthOfLanesOpposite[b_i];
  }

  memset(&Lane_boundary[0], 0, 24U * sizeof(double));
  for (b_i = 0; b_i < 7; b_i++) {
    LaneCenterline[b_i] = 0.0;
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
    Lane_boundary[b_i + 16] = b_i + (short)i;
  }

  y = (WidthOfLanes[0] * 0.5 + pos_l_CurrentLane) - Lane_boundary[0];
  for (b_i = 0; b_i < 8; b_i++) {
    double d;
    double d1;
    d = Lane_boundary[b_i] + y;
    Lane_boundary[b_i + 8] = d;
    d1 = d - WidthOfLanes[b_i];
    Lane_boundary[b_i] = d1;
    WidthOfLanes[b_i] = 0.5 * (d + d1);
  }

  i = NumOfLanesOpposite + 2;
  if (NumOfLanesOpposite + 2 > 32767) {
    i = 32767;
  }

  if ((short)i < 3) {
    b_i = 0;
    i = 0;
  } else {
    b_i = 2;
    i = (short)i;
  }

  k = i - b_i;
  for (i = 0; i < k; i++) {
    LaneCenterline[i] = WidthOfLanes[b_i + i];
  }

  LaneCenterline[k] = WidthOfLanes[0];
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
    Lane_boundary[b_i + 16] = b_i + (short)i;
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

  return LaneIndexOfPoint;
}

/*
 * NumOfLanes=2; % 车道数量
 *  LanesWithFail=[1 2]; % 故障车所在车道序号，车道序号自小到大排列，最左侧车道序号为1
 *  TargetLaneIndex=2; % 原目标车道
 *  CurrentLaneIndex=1; 本车所在车道
 *
 * Arguments    : const double WidthOfLanes[6]
 *                const short LanesWithFail[6]
 *                short *TargetLaneIndex
 *                short CurrentLaneIndex
 * Return Type  : short
 */
static short LaneSelectionWithBlockedLanes(const double WidthOfLanes[6], const
  short LanesWithFail[6], short *TargetLaneIndex, short CurrentLaneIndex)
{
  emxArray_boolean_T *b_x;
  emxArray_int8_T *Neighbor2Current;
  emxArray_int8_T *ii;
  emxArray_int8_T *r;
  emxArray_uint16_T *LaneInfos;
  int i;
  int idx;
  int last;
  int nx;
  int nz;
  short BackupTargetLaneIndex;
  unsigned short *LaneInfos_data;
  signed char *Neighbor2Current_data;
  signed char *ii_data;
  boolean_T x[6];
  boolean_T guard1 = false;
  boolean_T *x_data;
  for (i = 0; i < 6; i++) {
    x[i] = (WidthOfLanes[i] != 0.0);
  }

  nz = x[0];
  for (idx = 0; idx < 5; idx++) {
    nz += x[idx + 1];
  }

  emxInit_uint16_T(&LaneInfos);
  i = LaneInfos->size[0] * LaneInfos->size[1];
  LaneInfos->size[0] = nz;
  LaneInfos->size[1] = 2;
  emxEnsureCapacity_uint16_T(LaneInfos, i);
  LaneInfos_data = LaneInfos->data;
  last = nz << 1;
  for (i = 0; i < last; i++) {
    LaneInfos_data[i] = 0U;
  }

  /*  Failfalg Dis2Tar+Dis2Cur */
  for (idx = 0; idx < 6; idx++) {
    short i1;
    i1 = LanesWithFail[idx];
    if (i1 != 0) {
      LaneInfos_data[i1 - 1] = 1U;
    }
  }

  for (idx = 0; idx < nz; idx++) {
    i = (idx - *TargetLaneIndex) + 1;
    if (i > 32767) {
      i = 32767;
    }

    last = (idx - CurrentLaneIndex) + 1;
    if (last > 32767) {
      last = 32767;
    }

    if ((short)i < 0) {
      i = (short)-(short)i;
    } else {
      i = (short)i;
    }

    if ((short)last < 0) {
      last = (short)-(short)last;
    } else {
      last = (short)last;
    }

    i += last;
    if (i > 32767) {
      i = 32767;
    } else if (i < -32768) {
      i = -32768;
    }

    LaneInfos_data[idx + LaneInfos->size[0]] = (unsigned short)i;

    /*      LaneInfos(i,2)=abs(i-TargetLaneIndex); */
    /*      LaneInfos(i,3)=abs(i-CurrentLaneIndex); */
  }

  emxInit_int8_T(&Neighbor2Current, 2);
  Neighbor2Current_data = Neighbor2Current->data;
  Neighbor2Current->size[0] = 0;
  Neighbor2Current->size[1] = 0;
  emxInit_boolean_T(&b_x);
  i = b_x->size[0];
  b_x->size[0] = LaneInfos->size[0];
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  last = LaneInfos->size[0];
  for (i = 0; i < last; i++) {
    x_data[i] = (LaneInfos_data[i] == 0);
  }

  nx = b_x->size[0] - 1;
  last = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (x_data[idx]) {
      last++;
    }
  }

  emxInit_int8_T(&ii, 1);
  i = ii->size[0];
  ii->size[0] = last;
  emxEnsureCapacity_int8_T(ii, i);
  ii_data = ii->data;
  last = 0;
  for (idx = 0; idx <= nx; idx++) {
    if (x_data[idx]) {
      ii_data[last] = (signed char)(idx + 1);
      last++;
    }
  }

  if (ii->size[0] != 0) {
    boolean_T exitg1;
    i = b_x->size[0];
    b_x->size[0] = LaneInfos->size[0];
    emxEnsureCapacity_boolean_T(b_x, i);
    x_data = b_x->data;
    last = LaneInfos->size[0];
    for (i = 0; i < last; i++) {
      x_data[i] = (LaneInfos_data[i] == 0);
    }

    nx = b_x->size[0] - 1;
    last = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (x_data[idx]) {
        last++;
      }
    }

    emxInit_int8_T(&r, 1);
    i = r->size[0];
    r->size[0] = last;
    emxEnsureCapacity_int8_T(r, i);
    Neighbor2Current_data = r->data;
    last = 0;
    for (idx = 0; idx <= nx; idx++) {
      if (x_data[idx]) {
        Neighbor2Current_data[last] = (signed char)(idx + 1);
        last++;
      }
    }

    last = r->size[0];
    if (r->size[0] <= 2) {
      if (r->size[0] == 1) {
        nx = LaneInfos_data[(Neighbor2Current_data[0] + LaneInfos->size[0]) - 1];
      } else if (LaneInfos_data[(Neighbor2Current_data[0] + LaneInfos->size[0])
                 - 1] > LaneInfos_data[(Neighbor2Current_data[r->size[0] - 1] +
                  LaneInfos->size[0]) - 1]) {
        nx = LaneInfos_data[(Neighbor2Current_data[r->size[0] - 1] +
                             LaneInfos->size[0]) - 1];
      } else {
        nx = LaneInfos_data[(Neighbor2Current_data[0] + LaneInfos->size[0]) - 1];
      }
    } else {
      nx = LaneInfos_data[(Neighbor2Current_data[0] + LaneInfos->size[0]) - 1];
      for (idx = 2; idx <= last; idx++) {
        i = LaneInfos_data[(Neighbor2Current_data[idx - 1] + LaneInfos->size[0])
          - 1];
        if (nx > i) {
          nx = i;
        }
      }
    }

    emxFree_int8_T(&r);
    i = b_x->size[0];
    b_x->size[0] = LaneInfos->size[0];
    emxEnsureCapacity_boolean_T(b_x, i);
    x_data = b_x->data;
    last = LaneInfos->size[0];
    for (i = 0; i < last; i++) {
      x_data[i] = ((LaneInfos_data[i + LaneInfos->size[0]] == nx) &&
                   (LaneInfos_data[i] == 0));
    }

    nx = b_x->size[0];
    idx = 0;
    i = ii->size[0];
    ii->size[0] = b_x->size[0];
    emxEnsureCapacity_int8_T(ii, i);
    ii_data = ii->data;
    last = 0;
    exitg1 = false;
    while ((!exitg1) && (last <= nx - 1)) {
      if (x_data[last]) {
        idx++;
        ii_data[idx - 1] = (signed char)(last + 1);
        if (idx >= nx) {
          exitg1 = true;
        } else {
          last++;
        }
      } else {
        last++;
      }
    }

    if (b_x->size[0] == 1) {
      if (idx == 0) {
        ii->size[0] = 0;
      }
    } else {
      i = ii->size[0];
      if (idx < 1) {
        ii->size[0] = 0;
      } else {
        ii->size[0] = idx;
      }

      emxEnsureCapacity_int8_T(ii, i);
      ii_data = ii->data;
    }

    i = Neighbor2Current->size[0] * Neighbor2Current->size[1];
    Neighbor2Current->size[0] = ii->size[0];
    Neighbor2Current->size[1] = 1;
    emxEnsureCapacity_int8_T(Neighbor2Current, i);
    Neighbor2Current_data = Neighbor2Current->data;
    last = ii->size[0];
    for (i = 0; i < last; i++) {
      Neighbor2Current_data[i] = ii_data[i];
    }
  }

  emxFree_int8_T(&ii);
  emxFree_boolean_T(&b_x);

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
  guard1 = false;
  if ((Neighbor2Current->size[0] != 0) && (Neighbor2Current->size[1] != 0)) {
    if ((Neighbor2Current->size[0] == 2) || (Neighbor2Current->size[0] == 1)) {
      *TargetLaneIndex = Neighbor2Current_data[0];
    }

    if (Neighbor2Current->size[0] == 2) {
      BackupTargetLaneIndex = Neighbor2Current_data[1];
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 && (*TargetLaneIndex > CurrentLaneIndex)) {
    i = CurrentLaneIndex - 1;
    if (CurrentLaneIndex - 1 < -32768) {
      i = -32768;
    }

    if (i >= 1) {
      i = CurrentLaneIndex - 1;
      if (CurrentLaneIndex - 1 < -32768) {
        i = -32768;
      }

      if (LaneInfos_data[i - 1] == 0) {
        i = CurrentLaneIndex - 1;
        if (CurrentLaneIndex - 1 < -32768) {
          i = -32768;
        }

        BackupTargetLaneIndex = (short)i;
      }
    }
  }

  emxFree_int8_T(&Neighbor2Current);
  if ((BackupTargetLaneIndex == -1) && (*TargetLaneIndex < CurrentLaneIndex)) {
    i = CurrentLaneIndex + 1;
    if (CurrentLaneIndex + 1 > 32767) {
      i = 32767;
    }

    if (i <= nz) {
      i = CurrentLaneIndex + 1;
      if (CurrentLaneIndex + 1 > 32767) {
        i = 32767;
      }

      if (LaneInfos_data[i - 1] == 0) {
        i = CurrentLaneIndex + 1;
        if (CurrentLaneIndex + 1 > 32767) {
          i = 32767;
        }

        BackupTargetLaneIndex = (short)i;
      }
    }
  }

  emxFree_uint16_T(&LaneInfos);
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
 *                double l_veh
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
  dec2line, double l_veh, double R1[2], double R2[2], double R3[2], double
  pos_start[2], double p1[4], double p2[4], double pos_end[2])
{
  double a;
  double alph;
  double alph_tmp;
  double l_p1;
  double l_p2;
  double lr1;
  double lr2;
  double lr3;
  double s_p2;
  double sr1_tmp;
  double sr2;
  double sr3;
  l_boundry -= dec2line;
  sr1_tmp = ((s_turnround_border - D_safe) - w_veh / 2.0) - R;
  lr1 = l_current + R;
  a = 0.5 * l_veh;
  alph_tmp = R * R;
  alph = 57.295779513082323 * acos((l_boundry - lr1) / -sqrt(alph_tmp + a * a))
    - 57.295779513082323 * atan(0.5 * l_veh / R);
  a = alph;
  b_cosd(&a);
  l_p1 = lr1 - R * a;
  b_sind(&alph);
  alph = sr1_tmp + R * alph;

  /*  if l_boundry-lr1>0 */
  /*      l_p1= lr1+(l_boundry-lr1)*(R/(R+w_veh/2)); */
  /*      s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2); */
  /*  elseif l_boundry-lr1<0 */
  /*      l_p1= lr1+(l_boundry-lr1)*((R+w_veh/2)/R); */
  /*      s_p1=sr1+sqrt(R^2-(l_p1-lr1)^2); */
  /*  else */
  /*      l_p1=lr1+R; */
  /*      s_p1=sr1; */
  /*  end */
  sr2 = 2.0 * alph - sr1_tmp;
  lr2 = 2.0 * l_p1 - lr1;
  lr3 = l_targetlane - R;
  a = lr2 - lr3;
  sr3 = sr2 - sqrt(4.0 * alph_tmp - a * a);
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
  p1[0] = alph;
  p1[1] = l_p1;
  p1[2] = atan((l_p1 - lr1) / (alph - sr1_tmp));
  p1[3] = 0.0;

  /*  theta2=-theta2; */
  p2[0] = s_p2;
  p2[1] = l_p2;
  p2[2] = atan((l_p2 - lr2) / (s_p2 - sr2));
  p2[3] = 0.0;
  pos_start[0] = sr1_tmp;
  pos_start[1] = l_current;
  pos_end[0] = sr3 + sqrt(alph_tmp - a * a);
  pos_end[1] = l_targetlane;
}

/*
 * 调用时LaneCenterline(TargetLaneIndexOpposite)代替LaneCenterlineTargetLane
 *
 * Arguments    : double LaneCenterlineTargetLane
 *                const double PosCircle1[2]
 *                const double PosCircle2[2]
 *                double TurningRadius
 *                double pos_s
 *                double *NumRefLaneTurnAround
 *                emxArray_real_T *SRefLaneTurnAround
 *                emxArray_real_T *LRefLaneTurnAround
 *                double *SEnd
 * Return Type  : void
 */
static void PathPlanTurnAroundDecider(double LaneCenterlineTargetLane, const
  double PosCircle1[2], const double PosCircle2[2], double TurningRadius, double
  pos_s, double *NumRefLaneTurnAround, emxArray_real_T *SRefLaneTurnAround,
  emxArray_real_T *LRefLaneTurnAround, double *SEnd)
{
  emxArray_int8_T *LRefLaneTurnAroundBefore;
  emxArray_real_T *LRefLaneTurnAroundDuring1;
  emxArray_real_T *LRefLaneTurnAroundDuring2;
  emxArray_real_T *LRefLaneTurnAroundDuring3;
  emxArray_real_T *LRefLaneTurnAroundTransition;
  emxArray_real_T *SRefLaneTurnAroundBefore;
  emxArray_real_T *SRefLaneTurnAroundDuring1;
  emxArray_real_T *SRefLaneTurnAroundDuring2;
  emxArray_real_T *SRefLaneTurnAroundDuring3;
  emxArray_real_T *SRefLaneTurnAroundTransition;
  double dv[5];
  double NumRefLaneTurnAroundBefore;
  double NumRefLaneTurnAroundDuring1_tmp;
  double targetAngle;
  double *LRefLaneTurnAroundDuring1_data;
  double *LRefLaneTurnAroundDuring2_data;
  double *LRefLaneTurnAroundDuring3_data;
  double *SRefLaneTurnAroundBefore_data;
  double *SRefLaneTurnAroundDuring1_data;
  double *SRefLaneTurnAroundDuring2_data;
  double *SRefLaneTurnAroundDuring3_data;
  double *SRefLaneTurnAround_data;
  double *c_LRefLaneTurnAroundTransition_;
  double *c_SRefLaneTurnAroundTransition_;
  int b_input_sizes_idx_1;
  int b_unnamed_idx_1;
  int c_input_sizes_idx_1;
  int c_unnamed_idx_1;
  int d_input_sizes_idx_1;
  int e_input_sizes_idx_1;
  int f_input_sizes_idx_1;
  int g_input_sizes_idx_1;
  int i;
  int i1;
  int input_sizes_idx_1;
  int unnamed_idx_1;
  signed char *LRefLaneTurnAroundBefore_data;
  targetAngle = PosCircle1[0] - pos_s;
  NumRefLaneTurnAroundBefore = rt_roundd_snf(targetAngle / 5.0);
  emxInit_real_T(&SRefLaneTurnAroundBefore, 2);
  b_linspace(pos_s, PosCircle1[0] - targetAngle / NumRefLaneTurnAroundBefore,
             NumRefLaneTurnAroundBefore, SRefLaneTurnAroundBefore);
  SRefLaneTurnAroundBefore_data = SRefLaneTurnAroundBefore->data;
  emxInit_int8_T(&LRefLaneTurnAroundBefore, 2);
  if (!(NumRefLaneTurnAroundBefore >= 0.0)) {
    LRefLaneTurnAroundBefore->size[1] = 0;
  } else {
    i = LRefLaneTurnAroundBefore->size[0] * LRefLaneTurnAroundBefore->size[1];
    LRefLaneTurnAroundBefore->size[0] = 1;
    LRefLaneTurnAroundBefore->size[1] = (int)NumRefLaneTurnAroundBefore;
    emxEnsureCapacity_int8_T(LRefLaneTurnAroundBefore, i);
    LRefLaneTurnAroundBefore_data = LRefLaneTurnAroundBefore->data;
    if ((int)NumRefLaneTurnAroundBefore >= 1) {
      LRefLaneTurnAroundBefore_data[(int)NumRefLaneTurnAroundBefore - 1] = 0;
      if (LRefLaneTurnAroundBefore->size[1] >= 3) {
        i = LRefLaneTurnAroundBefore->size[1] - 1;
        for (input_sizes_idx_1 = 2; input_sizes_idx_1 <= i; input_sizes_idx_1++)
        {
          LRefLaneTurnAroundBefore_data[input_sizes_idx_1 - 1] = 0;
        }

        if ((LRefLaneTurnAroundBefore->size[1] & 1) == 1) {
          LRefLaneTurnAroundBefore_data[LRefLaneTurnAroundBefore->size[1] >> 1] =
            0;
        }
      }
    }
  }

  NumRefLaneTurnAroundDuring1_tmp = rt_roundd_snf(TurningRadius * 0.5 *
    3.1415926535897931 / 0.5);
  emxInit_real_T(&SRefLaneTurnAroundDuring1, 2);
  i = SRefLaneTurnAroundDuring1->size[0] * SRefLaneTurnAroundDuring1->size[1];
  SRefLaneTurnAroundDuring1->size[0] = 1;
  input_sizes_idx_1 = (int)NumRefLaneTurnAroundDuring1_tmp;
  SRefLaneTurnAroundDuring1->size[1] = (int)NumRefLaneTurnAroundDuring1_tmp;
  emxEnsureCapacity_real_T(SRefLaneTurnAroundDuring1, i);
  SRefLaneTurnAroundDuring1_data = SRefLaneTurnAroundDuring1->data;
  for (i = 0; i < input_sizes_idx_1; i++) {
    SRefLaneTurnAroundDuring1_data[i] = 0.0;
  }

  emxInit_real_T(&LRefLaneTurnAroundDuring1, 2);
  i = LRefLaneTurnAroundDuring1->size[0] * LRefLaneTurnAroundDuring1->size[1];
  LRefLaneTurnAroundDuring1->size[0] = 1;
  LRefLaneTurnAroundDuring1->size[1] = (int)NumRefLaneTurnAroundDuring1_tmp;
  emxEnsureCapacity_real_T(LRefLaneTurnAroundDuring1, i);
  LRefLaneTurnAroundDuring1_data = LRefLaneTurnAroundDuring1->data;
  for (i = 0; i < input_sizes_idx_1; i++) {
    LRefLaneTurnAroundDuring1_data[i] = 0.0;
  }

  for (b_input_sizes_idx_1 = 0; b_input_sizes_idx_1 < input_sizes_idx_1;
       b_input_sizes_idx_1++) {
    targetAngle = (((double)b_input_sizes_idx_1 + 1.0) - 1.0) *
      3.1415926535897931 / 2.0 / NumRefLaneTurnAroundDuring1_tmp -
      1.5707963267948966;
    SRefLaneTurnAroundDuring1_data[b_input_sizes_idx_1] = PosCircle1[0] + cos
      (targetAngle) * TurningRadius;
    LRefLaneTurnAroundDuring1_data[b_input_sizes_idx_1] = PosCircle1[1] + sin
      (targetAngle) * TurningRadius;
  }

  NumRefLaneTurnAroundBefore = PosCircle2[1] - PosCircle1[1];
  targetAngle = rt_roundd_snf(NumRefLaneTurnAroundBefore / 0.5);
  emxInit_real_T(&SRefLaneTurnAroundTransition, 2);
  c_SRefLaneTurnAroundTransition_ = SRefLaneTurnAroundTransition->data;
  SRefLaneTurnAroundTransition->size[0] = 0;
  SRefLaneTurnAroundTransition->size[1] = 0;
  emxInit_real_T(&SRefLaneTurnAroundDuring2, 2);
  SRefLaneTurnAroundDuring2_data = SRefLaneTurnAroundDuring2->data;
  SRefLaneTurnAroundDuring2->size[0] = 0;
  SRefLaneTurnAroundDuring2->size[1] = 0;
  emxInit_real_T(&LRefLaneTurnAroundDuring2, 2);
  LRefLaneTurnAroundDuring2_data = LRefLaneTurnAroundDuring2->data;
  LRefLaneTurnAroundDuring2->size[0] = 0;
  LRefLaneTurnAroundDuring2->size[1] = 0;
  emxInit_real_T(&LRefLaneTurnAroundTransition, 2);
  c_LRefLaneTurnAroundTransition_ = LRefLaneTurnAroundTransition->data;
  if (targetAngle != 0.0) {
    b_linspace(PosCircle1[0] + TurningRadius, PosCircle2[0] + TurningRadius,
               targetAngle, LRefLaneTurnAroundTransition);
    c_LRefLaneTurnAroundTransition_ = LRefLaneTurnAroundTransition->data;
    i = SRefLaneTurnAroundDuring2->size[0] * SRefLaneTurnAroundDuring2->size[1];
    SRefLaneTurnAroundDuring2->size[0] = 1;
    SRefLaneTurnAroundDuring2->size[1] = LRefLaneTurnAroundTransition->size[1];
    emxEnsureCapacity_real_T(SRefLaneTurnAroundDuring2, i);
    SRefLaneTurnAroundDuring2_data = SRefLaneTurnAroundDuring2->data;
    b_input_sizes_idx_1 = LRefLaneTurnAroundTransition->size[1];
    for (i = 0; i < b_input_sizes_idx_1; i++) {
      SRefLaneTurnAroundDuring2_data[i] = c_LRefLaneTurnAroundTransition_[i];
    }

    b_linspace(PosCircle1[1], PosCircle2[1] - NumRefLaneTurnAroundBefore /
               targetAngle, targetAngle, LRefLaneTurnAroundTransition);
    c_LRefLaneTurnAroundTransition_ = LRefLaneTurnAroundTransition->data;
    i = LRefLaneTurnAroundDuring2->size[0] * LRefLaneTurnAroundDuring2->size[1];
    LRefLaneTurnAroundDuring2->size[0] = 1;
    LRefLaneTurnAroundDuring2->size[1] = LRefLaneTurnAroundTransition->size[1];
    emxEnsureCapacity_real_T(LRefLaneTurnAroundDuring2, i);
    LRefLaneTurnAroundDuring2_data = LRefLaneTurnAroundDuring2->data;
    b_input_sizes_idx_1 = LRefLaneTurnAroundTransition->size[1];
    for (i = 0; i < b_input_sizes_idx_1; i++) {
      LRefLaneTurnAroundDuring2_data[i] = c_LRefLaneTurnAroundTransition_[i];
    }
  }

  emxInit_real_T(&SRefLaneTurnAroundDuring3, 2);
  i = SRefLaneTurnAroundDuring3->size[0] * SRefLaneTurnAroundDuring3->size[1];
  SRefLaneTurnAroundDuring3->size[0] = 1;
  SRefLaneTurnAroundDuring3->size[1] = (int)NumRefLaneTurnAroundDuring1_tmp;
  emxEnsureCapacity_real_T(SRefLaneTurnAroundDuring3, i);
  SRefLaneTurnAroundDuring3_data = SRefLaneTurnAroundDuring3->data;
  for (i = 0; i < input_sizes_idx_1; i++) {
    SRefLaneTurnAroundDuring3_data[i] = 0.0;
  }

  emxInit_real_T(&LRefLaneTurnAroundDuring3, 2);
  i = LRefLaneTurnAroundDuring3->size[0] * LRefLaneTurnAroundDuring3->size[1];
  LRefLaneTurnAroundDuring3->size[0] = 1;
  LRefLaneTurnAroundDuring3->size[1] = (int)NumRefLaneTurnAroundDuring1_tmp;
  emxEnsureCapacity_real_T(LRefLaneTurnAroundDuring3, i);
  LRefLaneTurnAroundDuring3_data = LRefLaneTurnAroundDuring3->data;
  for (i = 0; i < input_sizes_idx_1; i++) {
    LRefLaneTurnAroundDuring3_data[i] = 0.0;
  }

  for (b_input_sizes_idx_1 = 0; b_input_sizes_idx_1 < input_sizes_idx_1;
       b_input_sizes_idx_1++) {
    targetAngle = (((double)b_input_sizes_idx_1 + 1.0) - 1.0) *
      3.1415926535897931 / 2.0 / NumRefLaneTurnAroundDuring1_tmp;
    SRefLaneTurnAroundDuring3_data[b_input_sizes_idx_1] = PosCircle1[0] + cos
      (targetAngle) * TurningRadius;
    LRefLaneTurnAroundDuring3_data[b_input_sizes_idx_1] = PosCircle2[1] + sin
      (targetAngle) * TurningRadius;
  }

  NumRefLaneTurnAroundBefore = PosCircle2[1] + TurningRadius;
  targetAngle = rt_roundd_snf((NumRefLaneTurnAroundBefore -
    LaneCenterlineTargetLane) / 0.1);
  LRefLaneTurnAroundTransition->size[0] = 0;
  LRefLaneTurnAroundTransition->size[1] = 0;
  if (targetAngle != 0.0) {
    b_linspace(PosCircle2[0], (PosCircle2[0] - targetAngle) + 1.0, targetAngle,
               LRefLaneTurnAroundTransition);
    c_LRefLaneTurnAroundTransition_ = LRefLaneTurnAroundTransition->data;
    i = SRefLaneTurnAroundTransition->size[0] *
      SRefLaneTurnAroundTransition->size[1];
    SRefLaneTurnAroundTransition->size[0] = 1;
    SRefLaneTurnAroundTransition->size[1] = LRefLaneTurnAroundTransition->size[1];
    emxEnsureCapacity_real_T(SRefLaneTurnAroundTransition, i);
    c_SRefLaneTurnAroundTransition_ = SRefLaneTurnAroundTransition->data;
    b_input_sizes_idx_1 = LRefLaneTurnAroundTransition->size[1];
    for (i = 0; i < b_input_sizes_idx_1; i++) {
      c_SRefLaneTurnAroundTransition_[i] = c_LRefLaneTurnAroundTransition_[i];
    }

    b_linspace(NumRefLaneTurnAroundBefore, LaneCenterlineTargetLane + 0.1,
               targetAngle, LRefLaneTurnAroundTransition);
    c_LRefLaneTurnAroundTransition_ = LRefLaneTurnAroundTransition->data;
  }

  if ((SRefLaneTurnAroundDuring2->size[0] != 0) &&
      (SRefLaneTurnAroundDuring2->size[1] != 0)) {
    c_input_sizes_idx_1 = SRefLaneTurnAroundDuring2->size[1];
  } else {
    c_input_sizes_idx_1 = 0;
  }

  if (SRefLaneTurnAroundDuring3->size[1] != 0) {
    d_input_sizes_idx_1 = SRefLaneTurnAroundDuring3->size[1];
  } else {
    d_input_sizes_idx_1 = 0;
  }

  if ((SRefLaneTurnAroundTransition->size[0] != 0) &&
      (SRefLaneTurnAroundTransition->size[1] != 0)) {
    e_input_sizes_idx_1 = SRefLaneTurnAroundTransition->size[1];
  } else {
    e_input_sizes_idx_1 = 0;
  }

  *SEnd = PosCircle2[0] - targetAngle;
  c_linspace(*SEnd, *SEnd - 20.0, dv);
  if (SRefLaneTurnAroundBefore->size[1] != 0) {
    unnamed_idx_1 = SRefLaneTurnAroundBefore->size[1];
  } else {
    unnamed_idx_1 = 0;
  }

  if (SRefLaneTurnAroundDuring1->size[1] != 0) {
    b_unnamed_idx_1 = SRefLaneTurnAroundDuring1->size[1];
  } else {
    b_unnamed_idx_1 = 0;
  }

  if ((SRefLaneTurnAroundDuring2->size[0] != 0) &&
      (SRefLaneTurnAroundDuring2->size[1] != 0)) {
    c_unnamed_idx_1 = SRefLaneTurnAroundDuring2->size[1];
  } else {
    c_unnamed_idx_1 = 0;
  }

  if (SRefLaneTurnAroundDuring3->size[1] != 0) {
    f_input_sizes_idx_1 = SRefLaneTurnAroundDuring3->size[1];
  } else {
    f_input_sizes_idx_1 = 0;
  }

  if ((SRefLaneTurnAroundTransition->size[0] != 0) &&
      (SRefLaneTurnAroundTransition->size[1] != 0)) {
    g_input_sizes_idx_1 = SRefLaneTurnAroundTransition->size[1];
  } else {
    g_input_sizes_idx_1 = 0;
  }

  i = SRefLaneTurnAround->size[0] * SRefLaneTurnAround->size[1];
  SRefLaneTurnAround->size[0] = 1;
  i1 = unnamed_idx_1 + b_unnamed_idx_1;
  SRefLaneTurnAround->size[1] = (((i1 + c_input_sizes_idx_1) +
    d_input_sizes_idx_1) + e_input_sizes_idx_1) + 5;
  emxEnsureCapacity_real_T(SRefLaneTurnAround, i);
  SRefLaneTurnAround_data = SRefLaneTurnAround->data;
  for (i = 0; i < unnamed_idx_1; i++) {
    SRefLaneTurnAround_data[i] = SRefLaneTurnAroundBefore_data[i];
  }

  emxFree_real_T(&SRefLaneTurnAroundBefore);
  for (i = 0; i < b_unnamed_idx_1; i++) {
    SRefLaneTurnAround_data[i + unnamed_idx_1] =
      SRefLaneTurnAroundDuring1_data[i];
  }

  emxFree_real_T(&SRefLaneTurnAroundDuring1);
  for (i = 0; i < c_input_sizes_idx_1; i++) {
    SRefLaneTurnAround_data[(i + unnamed_idx_1) + b_unnamed_idx_1] =
      SRefLaneTurnAroundDuring2_data[i];
  }

  emxFree_real_T(&SRefLaneTurnAroundDuring2);
  for (i = 0; i < d_input_sizes_idx_1; i++) {
    SRefLaneTurnAround_data[((i + unnamed_idx_1) + b_unnamed_idx_1) +
      c_unnamed_idx_1] = SRefLaneTurnAroundDuring3_data[i];
  }

  emxFree_real_T(&SRefLaneTurnAroundDuring3);
  for (i = 0; i < e_input_sizes_idx_1; i++) {
    SRefLaneTurnAround_data[(((i + unnamed_idx_1) + b_unnamed_idx_1) +
      c_unnamed_idx_1) + f_input_sizes_idx_1] =
      c_SRefLaneTurnAroundTransition_[i];
  }

  emxFree_real_T(&SRefLaneTurnAroundTransition);
  for (i = 0; i < 5; i++) {
    SRefLaneTurnAround_data[(((i + i1) + c_unnamed_idx_1) + f_input_sizes_idx_1)
      + g_input_sizes_idx_1] = dv[i];
  }

  if (LRefLaneTurnAroundBefore->size[1] != 0) {
    c_input_sizes_idx_1 = LRefLaneTurnAroundBefore->size[1];
  } else {
    c_input_sizes_idx_1 = 0;
  }

  if (LRefLaneTurnAroundDuring1->size[1] != 0) {
    d_input_sizes_idx_1 = LRefLaneTurnAroundDuring1->size[1];
  } else {
    d_input_sizes_idx_1 = 0;
  }

  if ((LRefLaneTurnAroundDuring2->size[0] != 0) &&
      (LRefLaneTurnAroundDuring2->size[1] != 0)) {
    e_input_sizes_idx_1 = LRefLaneTurnAroundDuring2->size[1];
  } else {
    e_input_sizes_idx_1 = 0;
  }

  if (LRefLaneTurnAroundDuring3->size[1] != 0) {
    input_sizes_idx_1 = LRefLaneTurnAroundDuring3->size[1];
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((LRefLaneTurnAroundTransition->size[0] != 0) &&
      (LRefLaneTurnAroundTransition->size[1] != 0)) {
    b_input_sizes_idx_1 = LRefLaneTurnAroundTransition->size[1];
  } else {
    b_input_sizes_idx_1 = 0;
  }

  c_linspace(LaneCenterlineTargetLane, LaneCenterlineTargetLane, dv);
  if (LRefLaneTurnAroundBefore->size[1] != 0) {
    unnamed_idx_1 = LRefLaneTurnAroundBefore->size[1];
  } else {
    unnamed_idx_1 = 0;
  }

  emxFree_int8_T(&LRefLaneTurnAroundBefore);
  if (LRefLaneTurnAroundDuring1->size[1] != 0) {
    b_unnamed_idx_1 = LRefLaneTurnAroundDuring1->size[1];
  } else {
    b_unnamed_idx_1 = 0;
  }

  if ((LRefLaneTurnAroundDuring2->size[0] != 0) &&
      (LRefLaneTurnAroundDuring2->size[1] != 0)) {
    c_unnamed_idx_1 = LRefLaneTurnAroundDuring2->size[1];
  } else {
    c_unnamed_idx_1 = 0;
  }

  if (LRefLaneTurnAroundDuring3->size[1] != 0) {
    f_input_sizes_idx_1 = LRefLaneTurnAroundDuring3->size[1];
  } else {
    f_input_sizes_idx_1 = 0;
  }

  if ((LRefLaneTurnAroundTransition->size[0] != 0) &&
      (LRefLaneTurnAroundTransition->size[1] != 0)) {
    g_input_sizes_idx_1 = LRefLaneTurnAroundTransition->size[1];
  } else {
    g_input_sizes_idx_1 = 0;
  }

  i = LRefLaneTurnAround->size[0] * LRefLaneTurnAround->size[1];
  LRefLaneTurnAround->size[0] = 1;
  LRefLaneTurnAround->size[1] = ((((c_input_sizes_idx_1 + d_input_sizes_idx_1) +
    e_input_sizes_idx_1) + input_sizes_idx_1) + b_input_sizes_idx_1) + 5;
  emxEnsureCapacity_real_T(LRefLaneTurnAround, i);
  SRefLaneTurnAroundBefore_data = LRefLaneTurnAround->data;
  for (i = 0; i < c_input_sizes_idx_1; i++) {
    SRefLaneTurnAroundBefore_data[i] = 0.0;
  }

  for (i = 0; i < d_input_sizes_idx_1; i++) {
    SRefLaneTurnAroundBefore_data[i + unnamed_idx_1] =
      LRefLaneTurnAroundDuring1_data[i];
  }

  emxFree_real_T(&LRefLaneTurnAroundDuring1);
  for (i = 0; i < e_input_sizes_idx_1; i++) {
    SRefLaneTurnAroundBefore_data[(i + unnamed_idx_1) + b_unnamed_idx_1] =
      LRefLaneTurnAroundDuring2_data[i];
  }

  emxFree_real_T(&LRefLaneTurnAroundDuring2);
  for (i = 0; i < input_sizes_idx_1; i++) {
    SRefLaneTurnAroundBefore_data[((i + unnamed_idx_1) + b_unnamed_idx_1) +
      c_unnamed_idx_1] = LRefLaneTurnAroundDuring3_data[i];
  }

  emxFree_real_T(&LRefLaneTurnAroundDuring3);
  for (i = 0; i < b_input_sizes_idx_1; i++) {
    SRefLaneTurnAroundBefore_data[(((i + unnamed_idx_1) + b_unnamed_idx_1) +
      c_unnamed_idx_1) + f_input_sizes_idx_1] =
      c_LRefLaneTurnAroundTransition_[i];
  }

  emxFree_real_T(&LRefLaneTurnAroundTransition);
  for (i = 0; i < 5; i++) {
    SRefLaneTurnAroundBefore_data[((((i + unnamed_idx_1) + b_unnamed_idx_1) +
      c_unnamed_idx_1) + f_input_sizes_idx_1) + g_input_sizes_idx_1] = dv[i];
  }

  c_input_sizes_idx_1 = SRefLaneTurnAround->size[1];
  if (SRefLaneTurnAround->size[1] < 100) {
    NumRefLaneTurnAroundBefore = 100.0 - (double)SRefLaneTurnAround->size[1];
    i = SRefLaneTurnAround->size[1];
    b_input_sizes_idx_1 = 100 - SRefLaneTurnAround->size[1];
    i1 = SRefLaneTurnAround->size[0] * SRefLaneTurnAround->size[1];
    SRefLaneTurnAround->size[1] = 100;
    emxEnsureCapacity_real_T(SRefLaneTurnAround, i1);
    SRefLaneTurnAround_data = SRefLaneTurnAround->data;
    for (i1 = 0; i1 < b_input_sizes_idx_1; i1++) {
      SRefLaneTurnAround_data[i + i1] = 0.0;
    }

    i = LRefLaneTurnAround->size[1];
    input_sizes_idx_1 = (int)NumRefLaneTurnAroundBefore;
    i1 = LRefLaneTurnAround->size[0] * LRefLaneTurnAround->size[1];
    LRefLaneTurnAround->size[1] += (int)NumRefLaneTurnAroundBefore;
    emxEnsureCapacity_real_T(LRefLaneTurnAround, i1);
    SRefLaneTurnAroundBefore_data = LRefLaneTurnAround->data;
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      SRefLaneTurnAroundBefore_data[i + i1] = 0.0;
    }
  }

  /*  过渡结束位置的s坐标 */
  /*  figure; */
  /*  plot(SRefLaneTurnAroundBefore,LRefLaneTurnAroundBefore,'ro'); */
  /*  hold on; */
  /*  plot(SRefLaneTurnAroundDuring1,LRefLaneTurnAroundDuring1,'b*'); */
  /*  hold on; */
  /*  plot(SRefLaneTurnAroundDuring2,LRefLaneTurnAroundDuring2,'r+'); */
  /*  hold on; */
  /*  plot(SRefLaneTurnAroundDuring3,LRefLaneTurnAroundDuring3,'gs'); */
  /*  hold on; */
  /*  plot(SRefLaneTurnAroundTransition,LRefLaneTurnAroundTransition,'cd'); */
  /*  hold on; */
  /*  plot(SRefLaneTurnAroundAfter,LRefLaneTurnAroundAfter,'bl>'); */
  /*  axis equal; */
  /*  figure; */
  /*  plot(SRefLaneTurnAround,LRefLaneTurnAround,'b*'); */
  /*  axis equal; */
  *NumRefLaneTurnAround = c_input_sizes_idx_1;
}

/*
 * Arguments    : double Rreplan
 *                double pos_s
 *                double pos_l
 *                double pos_l_CurrentLane
 *                double pos_psi
 *                double *CenterS
 *                double *CenterL
 * Return Type  : void
 */
static void ReplanCenter(double Rreplan, double pos_s, double pos_l, double
  pos_l_CurrentLane, double pos_psi, double *CenterS, double *CenterL)
{
  double u;
  u = pos_l - pos_l_CurrentLane;
  if (!rtIsNaN(u)) {
    if (u < 0.0) {
      u = -1.0;
    } else {
      u = (u > 0.0);
    }
  }

  if ((u == 1.0) && (pos_psi <= 90.0)) {
    u = pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = pos_psi;
    b_sind(&u);
    *CenterL = pos_l - Rreplan * u;
  } else if ((u == 1.0) && (pos_psi > 90.0)) {
    u = 180.0 - pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = 180.0 - pos_psi;
    b_sind(&u);
    *CenterL = pos_l - Rreplan * u;
  } else if ((u == -1.0) && (pos_psi <= 90.0)) {
    u = pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = pos_psi;
    b_sind(&u);
    *CenterL = pos_l + Rreplan * u;
  } else if ((u == -1.0) && (pos_psi > 90.0)) {
    u = 180.0 - pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = 180.0 - pos_psi;
    b_sind(&u);
    *CenterL = pos_l + Rreplan * u;
  } else if (pos_psi < 90.0) {
    u = pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = pos_psi;
    b_sind(&u);
    *CenterL = pos_l - Rreplan * u;
  } else {
    u = 180.0 - pos_psi;
    b_cosd(&u);
    *CenterS = pos_s + Rreplan * u;
    u = 180.0 - pos_psi;
    b_sind(&u);
    *CenterL = pos_l + Rreplan * u;
  }
}

/*
 * Arguments    : double fun_x_workspace_s0
 *                const double c_fun_x_workspace_fun_a_workspa[4]
 *                const double d_fun_x_workspace_fun_a_workspa[9]
 *                double length
 *                double x
 * Return Type  : double
 */
static double ReplanTrajPosCalc3_anonFcn3(double fun_x_workspace_s0, const
  double c_fun_x_workspace_fun_a_workspa[4], const double
  d_fun_x_workspace_fun_a_workspa[9], double length, double x)
{
  double c[50];
  double varargin_1_tmp[50];
  double y[50];
  double varargout_1;
  int ia;
  int iac;
  int ix;
  linspace(fun_x_workspace_s0, x, varargin_1_tmp);
  d_ppval(c_fun_x_workspace_fun_a_workspa, d_fun_x_workspace_fun_a_workspa,
          varargin_1_tmp, c);
  for (ix = 0; ix < 50; ix++) {
    double d;
    d = c[ix];
    y[ix] = sqrt(d * d + 1.0);
  }

  c[0] = 0.5 * (varargin_1_tmp[1] - varargin_1_tmp[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (varargin_1_tmp[ix + 2] - varargin_1_tmp[ix]);
  }

  c[49] = 0.5 * (varargin_1_tmp[49] - varargin_1_tmp[48]);
  varargout_1 = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    int b_iac;
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      varargout_1 += y[ia - 1] * c[ix];
    }

    ix++;
  }

  varargout_1 -= length;
  return varargout_1;
}

/*
 * Arguments    : double fun_x_workspace_s0
 *                const double c_fun_x_workspace_fun_a_workspa[5]
 *                const double d_fun_x_workspace_fun_a_workspa[12]
 *                double length
 *                double x
 * Return Type  : double
 */
static double ReplanTrajPosCalc4_anonFcn3(double fun_x_workspace_s0, const
  double c_fun_x_workspace_fun_a_workspa[5], const double
  d_fun_x_workspace_fun_a_workspa[12], double length, double x)
{
  double c[50];
  double varargin_1_tmp[50];
  double y[50];
  double varargout_1;
  int ia;
  int iac;
  int ix;
  linspace(fun_x_workspace_s0, x, varargin_1_tmp);
  ppval(c_fun_x_workspace_fun_a_workspa, d_fun_x_workspace_fun_a_workspa,
        varargin_1_tmp, c);
  for (ix = 0; ix < 50; ix++) {
    double d;
    d = c[ix];
    y[ix] = sqrt(d * d + 1.0);
  }

  c[0] = 0.5 * (varargin_1_tmp[1] - varargin_1_tmp[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (varargin_1_tmp[ix + 2] - varargin_1_tmp[ix]);
  }

  c[49] = 0.5 * (varargin_1_tmp[49] - varargin_1_tmp[48]);
  varargout_1 = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    int b_iac;
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      varargout_1 += y[ia - 1] * c[ix];
    }

    ix++;
  }

  varargout_1 -= length;
  return varargout_1;
}

/*
 * ,
 *
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
  double a_soll;
  double ex;
  double x_idx_0;
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
  } else if ((d_veh2waitingArea <= 0.5 * Parameters_l_veh) ||
             (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle ==
              1)) {
    dec_avoidOncomingVehicle = 0;
  }

  /*  停车决策 */
  if ((speed < 1.0E-5) || rtIsNaN(speed)) {
    ex = 1.0E-5;
  } else {
    ex = speed;
  }

  for (i = 0; i < 6; i++) {
    x_idx_0 = ((d_veh2conflict[i] + Parameters_l_veh) / ex * v_veh[i] + 0.5 *
               Parameters_w_veh) + f_CalibrationVars_SpeedPlanAvoi;
    if ((x_idx_0 < 0.0) || rtIsNaN(x_idx_0)) {
      d_veh[i] = 0.0;
    } else {
      d_veh[i] = x_idx_0;
    }
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
      ex = v_veh[i];
      if ((ex < 1.0E-5) || rtIsNaN(ex)) {
        ex = 1.0E-5;
      }

      ex = ((s_veh[i] - 0.5 * Parameters_w_veh) -
            f_CalibrationVars_SpeedPlanAvoi) / ex;
      if (!(ex > 0.0)) {
        ex = 0.0;
      }

      x_idx_0 = speed + c_CalibrationVars_SpeedPlanAvoi * ex;
      if ((x_idx_0 > e_CalibrationVars_SpeedPlanAvoi) || (rtIsNaN(x_idx_0) &&
           (!rtIsNaN(e_CalibrationVars_SpeedPlanAvoi)))) {
        x_idx_0 = e_CalibrationVars_SpeedPlanAvoi;
      }

      d_veh[i] = 0.5 * (x_idx_0 + speed) * ex;
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
    ex = d_veh2waitingArea + CalibrationVars_ACC->d_wait;
    if (!(ex > 0.0)) {
      ex = 0.0;
    }

    a_soll = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed, 1,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
    ex = b_ACC(e_CalibrationVars_SpeedPlanAvoi, 0.0, ex, speed, 1,
               CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
               CalibrationVars_ACC->d_wait2faultyCar,
               CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
               CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
               CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
               CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    if ((a_soll > ex) || (rtIsNaN(a_soll) && (!rtIsNaN(ex)))) {
      a_soll = ex;
    }
  } else if (dec_avoidOncomingVehicle == 1) {
    a_soll = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed,
                   wait_avoidOncomingVehicle, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else if (d_veh2waitingArea > 2.0 * Parameters_l_veh) {
    a_soll = b_ACC(v_max, v_b, s_b, speed, wait_avoidOncomingVehicle,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else {
    a_soll = b_ACC(e_CalibrationVars_SpeedPlanAvoi, v_b, s_b, speed,
                   wait_avoidOncomingVehicle, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
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
  return a_soll;
}

/*
 * pos_s为网联车车头s坐标
 *  s_ped,l_ped,v_ped,psi_ped皆为大小40的数组，默认值为0,0,-1,0，s_ped按从小到大排列，psi_ped范围为0到360（沿l轴为0度，沿s轴为90度）
 *  d_veh2cross=max([0 d_veh2cross]);
 *
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
  double jerk;
  double tend;
  int d_veh2ped_size[2];
  int i;
  int partialTrueCount;
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
  if (GlobVars->Decider.a_sollpre2traj <= c_CalibrationVars_SpeedPlanAvoi.a_min)
  {
    jerk = (0.0 - speed * speed) / (2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min);
  } else {
    jerk = (c_CalibrationVars_SpeedPlanAvoi.a_min *
            c_CalibrationVars_SpeedPlanAvoi.a_min -
            GlobVars->Decider.a_sollpre2traj * GlobVars->Decider.a_sollpre2traj)
      / (2.0 * (-speed + 2.2204460492503131E-16));
    if ((jerk < -2.0) || (jerk >= 0.0)) {
      tend = (c_CalibrationVars_SpeedPlanAvoi.a_min -
              GlobVars->Decider.a_sollpre2traj) / -2.0;
      jerk = (speed + GlobVars->Decider.a_sollpre2traj * tend) - tend * tend;
      jerk = ((speed * tend + 0.5 * GlobVars->Decider.a_sollpre2traj * (tend *
                tend)) + -0.33333333333333331 * rt_powd_snf(tend, 3.0)) + (0.0 -
        jerk * jerk) / (2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min);
    } else {
      tend = (0.0 - speed) / (0.5 * (GlobVars->Decider.a_sollpre2traj +
        c_CalibrationVars_SpeedPlanAvoi.a_min));
      jerk = (speed * tend + 0.5 * GlobVars->Decider.a_sollpre2traj * (tend *
               tend)) + 0.16666666666666666 * jerk * rt_powd_snf(tend, 3.0);
    }
  }

  /*  d_bre=(0-speed.^2)/(2*a_min); */
  if (GlobVars->SpeedPlanAvoidPedestrian.dec_ped == 0) {
    double d_veh2ped_data[40];
    int trueCount;

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

    if ((e_minimum(d_veh2ped_data, d_veh2ped_size) <= jerk) || ((d_veh2cross <=
          jerk) && (d_veh2cross > 0.0))) {
      /*  1原为l_veh 01.21修改 */
      dec_ped = 1;
    }
  } else {
    double d_veh2ped_data[40];
    int trueCount;

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

    jerk = fmax(jerk, (0.0 - c_CalibrationVars_SpeedPlanAvoi.v_max_int
                       * c_CalibrationVars_SpeedPlanAvoi.v_max_int) / (2.0 *
      c_CalibrationVars_SpeedPlanAvoi.a_min)) + 10.0;
    if ((e_minimum(d_veh2ped_data, d_veh2ped_size) > jerk) && ((d_veh2cross ==
          0.0) || (d_veh2cross > jerk))) {
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
          jerk = psi_ped[i] / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        } else if ((psi_ped[i] > 90.0) && (psi_ped[i] <= 270.0)) {
          jerk = (psi_ped[i] + 180.0) / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        } else {
          jerk = (psi_ped[i] + 360.0) / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        }

        if ((speed < 1.0E-5) || rtIsNaN(speed)) {
          tend = 1.0E-5;
        } else {
          tend = speed;
        }

        jerk = d_veh2ped[i] / tend * jerk + 0.5 * Parameters_w_veh * 1.2;
        if ((jerk < 0.0) || rtIsNaN(jerk)) {
          jerk = 0.0;
        }

        if (fabs(l_ped[i]) <= jerk) {
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
          jerk = psi_ped[i] / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        } else if ((psi_ped[i] > 90.0) && (psi_ped[i] <= 270.0)) {
          jerk = (psi_ped[i] + 180.0) / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        } else {
          jerk = (psi_ped[i] + 360.0) / 2.0;
          b_cosd(&jerk);
          if (rtIsNaN(l_ped[i])) {
            tend = rtNaN;
          } else if (l_ped[i] < 0.0) {
            tend = -1.0;
          } else {
            tend = (l_ped[i] > 0.0);
          }

          jerk = fmax(0.0, -v_ped[i] * tend * jerk);
        }

        tend = (fabs(l_ped[i]) - 0.5 * Parameters_w_veh * 1.2) / (jerk +
          2.2204460492503131E-16);
        if (!(tend > 0.0)) {
          tend = 0.0;
        }

        /*  if ~(d_veh2ped(i)<10 && s_max>d_veh2ped(i)) % 是否考虑车长？ */
        jerk = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * tend;
        if ((jerk > c_CalibrationVars_SpeedPlanAvoi.v_max_int) || (rtIsNaN(jerk)
             && (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max_int)))) {
          jerk = c_CalibrationVars_SpeedPlanAvoi.v_max_int;
        }

        if (!(0.5 * (jerk + speed) * tend > d_veh2ped[i])) {
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
                      CalibrationVars_ACC->a_min,
                      CalibrationVars_ACC->d_wait2faultyCar,
                      CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                      CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                      CalibrationVars_ACC->tau_v_emg,
                      CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                      CalibrationVars_ACC->d_wait);
    } else {
      *a_soll = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, v_b, s_b, speed,
                      0, CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                      CalibrationVars_ACC->d_wait2faultyCar,
                      CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                      CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                      CalibrationVars_ACC->tau_v_emg,
                      CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                      CalibrationVars_ACC->d_wait);
    }
  } else {
    *a_soll = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, v_b, s_b, speed,
                    wait_ped, CalibrationVars_ACC->a_max,
                    CalibrationVars_ACC->a_min,
                    CalibrationVars_ACC->d_wait2faultyCar,
                    CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                    CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                    CalibrationVars_ACC->tau_v_emg,
                    CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                    CalibrationVars_ACC->d_wait);
    jerk = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max_int, 0.0,
                 (*d_veh2stopline + CalibrationVars_ACC->d_wait) - 0.5, speed,
                 wait_ped, CalibrationVars_ACC->a_max,
                 CalibrationVars_ACC->a_min,
                 CalibrationVars_ACC->d_wait2faultyCar,
                 CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                 CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                 CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
                 CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    if ((*a_soll > jerk) || (rtIsNaN(*a_soll) && (!rtIsNaN(jerk)))) {
      *a_soll = jerk;
    }
  }

  GlobVars->SpeedPlanAvoidPedestrian.dec_ped = dec_ped;
  GlobVars->SpeedPlanAvoidPedestrian.wait_ped = wait_ped;
}

/*
 * CurrentLaneFrontDisAvoidVehicle,CurrentLaneFrontVelAvoidVehicle,TargetLaneFrontDisAvoidVehicle,TargetLaneFrontVelAvoidVehicle,TargetLaneBehindDisAvoidVehicle,TargetLaneBehindVelAvoidVehicle
 * globalVariable----------------------------------------------------------------------------------------------------------------------
 *
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
  double x[2];
  double a_soll;
  double d;
  double d1;
  double d2;
  double d_ist;
  double d_ist_tmp;
  double ex;
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
  dec_fol = GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle;
  dec_bre = GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle;
  b_wait = GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle;
  x[0] = 1.0E-5;
  x[1] = v_a;
  v_a = maximum(x);
  x[0] = 1.0E-5;
  x[1] = v_b;
  v_b = maximum(x);
  x[0] = 1.0E-5;
  x[1] = v_c;
  v_c = maximum(x);

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
    x[0] = (0.0 - speed * speed) / (2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min)
      + 10.0;
    if ((x[0] > 15.0) || rtIsNaN(x[0])) {
      ex = 15.0;
    } else {
      ex = x[0];
    }

    if ((d_veh2stopline <= ex) && (d_veh2stopline > 0.0)) {
      dec_bre = 1;
    }
  } else if ((d_veh2stopline <= 0.0) ||
             (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1)) {
    dec_bre = 0;
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
        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_b2int;
        if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
             (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
          ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
        } else {
          ex = x[0];
        }

        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_b2int;
        x[1] = 0.0;
        d /= c_CalibrationVars_SpeedPlanAvoi.gapIndex;
        if (!(fmax(0.5 * (maximum(x) + speed) * t_b2int, (d_veh2int
               + Parameters_l_veh) + d) < fmin(0.5 * (ex + speed) * t_b2int,
              (s_b_end - l_a) - d))) {
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
        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_a2int;
        if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
             (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
          ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
        } else {
          ex = x[0];
        }

        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_a2int;
        x[1] = 0.0;
        d /= c_CalibrationVars_SpeedPlanAvoi.gapIndex;
        if (fmax(0.5 * (maximum(x) + speed) * t_a2int, (d_veh2int
              + Parameters_l_veh) + d) < fmin(0.5 * (ex + speed) * t_a2int,
             (s_b_end - l_b) - d)) {
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
        dv[1] = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[2] = (d1 - v_c * v_c) / d2;
        if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
               (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
            ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
          } else {
            ex = x[0];
          }

          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          x[1] = 0.0;
          if (fmax(0.5 * (maximum(x) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + v_c * c_CalibrationVars_SpeedPlanAvoi.t_re
                   / c_CalibrationVars_SpeedPlanAvoi.gapIndex) < fmin(0.5 * (ex
                + speed) * t_c2int, (s_b_end - l_b) - v_c *
               c_CalibrationVars_SpeedPlanAvoi.t_re /
               c_CalibrationVars_SpeedPlanAvoi.gapIndex)) {
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
        d = v_c * c_CalibrationVars_SpeedPlanAvoi.t_re;
        dv[1] = d;
        dv[2] = (d1 - v_c * v_c) / d2;
        if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
               (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
            ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
          } else {
            ex = x[0];
          }

          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          x[1] = 0.0;
          d /= c_CalibrationVars_SpeedPlanAvoi.gapIndex;
          if (!(fmax(0.5 * (maximum(x) + speed) * t_c2int, (d_veh2int
                 + Parameters_l_veh) + d) < fmin(0.5 * (ex + speed) * t_c2int,
                (s_b_end - l_a) - d))) {
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
    boolean_T guard6 = false;
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
        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_b2int;
        if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
             (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
          ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
        } else {
          ex = x[0];
        }

        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_b2int;
        x[1] = 0.0;
        d = v_b * c_CalibrationVars_SpeedPlanAvoi.t_re /
          c_CalibrationVars_SpeedPlanAvoi.gapIndex;
        if (fmax(0.5 * (maximum(x) + speed) * t_b2int, (d_veh2int
              + Parameters_l_veh) + d) < fmin(0.5 * (ex + speed) * t_b2int,
             (s_b_end - l_a) - d)) {
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
      d = v_a * v_a;
      d1 = 2.0 * c_CalibrationVars_SpeedPlanAvoi.a_min;
      dv[2] = (v_b * v_b - d) / d1;
      if (((s_b_end - d_veh2int) - l_b) - Parameters_l_veh > b_maximum(dv)) {
        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_a2int;
        if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
             (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
          ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
        } else {
          ex = x[0];
        }

        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_a2int;
        x[1] = 0.0;
        d2 = v_a * c_CalibrationVars_SpeedPlanAvoi.t_re /
          c_CalibrationVars_SpeedPlanAvoi.gapIndex;
        if (fmax(0.5 * (maximum(x) + speed) * t_a2int, (d_veh2int
              + Parameters_l_veh) + d2) < fmin(0.5 * (ex + speed) * t_a2int,
             (s_b_end - l_b) - d2)) {
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
          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
               (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
            ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
          } else {
            ex = x[0];
          }

          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          x[1] = 0.0;
          if (fmax(0.5 * (maximum(x) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + v_c * c_CalibrationVars_SpeedPlanAvoi.t_re
                   / c_CalibrationVars_SpeedPlanAvoi.gapIndex) < fmin(0.5 * (ex
                + speed) * t_c2int, (s_b_end - l_b) - v_c *
               c_CalibrationVars_SpeedPlanAvoi.t_re /
               c_CalibrationVars_SpeedPlanAvoi.gapIndex)) {
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
        dv[2] = (d - v_c * v_c) / d1;
        if (((s_b_end - d_veh2int) - l_a) - Parameters_l_veh > b_maximum(dv)) {
          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
          if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
               (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
            ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
          } else {
            ex = x[0];
          }

          x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
          x[1] = 0.0;
          if (fmax(0.5 * (maximum(x) + speed) * t_c2int, (d_veh2int
                + Parameters_l_veh) + v_c * c_CalibrationVars_SpeedPlanAvoi.t_re
                   / c_CalibrationVars_SpeedPlanAvoi.gapIndex) < fmin(0.5 * (ex
                + speed) * t_c2int, (s_b_end - l_a) - v_c *
               c_CalibrationVars_SpeedPlanAvoi.t_re /
               c_CalibrationVars_SpeedPlanAvoi.gapIndex)) {
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
    boolean_T prereq4;
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
        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_max * t_c2int;
        if ((x[0] > c_CalibrationVars_SpeedPlanAvoi.v_max) || (rtIsNaN(x[0]) &&
             (!rtIsNaN(c_CalibrationVars_SpeedPlanAvoi.v_max)))) {
          ex = c_CalibrationVars_SpeedPlanAvoi.v_max;
        } else {
          ex = x[0];
        }

        x[0] = speed + c_CalibrationVars_SpeedPlanAvoi.a_min * t_c2int;
        x[1] = 0.0;
        d /= c_CalibrationVars_SpeedPlanAvoi.gapIndex;
        if ((fmax(0.5 * (maximum(x) + speed) * t_c2int, (d_veh2int
               + Parameters_l_veh) + d) < fmin(0.5 * (ex + speed) * t_c2int,
              (s_b_end - l_b) - d)) && prereq4) {
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
                  CalibrationVars_ACC->d_wait2faultyCar,
                  CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                  CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                  CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
                  CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
  a_soll = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, v_soll, d_ist, speed,
                 b_wait, CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                 CalibrationVars_ACC->d_wait2faultyCar,
                 CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                 CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                 CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
                 CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
  if ((a_soll > s_b_end) || (rtIsNaN(a_soll) && (!rtIsNaN(s_b_end)))) {
    a_soll = s_b_end;
  }

  if (b_wait == 0) {
    if (d_veh2stopline <= 0.0) {
      d = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, v_b, s_b - l_b, speed, 0,
                CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                CalibrationVars_ACC->d_wait2faultyCar,
                CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
                CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
      if ((a_soll > d) || (rtIsNaN(a_soll) && (!rtIsNaN(d)))) {
        a_soll = d;
      }
    }
  } else {
    /*  停车待通行状态下速度规划 */
    x[0] = 0.0;
    x[1] = d_veh2stopline + CalibrationVars_ACC->d_wait;
    a_soll = b_ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, 0.0, maximum(x), speed,
                   b_wait, CalibrationVars_ACC->a_max,
                   CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
    if ((a_soll > s_b_end) || (rtIsNaN(a_soll) && (!rtIsNaN(s_b_end)))) {
      a_soll = s_b_end;
    }
  }

  if (dec_fol == 1) {
    /* 跟车状态下，跟主路车加速度和停车加速度取最大且与匝道前车加速度取最小 */
    x[0] = 0.0;
    x[1] = d_veh2stopline + CalibrationVars_ACC->d_wait;
    d = maximum(x);
    x[0] = a_soll;
    x[1] = ACC(c_CalibrationVars_SpeedPlanAvoi.v_max, 0.0, d, speed, 1.0,
               CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
               CalibrationVars_ACC->d_wait2faultyCar,
               CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
               CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
               CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
               CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    a_soll = fmin(s_b_end, maximum(x));
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
 *
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
  double a_soll;
  double ex;
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
  /*  v_max=v_max_int;%实车测试时因信号灯停止线前刹不住，改为场景激活就限速 */
  /* 1.5; */
  /* Parameters-------------------------------------------------------------------------------------------------------------------------- */
  /*  w_veh=Parameters.w_veh; */
  decel = 0;
  if (!(d_veh2int > 0.0)) {
    d_veh2int = 0.0;
  }

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
  } else if ((d_veh2int <= 1.0) ||
             (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
    /*  1原为10 12.31修改 */
    dec_bre = 0;
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
    ex = ACC(v_max, v_b, s_b, speed, 0.0, CalibrationVars_ACC->a_max,
             CalibrationVars_ACC->a_min, CalibrationVars_ACC->d_wait2faultyCar,
             CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
             CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
             CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
             CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    if ((ex > d_CalibrationVars_SpeedPlanTraf) || (rtIsNaN(ex) && (!rtIsNaN
          (d_CalibrationVars_SpeedPlanTraf)))) {
      ex = d_CalibrationVars_SpeedPlanTraf;
    }

    ex = speed + ex * time2nextSwitch;
    if ((ex > f_CalibrationVars_SpeedPlanTraf) || (rtIsNaN(ex) && (!rtIsNaN
          (f_CalibrationVars_SpeedPlanTraf)))) {
      ex = f_CalibrationVars_SpeedPlanTraf;
    }

    if (0.5 * (ex + speed) * time2nextSwitch > d_veh2int) {
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
                   CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
  } else if (decel == 1) {
    /*          a_soll=min([ACC(v_max_int,v_soll,d_ist,speed,wait,CalibrationVars) ACCcust(v_max_int,0,d_veh2int+CalibrationVars.ACC.d_wait-0.5,speed,a_max,a_min_com,t_acc,CalibrationVars)]);            */
    ex = ACC(f_CalibrationVars_SpeedPlanTraf, 0.0, (d_veh2int
              + CalibrationVars_ACC->d_wait) - 0.5, speed, 1.0,
             CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
             CalibrationVars_ACC->d_wait2faultyCar,
             CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
             CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
             CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
             CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    if ((ex < c_CalibrationVars_SpeedPlanTraf) || (rtIsNaN(ex) && (!rtIsNaN
          (c_CalibrationVars_SpeedPlanTraf)))) {
      ex = c_CalibrationVars_SpeedPlanTraf;
    }

    a_soll = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
    if ((a_soll > ex) || (rtIsNaN(a_soll) && (!rtIsNaN(ex)))) {
      a_soll = ex;
    }

    /* 20220712,去掉ACCcust */
  } else if (b_wait == 1) {
    a_soll = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, 1,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
                   CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
                   CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
                   CalibrationVars_ACC->tau_v_emg,
                   CalibrationVars_ACC->tau_d_emg, CalibrationVars_ACC->t_acc,
                   CalibrationVars_ACC->d_wait);
    ex = b_ACC(f_CalibrationVars_SpeedPlanTraf, 0.0, (d_veh2int
                + CalibrationVars_ACC->d_wait) - 0.5, speed, 1,
               CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
               CalibrationVars_ACC->d_wait2faultyCar,
               CalibrationVars_ACC->tau_v_com, CalibrationVars_ACC->tau_v,
               CalibrationVars_ACC->tau_d, CalibrationVars_ACC->tau_v_bre,
               CalibrationVars_ACC->tau_v_emg, CalibrationVars_ACC->tau_d_emg,
               CalibrationVars_ACC->t_acc, CalibrationVars_ACC->d_wait);
    if ((a_soll > ex) || (rtIsNaN(a_soll) && (!rtIsNaN(ex)))) {
      a_soll = ex;
    }

    /*  a_soll */
    /*  greenLight */
  } else {
    a_soll = b_ACC(f_CalibrationVars_SpeedPlanTraf, v_b, s_b, speed, b_wait,
                   CalibrationVars_ACC->a_max, CalibrationVars_ACC->a_min,
                   CalibrationVars_ACC->d_wait2faultyCar,
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
 * ,
 *
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
 *                double pos_l
 *                short CurrentLaneIndex
 *                short TargetLaneIndex
 *                short GoalLaneIndex
 *                short BackupTargetLaneIndex
 *                double d_veh2int
 *                double d_veh2goal
 *                double WidthOfLanes[6]
 *                double v_max
 *                const short LanesWithFail[6]
 *                short AEBActive
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
  double speed, double pos_s, double pos_l_CurrentLane, double pos_l, short
  CurrentLaneIndex, short TargetLaneIndex, short GoalLaneIndex, short
  BackupTargetLaneIndex, double d_veh2int, double d_veh2goal, double
  WidthOfLanes[6], double v_max, const short LanesWithFail[6], short AEBActive,
  TypeGlobVars *GlobVars, const TypeCalibrationVars *CalibrationVars, const
  TypeParameters Parameters, double *a_soll, double traj_s[80], double traj_l[80],
  double traj_psi[80], double traj_vs[80], double traj_vl[80], double
  traj_omega[80])
{
  double traj[720];
  double S_traj[80];
  double V_traj[80];
  double X_traj[80];
  double para_ST[4];
  double b_S_b_end[3];
  double b_speed[2];
  double S_b_end;
  double S_b_end_tmp;
  double S_c_end;
  double S_d_end;
  double S_end;
  double S_min;
  double S_min_dyn;
  double S_tlc;
  double TargetLaneBehindDis;
  double TargetLaneBehindVel;
  double TargetLaneFrontDis;
  double TargetLaneFrontVel;
  double V_b_end;
  double V_c_end;
  double V_end;
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
  int i;
  int i3;
  int traj_tmp;
  short b_CurrentLaneIndex[2];
  short CountLaneChange;
  short CurrentTargetLaneIndex;
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
  boolean_T prereq5;

  /* globalVariable---------------------------------------------------------------------------------------------------------------------- */
  CountLaneChange = GlobVars->TrajPlanLaneChange.countLaneChange;
  DurationLaneChange = GlobVars->TrajPlanLaneChange.durationLaneChange;
  t_lc_traj = GlobVars->TrajPlanLaneChange.t_lc_traj;
  CurrentTargetLaneIndex = GlobVars->TrajPlanLaneChange.currentTargetLaneIndex;

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
    i = 6;
  } else {
    i = (short)(CurrentLaneIndex + 1);
  }

  w_lane_right = 0.5 * WidthOfLanes[i - 1] + S_tlc;
  w_lane = (w_lane_left + w_lane_right) / 2.0;

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
    if (BackupTargetLaneIndex <= CurrentLaneIndex) {
      s_d = LeftLaneBehindDis;
      v_d = LeftLaneBehindVel;
      s_e = LeftLaneFrontDis;
      v_e = LeftLaneFrontVel;
    } else {
      s_d = RightLaneBehindDis;
      v_d = RightLaneBehindVel;
      s_e = RightLaneFrontDis;
      v_e = RightLaneFrontVel;
    }

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
  /* 1; */
  /* -3.5; */
  /* 2.5; */
  /* -1; */
  *a_soll = 100.0;

  /*  t_lc=max([2-0.05*(speed-10) 1.7]); */
  /*  t_lc=min([t_lc 2.3]); */
  b_speed[0] = 2.0 - 0.04 * (speed - 15.0);
  b_speed[1] = 2.0;
  S_b_end_tmp = maximum(b_speed);
  b_speed[0] = S_b_end_tmp;
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
  if ((GlobVars->TrajPlanLaneChange.countLaneChange == 0) && (CurrentLaneIndex
       != TargetLaneIndex) && (CurrentLaneIndex != BackupTargetLaneIndex)) {
    if (TargetLaneIndex <= CurrentLaneIndex) {
      w_lane = w_lane_left - (pos_l - pos_l_CurrentLane);
    } else {
      w_lane = w_lane_right + (pos_l - pos_l_CurrentLane);
    }

    /*      w_lane=0.5*WidthOfLanes(CurrentLaneIndex)+0.5*WidthOfLanes(TargetLaneIndex);%20220328 */
    /*      w_lane=w_lane+double(sign(TargetLaneIndex-CurrentLaneIndex))*(pos_l-pos_l_CurrentLane); */
    if (speed < 5.0) {
      b_speed[0] = 10.0;
      b_speed[1] = t_lc * speed;
      S_end = maximum(b_speed);
      fzero(speed, S_end, w_lane, &S_d_end, &S_tlc, &S_b_end);
      b_speed[0] = t_lc;
      b_speed[1] = S_d_end;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      speed_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_speed[1] = (0.0 - speed * speed) / speed_tmp;
      S_min_dyn = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_b_end = maximum(b_speed);
      S_d_end = w_lane * w_lane;
      b_speed[0] = sqrt(S_min_dyn * S_min_dyn - S_d_end);
      b_speed[1] = sqrt(S_b_end * S_b_end - S_d_end);
      S_min_dyn = maximum(b_speed);

      /*          V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]); */
      /*          S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc; */
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_speed[0] = ACC(v_max, TargetLaneFrontVel, TargetLaneFrontDis -
                       TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.d_wait2faultyCar,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_b_end_tmp = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + S_b_end_tmp *
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
      S_b_end_tmp = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_b_end_tmp) && (TargetLaneFrontDis <= speed *
           t_lc)) {
        /* , */
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        S_tlc = w_lane * w_lane;
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = V_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_b_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - V_b_end * V_b_end) / speed_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        S_min_dyn = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_b_end_tmp = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_b_end_tmp;
        b_S_b_end[1] = S_b_end_tmp - (V_c_end * V_c_end - V_end * V_end) /
          speed_tmp;
        b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - S_tlc);
        TargetLaneFrontVel = d_minimum(b_S_b_end);
      } else {
        guard1 = false;
        if (-TargetLaneBehindDis <= S_b_end_tmp) {
          d = speed * t_lc;
          if (TargetLaneFrontDis > d) {
            /* , */
            /*                  b车存在 c车不存在 */
            /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
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
            b_S_b_end[2] = S_min_dyn;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
            S_min_dyn = d + 0.5 * CalibrationVars->TrajPlanLaneChange.a_max *
              t_lc * t_lc;
            b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
              V_end;
            b_S_b_end[1] = (S_c_end - V_end *
                            CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end
              * V_c_end - V_end * V_end) / speed_tmp;
            b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - w_lane * w_lane);
            TargetLaneFrontVel = d_minimum(b_S_b_end);
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if ((-TargetLaneBehindDis > S_b_end_tmp) && (TargetLaneFrontDis >
               speed * t_lc)) {
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
            b_S_b_end[2] = S_min_dyn;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
            S_min_dyn = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
            b_S_b_end[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
              V_end;
            b_S_b_end[1] = (S_c_end - V_end *
                            CalibrationVars->TrajPlanLaneChange.t_re) - (V_c_end
              * V_c_end - V_end * V_end) / (2.0 *
              CalibrationVars->TrajPlanLaneChange.a_min);
            b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - w_lane * w_lane);
            TargetLaneFrontVel = d_minimum(b_S_b_end);
          } else {
            /*                  b车存在 c车存在 */
            /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
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
            b_S_b_end[2] = S_min_dyn;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([S_c_end-t_re*V_end-l_veh S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
            S_min_dyn = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
            S_b_end_tmp = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
              V_end;
            b_S_b_end[0] = S_b_end_tmp;
            b_S_b_end[1] = S_b_end_tmp - (V_c_end * V_c_end - V_end * V_end) /
              (2.0 * CalibrationVars->TrajPlanLaneChange.a_min);
            b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - w_lane * w_lane);
            TargetLaneFrontVel = d_minimum(b_S_b_end);
          }
        }
      }

      /* 目标车道前车减速，后车加速极限条件下仍可避免碰撞 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end));%被条件9覆盖，暂时注释 */
      /* 换道完成速度高于以最小减速度减速的速度 */
      /* 换道完成速度低于以最大加速度加速的速度 */
      if ((TargetLaneFrontVel > S_end) && (S_end > S_min)) {
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
          CurrentTargetLaneIndex = TargetLaneIndex;
        }
      }
    } else {
      /*          V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]); */
      /*          S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc; */
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneFrontVel +
        CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      V_c_end = maximum(b_speed);
      S_c_end = TargetLaneFrontDis + 0.5 * (V_c_end + TargetLaneFrontVel) * t_lc;
      b_speed[0] = ACC(v_max, TargetLaneFrontVel, TargetLaneFrontDis -
                       TargetLaneBehindDis, TargetLaneBehindVel, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.d_wait2faultyCar,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_b_end_tmp = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = TargetLaneBehindVel + S_b_end_tmp *
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
      S_b_end_tmp = TargetLaneBehindVel * t_lc;
      if ((-TargetLaneBehindDis > S_b_end_tmp) && (TargetLaneFrontDis <= speed *
           (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车存在 */
        b_speed[0] = 0.5 * (speed + V_c_end);
        b_speed[1] = speed;
        V_end = c_minimum(b_speed);
        S_b_end_tmp = S_b_end + V_b_end *
          CalibrationVars->TrajPlanLaneChange.t_re;
        b_S_b_end[0] = S_b_end_tmp + Parameters.l_veh;
        TargetLaneFrontVel = V_end * V_end;
        S_tlc = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = (S_b_end_tmp + (TargetLaneFrontVel - V_b_end * V_b_end) /
                        S_tlc) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_b_end_tmp = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_b_end_tmp;
        b_S_b_end[1] = S_b_end_tmp - (V_c_end * V_c_end - TargetLaneFrontVel) /
          S_tlc;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneFrontVel = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = TargetLaneFrontVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis <= S_b_end_tmp) && (TargetLaneFrontDis >
                  speed * (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车存在 c车不存在 */
        b_speed[0] = 0.5 * (speed + V_b_end);
        b_speed[1] = speed;
        V_end = c_minimum(b_speed);
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_b_end_tmp = V_end * V_end;
        b_S_b_end[1] = ((S_b_end + V_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) +
                        (S_b_end_tmp - V_b_end * V_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        TargetLaneFrontVel = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[0] = TargetLaneFrontVel;
        b_S_b_end[1] = TargetLaneFrontVel - (V_c_end * V_c_end - S_b_end_tmp) /
          (2.0 * CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneFrontVel = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = TargetLaneFrontVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      } else if ((-TargetLaneBehindDis > S_b_end_tmp) && (TargetLaneFrontDis >
                  speed * (t_lc + CalibrationVars->TrajPlanLaneChange.t_re))) {
        /* , */
        /*                  b车不存在 c车不存在 */
        S_end = speed * t_lc;
        V_end = speed;
        b_S_b_end[0] = (S_b_end + V_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_b_end_tmp = speed * speed;
        TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_b_end + V_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) +
                        (S_b_end_tmp - V_b_end * V_b_end) / TargetLaneFrontVel)
          + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_c_end-t_re*V_end-l_veh) S_c_end-V_end*t_re-(V_c_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        S_tlc = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
        b_S_b_end[0] = S_tlc;
        b_S_b_end[1] = S_tlc - (V_c_end * V_c_end - S_b_end_tmp) /
          TargetLaneFrontVel;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneFrontVel = d_minimum(b_S_b_end);
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
        TargetLaneFrontVel = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = TargetLaneFrontVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      }

      /*  参见TrajPlanLaneChange_S_max_withAccel.bmp */
      b_speed[0] = V_end;
      b_speed[1] = speed;
      S_min_dyn = (CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc -
                   fabs(V_end - speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移  */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (TargetLaneFrontVel >= S_min) {
        S_b_end_tmp = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort *
          t_lc * t_lc;
        d = S_min_dyn * S_min_dyn /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_speed) + S_b_end_tmp) - d) && (S_end >=
             (t_lc * maximum(b_speed) - S_b_end_tmp) + d)) {
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
           speed + CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc) &&
          prereq5 && (d_veh2int >= S_end +
                      CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int
                      * Parameters.l_veh) && (TargetLaneBehindDis <=
           -Parameters.l_veh) && (TargetLaneFrontDis >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel +
          CalibrationVars->TrajPlanLaneChange.index_accel *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          CurrentTargetLaneIndex = TargetLaneIndex;
        }
      }
    }
  }

  if ((CountLaneChange == 0) && (CurrentLaneIndex != TargetLaneIndex) &&
      (CurrentLaneIndex != BackupTargetLaneIndex) && (BackupTargetLaneIndex !=
       -1)) {
    if (BackupTargetLaneIndex <= CurrentLaneIndex) {
      w_lane = w_lane_left - (pos_l - pos_l_CurrentLane);
    } else {
      w_lane = w_lane_right + (pos_l - pos_l_CurrentLane);
    }

    if (speed < 5.0) {
      b_speed[0] = 10.0;
      b_speed[1] = t_lc * speed;
      S_end = maximum(b_speed);
      fzero(speed, S_end, w_lane, &S_d_end, &S_tlc, &S_b_end);
      b_speed[0] = t_lc;
      b_speed[1] = S_d_end;
      t_lc = 0.1 * rt_roundd_snf(maximum(b_speed) / 0.1);
      b_speed[0] = w_lane;
      b_speed[1] = (0.0 - speed * speed) / (2.0 *
        CalibrationVars->TrajPlanLaneChange.a_min);
      S_min_dyn = maximum(b_speed);
      b_speed[0] = w_lane;
      b_speed[1] = speed * t_lc + 0.5 *
        CalibrationVars->TrajPlanLaneChange.a_min * t_lc * t_lc;
      S_b_end = maximum(b_speed);
      speed_tmp = w_lane * w_lane;
      b_speed[0] = sqrt(S_min_dyn * S_min_dyn - speed_tmp);
      b_speed[1] = sqrt(S_b_end * S_b_end - speed_tmp);
      S_min_dyn = maximum(b_speed);

      /*          V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]); */
      /*          S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc; */
      b_speed[0] = 0.0;
      b_speed[1] = v_e + CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc;
      TargetLaneBehindVel = maximum(b_speed);
      V_b_end = s_e + 0.5 * (TargetLaneBehindVel + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.d_wait2faultyCar,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_b_end_tmp = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + S_b_end_tmp *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_b_end = maximum(b_speed);
      S_d_end = s_d + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;
      S_b_end_tmp = v_d * t_lc;
      if ((-s_d > S_b_end_tmp) && (s_e <= speed * t_lc)) {
        /* , */
        /*                  b车不存在 c车存在 */
        /*  V_end=min([0.5*(V_0+V_c_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        S_tlc = w_lane * w_lane;
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_d_end + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_b_end_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_d_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / S_b_end_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        S_min_dyn = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        TargetLaneFrontVel = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[0] = TargetLaneFrontVel;
        b_S_b_end[1] = TargetLaneFrontVel - (TargetLaneBehindVel *
          TargetLaneBehindVel - V_end * V_end) / S_b_end_tmp;
        b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - S_tlc);
        TargetLaneFrontVel = d_minimum(b_S_b_end);
      } else if ((-s_d <= S_b_end_tmp) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b车存在 c车不存在 */
        /*  V_end=min([0.5*(V_0+V_b_end) ((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0]); */
        S_tlc = w_lane * w_lane;
        V_end = sqrt(S_end * S_end + S_tlc) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_d_end + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        S_b_end_tmp = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
        b_S_b_end[1] = ((S_d_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / S_b_end_tmp) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        S_min_dyn = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        TargetLaneFrontVel = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[0] = TargetLaneFrontVel;
        b_S_b_end[1] = TargetLaneFrontVel - (TargetLaneBehindVel *
          TargetLaneBehindVel - V_end * V_end) / S_b_end_tmp;
        b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - S_tlc);
        TargetLaneFrontVel = d_minimum(b_S_b_end);
      } else if ((-s_d > S_b_end_tmp) && (s_e > speed * t_lc)) {
        /* , */
        /*                  b车不存在 c车不存在 */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_d_end + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_d_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        S_min_dyn = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        b_S_b_end[0] = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (V_b_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) -
          (TargetLaneBehindVel * TargetLaneBehindVel - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - w_lane * w_lane);
        TargetLaneFrontVel = d_minimum(b_S_b_end);
      } else {
        /*                  b车存在 c车存在 */
        /*  V_end=min([((S_end.^2+w_lane.^2).^0.5)/t_lc*2-V_0 0.5*(V_b_end+V_c_end)]); */
        V_end = sqrt(S_end * S_end + w_lane * w_lane) / t_lc * 2.0 - speed;
        b_speed[0] = S_b_end - V_end;
        b_speed[1] = 0.0;
        b_S_b_end[0] = (S_d_end + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_d_end + maximum(b_speed) *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = S_min_dyn;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([S_e_end-t_re*V_end-l_veh S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh ((S_0+V_0*t_lc+0.5*a_max*t_lc*t_lc).^2-w_lane.^2).^0.5]); */
        S_min_dyn = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max * t_lc * t_lc;
        S_b_end_tmp = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re * V_end;
        b_S_b_end[0] = S_b_end_tmp;
        b_S_b_end[1] = S_b_end_tmp - (TargetLaneBehindVel * TargetLaneBehindVel
          - V_end * V_end) / (2.0 * CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = sqrt(S_min_dyn * S_min_dyn - w_lane * w_lane);
        TargetLaneFrontVel = d_minimum(b_S_b_end);
      }

      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if ((TargetLaneFrontVel > S_end) && (S_end > S_min)) {
        prereq5 = true;
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      if (((V_b_end - S_d_end) - Parameters.l_veh > S_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (TargetLaneBehindVel *
            TargetLaneBehindVel - S_b_end * S_b_end) / (2.0 *
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
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    } else {
      boolean_T guard2 = false;

      /*          V_a_end=max([0 v_a+(index_accel*a_min_comfort)*t_lc]); */
      /*          S_a_end=s_a+0.5*(V_a_end+v_a)*t_lc; */
      b_speed[0] = 0.0;
      speed_tmp = CalibrationVars->TrajPlanLaneChange.index_accel *
        CalibrationVars->TrajPlanLaneChange.a_min_comfort;
      b_speed[1] = v_e + speed_tmp * t_lc;
      TargetLaneBehindVel = maximum(b_speed);
      V_b_end = s_e + 0.5 * (TargetLaneBehindVel + v_e) * t_lc;
      b_speed[0] = ACC(v_max, v_e, s_e - s_d, v_d, 0.0,
                       CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                       CalibrationVars->ACC.d_wait2faultyCar,
                       CalibrationVars->ACC.tau_v_com,
                       CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                       CalibrationVars->ACC.tau_v_bre,
                       CalibrationVars->ACC.tau_v_emg,
                       CalibrationVars->ACC.tau_d_emg,
                       CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait) /
        CalibrationVars->TrajPlanLaneChange.a_max_comfort;
      b_speed[1] = 0.5;
      S_b_end_tmp = maximum(b_speed);
      b_speed[0] = 0.0;
      b_speed[1] = v_d + S_b_end_tmp *
        CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_b_end = maximum(b_speed);
      S_d_end = s_d + 0.5 * (S_b_end + v_d) * t_lc;
      t_mid = 0.5 * t_lc;
      S_b_end_tmp = v_d * t_lc;
      guard1 = false;
      guard2 = false;
      if (-s_d > S_b_end_tmp) {
        d = speed * t_lc;
        if (s_e <= d) {
          /* , */
          /*                  b车不存在 c车存在 */
          b_speed[0] = 0.5 * (speed + TargetLaneBehindVel);
          b_speed[1] = speed;
          V_end = c_minimum(b_speed);
          S_b_end_tmp = S_d_end + S_b_end *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = S_b_end_tmp + Parameters.l_veh;
          TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
          b_S_b_end[1] = (S_b_end_tmp + (V_end * V_end - S_b_end * S_b_end) /
                          TargetLaneFrontVel) + Parameters.l_veh;
          b_S_b_end[2] = d + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          S_min = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          S_b_end_tmp = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
            V_end;
          b_S_b_end[0] = S_b_end_tmp;
          b_S_b_end[1] = S_b_end_tmp - (TargetLaneBehindVel *
            TargetLaneBehindVel - V_end * V_end) / TargetLaneFrontVel;
          b_S_b_end[2] = d + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          TargetLaneFrontVel = d_minimum(b_S_b_end);
          b_S_b_end[0] = S_min;
          b_S_b_end[1] = TargetLaneFrontVel;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }

      if (guard2) {
        if ((-s_d <= S_b_end_tmp) && (s_e > speed * t_lc)) {
          /* , */
          /*                  b车存在 c车不存在 */
          b_speed[0] = 0.5 * (speed + S_b_end);
          b_speed[1] = speed;
          V_end = c_minimum(b_speed);
          S_b_end_tmp = S_d_end + S_b_end *
            CalibrationVars->TrajPlanLaneChange.t_re;
          b_S_b_end[0] = S_b_end_tmp + Parameters.l_veh;
          TargetLaneFrontVel = V_end * V_end;
          b_S_b_end[1] = (S_b_end_tmp + (TargetLaneFrontVel - S_b_end * S_b_end)
                          / (2.0 * CalibrationVars->TrajPlanLaneChange.a_min)) +
            Parameters.l_veh;
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
          S_min = b_maximum(b_S_b_end);

          /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
          S_b_end_tmp = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
            V_end;
          b_S_b_end[0] = S_b_end_tmp;
          b_S_b_end[1] = S_b_end_tmp - (TargetLaneBehindVel *
            TargetLaneBehindVel - TargetLaneFrontVel) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min);
          b_S_b_end[2] = speed * t_lc + 0.5 *
            CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
          TargetLaneFrontVel = d_minimum(b_S_b_end);
          b_S_b_end[0] = S_min;
          b_S_b_end[1] = TargetLaneFrontVel;
          b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
          S_end = median(b_S_b_end);
        } else if (-s_d > S_b_end_tmp) {
          S_end = speed * t_lc;
          if (s_e > S_end) {
            /* , */
            /*                  b车不存在 c车不存在 */
            V_end = speed;
            b_S_b_end[0] = (S_d_end + S_b_end *
                            CalibrationVars->TrajPlanLaneChange.t_re) +
              Parameters.l_veh;
            S_b_end_tmp = speed * speed;
            TargetLaneFrontVel = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
            b_S_b_end[1] = ((S_d_end + S_b_end *
                             CalibrationVars->TrajPlanLaneChange.t_re) +
                            (S_b_end_tmp - S_b_end * S_b_end) /
                            TargetLaneFrontVel) + Parameters.l_veh;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
            S_min = b_maximum(b_S_b_end);

            /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
            S_tlc = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re * speed;
            b_S_b_end[0] = S_tlc;
            b_S_b_end[1] = S_tlc - (TargetLaneBehindVel * TargetLaneBehindVel -
              S_b_end_tmp) / TargetLaneFrontVel;
            b_S_b_end[2] = speed * t_lc + 0.5 *
              CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
            TargetLaneFrontVel = d_minimum(b_S_b_end);
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
        b_speed[1] = 0.5 * (S_b_end + TargetLaneBehindVel);
        V_end = c_minimum(b_speed);
        b_S_b_end[0] = (S_d_end + S_b_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) +
          Parameters.l_veh;
        b_S_b_end[1] = ((S_d_end + S_b_end *
                         CalibrationVars->TrajPlanLaneChange.t_re) + (V_end *
          V_end - S_b_end * S_b_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min)) + Parameters.l_veh;
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc * t_lc;
        S_min = b_maximum(b_S_b_end);

        /*              S_max=min([(S_e_end-t_re*V_end-l_veh) S_e_end-V_end*t_re-(V_e_end.^2-V_end.^2)/(2*a_min)-l_veh S_0+V_0*t_lc+0.5*a_max_comfort*t_lc*t_lc]); */
        b_S_b_end[0] = V_b_end - CalibrationVars->TrajPlanLaneChange.t_re *
          V_end;
        b_S_b_end[1] = (V_b_end - V_end *
                        CalibrationVars->TrajPlanLaneChange.t_re) -
          (TargetLaneBehindVel * TargetLaneBehindVel - V_end * V_end) / (2.0 *
          CalibrationVars->TrajPlanLaneChange.a_min);
        b_S_b_end[2] = speed * t_lc + 0.5 *
          CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc * t_lc;
        TargetLaneFrontVel = d_minimum(b_S_b_end);
        b_S_b_end[0] = S_min;
        b_S_b_end[1] = TargetLaneFrontVel;
        b_S_b_end[2] = t_lc * 0.5 * (V_end + speed);
        S_end = median(b_S_b_end);
      }

      b_speed[0] = V_end;
      b_speed[1] = speed;
      S_tlc = CalibrationVars->TrajPlanLaneChange.a_max_comfort * t_lc;
      S_min_dyn = (S_tlc - fabs(V_end - speed)) / 2.0;

      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最大位移 */
      /*  换道过程中只进行匀加速和匀速或只进行匀减速和匀速 -> 最小位移 */
      /*          prereq2=(S_a_end>0.5*(S_0+S_end)); */
      if (TargetLaneFrontVel >= S_min) {
        S_b_end_tmp = 0.5 * CalibrationVars->TrajPlanLaneChange.a_max_comfort *
          t_lc * t_lc;
        d = S_min_dyn * S_min_dyn /
          CalibrationVars->TrajPlanLaneChange.a_max_comfort;
        if ((S_end <= (t_lc * c_minimum(b_speed) + S_b_end_tmp) - d) && (S_end >=
             (t_lc * maximum(b_speed) - S_b_end_tmp) + d)) {
          prereq5 = true;
        } else {
          prereq5 = false;
        }
      } else {
        prereq5 = false;
      }

      /*  距离路口过近时不允许换道 */
      /*  prereq6=(speed>=5 && d_veh2int>=speed*t_permit); % 速度较低时或距离路口过近时不允许换道 */
      if (((V_b_end - S_d_end) - Parameters.l_veh > S_b_end *
           CalibrationVars->TrajPlanLaneChange.t_re + (TargetLaneBehindVel *
            TargetLaneBehindVel - S_b_end * S_b_end) / (2.0 *
            CalibrationVars->TrajPlanLaneChange.a_min)) && (V_end >= speed +
           CalibrationVars->TrajPlanLaneChange.a_min_comfort * t_lc) && (V_end <=
           speed + S_tlc) && prereq5 && (d_veh2int >= S_end +
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int *
           Parameters.l_veh) && (s_d <= -Parameters.l_veh) && (s_e >= 0.0)) {
        b_speed[0] = 0.0;
        b_speed[1] = CurrentLaneFrontVel + speed_tmp * t_mid;
        if ((CurrentLaneFrontDis + 0.5 * (maximum(b_speed) + CurrentLaneFrontVel)
             * t_mid > 0.5 * S_end) && (CurrentLaneFrontDis >= 0.25 * t_lc *
             (0.75 * speed + 0.25 * V_end))) {
          CountLaneChange = 1;
          TargetLaneIndex = BackupTargetLaneIndex;
          CurrentTargetLaneIndex = BackupTargetLaneIndex;
        }
      }
    }
  }

  /*  para=[]; */
  /*  path generation */
  /*  if TargetLaneIndex<=CurrentLaneIndex */
  /*      w_lane=w_lane_left-(pos_l-pos_l_CurrentLane); */
  /*  else */
  /*      w_lane=w_lane_right+(pos_l-pos_l_CurrentLane); */
  /*  end */
  if (CountLaneChange == 1) {
    double b[100];
    double x1[100];
    double dv[9];
    double para[3];
    t_lc_traj = t_lc;
    S_end -= 0.5 * Parameters.l_veh;

    /* 由车头位置的S_end改为车中心位置的S_end，用于轨迹生成 */
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
    b_S_b_end[2] = w_lane * 1.0E+6;
    mldivide(dv, b_S_b_end, para);

    /*  SpeedPlanner */
    d_linspace(S_end, x1);
    S_min_dyn = 3.0 * para[0];
    S_b_end = 4.0 * para[1];
    S_tlc = 5.0 * para[2];
    for (SwitchACC = 0; SwitchACC < 100; SwitchACC++) {
      S_b_end_tmp = x1[SwitchACC];
      S_b_end_tmp = ((S_min_dyn * (S_b_end_tmp * S_b_end_tmp) + S_b_end *
                      rt_powd_snf(S_b_end_tmp, 3.0)) + S_tlc * rt_powd_snf
                     (S_b_end_tmp, 4.0)) / 1.0E+6;
      b[SwitchACC] = sqrt(S_b_end_tmp * S_b_end_tmp + 1.0);
    }

    double dv1[16];
    S_tlc = trapz(x1, b);
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
    S_b_end_tmp = t_lc * t_lc;
    dv1[10] = S_b_end_tmp;
    dv1[14] = rt_powd_snf(t_lc, 3.0);
    dv1[3] = 0.0;
    dv1[7] = 1.0;
    dv1[11] = 2.0 * t_lc;
    dv1[15] = 3.0 * S_b_end_tmp;
    b_mldivide(dv1, para_ST);

    /*  ST参数到轨迹 */
    S_b_end_tmp = rt_roundd_snf(t_lc / 0.05);
    i = (int)(S_b_end_tmp + 1.0);
    if ((int)(S_b_end_tmp + 1.0) - 1 >= 0) {
      b_speed[0] = 0.0;
      b_speed[1] = S_tlc;
    }

    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      S_tlc = 0.05 * (((double)SwitchACC + 1.0) - 1.0);
      S_b_end = S_tlc * S_tlc;
      S_traj[SwitchACC] = ((para_ST[0] + para_ST[1] * S_tlc) + para_ST[2] *
                           S_b_end) + para_ST[3] * rt_powd_snf(S_tlc, 3.0);
      V_traj[SwitchACC] = (para_ST[1] + para_ST[2] * 2.0 * S_tlc) + para_ST[3] *
        3.0 * S_b_end;
      b_fzero(para, S_traj, (double)SwitchACC + 1.0, b_speed, &X_traj[SwitchACC],
              &S_tlc, &S_b_end);
    }

    i = (int)(t_lc / 0.05);
    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      d = X_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC] = pos_s + d;
      i3 = CurrentLaneIndex - TargetLaneIndex;
      S_tlc = rt_powd_snf(d, 3.0);
      S_b_end = rt_powd_snf(d, 4.0);
      GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 120] = pos_l +
        (double)i3 * ((para[0] * S_tlc + para[1] * S_b_end) + para[2] *
                      rt_powd_snf(d, 5.0)) * 1.0E-6;
      d = atan(((para[0] * 3.0 * (d * d) + para[1] * 4.0 * S_tlc) + para[2] *
                5.0 * S_b_end) * 1.0E-6);
      S_tlc = 90.0 - (double)i3 * 180.0 / 3.1415926535897931 * d;
      GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 240] = S_tlc;
      d *= 57.295779513082323;
      S_b_end = d;
      b_cosd(&S_b_end);
      S_d_end = V_traj[SwitchACC + 1];
      GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 360] = S_d_end *
        S_b_end;
      b_sind(&d);
      GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 480] = (double)i3 *
        S_d_end * d;
      if ((unsigned int)SwitchACC + 1U == 1U) {
        GlobVars->TrajPlanLaneChange.laneChangePath[600] = 0.0;
      } else {
        GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 600] = (S_tlc -
          GlobVars->TrajPlanLaneChange.laneChangePath[SwitchACC + 239]) / 0.05;
      }
    }

    if (GlobVars->TrajPlanLaneChange.laneChangePath[(int)S_b_end_tmp + 359] ==
        0.0) {
      GlobVars->TrajPlanLaneChange.laneChangePath[(int)rt_roundd_snf(t_lc / 0.05)
        + 359] = V_traj[(int)(rt_roundd_snf(t_lc / 0.05) + 1.0) - 1];
    }

    i1 = GlobVars->TrajPlanLaneChange.durationLaneChange;
    i = i1 + 1;
    if (i1 + 1 > 32767) {
      i = 32767;
    }

    DurationLaneChange = (short)i;
    CountLaneChange = 2;
  }

  /* 换道中止判断 */
  if (DurationLaneChange != 0) {
    if (CurrentTargetLaneIndex <= CurrentLaneIndex) {
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

    S_d_end = rt_roundd_snf(t_lc_traj / 0.05);
    S_end = GlobVars->TrajPlanLaneChange.laneChangePath[(int)S_d_end - 1] + 0.5 *
      Parameters.l_veh;

    /* 此处S_end为车头位置，用于下方的重归划判断 */
    i = DurationLaneChange - 1;
    if (DurationLaneChange - 1 < -32768) {
      i = -32768;
    }

    b_speed[0] = t_lc_traj - (double)i * 0.1;
    b_speed[1] = 0.0;
    S_tlc = maximum(b_speed);
    b_speed[0] = 0.0;
    b_speed[1] = TargetLaneFrontVel + 0.0 *
      CalibrationVars->TrajPlanLaneChange.a_min_comfort * S_tlc;
    V_c_end = maximum(b_speed);
    S_c_end = (TargetLaneFrontDis + pos_s) + 0.5 * (V_c_end + TargetLaneFrontVel)
      * S_tlc;
    b_speed[0] = 0.0;
    b_speed[1] = TargetLaneBehindVel + 0.0 *
      CalibrationVars->TrajPlanLaneChange.a_max_comfort * S_tlc;
    V_b_end = maximum(b_speed);

    /* 20220706 */
    if ((CurrentLaneIndex != CurrentTargetLaneIndex) && (DurationLaneChange != 0))
    {
      speed_tmp = ((TargetLaneBehindDis + pos_s) + 0.5 * (V_b_end +
        TargetLaneBehindVel) * S_tlc) + V_b_end *
        CalibrationVars->TrajPlanLaneChange.t_re;
      b_speed[0] = speed_tmp + Parameters.l_veh;
      S_d_end = GlobVars->TrajPlanLaneChange.laneChangePath[(int)S_d_end + 359];
      S_tlc = S_d_end * S_d_end;
      S_b_end = 2.0 * CalibrationVars->TrajPlanLaneChange.a_min;
      b_speed[1] = (speed_tmp + (S_tlc - V_b_end * V_b_end) / S_b_end) +
        Parameters.l_veh;
      guard1 = false;
      if (rt_roundd_snf(S_end * 1000.0) < rt_roundd_snf(maximum(b_speed) *
           1000.0)) {
        guard1 = true;
      } else {
        b_speed[0] = S_c_end - CalibrationVars->TrajPlanLaneChange.t_re *
          S_d_end;
        b_speed[1] = S_c_end - (V_c_end * V_c_end - S_tlc) / S_b_end;
        if (rt_roundd_snf(S_end * 1000.0) > rt_roundd_snf(c_minimum(b_speed) *
             1000.0)) {
          guard1 = true;
        }
      }

      if (guard1) {
        /* 判断换道条件是否满足，不满足换道终止 */
        /*          GlobVars.TrajPlanLaneChange.CountLaneChange=0*GlobVars.TrajPlanLaneChange.CountLaneChange;%换道终止s */
        /*          GlobVars.TrajPlanLaneChange.DurationLaneChange=0*DurationLaneChange; */
        CountLaneChange = 0;
        DurationLaneChange = 0;
        CurrentTargetLaneIndex = CurrentLaneIndex;
      }
    }
  }

  if ((DurationLaneChange > 0) && (DurationLaneChange <= rt_roundd_snf(10.0 *
        t_lc_traj))) {
    /*  for count_1=1:1:2*(21-DurationLaneChange) */
    S_b_end_tmp = rt_roundd_snf(t_lc_traj / 0.1);
    d = (S_b_end_tmp + 1.0) - (double)DurationLaneChange;
    if (d < 32768.0) {
      if (d >= -32768.0) {
        i1 = (short)d;
      } else {
        i1 = MIN_int16_T;
      }
    } else if (d >= 32768.0) {
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
    if (i - 1 >= 0) {
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

    for (SwitchACC = 0; SwitchACC < i; SwitchACC++) {
      i3 = (i2 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 - 3];
      i3 = (count_2 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC + 1] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 +
        117];
      i3 = (i4 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC + 2] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 +
        237];
      i3 = (i5 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC + 3] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 +
        357];
      i3 = (i6 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC + 4] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 +
        477];
      i3 = (i7 + SwitchACC) + 1;
      if (i3 > 32767) {
        i3 = 32767;
      }

      traj[6 * SwitchACC + 5] = GlobVars->TrajPlanLaneChange.laneChangePath[i3 +
        597];
    }

    /*  for count_2=2*(21-DurationLaneChange)+1:1:80 */
    i = i1 + 1;
    if (i1 + 1 > 32767) {
      i = 32767;
    }

    i2 = (short)i;
    if (i2 <= 120) {
      traj_tmp = (int)(2.0 * rt_roundd_snf(t_lc_traj / 0.1));
      d = (rt_roundd_snf(t_lc_traj / 0.1) + 1.0) - (double)DurationLaneChange;
      if (d < 32768.0) {
        if (d >= -32768.0) {
          count_2 = (short)d;
        } else {
          count_2 = MIN_int16_T;
        }
      } else if (d >= 32768.0) {
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
      SwitchACC = 6 * (count_2 - 1);
      traj[SwitchACC] = GlobVars->TrajPlanLaneChange.laneChangePath[(int)(2.0 *
        S_b_end_tmp) - 1] + traj[6 * (i1 - 1) + 3] * 0.05 * (double)(count_2 -
        i1);
      traj[SwitchACC + 1] = GlobVars->TrajPlanLaneChange.laneChangePath[traj_tmp
        + 119];
      traj[SwitchACC + 2] = GlobVars->TrajPlanLaneChange.laneChangePath[traj_tmp
        + 239];
      traj[SwitchACC + 3] = traj[6 * (i8 - 1) + 3];
      traj[SwitchACC + 4] = 0.0;
      traj[SwitchACC + 5] = 0.0;
    }

    if (AEBActive == 5) {
      CountLaneChange = 0;
      DurationLaneChange = 0;
    } else {
      i = DurationLaneChange + 1;
      if (DurationLaneChange + 1 > 32767) {
        i = 32767;
      }

      DurationLaneChange = (short)i;
    }

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
                    CalibrationVars->ACC.a_min,
                    CalibrationVars->ACC.d_wait2faultyCar,
                    CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                    CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                    CalibrationVars->ACC.tau_v_emg,
                    CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                    CalibrationVars->ACC.d_wait);
    } else {
      *a_soll = ACC(v_max, CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                    b_wait, CalibrationVars->ACC.a_max,
                    CalibrationVars->ACC.a_min,
                    CalibrationVars->ACC.d_wait2faultyCar,
                    CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                    CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
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

  GlobVars->TrajPlanLaneChange.countLaneChange = CountLaneChange;
  GlobVars->TrajPlanLaneChange.durationLaneChange = DurationLaneChange;
  GlobVars->TrajPlanLaneChange.t_lc_traj = t_lc_traj;
  GlobVars->TrajPlanLaneChange.currentTargetLaneIndex = CurrentTargetLaneIndex;

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
}

/*
 * ,
 *
 * Arguments    : double a_soll
 *                double speed
 *                double pos_s
 *                double pos_l
 *                double pos_psi
 *                double pos_l_CurrentLane
 *                double stopdistance
 *                double SampleTime
 *                double a_soll_ACC
 *                double CurrentLaneFrontVel
 *                TypeGlobVars *GlobVars
 *                double c_CalibrationVars_TrajPlanLaneC
 *                double d_CalibrationVars_TrajPlanLaneC
 *                double c_CalibrationVars_UrbanPlanner_
 *                double Parameter_l_veh
 *                double traj_s[80]
 *                double traj_l[80]
 *                double traj_psi[80]
 *                double traj_vs[80]
 *                double traj_vl[80]
 *                double traj_omega[80]
 * Return Type  : void
 */
static void TrajPlanLaneChange_RePlan(double a_soll, double speed, double pos_s,
  double pos_l, double pos_psi, double pos_l_CurrentLane, double stopdistance,
  double SampleTime, double a_soll_ACC, double CurrentLaneFrontVel, TypeGlobVars
  *GlobVars, double c_CalibrationVars_TrajPlanLaneC, double
  d_CalibrationVars_TrajPlanLaneC, double c_CalibrationVars_UrbanPlanner_,
  double Parameter_l_veh, double traj_s[80], double traj_l[80], double traj_psi
  [80], double traj_vs[80], double traj_vl[80], double traj_omega[80])
{
  static const signed char b[16] = { 0, 0, 0, 0, 3, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1,
    0 };

  double traj[480];
  double b_paraNew_coefs[16];
  double para4_coefs[16];
  double para3_coefs[12];
  double paraNew_coefs[12];
  double fprime_coefs[9];
  double para4_breaks[5];
  double para3_breaks[4];
  double CenterS;
  double Rreplan;
  double Rreplan_extre;
  double Rreplan_tmp;
  double S_end;
  double d;
  int IsStopSpeedPlan;
  int iter;
  int para_breaks_size_idx_1;
  int para_pieces;
  short DurationLaneChange_RePlan;

  /* globalVariable---------------------------------------------------------------------------------------------------------------------- */
  DurationLaneChange_RePlan =
    GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan;

  /*  初始值15度 */
  /*  默认为4 */
  /* 初值------------------------------------------------------------ */
  for (iter = 0; iter < 5; iter++) {
    para4_breaks[iter] = GlobVars->TrajPlanLaneChange_RePlan.para1[iter];
  }

  memcpy(&para4_coefs[0], &GlobVars->TrajPlanLaneChange_RePlan.para2[0], 16U *
         sizeof(double));
  for (iter = 0; iter < 4; iter++) {
    para3_breaks[iter] = GlobVars->TrajPlanLaneChange_RePlan.para1[iter];
    IsStopSpeedPlan = iter << 2;
    para3_coefs[3 * iter] = GlobVars->
      TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan];
    para3_coefs[3 * iter + 1] = GlobVars->
      TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan + 1];
    para3_coefs[3 * iter + 2] = GlobVars->
      TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan + 2];
  }

  /* ---------------------------------------------------------------- */
  /*  para.form='pp'; */
  /*  if GlobVars.TrajPlanLaneChange_RePlan.para3==4 */
  /*      para.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1; */
  /*      para.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2; */
  /*      para.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3;  */
  /*  else */
  /*      para.breaks=GlobVars.TrajPlanLaneChange_RePlan.para1(1,1:4); */
  /*      para.coefs=GlobVars.TrajPlanLaneChange_RePlan.para2(1:3,:); */
  /*      para.pieces=GlobVars.TrajPlanLaneChange_RePlan.para3; */
  /*  end */
  /*  para.order=4; */
  /*  para.dim=1; */
  S_end = GlobVars->TrajPlanLaneChange_RePlan.s_end;

  /*  初始值0 */
  memset(&traj[0], 0, 480U * sizeof(double));

  /*  过渡路径生成 */
  if (GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan == 0) {
    double para_breaks_data[5];
    d = d_CalibrationVars_TrajPlanLaneC;
    b_cotd(&d);
    Rreplan_tmp = speed * speed;
    Rreplan = fmax(Rreplan_tmp / c_CalibrationVars_TrajPlanLaneC,
                   Parameter_l_veh * d);
    ReplanCenter(Rreplan, pos_s, pos_l, pos_l_CurrentLane, pos_psi, &CenterS,
                 &Rreplan_extre);
    Rreplan_extre = pos_l_CurrentLane - Rreplan_extre;
    S_end = CenterS + sqrt(Rreplan * Rreplan - Rreplan_extre * Rreplan_extre);

    /*      if pos_psi~=90 && (pos_psi-90)*(pos_l-pos_l_CurrentLane)<=0 % 输入para函数的为三个点 */
    if ((fabs(pos_psi - 90.0) >= 0.01) && ((pos_psi - 90.0) * (pos_l -
          pos_l_CurrentLane) <= 0.0)) {
      double b_paraNew_breaks[5];
      double c_pos_l[5];

      /*  输入para函数的为三个点 */
      d = d_CalibrationVars_TrajPlanLaneC * 2.0;
      b_cotd(&d);
      Rreplan_extre = fmax(Rreplan_tmp / (c_CalibrationVars_TrajPlanLaneC * 2.5),
                           Parameter_l_veh * d);
      ReplanCenter(Rreplan_extre, pos_s, pos_l, pos_l_CurrentLane, pos_psi,
                   &Rreplan, &CenterS);

      /* 计算三次曲线参数 */
      d = 90.0 - pos_psi;
      b_cosd(&d);
      Rreplan_tmp = 90.0 - pos_psi;
      b_sind(&Rreplan_tmp);
      para_breaks_data[0] = pos_s - 0.1 * d;
      para_breaks_data[1] = pos_s;
      para_breaks_data[2] = Rreplan;
      para_breaks_data[3] = S_end;
      para_breaks_data[4] = S_end + 1.0;
      c_pos_l[0] = pos_l - Rreplan_tmp * 0.1;
      c_pos_l[1] = pos_l;
      if (rtIsNaN(pos_psi - 90.0)) {
        d = rtNaN;
      } else if (pos_psi - 90.0 < 0.0) {
        d = -1.0;
      } else {
        d = (pos_psi - 90.0 > 0.0);
      }

      c_pos_l[2] = CenterS - Rreplan_extre * d;
      c_pos_l[3] = pos_l_CurrentLane;
      c_pos_l[4] = pos_l_CurrentLane;
      pchip(para_breaks_data, c_pos_l, b_paraNew_breaks, b_paraNew_coefs);
      para_breaks_size_idx_1 = 5;
      for (iter = 0; iter < 5; iter++) {
        para_breaks_data[iter] = b_paraNew_breaks[iter];
      }

      IsStopSpeedPlan = 4;
      para_pieces = 4;
    } else {
      double b_pos_l[4];
      double b_pos_s[4];
      double paraNew_breaks[4];

      /*  输入para函数的为两个点 */
      /* 计算三次曲线参数 */
      d = 90.0 - pos_psi;
      b_cosd(&d);
      Rreplan_tmp = 90.0 - pos_psi;
      b_sind(&Rreplan_tmp);
      b_pos_s[0] = pos_s - 0.1 * d;
      b_pos_s[1] = pos_s;
      b_pos_s[2] = S_end;
      b_pos_s[3] = S_end + 1.0;
      b_pos_l[0] = pos_l - Rreplan_tmp * 0.1;
      b_pos_l[1] = pos_l;
      b_pos_l[2] = pos_l_CurrentLane;
      b_pos_l[3] = pos_l_CurrentLane;
      b_pchip(b_pos_s, b_pos_l, paraNew_breaks, paraNew_coefs);
      para_breaks_size_idx_1 = 4;
      para_breaks_data[0] = paraNew_breaks[0];
      para_breaks_data[1] = paraNew_breaks[1];
      para_breaks_data[2] = paraNew_breaks[2];
      para_breaks_data[3] = paraNew_breaks[3];
      IsStopSpeedPlan = 3;
      memcpy(&b_paraNew_coefs[0], &paraNew_coefs[0], 12U * sizeof(double));
      para_pieces = 3;
    }

    for (iter = 0; iter < 5; iter++) {
      if (iter + 1 <= para_breaks_size_idx_1) {
        GlobVars->TrajPlanLaneChange_RePlan.para1[iter] = para_breaks_data[iter];
      } else {
        GlobVars->TrajPlanLaneChange_RePlan.para1[4] = 0.0;
      }
    }

    for (iter = 0; iter < 4; iter++) {
      if (iter + 1 <= IsStopSpeedPlan) {
        GlobVars->TrajPlanLaneChange_RePlan.para2[iter] = b_paraNew_coefs[iter];
        GlobVars->TrajPlanLaneChange_RePlan.para2[iter + 4] =
          b_paraNew_coefs[iter + IsStopSpeedPlan];
        GlobVars->TrajPlanLaneChange_RePlan.para2[iter + 8] =
          b_paraNew_coefs[iter + IsStopSpeedPlan * 2];
        GlobVars->TrajPlanLaneChange_RePlan.para2[iter + 12] =
          b_paraNew_coefs[iter + IsStopSpeedPlan * 3];
      } else {
        GlobVars->TrajPlanLaneChange_RePlan.para2[3] = 0.0;
        GlobVars->TrajPlanLaneChange_RePlan.para2[7] = 0.0;
        GlobVars->TrajPlanLaneChange_RePlan.para2[11] = 0.0;
        GlobVars->TrajPlanLaneChange_RePlan.para2[15] = 0.0;
      }
    }

    /*      GlobVars.TrajPlanLaneChange_RePlan.para1=[para.breaks zeros([1 5-length(para.breaks)])]; */
    /*      GlobVars.TrajPlanLaneChange_RePlan.para2=[para.coefs;zeros([4-size(para.coefs,1) 4])]; */
    GlobVars->TrajPlanLaneChange_RePlan.para3 = para_pieces;
    GlobVars->TrajPlanLaneChange_RePlan.s_end = S_end;
    DurationLaneChange_RePlan = 1;

    /*  画图 */
    /*      s=linspace(pos_s,S_end,50); */
    /*      l = ppval(para,s); */
    /*      plot(s-pos_s,l,'r') */
    /*      hold on */
    /*      x1=pos_s:0.1:S_end; */
    /*      y1=sqrt(Rreplan.^2-(x1-CenterS).^2)+CenterL; */
    /*      plot(x1-pos_s,y1,'k--') */
    /*      hold on */
    /*      x2=pos_s:0.1:CenterS_extre+sqrt(Rreplan_extre.^2-(pos_l_CurrentLane-CenterL_extre).^2); */
    /*      y2=sqrt(Rreplan_extre.^2-(x2-CenterS_extre).^2)+CenterL_extre; */
    /*      plot(x2-pos_s,y2,'k--') */
    /*      axis([0 S_end-pos_s+0.5 0 8]) */
    /*      axis equal */
    /*      ax=gca; */
    /*      ax.XAxisLocation='origin'; */
    /*      ax.YAxisLocation='origin'; */
    /*      ax.Box='off'; */
  }

  if (GlobVars->TrajPlanLaneChange_RePlan.para3 == 4.0) {
    for (iter = 0; iter < 5; iter++) {
      para4_breaks[iter] = GlobVars->TrajPlanLaneChange_RePlan.para1[iter];
    }

    memcpy(&para4_coefs[0], &GlobVars->TrajPlanLaneChange_RePlan.para2[0], 16U *
           sizeof(double));
  } else {
    for (iter = 0; iter < 4; iter++) {
      para3_breaks[iter] = GlobVars->TrajPlanLaneChange_RePlan.para1[iter];
      IsStopSpeedPlan = iter << 2;
      para3_coefs[3 * iter] = GlobVars->
        TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan];
      para3_coefs[3 * iter + 1] = GlobVars->
        TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan + 1];
      para3_coefs[3 * iter + 2] = GlobVars->
        TrajPlanLaneChange_RePlan.para2[IsStopSpeedPlan + 2];
    }
  }

  if ((DurationLaneChange_RePlan > 0) && (pos_s < S_end)) {
    double a[50];
    double varargin_1_tmp[50];
    double y[50];
    double b_GlobVars[3];
    double c_pos_s[2];
    double length_count_1;
    double v_count_1;

    /* 生成轨迹 */
    /* ------------------------------------------------------------------------------------------------------------------------------  */
    IsStopSpeedPlan = 0;
    if (stopdistance < 200.0) {
      d = speed * speed;
      if (d / 8.0 <= stopdistance) {
        d *= 0.44444444444444442;
        Rreplan_tmp = -(d / (0.66666666666666663 * stopdistance));
        if ((Rreplan_tmp <= a_soll_ACC) || (CurrentLaneFrontVel < 0.2)) {
          a_soll = fmax(a_soll, Rreplan_tmp);
          if (GlobVars->Decider.a_sollpre2traj != 100.0) {
            if (a_soll > -2.0) {
              b_GlobVars[0] = GlobVars->Decider.a_sollpre2traj +
                c_CalibrationVars_UrbanPlanner_ * SampleTime;
              b_GlobVars[1] = a_soll;
              b_GlobVars[2] = GlobVars->Decider.a_sollpre2traj -
                c_CalibrationVars_UrbanPlanner_ * SampleTime;
              a_soll = median(b_GlobVars);
            } else {
              b_GlobVars[0] = GlobVars->Decider.a_sollpre2traj +
                c_CalibrationVars_UrbanPlanner_ * SampleTime;
              b_GlobVars[1] = a_soll;
              b_GlobVars[2] = GlobVars->Decider.a_sollpre2traj - 2.5 *
                c_CalibrationVars_UrbanPlanner_ * SampleTime;
              a_soll = median(b_GlobVars);
            }
          }

          if (a_soll >= Rreplan_tmp) {
            double jerk;
            double tend;
            tend = (3.0 * sqrt(fmax(0.0, d + 0.66666666666666663 * a_soll *
                      stopdistance)) - 2.0 * speed) / (a_soll +
              2.2204460492503131E-16);
            jerk = -2.0 * (speed + a_soll * tend) / (tend * tend);
            for (para_breaks_size_idx_1 = 0; para_breaks_size_idx_1 < 80;
                 para_breaks_size_idx_1++) {
              Rreplan = 0.05 * ((double)para_breaks_size_idx_1 + 1.0);
              if (Rreplan <= tend) {
                CenterS = Rreplan * Rreplan;
                v_count_1 = (speed + a_soll * Rreplan) + 0.5 * jerk * CenterS;
                length_count_1 = (speed * Rreplan + 0.5 * a_soll * CenterS) +
                  0.16666666666666666 * jerk * rt_powd_snf(Rreplan, 3.0);
              } else {
                v_count_1 = 0.0;
                length_count_1 = stopdistance;
              }

              if (GlobVars->TrajPlanLaneChange_RePlan.para3 == 4.0) {
                /* 计算沿曲线给定长度的点位置 */
                /*  fprime = fnder(para,1); */
                /* fprime = fnder(para,1);--------------------------------------------------- */
                /*  fprime=para; */
                /*  fprime.coefs=dcoefs; */
                /*  fprime.order=para.order-1; */
                for (iter = 0; iter < 4; iter++) {
                  d = para4_coefs[iter];
                  Rreplan_tmp = para4_coefs[iter + 4];
                  Rreplan = para4_coefs[iter + 8];
                  CenterS = para4_coefs[iter + 12];
                  for (para_pieces = 0; para_pieces < 4; para_pieces++) {
                    IsStopSpeedPlan = para_pieces << 2;
                    b_paraNew_coefs[iter + IsStopSpeedPlan] = ((d * (double)
                      b[IsStopSpeedPlan] + Rreplan_tmp * (double)
                      b[IsStopSpeedPlan + 1]) + Rreplan * (double)
                      b[IsStopSpeedPlan + 2]) + CenterS * (double)
                      b[IsStopSpeedPlan + 3];
                  }
                }

                memcpy(&paraNew_coefs[0], &b_paraNew_coefs[4], 12U * sizeof
                       (double));

                /* -------------------------------------------------------------------------- */
                linspace(pos_s, S_end, varargin_1_tmp);
                ppval(para4_breaks, paraNew_coefs, varargin_1_tmp, a);
                for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan
                     ++) {
                  d = a[IsStopSpeedPlan];
                  y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
                }

                if (b_trapz(varargin_1_tmp, y) >= length_count_1) {
                  c_pos_s[0] = pos_s;
                  c_pos_s[1] = S_end;
                  c_fzero(pos_s, para4_breaks, paraNew_coefs, length_count_1,
                          c_pos_s, &Rreplan_extre, &Rreplan, &CenterS);
                  traj[6 * para_breaks_size_idx_1] = Rreplan_extre;
                  d = b_ppval(para4_breaks, para4_coefs, Rreplan_extre);
                  Rreplan_tmp = 90.0 - 57.295779513082323 * atan(c_ppval
                    (para4_breaks, paraNew_coefs, Rreplan_extre));
                } else {
                  linspace(pos_s, S_end, varargin_1_tmp);
                  ppval(para4_breaks, paraNew_coefs, varargin_1_tmp, a);
                  for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50;
                       IsStopSpeedPlan++) {
                    d = a[IsStopSpeedPlan];
                    y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
                  }

                  traj[6 * para_breaks_size_idx_1] = (length_count_1 - b_trapz
                    (varargin_1_tmp, y)) + S_end;
                  d = pos_l_CurrentLane;
                  Rreplan_tmp = 90.0;
                }

                traj[6 * para_breaks_size_idx_1 + 1] = d;
                traj[6 * para_breaks_size_idx_1 + 2] = Rreplan_tmp;
              } else {
                /* 计算沿曲线给定长度的点位置 */
                /*  fprime = fnder(para,1); */
                /* fprime = fnder(para,1);--------------------------------------------------- */
                /*  fprime=para; */
                /*  fprime.coefs=dcoefs; */
                /*  fprime.order=para.order-1; */
                for (iter = 0; iter < 3; iter++) {
                  d = para3_coefs[iter];
                  Rreplan_tmp = para3_coefs[iter + 3];
                  Rreplan = para3_coefs[iter + 6];
                  CenterS = para3_coefs[iter + 9];
                  for (para_pieces = 0; para_pieces < 4; para_pieces++) {
                    IsStopSpeedPlan = para_pieces << 2;
                    paraNew_coefs[iter + 3 * para_pieces] = ((d * (double)
                      b[IsStopSpeedPlan] + Rreplan_tmp * (double)
                      b[IsStopSpeedPlan + 1]) + Rreplan * (double)
                      b[IsStopSpeedPlan + 2]) + CenterS * (double)
                      b[IsStopSpeedPlan + 3];
                  }
                }

                memcpy(&fprime_coefs[0], &paraNew_coefs[3], 9U * sizeof(double));

                /* -------------------------------------------------------------------------- */
                linspace(pos_s, S_end, varargin_1_tmp);
                d_ppval(para3_breaks, fprime_coefs, varargin_1_tmp, a);
                for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan
                     ++) {
                  d = a[IsStopSpeedPlan];
                  y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
                }

                if (b_trapz(varargin_1_tmp, y) >= length_count_1) {
                  c_pos_s[0] = pos_s;
                  c_pos_s[1] = S_end;
                  d_fzero(pos_s, para3_breaks, fprime_coefs, length_count_1,
                          c_pos_s, &Rreplan_extre, &Rreplan, &CenterS);
                  traj[6 * para_breaks_size_idx_1] = Rreplan_extre;
                  d = e_ppval(para3_breaks, para3_coefs, Rreplan_extre);
                  Rreplan_tmp = 90.0 - 57.295779513082323 * atan(f_ppval
                    (para3_breaks, fprime_coefs, Rreplan_extre));
                } else {
                  linspace(pos_s, S_end, varargin_1_tmp);
                  d_ppval(para3_breaks, fprime_coefs, varargin_1_tmp, a);
                  for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50;
                       IsStopSpeedPlan++) {
                    d = a[IsStopSpeedPlan];
                    y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
                  }

                  traj[6 * para_breaks_size_idx_1] = (length_count_1 - b_trapz
                    (varargin_1_tmp, y)) + S_end;
                  d = pos_l_CurrentLane;
                  Rreplan_tmp = 90.0;
                }

                traj[6 * para_breaks_size_idx_1 + 1] = d;
                traj[6 * para_breaks_size_idx_1 + 2] = Rreplan_tmp;
              }

              d = 90.0 - Rreplan_tmp;
              b_cosd(&d);
              traj[6 * para_breaks_size_idx_1 + 3] = v_count_1 * d;
              d = 90.0 - Rreplan_tmp;
              b_sind(&d);
              traj[6 * para_breaks_size_idx_1 + 4] = v_count_1 * d;
              if (para_breaks_size_idx_1 + 1 == 1) {
                traj[5] = 0.0;
              } else {
                traj[6 * para_breaks_size_idx_1 + 5] = (Rreplan_tmp - traj[6 *
                  (para_breaks_size_idx_1 - 1) + 2]) / 0.05;
              }
            }

            IsStopSpeedPlan = 1;
          }
        }
      }
    }

    /* ------------------------------------------------------------------------------------------------------------------------------ */
    if (IsStopSpeedPlan == 0) {
      if (GlobVars->Decider.a_sollpre2traj != 100.0) {
        if (a_soll > -2.0) {
          b_GlobVars[0] = GlobVars->Decider.a_sollpre2traj +
            c_CalibrationVars_UrbanPlanner_ * SampleTime;
          b_GlobVars[1] = a_soll;
          b_GlobVars[2] = GlobVars->Decider.a_sollpre2traj -
            c_CalibrationVars_UrbanPlanner_ * SampleTime;
          a_soll = median(b_GlobVars);
        } else {
          b_GlobVars[0] = GlobVars->Decider.a_sollpre2traj +
            c_CalibrationVars_UrbanPlanner_ * SampleTime;
          b_GlobVars[1] = a_soll;
          b_GlobVars[2] = GlobVars->Decider.a_sollpre2traj - 2.5 *
            c_CalibrationVars_UrbanPlanner_ * SampleTime;
          a_soll = median(b_GlobVars);
        }
      }

      for (para_breaks_size_idx_1 = 0; para_breaks_size_idx_1 < 80;
           para_breaks_size_idx_1++) {
        Rreplan = 0.05 * ((double)para_breaks_size_idx_1 + 1.0);
        c_pos_s[0] = 0.0;
        c_pos_s[1] = speed + a_soll * Rreplan;
        v_count_1 = maximum(c_pos_s);
        if (v_count_1 == 0.0) {
          length_count_1 = (0.0 - speed * speed) / (2.0 * a_soll +
            2.2204460492503131E-16);
        } else {
          length_count_1 = (v_count_1 + speed) * Rreplan / 2.0;
        }

        if (GlobVars->TrajPlanLaneChange_RePlan.para3 == 4.0) {
          /* 计算沿曲线给定长度的点位置 */
          /*  fprime = fnder(para,1); */
          /* fprime = fnder(para,1);--------------------------------------------------- */
          /*  fprime=para; */
          /*  fprime.coefs=dcoefs; */
          /*  fprime.order=para.order-1; */
          for (iter = 0; iter < 4; iter++) {
            d = para4_coefs[iter];
            Rreplan_tmp = para4_coefs[iter + 4];
            Rreplan = para4_coefs[iter + 8];
            CenterS = para4_coefs[iter + 12];
            for (para_pieces = 0; para_pieces < 4; para_pieces++) {
              IsStopSpeedPlan = para_pieces << 2;
              b_paraNew_coefs[iter + IsStopSpeedPlan] = ((d * (double)
                b[IsStopSpeedPlan] + Rreplan_tmp * (double)b[IsStopSpeedPlan + 1])
                + Rreplan * (double)b[IsStopSpeedPlan + 2]) + CenterS * (double)
                b[IsStopSpeedPlan + 3];
            }
          }

          memcpy(&paraNew_coefs[0], &b_paraNew_coefs[4], 12U * sizeof(double));

          /* -------------------------------------------------------------------------- */
          linspace(pos_s, S_end, varargin_1_tmp);
          ppval(para4_breaks, paraNew_coefs, varargin_1_tmp, a);
          for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan++) {
            d = a[IsStopSpeedPlan];
            y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
          }

          if (b_trapz(varargin_1_tmp, y) >= length_count_1) {
            c_pos_s[0] = pos_s;
            c_pos_s[1] = S_end;
            c_fzero(pos_s, para4_breaks, paraNew_coefs, length_count_1, c_pos_s,
                    &Rreplan_extre, &Rreplan, &CenterS);
            traj[6 * para_breaks_size_idx_1] = Rreplan_extre;
            d = b_ppval(para4_breaks, para4_coefs, Rreplan_extre);
            Rreplan_tmp = 90.0 - 57.295779513082323 * atan(c_ppval(para4_breaks,
              paraNew_coefs, Rreplan_extre));
          } else {
            linspace(pos_s, S_end, varargin_1_tmp);
            ppval(para4_breaks, paraNew_coefs, varargin_1_tmp, a);
            for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan++) {
              d = a[IsStopSpeedPlan];
              y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
            }

            traj[6 * para_breaks_size_idx_1] = (length_count_1 - b_trapz
              (varargin_1_tmp, y)) + S_end;
            d = pos_l_CurrentLane;
            Rreplan_tmp = 90.0;
          }

          traj[6 * para_breaks_size_idx_1 + 1] = d;
          traj[6 * para_breaks_size_idx_1 + 2] = Rreplan_tmp;
        } else {
          /* 计算沿曲线给定长度的点位置 */
          /*  fprime = fnder(para,1); */
          /* fprime = fnder(para,1);--------------------------------------------------- */
          /*  fprime=para; */
          /*  fprime.coefs=dcoefs; */
          /*  fprime.order=para.order-1; */
          for (iter = 0; iter < 3; iter++) {
            d = para3_coefs[iter];
            Rreplan_tmp = para3_coefs[iter + 3];
            Rreplan = para3_coefs[iter + 6];
            CenterS = para3_coefs[iter + 9];
            for (para_pieces = 0; para_pieces < 4; para_pieces++) {
              IsStopSpeedPlan = para_pieces << 2;
              paraNew_coefs[iter + 3 * para_pieces] = ((d * (double)
                b[IsStopSpeedPlan] + Rreplan_tmp * (double)b[IsStopSpeedPlan + 1])
                + Rreplan * (double)b[IsStopSpeedPlan + 2]) + CenterS * (double)
                b[IsStopSpeedPlan + 3];
            }
          }

          memcpy(&fprime_coefs[0], &paraNew_coefs[3], 9U * sizeof(double));

          /* -------------------------------------------------------------------------- */
          linspace(pos_s, S_end, varargin_1_tmp);
          d_ppval(para3_breaks, fprime_coefs, varargin_1_tmp, a);
          for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan++) {
            d = a[IsStopSpeedPlan];
            y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
          }

          if (b_trapz(varargin_1_tmp, y) >= length_count_1) {
            c_pos_s[0] = pos_s;
            c_pos_s[1] = S_end;
            d_fzero(pos_s, para3_breaks, fprime_coefs, length_count_1, c_pos_s,
                    &Rreplan_extre, &Rreplan, &CenterS);
            traj[6 * para_breaks_size_idx_1] = Rreplan_extre;
            d = e_ppval(para3_breaks, para3_coefs, Rreplan_extre);
            Rreplan_tmp = 90.0 - 57.295779513082323 * atan(f_ppval(para3_breaks,
              fprime_coefs, Rreplan_extre));
          } else {
            linspace(pos_s, S_end, varargin_1_tmp);
            d_ppval(para3_breaks, fprime_coefs, varargin_1_tmp, a);
            for (IsStopSpeedPlan = 0; IsStopSpeedPlan < 50; IsStopSpeedPlan++) {
              d = a[IsStopSpeedPlan];
              y[IsStopSpeedPlan] = sqrt(d * d + 1.0);
            }

            traj[6 * para_breaks_size_idx_1] = (length_count_1 - b_trapz
              (varargin_1_tmp, y)) + S_end;
            d = pos_l_CurrentLane;
            Rreplan_tmp = 90.0;
          }

          traj[6 * para_breaks_size_idx_1 + 1] = d;
          traj[6 * para_breaks_size_idx_1 + 2] = Rreplan_tmp;
        }

        d = 90.0 - Rreplan_tmp;
        b_cosd(&d);
        traj[6 * para_breaks_size_idx_1 + 3] = v_count_1 * d;
        d = 90.0 - Rreplan_tmp;
        b_sind(&d);
        traj[6 * para_breaks_size_idx_1 + 4] = v_count_1 * d;
        if (para_breaks_size_idx_1 + 1 == 1) {
          traj[5] = 0.0;
        } else {
          traj[6 * para_breaks_size_idx_1 + 5] = (Rreplan_tmp - traj[6 *
            (para_breaks_size_idx_1 - 1) + 2]) / 0.05;
        }
      }
    }

    iter = DurationLaneChange_RePlan + 1;
    if (DurationLaneChange_RePlan + 1 > 32767) {
      iter = 32767;
    }

    DurationLaneChange_RePlan = (short)iter;
  }

  if ((traj[6] >= S_end) || (pos_s >= S_end)) {
    DurationLaneChange_RePlan = 0;
  }

  for (iter = 0; iter < 80; iter++) {
    traj_s[iter] = traj[6 * iter];
    traj_l[iter] = traj[6 * iter + 1];
    traj_psi[iter] = traj[6 * iter + 2];
    traj_vs[iter] = traj[6 * iter + 3];
    traj_vl[iter] = traj[6 * iter + 4];
    traj_omega[iter] = traj[6 * iter + 5];
  }

  GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan =
    DurationLaneChange_RePlan;
  GlobVars->Decider.a_sollpre2traj = a_soll;

  /*  t=(0.05:0.05:4); */
  /*  traj_s */
  /*  plot(t,traj_s-40); */
  /*  if GlobVars.TrajPlanLaneChange_RePlan.DurationLaneChange_RePlan>0 */
  /*      figure; */
  /*      plot(traj_s,traj_l) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(traj_l) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(traj_psi) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(diff(traj_s)/0.05) */
  /*      hold on; */
  /*      plot(traj_vs) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(diff(traj_l)/0.05) */
  /*      hold on; */
  /*      plot(traj_vl) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(diff(traj_psi)/0.05) */
  /*      hold on; */
  /*      plot(traj_omega) */
  /*      legend; */
  /*       */
  /*      figure; */
  /*      plot(diff((traj_vl.^2+traj_vs.^2).^0.5)/0.05) */
  /*      figure; */
  /*      plot((traj_vl.^2+traj_vs.^2).^0.5) */
  /*  end */
}

/*
 * Arguments    : const double c_fun_x_workspace_fun_a_workspa[3]
 *                const double S_traj[80]
 *                double i_traj
 *                double x
 * Return Type  : double
 */
static double TrajPlanLaneChange_anonFcn8(const double
  c_fun_x_workspace_fun_a_workspa[3], const double S_traj[80], double i_traj,
  double x)
{
  double b[50];
  double c[50];
  double varargin_1_tmp[50];
  double a;
  double b_a;
  double c_a;
  double varargout_1;
  int ia;
  int iac;
  int ix;
  linspace(0.0, x, varargin_1_tmp);
  a = 3.0 * c_fun_x_workspace_fun_a_workspa[0];
  b_a = 4.0 * c_fun_x_workspace_fun_a_workspa[1];
  c_a = 5.0 * c_fun_x_workspace_fun_a_workspa[2];
  for (ix = 0; ix < 50; ix++) {
    double d;
    d = varargin_1_tmp[ix];
    d = ((a * (d * d) + b_a * rt_powd_snf(d, 3.0)) + c_a * rt_powd_snf(d, 4.0)) /
      1.0E+6;
    b[ix] = sqrt(d * d + 1.0);
  }

  c[0] = 0.5 * (varargin_1_tmp[1] - varargin_1_tmp[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (varargin_1_tmp[ix + 2] - varargin_1_tmp[ix]);
  }

  c[49] = 0.5 * (varargin_1_tmp[49] - varargin_1_tmp[48]);
  varargout_1 = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    int b_iac;
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      varargout_1 += b[ia - 1] * c[ix];
    }

    ix++;
  }

  varargout_1 -= S_traj[(int)i_traj - 1];
  return varargout_1;
}

/*
 * ,
 * ,
 * ,
 *
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
 *                const double LengthCodirectCar[10]
 *                short CurrentLane
 *                double v_max
 *                double a_soll
 *                short CurrentGear
 *                short *TurnAroundActive
 *                short *AEBactive
 *                double stopdistance
 *                double a_soll_ACC
 *                double SampleTime
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 *                double *a_soll_TrajPlanTurnAround
 *                double *a_sollTurnAround2Decider
 *                struct2_T *Refline
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
  double PosSCodirectCar[10], const double LengthCodirectCar[10], short
  CurrentLane, double v_max, double a_soll, short CurrentGear, short
  *TurnAroundActive, short *AEBactive, double stopdistance, double a_soll_ACC,
  double SampleTime, TypeGlobVars *GlobVars, const TypeCalibrationVars
  *CalibrationVars, const TypeParameters Parameters, double
  *a_soll_TrajPlanTurnAround, double *a_sollTurnAround2Decider, struct2_T
  *Refline, double traj_s[80], double traj_l[80], double traj_psi[80], double
  traj_vs[80], double traj_vl[80], double traj_omega[80], short *TargetGear)
{
  emxArray_int16_T *b_Lanes2Search;
  emxArray_int16_T *b_OccupiedLanes;
  emxArray_int32_T *b_ia;
  emxArray_real_T *Lanes2Search;
  emxArray_real_T *OccupiedLanes;
  emxArray_real_T *OccupiedLanesPosMid1;
  emxArray_real_T *OccupiedLanesPosMid2;
  emxArray_real_T *ia;
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
  double a_max_com;
  double a_predict;
  double b;
  double d;
  double d_cur2pot_tar;
  double dis2pos_mid2;
  double disOppositeCar2circle2;
  double jerk;
  double k;
  double l_veh;
  double pos_l_TargetLane;
  double targetSpeed;
  double timeGap;
  double v_max_turnAround_tmp_tmp;
  double w_veh;
  double *Lanes2Search_data;
  double *OccupiedLanesPosMid2_data;
  double *OccupiedLanes_data;
  double *ia_data;
  int WidthOfLanesOpposite_size[2];
  int b_WidthOfLanesOpposite_size[2];
  int IsStopSpeedPlan;
  int b_i;
  int i;
  int j;
  int loop_ub;
  int trueCount;
  int *b_ia_data;
  short IndexOfLaneOppositeCarFront[6];
  short iv[2];
  short TargetLaneIndexOpposite;
  short TurnAroundState;
  short TypeOfTurnAround;
  short dec_trunAround;
  short i1;
  short wait_turnAround;
  short *b_Lanes2Search_data;
  short *b_OccupiedLanes_data;
  boolean_T exitg1;
  boolean_T exitg2;

  /* -------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* Parameters */
  TurningRadius = Parameters.turningRadius;
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
  D_safe2 = CalibrationVars->TrajPlanTurnAround.d_safe2;
  a_max_com = CalibrationVars->TrajPlanTurnAround.a_max_com;
  v_max_turnAround_tmp_tmp =
    CalibrationVars->TrajPlanTurnAround.v_max_turnAround;

  /* --------------------------------------------------------------------------------------------------------------------------------------------------------- */
  /* globalVariable */
  dec_trunAround = GlobVars->TrajPlanTurnAround.dec_trunAround;
  wait_turnAround = GlobVars->TrajPlanTurnAround.wait_turnAround;
  TypeOfTurnAround = GlobVars->TrajPlanTurnAround.typeOfTurnAround;
  TurnAroundState = GlobVars->TrajPlanTurnAround.turnAroundState;
  TargetLaneIndexOpposite = GlobVars->TrajPlanTurnAround.targetLaneIndexOpposite;

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
  Refline->NumRefLaneTurnAround = 0;
  memset(&Refline->SRefLaneTurnAround[0], 0, 100U * sizeof(double));
  memset(&Refline->LRefLaneTurnAround[0], 0, 100U * sizeof(double));
  Refline->TurnAroundReflineState = 0;
  *TargetGear = CurrentGear;
  for (i = 0; i < 6; i++) {
    IndexOfLaneOppositeCarFront[i] = 0;
    SpeedOppositeCarFront[i] = 0.0;
    PosSOppositeCarFront[i] = 200.0;
    SpeedOppositeCarRear[i] = 0.0;
    PosSOppositeCarRear[i] = -200.0;
    LengthOppositeCarRear[i] = 5.0;
  }

  dis2pos_mid2 = 200.0;

  /*  目标车道选择 ----(判断掉头一次二次，选择掉头目标车道，只计算一次) */
  pos_l_TargetLane = 0.0;

  /* 20220324因为只用一次，所以可以设初值 */
  jerk = WidthOfLanes[CurrentLane - 1];
  emxInit_real_T(&OccupiedLanesPosMid2, 2);
  OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
  emxInit_real_T(&Lanes2Search, 2);
  if (GlobVars->TrajPlanTurnAround.typeOfTurnAround == 0) {
    /*  d_cur2tar=0.5*(WidthOfLanesOpposite(1)-w_veh)+0.5*WidthOfLaneCurrent+WidthOfGap+WidthOfLanesOpposite(1); */
    TargetLaneIndexOpposite = 1;
    TypeOfTurnAround = 2;

    /*  1为一次顺车掉头，2为二次顺车掉头 */
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= NumOfLanesOpposite - 1)) {
      d_cur2pot_tar = (0.5 * jerk + WidthOfGap) + 0.5 * WidthOfLanesOpposite[i];
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
      } else if ((0.5 * (WidthOfLanesOpposite[i] - w_veh) -
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
      short i2;
      unsigned short u;
      d = ((s_turnaround_border - Parameters.turningRadius) - 0.5 *
           Parameters.w_veh) - CalibrationVars->TrajPlanTurnAround.d_safe1;
      GlobVars->TrajPlanTurnAround.posCircle[0] = d;
      GlobVars->TrajPlanTurnAround.posCircle[1] = pos_l_CurrentLane +
        Parameters.turningRadius;

      /*  s_start=PosCircle1(1); */
      GlobVars->TrajPlanTurnAround.posCircle2[0] = d;
      GlobVars->TrajPlanTurnAround.posCircle2[1] = pos_l_TargetLane -
        Parameters.turningRadius;
      LaneCenterCal(CurrentLane, pos_l_CurrentLane, jerk, WidthOfGap,
                    WidthOfLanesOpposite, NumOfLanesOpposite,
                    GlobVars->TrajPlanTurnAround.laneCenterline);

      /*  车道中心线位置 全局变量 */
      /* 掉头参考线输出及参考线过渡结束位置的坐标 */
      PathPlanTurnAroundDecider(GlobVars->
        TrajPlanTurnAround.laneCenterline[TargetLaneIndexOpposite - 1],
        GlobVars->TrajPlanTurnAround.posCircle,
        GlobVars->TrajPlanTurnAround.posCircle2, Parameters.turningRadius, pos_s,
        &pos_l_TargetLane, Lanes2Search, OccupiedLanesPosMid2,
        &GlobVars->TrajPlanTurnAround.reflineSend);
      OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
      Lanes2Search_data = Lanes2Search->data;
      d = rt_roundd_snf(pos_l_TargetLane);
      if (d < 32768.0) {
        if (d >= -32768.0) {
          i2 = (short)d;
          i1 = i2;
        } else {
          i2 = MIN_int16_T;
          i1 = MIN_int16_T;
        }
      } else if (d >= 32768.0) {
        i2 = MAX_int16_T;
        i1 = MAX_int16_T;
      } else {
        i2 = 0;
        i1 = 0;
      }

      Refline->NumRefLaneTurnAround = i2;
      if ((!rtIsNaN(pos_l_TargetLane)) && (pos_l_TargetLane < 100.0)) {
        u = (unsigned short)i1;
      } else {
        u = 100U;
      }

      if (u > 32767) {
        u = 32767U;
      }

      b_i = u;
      for (IsStopSpeedPlan = 0; IsStopSpeedPlan < b_i; IsStopSpeedPlan++) {
        Refline->SRefLaneTurnAround[IsStopSpeedPlan] =
          Lanes2Search_data[IsStopSpeedPlan];
        Refline->LRefLaneTurnAround[IsStopSpeedPlan] =
          OccupiedLanesPosMid2_data[IsStopSpeedPlan];
      }

      GlobVars->TrajPlanTurnAround.reflineLend =
        GlobVars->TrajPlanTurnAround.laneCenterline[TargetLaneIndexOpposite - 1];
    }

    /*  二次顺车掉头路径生成 */
    if (TypeOfTurnAround == 2) {
      b_i = NumOfLanesOpposite - 1;
      if (NumOfLanesOpposite - 1 < -32768) {
        b_i = -32768;
      }

      if ((short)b_i < 1) {
        loop_ub = 0;
      } else {
        loop_ub = (short)b_i;
      }

      /* , */
      TargetLaneIndexOpposite = NumOfLanesOpposite;
      if ((short)(NumOfLanesOpposite - 1) < 1) {
        IsStopSpeedPlan = 0;
      } else {
        IsStopSpeedPlan = (short)(NumOfLanesOpposite - 1);
      }

      WidthOfLanesOpposite_size[0] = 1;
      WidthOfLanesOpposite_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], (unsigned int)
               loop_ub * sizeof(double));
      }

      b_WidthOfLanesOpposite_size[0] = 1;
      b_WidthOfLanesOpposite_size[1] = IsStopSpeedPlan;
      if (IsStopSpeedPlan - 1 >= 0) {
        memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
               (unsigned int)IsStopSpeedPlan * sizeof(double));
      }

      PathPlanTurnAround(s_turnaround_border, Parameters.w_veh,
                         Parameters.turningRadius,
                         CalibrationVars->TrajPlanTurnAround.d_safe1,
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
                         Parameters.l_veh,
                         GlobVars->TrajPlanTurnAround.posCircle,
                         GlobVars->TrajPlanTurnAround.posCircle2,
                         GlobVars->TrajPlanTurnAround.posCircle3,
                         GlobVars->TrajPlanTurnAround.pos_start,
                         GlobVars->TrajPlanTurnAround.pos_mid1,
                         GlobVars->TrajPlanTurnAround.pos_mid2,
                         GlobVars->TrajPlanTurnAround.pos_end);

      /*          pos_mid1_rear=[pos_mid1(1)+sin(pos_mid1(3))*l_veh pos_mid1(2)-cos(pos_mid1(3))*l_veh 0 0]; */
      /*          pos_mid2_rear=[pos_mid2(1)+sin(pos_mid2(3))*l_veh pos_mid2(2)-cos(pos_mid2(3))*l_veh 0 0]; */
      d = cos(GlobVars->TrajPlanTurnAround.pos_mid1[2]);
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] =
        GlobVars->TrajPlanTurnAround.pos_mid1[0] + sin
        (GlobVars->TrajPlanTurnAround.pos_mid1[2]) * Parameters.l_veh * 0.5;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] =
        GlobVars->TrajPlanTurnAround.pos_mid1[1] - d * Parameters.l_veh * 0.5;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[2] = 0.0;
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] = 0.0;
      d = cos(GlobVars->TrajPlanTurnAround.pos_mid2[2]);
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] =
        GlobVars->TrajPlanTurnAround.pos_mid2[0] + sin
        (GlobVars->TrajPlanTurnAround.pos_mid2[2]) * Parameters.l_veh * 0.5;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] =
        GlobVars->TrajPlanTurnAround.pos_mid2[1] - d * Parameters.l_veh * 0.5;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[2] = 0.0;
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] = 0.0;

      /*  OccupiedLanesPosMid1=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1_rear(2)):1:..., */
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid1(2)); % 例如[2 1] 全局变量 对向车道序号为正，最左侧为1 */
      GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, jerk, WidthOfGap, WidthOfLanesOpposite,
        NumOfLanesOpposite, GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]);
      GlobVars->TrajPlanTurnAround.pos_mid1[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, jerk, WidthOfGap, WidthOfLanesOpposite,
        NumOfLanesOpposite, GlobVars->TrajPlanTurnAround.pos_mid1[1]);

      /*  OccupiedLanesPosMid2=LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2_rear(2)):1:..., */
      /*      LaneIndexJudge(CurrentLane,pos_l_CurrentLane,WidthOfLaneCurrent,WidthOfGap,WidthOfLanesOpposite,NumOfLanesOpposite,pos_mid2(2)); % 例如[1 -1] 全局变量 掉头前道路车道序号为负，最左侧为-1 */
      GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, jerk, WidthOfGap, WidthOfLanesOpposite,
        NumOfLanesOpposite, GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]);
      GlobVars->TrajPlanTurnAround.pos_mid2[3] = LaneIndexJudge(CurrentLane,
        pos_l_CurrentLane, jerk, WidthOfGap, WidthOfLanesOpposite,
        NumOfLanesOpposite, GlobVars->TrajPlanTurnAround.pos_mid2[1]);
      LaneCenterCal(CurrentLane, pos_l_CurrentLane, WidthOfLanes[CurrentLane - 1],
                    WidthOfGap, WidthOfLanesOpposite, NumOfLanesOpposite,
                    GlobVars->TrajPlanTurnAround.laneCenterline);

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
      if ((GlobVars->TrajPlanTurnAround.laneCenterline[i] != 0.0) && (i + 1 <=
           TargetLaneIndexOpposite)) {
        if (GlobVars->TrajPlanTurnAround.laneCenterline[i] <
            GlobVars->TrajPlanTurnAround.posCircle[1]) {
          targetSpeed = GlobVars->TrajPlanTurnAround.posCircle[1] -
            GlobVars->TrajPlanTurnAround.laneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            targetSpeed * targetSpeed) + GlobVars->TrajPlanTurnAround.posCircle
            [0];
        } else if ((GlobVars->TrajPlanTurnAround.laneCenterline[i] >=
                    GlobVars->TrajPlanTurnAround.posCircle[1]) &&
                   (GlobVars->TrajPlanTurnAround.laneCenterline[i] <=
                    GlobVars->TrajPlanTurnAround.posCircle2[1])) {
          posOfLaneCenterline[i] = TurningRadius +
            GlobVars->TrajPlanTurnAround.posCircle[0];
        } else if (GlobVars->TrajPlanTurnAround.laneCenterline[i] >
                   GlobVars->TrajPlanTurnAround.posCircle2[1]) {
          targetSpeed = GlobVars->TrajPlanTurnAround.posCircle2[1] -
            GlobVars->TrajPlanTurnAround.laneCenterline[i];
          posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
            targetSpeed * targetSpeed) + GlobVars->
            TrajPlanTurnAround.posCircle2[0];
        }
      }
    }
  } else if ((TypeOfTurnAround == 2) &&
             (GlobVars->TrajPlanTurnAround.turnAroundState < 2)) {
    for (i = 0; i < 7; i++) {
      if ((GlobVars->TrajPlanTurnAround.laneCenterline[i] != 0.0) && (i + 1 <=
           TargetLaneIndexOpposite)) {
        targetSpeed = GlobVars->TrajPlanTurnAround.posCircle[1] -
          GlobVars->TrajPlanTurnAround.laneCenterline[i];
        posOfLaneCenterline[i] = sqrt(TurningRadius * TurningRadius -
          targetSpeed * targetSpeed) + GlobVars->TrajPlanTurnAround.posCircle[0];
      }
    }
  }

  if ((TypeOfTurnAround == 1) || ((TypeOfTurnAround == 2) &&
       (GlobVars->TrajPlanTurnAround.turnAroundState < 2))) {
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
    if (b_i - 1 >= 0) {
      WidthOfLanesOpposite_size[0] = 1;
    }

    for (i = 0; i < b_i; i++) {
      if (i < 1) {
        loop_ub = 0;
      } else {
        loop_ub = i;
      }

      WidthOfLanesOpposite_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], (unsigned int)
               loop_ub * sizeof(double));
      }

      d = (0.5 * jerk + WidthOfGap) + 0.5 * WidthOfLanesOpposite[i];
      if (d + sum(posOfLaneCenterline, WidthOfLanesOpposite_size) <
          TurningRadius) {
        if (i < 1) {
          loop_ub = 0;
        } else {
          loop_ub = i;
        }

        b_WidthOfLanesOpposite_size[0] = 1;
        b_WidthOfLanesOpposite_size[1] = loop_ub;
        if (loop_ub - 1 >= 0) {
          memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                 (unsigned int)loop_ub * sizeof(double));
        }

        d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
          TurningRadius * acos((TurningRadius - (d + sum
          (WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size))) /
          TurningRadius);
      } else {
        if (i < 1) {
          loop_ub = 0;
        } else {
          loop_ub = i;
        }

        b_WidthOfLanesOpposite_size[0] = 1;
        b_WidthOfLanesOpposite_size[1] = loop_ub;
        if (loop_ub - 1 >= 0) {
          memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                 (unsigned int)loop_ub * sizeof(double));
        }

        if (d + sum(WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size) >
            (TurningRadius + GlobVars->TrajPlanTurnAround.posCircle2[1]) -
            GlobVars->TrajPlanTurnAround.posCircle[1]) {
          if (i < 1) {
            loop_ub = 0;
          } else {
            loop_ub = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                   (unsigned int)loop_ub * sizeof(double));
          }

          a_predict = GlobVars->TrajPlanTurnAround.posCircle2[1];
          timeGap = GlobVars->TrajPlanTurnAround.posCircle[1];
          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
            (((TurningRadius * asin(((((d + sum(WidthOfLanesOpposite_data,
                     b_WidthOfLanesOpposite_size)) - TurningRadius) - a_predict)
                 + timeGap) / TurningRadius) + a_predict) - timeGap) +
             TurningRadius * 3.1415926535897931 / 2.0);

          /* , */
        } else {
          if (i < 1) {
            loop_ub = 0;
          } else {
            loop_ub = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                   (unsigned int)loop_ub * sizeof(double));
          }

          d_veh2cross[i] = ((((((GlobVars->TrajPlanTurnAround.posCircle[0] -
            pos_s) + TurningRadius * 3.1415926535897931 / 2.0) + 0.5 * jerk) +
                              WidthOfGap) + 0.5 * WidthOfLanesOpposite[i]) + sum
                            (WidthOfLanesOpposite_data,
                             b_WidthOfLanesOpposite_size)) - TurningRadius;
        }
      }

      /*          d_veh2cross(i)=PosCircle1(1)-pos_s+0.5*WidthOfLaneCurrent+WidthOfGap+0.5*WidthOfLanesOpposite(i)+sum(WidthOfLanesOpposite(1:i-1)); % 可更精确 */
    }

    /*  策略模式判断 */
    if (GlobVars->TrajPlanTurnAround.dec_trunAround == 0) {
      d = GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s;
      if ((d <= (0.0 - speed * speed) / (2.0 *
            CalibrationVars->TrajPlanTurnAround.a_min) + 2.0 * Parameters.l_veh)
          && (d > 0.0) && (pos_l < GlobVars->TrajPlanTurnAround.posCircle[1])) {
        dec_trunAround = 1;
      }
    } else if ((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s <= 0.0) ||
               (GlobVars->TrajPlanTurnAround.wait_turnAround == 1) || (pos_l >
                GlobVars->TrajPlanTurnAround.posCircle[1])) {
      /* 20220225 */
      dec_trunAround = 0;
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
            (IndexOfLaneOppositeCarFront[j] > 0)) {
          b_speed[0] = speed;
          b_speed[1] = 1.0E-5;
          d = maximum(b_speed);
          b_speed[0] = ((d_veh2cross[IndexOfLaneOppositeCarFront[j] - 1] + l_veh
                         * 0.5) / d * SpeedOppositeCarFront[j] + 0.5 * w_veh) +
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
    if ((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.posCircle[0] -
         pos_s < 10.0)) {
      wait_turnAround = 0;
      a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                      CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                      CalibrationVars->ACC.d_wait2faultyCar,
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
            (IndexOfLaneOppositeCarFront[j] > 0)) {
          /*  timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)/max([SpeedOppositeCarFront(j) 0.00001])]); */
          /*  timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh-TurningRadius)/max([SpeedOppositeCarFront(j) 0.00001])]); */
          IsStopSpeedPlan = IndexOfLaneOppositeCarFront[j] - 1;
          b_speed[0] = ACC(13.888888888888889,
                           SpeedOppositeCarRear[IsStopSpeedPlan],
                           PosSOppositeCarFront[j] -
                           PosSOppositeCarRear[IsStopSpeedPlan],
                           SpeedOppositeCarFront[j], 0.0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.d_wait2faultyCar,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
          b_speed[1] = 0.0;
          disOppositeCar2circle2 = maximum(b_speed);
          if (SpeedOppositeCarFront[j] <= 0.01) {
            disOppositeCar2circle2 = 0.0;
          }

          /*  [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-(max(SpeedOppositeCarFront(j)+a_OppositeCarFront*t-v_max,0))^2/(2*a_OppositeCarFront+eps)..., */
          /*      -max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
          /*      [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
          b_speed[0] = 0.0;
          b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
          dv[0] = 1.0E-5;
          dv[1] = fmin(SpeedOppositeCarFront[j], v_max);
          d = maximum(b_speed);
          b_speed[0] = -0.01;
          b_speed[1] = d / maximum(dv) + 0.01;
          e_fzero(SpeedOppositeCarFront, (double)j + 1.0, fmin
                  (disOppositeCar2circle2, 1.5), v_max, PosSOppositeCarFront,
                  w_veh, D_safe2, b_speed, &timeGap, &d_cur2pot_tar,
                  &pos_l_TargetLane);

          /* , */
          /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
          b_speed[0] = a_predict;
          b_speed[1] = a_max_com;
          a_predict = c_minimum(b_speed);
          if (a_predict > 0.0) {
            d_cur2pot_tar = speed + a_predict * timeGap;
            targetSpeed = fmax(d_cur2pot_tar - v_max_turnAround_tmp_tmp, 0.0);
            pos_l_TargetLane = 0.5 * (d_cur2pot_tar + speed) * timeGap -
              targetSpeed * targetSpeed / (2.0 * a_predict +
              2.2204460492503131E-16);
          } else {
            dv1[0] = 0.0;
            dv1[1] = speed + a_predict * timeGap;
            dv1[2] = v_max_turnAround_tmp_tmp;
            pos_l_TargetLane = 0.5 * (median(dv1) + speed) * timeGap;
          }

          if (pos_l_TargetLane <= d_veh2cross[IsStopSpeedPlan] + l_veh * 0.5) {
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
    if ((GlobVars->TrajPlanTurnAround.turnAroundState == 0) && (pos_s >=
         GlobVars->TrajPlanTurnAround.pos_start[0]) && (pos_l <
         GlobVars->TrajPlanTurnAround.pos_start[1] + 0.5) && (pos_l >
         GlobVars->TrajPlanTurnAround.pos_start[1] - 0.5)) {
      TurnAroundState = 1;
      *TargetGear = 4;

      /*  P R N D /1 2 3 4 */
    }

    emxInit_real_T(&ia, 1);
    if (TurnAroundState == 1) {
      targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
      d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
      emxInit_real_T(&OccupiedLanesPosMid1, 2);
      ia_data = OccupiedLanesPosMid1->data;
      emxInit_real_T(&OccupiedLanes, 2);
      OccupiedLanes_data = OccupiedLanes->data;
      emxInit_int32_T(&b_ia);
      if ((sqrt(targetSpeed * targetSpeed + d_cur2pot_tar * d_cur2pot_tar) <
           0.15) && (speed <= 0.05)) {
        *TargetGear = 2;
        if (CurrentGear == 2) {
          boolean_T tf;

          /*  环境车允许倒车 % 倒车前决策 */
          /*  当前位置mid1车头车尾所在车道 → 确定已占据车道 */
          IsStopSpeedPlan = (GlobVars->TrajPlanTurnAround.pos_mid2[3] -
                             GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] >=
                             0.0) << 1;
          if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) || rtIsNaN
              (GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
            b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
            OccupiedLanesPosMid2->size[0] = 1;
            OccupiedLanesPosMid2->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
            OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
            OccupiedLanesPosMid2_data[0] = rtNaN;
          } else if ((IsStopSpeedPlan - 1 == 0) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid2[3]) &&
                      (IsStopSpeedPlan - 1 < 0)) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid2[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) &&
                      (IsStopSpeedPlan - 1 > 0))) {
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
            OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
            OccupiedLanesPosMid2_data[0] = rtNaN;
          } else if (floor(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ==
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) {
            d = GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
            b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
            OccupiedLanesPosMid2->size[0] = 1;
            loop_ub = (int)((GlobVars->TrajPlanTurnAround.pos_mid2[3] - d) /
                            ((double)IsStopSpeedPlan - 1.0));
            OccupiedLanesPosMid2->size[1] = loop_ub + 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
            OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
            for (b_i = 0; b_i <= loop_ub; b_i++) {
              OccupiedLanesPosMid2_data[b_i] = d + (double)((IsStopSpeedPlan - 1)
                * b_i);
            }
          } else {
            eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3],
                            (double)IsStopSpeedPlan - 1.0,
                            GlobVars->TrajPlanTurnAround.pos_mid2[3],
                            OccupiedLanesPosMid2);
            OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
          }

          /*  例如[1 -1] 掉头前道路车道序号为负，最左侧为-1 */
          IsStopSpeedPlan = (GlobVars->TrajPlanTurnAround.pos_mid1[3] -
                             GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] >=
                             0.0) << 1;
          if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) || rtIsNaN
              (GlobVars->TrajPlanTurnAround.pos_mid1[3])) {
            b_i = OccupiedLanesPosMid1->size[0] * OccupiedLanesPosMid1->size[1];
            OccupiedLanesPosMid1->size[0] = 1;
            OccupiedLanesPosMid1->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid1, b_i);
            ia_data = OccupiedLanesPosMid1->data;
            ia_data[0] = rtNaN;
          } else if ((IsStopSpeedPlan - 1 == 0) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid1_rear[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid1[3]) &&
                      (IsStopSpeedPlan - 1 < 0)) ||
                     ((GlobVars->TrajPlanTurnAround.pos_mid1[3] <
                       GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) &&
                      (IsStopSpeedPlan - 1 > 0))) {
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
            ia_data = OccupiedLanesPosMid1->data;
            ia_data[0] = rtNaN;
          } else if (floor(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) ==
                     GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]) {
            d = GlobVars->TrajPlanTurnAround.pos_mid1_rear[3];
            b_i = OccupiedLanesPosMid1->size[0] * OccupiedLanesPosMid1->size[1];
            OccupiedLanesPosMid1->size[0] = 1;
            loop_ub = (int)((GlobVars->TrajPlanTurnAround.pos_mid1[3] - d) /
                            ((double)IsStopSpeedPlan - 1.0));
            OccupiedLanesPosMid1->size[1] = loop_ub + 1;
            emxEnsureCapacity_real_T(OccupiedLanesPosMid1, b_i);
            ia_data = OccupiedLanesPosMid1->data;
            for (b_i = 0; b_i <= loop_ub; b_i++) {
              ia_data[b_i] = d + (double)((IsStopSpeedPlan - 1) * b_i);
            }
          } else {
            eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid1_rear[3],
                            (double)IsStopSpeedPlan - 1.0,
                            GlobVars->TrajPlanTurnAround.pos_mid1[3],
                            OccupiedLanesPosMid1);
            ia_data = OccupiedLanesPosMid1->data;
          }

          /*  例如[2 1] 对向车道序号为正，最左侧为1 */
          if (rtIsNaN(OccupiedLanesPosMid2_data[0]) || rtIsNaN
              (ia_data[OccupiedLanesPosMid1->size[1] - 1])) {
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            OccupiedLanes_data = OccupiedLanes->data;
            OccupiedLanes_data[0] = rtNaN;
          } else if (ia_data[OccupiedLanesPosMid1->size[1] - 1] <
                     OccupiedLanesPosMid2_data[0]) {
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 0;
          } else if ((rtIsInf(OccupiedLanesPosMid2_data[0]) || rtIsInf
                      (ia_data[OccupiedLanesPosMid1->size[1] - 1])) &&
                     (OccupiedLanesPosMid2_data[0] ==
                      ia_data[OccupiedLanesPosMid1->size[1] - 1])) {
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            OccupiedLanes->size[1] = 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            OccupiedLanes_data = OccupiedLanes->data;
            OccupiedLanes_data[0] = rtNaN;
          } else if (floor(OccupiedLanesPosMid2_data[0]) ==
                     OccupiedLanesPosMid2_data[0]) {
            d = OccupiedLanesPosMid2_data[0];
            b_i = OccupiedLanes->size[0] * OccupiedLanes->size[1];
            OccupiedLanes->size[0] = 1;
            loop_ub = (int)(ia_data[OccupiedLanesPosMid1->size[1] - 1] - d);
            OccupiedLanes->size[1] = loop_ub + 1;
            emxEnsureCapacity_real_T(OccupiedLanes, b_i);
            OccupiedLanes_data = OccupiedLanes->data;
            for (b_i = 0; b_i <= loop_ub; b_i++) {
              OccupiedLanes_data[b_i] = d + (double)b_i;
            }
          } else {
            b_eml_float_colon(OccupiedLanesPosMid2_data[0],
                              ia_data[OccupiedLanesPosMid1->size[1] - 1],
                              OccupiedLanes);
            OccupiedLanes_data = OccupiedLanes->data;
          }

          /*  目标位置mid2车尾所在车道 → 确定将侵入的车道 */
          do_vectors(OccupiedLanes, OccupiedLanesPosMid1, Lanes2Search, b_ia,
                     &IsStopSpeedPlan);
          b_ia_data = b_ia->data;
          b_i = ia->size[0];
          ia->size[0] = b_ia->size[0];
          emxEnsureCapacity_real_T(ia, b_i);
          ia_data = ia->data;
          loop_ub = b_ia->size[0];
          for (b_i = 0; b_i < loop_ub; b_i++) {
            ia_data[b_i] = b_ia_data[b_i];
          }

          sort(ia);
          ia_data = ia->data;
          b_i = Lanes2Search->size[0] * Lanes2Search->size[1];
          Lanes2Search->size[0] = 1;
          Lanes2Search->size[1] = ia->size[0];
          emxEnsureCapacity_real_T(Lanes2Search, b_i);
          Lanes2Search_data = Lanes2Search->data;
          loop_ub = ia->size[0];
          for (b_i = 0; b_i < loop_ub; b_i++) {
            Lanes2Search_data[b_i] = OccupiedLanes_data[(int)ia_data[b_i] - 1];
          }

          /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道） */
          IsStopSpeedPlan = ia->size[0] - 1;
          trueCount = 0;
          for (i = 0; i <= IsStopSpeedPlan; i++) {
            if (Lanes2Search_data[i] != 0.0) {
              trueCount++;
            }
          }

          loop_ub = 0;
          for (i = 0; i <= IsStopSpeedPlan; i++) {
            if (Lanes2Search_data[i] != 0.0) {
              Lanes2Search_data[loop_ub] = Lanes2Search_data[i];
              loop_ub++;
            }
          }

          b_i = Lanes2Search->size[0] * Lanes2Search->size[1];
          Lanes2Search->size[0] = 1;
          Lanes2Search->size[1] = trueCount;
          emxEnsureCapacity_real_T(Lanes2Search, b_i);
          Lanes2Search_data = Lanes2Search->data;

          /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将倒车路径简化为pos_mid1_rear到pos_mid2_rear的线段→d_veh2cross,timegap” */
          TurnAroundState = 2;
          a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                          CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                          CalibrationVars->ACC.d_wait2faultyCar,
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
            tf = false;
            IsStopSpeedPlan = 0;
            exitg2 = false;
            while ((!exitg2) && (IsStopSpeedPlan <= trueCount - 1)) {
              if (IndexOfLaneOppositeCar[i] == Lanes2Search_data[IsStopSpeedPlan])
              {
                tf = true;
                exitg2 = true;
              } else {
                IsStopSpeedPlan++;
              }
            }

            if (tf) {
              k = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]) /
                (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
              pos_l_TargetLane = GlobVars->
                TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[i] - 1];
              d_cur2pot_tar = pos_l_TargetLane -
                (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] *
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] *
                 GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]) /
                (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                 GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
              disOppositeCar2circle2 = PosSOppositeCar[i] - d_cur2pot_tar / k;
              if (disOppositeCar2circle2 > 0.0) {
                b_speed[0] = SpeedOppositeCar[i];
                b_speed[1] = 1.0E-5;
                d = maximum(b_speed);
                b_speed[0] = 0.0;
                b_speed[1] = ((disOppositeCar2circle2 - 0.5 * w_veh) - D_safe2) /
                  d;
                timeGap = maximum(b_speed);
                targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
                  d_cur2pot_tar / k;
                d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
                  pos_l_TargetLane;
                b_speed[0] = a_predict;
                b_speed[1] = a_max_com;
                d = c_minimum(b_speed);
                b_speed[0] = speed + d * timeGap;
                b_speed[1] = v_max_turnAround_tmp_tmp;
                if (0.5 * (c_minimum(b_speed) + speed) * timeGap <= sqrt
                    (targetSpeed * targetSpeed + d_cur2pot_tar * d_cur2pot_tar)
                    + l_veh * 0.5) {
                  TurnAroundState = 1;
                  exitg1 = true;
                } else {
                  i++;
                }
              } else if ((disOppositeCar2circle2 <= 0.0) &&
                         (disOppositeCar2circle2 > -LengthOppositeCar[i])) {
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
            loop_ub = -1;
            for (i = 0; i < 10; i++) {
              if (SpeedCodirectCar[i] > -1.0) {
                loop_ub++;
              }
            }

            j = 0;
            exitg1 = false;
            while ((!exitg1) && (j <= loop_ub)) {
              tf = false;
              IsStopSpeedPlan = 0;
              exitg2 = false;
              while ((!exitg2) && (IsStopSpeedPlan <= trueCount - 1)) {
                if (IndexOfLaneCodirectCar[j] ==
                    Lanes2Search_data[IsStopSpeedPlan]) {
                  tf = true;
                  exitg2 = true;
                } else {
                  IsStopSpeedPlan++;
                }
              }

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
                  disOppositeCar2circle2 =
                    (GlobVars->TrajPlanTurnAround.laneCenterline[6] - b) / k -
                    PosSCodirectCar[j];
                } else {
                  disOppositeCar2circle2 =
                    ((GlobVars->TrajPlanTurnAround.laneCenterline[6] - 3.2) - b)
                    / k - PosSCodirectCar[j];
                }

                if (disOppositeCar2circle2 > 0.0) {
                  b_speed[0] = SpeedCodirectCar[j];
                  b_speed[1] = 1.0E-5;
                  d = maximum(b_speed);
                  b_speed[0] = 0.0;
                  b_speed[1] = ((disOppositeCar2circle2 - 0.5 * w_veh) - D_safe2)
                    / d;
                  timeGap = maximum(b_speed);
                  if (IndexOfLaneCodirectCar[j] == -1) {
                    targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]
                      - (GlobVars->TrajPlanTurnAround.laneCenterline[6] - b) / k;
                    d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]
                      - GlobVars->TrajPlanTurnAround.laneCenterline[6];
                    d_cur2pot_tar = sqrt(targetSpeed * targetSpeed +
                                         d_cur2pot_tar * d_cur2pot_tar);
                  } else {
                    targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]
                      - ((GlobVars->TrajPlanTurnAround.laneCenterline[6] - 3.2)
                         - b) / k;
                    d_cur2pot_tar = (GlobVars->TrajPlanTurnAround.pos_mid1_rear
                                     [1] -
                                     GlobVars->
                                     TrajPlanTurnAround.laneCenterline[6]) + 3.2;
                    d_cur2pot_tar = sqrt(targetSpeed * targetSpeed +
                                         d_cur2pot_tar * d_cur2pot_tar);
                  }

                  b_speed[0] = a_predict;
                  b_speed[1] = a_max_com;
                  d = c_minimum(b_speed);
                  b_speed[0] = speed + d * timeGap;
                  b_speed[1] = v_max_turnAround_tmp_tmp;
                  if (0.5 * (c_minimum(b_speed) + speed) * timeGap <=
                      d_cur2pot_tar + l_veh * 0.5) {
                    TurnAroundState = 1;
                    exitg1 = true;
                  } else {
                    j++;
                  }
                } else if ((disOppositeCar2circle2 <= 0.0) &&
                           (disOppositeCar2circle2 > -LengthCodirectCar[j])) {
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
      d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
      pos_l_TargetLane = sqrt(targetSpeed * targetSpeed + d_cur2pot_tar *
        d_cur2pot_tar);
      if ((pos_l_TargetLane < 0.15) && (speed <= 0.05)) {
        *TargetGear = 4;
      }

      emxInit_int16_T(&b_OccupiedLanes);
      emxInit_int16_T(&b_Lanes2Search);
      emxInit_int32_T(&b_ia);
      if ((pos_l_TargetLane < 0.15) && (CurrentGear == 4) && (speed <= 0.05)) {
        /*  环境车允许前进 % 前进前决策 */
        /*  当前位置mid2车头车尾所在车道 → 确定已占据车道 */
        IsStopSpeedPlan = (GlobVars->TrajPlanTurnAround.pos_mid2[3] -
                           GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] >= 0.0)
          << 1;
        if (rtIsNaN(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) || rtIsNaN
            (GlobVars->TrajPlanTurnAround.pos_mid2[3])) {
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          OccupiedLanesPosMid2->size[1] = 1;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
          OccupiedLanesPosMid2_data[0] = rtNaN;
        } else if ((IsStopSpeedPlan - 1 == 0) ||
                   ((GlobVars->TrajPlanTurnAround.pos_mid2_rear[3] <
                     GlobVars->TrajPlanTurnAround.pos_mid2[3]) &&
                    (IsStopSpeedPlan - 1 < 0)) ||
                   ((GlobVars->TrajPlanTurnAround.pos_mid2[3] <
                     GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) &&
                    (IsStopSpeedPlan - 1 > 0))) {
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
          OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
          OccupiedLanesPosMid2_data[0] = rtNaN;
        } else if (floor(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) ==
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) {
          d = GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
          b_i = OccupiedLanesPosMid2->size[0] * OccupiedLanesPosMid2->size[1];
          OccupiedLanesPosMid2->size[0] = 1;
          loop_ub = (int)((GlobVars->TrajPlanTurnAround.pos_mid2[3] - d) /
                          ((double)IsStopSpeedPlan - 1.0));
          OccupiedLanesPosMid2->size[1] = loop_ub + 1;
          emxEnsureCapacity_real_T(OccupiedLanesPosMid2, b_i);
          OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
          for (b_i = 0; b_i <= loop_ub; b_i++) {
            OccupiedLanesPosMid2_data[b_i] = d + (double)((IsStopSpeedPlan - 1) *
              b_i);
          }
        } else {
          eml_float_colon(GlobVars->TrajPlanTurnAround.pos_mid2_rear[3], (double)
                          IsStopSpeedPlan - 1.0,
                          GlobVars->TrajPlanTurnAround.pos_mid2[3],
                          OccupiedLanesPosMid2);
          OccupiedLanesPosMid2_data = OccupiedLanesPosMid2->data;
        }

        /*  例如[1 -1] 掉头前道路车道序号为负，最左侧为-1 */
        eml_integer_colon_dispatcher(OccupiedLanesPosMid2_data[0],
          TargetLaneIndexOpposite, b_OccupiedLanes);
        b_OccupiedLanes_data = b_OccupiedLanes->data;
        b_do_vectors(b_OccupiedLanes, OccupiedLanesPosMid2, b_Lanes2Search, b_ia,
                     &IsStopSpeedPlan);
        b_ia_data = b_ia->data;
        b_i = ia->size[0];
        ia->size[0] = b_ia->size[0];
        emxEnsureCapacity_real_T(ia, b_i);
        ia_data = ia->data;
        loop_ub = b_ia->size[0];
        for (b_i = 0; b_i < loop_ub; b_i++) {
          ia_data[b_i] = b_ia_data[b_i];
        }

        sort(ia);
        ia_data = ia->data;
        b_i = b_Lanes2Search->size[0] * b_Lanes2Search->size[1];
        b_Lanes2Search->size[0] = 1;
        b_Lanes2Search->size[1] = ia->size[0];
        emxEnsureCapacity_int16_T(b_Lanes2Search, b_i);
        b_Lanes2Search_data = b_Lanes2Search->data;
        loop_ub = ia->size[0];
        for (b_i = 0; b_i < loop_ub; b_i++) {
          b_Lanes2Search_data[b_i] = b_OccupiedLanes_data[(int)ia_data[b_i] - 1];
        }

        /*  （非已占据车道的将侵入车道+已占据车道到将侵入的车道之间的车道） */
        IsStopSpeedPlan = ia->size[0] - 1;
        trueCount = 0;
        for (i = 0; i <= IsStopSpeedPlan; i++) {
          if (b_Lanes2Search_data[i] != 0) {
            trueCount++;
          }
        }

        loop_ub = 0;
        for (i = 0; i <= IsStopSpeedPlan; i++) {
          if (b_Lanes2Search_data[i] != 0) {
            b_Lanes2Search_data[loop_ub] = b_Lanes2Search_data[i];
            loop_ub++;
          }
        }

        b_i = b_Lanes2Search->size[0] * b_Lanes2Search->size[1];
        b_Lanes2Search->size[0] = 1;
        b_Lanes2Search->size[1] = trueCount;
        emxEnsureCapacity_int16_T(b_Lanes2Search, b_i);

        /*  （非已占据车道的目标车道+已占据车道到目标车道之间的车道）上搜寻前后车 → 判断碰撞可能性（起步决策） “将前进路径简化为pos_mid2到pos_end的线段→d_veh2cross,timegap” */
        TurnAroundState = 3;
        a_predict = ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                        CurrentLaneFrontVel, CurrentLaneFrontDis, speed, 0.0,
                        CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                        CalibrationVars->ACC.d_wait2faultyCar,
                        CalibrationVars->ACC.tau_v_com,
                        CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                        CalibrationVars->ACC.tau_v_bre,
                        CalibrationVars->ACC.tau_v_emg,
                        CalibrationVars->ACC.tau_d_emg,
                        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
        i = 0;
        exitg1 = false;
        while ((!exitg1) && (i < 20)) {
          if (isMember(IndexOfLaneOppositeCar[i], b_Lanes2Search)) {
            pos_l_TargetLane = GlobVars->
              TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[i] - 1];
            d_cur2pot_tar = (pos_l_TargetLane -
                             (GlobVars->TrajPlanTurnAround.pos_mid2[0] *
                              GlobVars->TrajPlanTurnAround.pos_end[1] -
                              GlobVars->TrajPlanTurnAround.pos_end[0] *
                              GlobVars->TrajPlanTurnAround.pos_mid2[1]) /
                             (GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                              GlobVars->TrajPlanTurnAround.pos_end[0])) /
              ((GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                GlobVars->TrajPlanTurnAround.pos_end[1]) /
               (GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                GlobVars->TrajPlanTurnAround.pos_end[0]));
            disOppositeCar2circle2 = PosSOppositeCar[i] - d_cur2pot_tar;
            if (disOppositeCar2circle2 > 0.0) {
              /*  a_OppositeCarFront=min(a_OppositeCarFront,1.5); */
              /*  [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])),..., */
              /*      [0-0.01 0.01+max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
              b_speed[0] = SpeedOppositeCar[i];
              b_speed[1] = 1.0E-5;
              d = maximum(b_speed);
              b_speed[0] = 0.0;
              b_speed[1] = ((disOppositeCar2circle2 - 0.5 * w_veh) - D_safe2) /
                d;
              timeGap = maximum(b_speed);
              targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid2[0] -
                d_cur2pot_tar;
              d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid2[1] -
                pos_l_TargetLane;
              b_speed[0] = a_predict;
              b_speed[1] = a_max_com;
              d = c_minimum(b_speed);
              b_speed[0] = speed + d * timeGap;
              b_speed[1] = v_max_turnAround_tmp_tmp;
              if (0.5 * (c_minimum(b_speed) + speed) * timeGap <= sqrt
                  (targetSpeed * targetSpeed + d_cur2pot_tar * d_cur2pot_tar) +
                  l_veh * 0.5) {
                TurnAroundState = 2;
                exitg1 = true;
              } else {
                i++;
              }
            } else if ((disOppositeCar2circle2 <= 0.0) &&
                       (disOppositeCar2circle2 > -LengthOppositeCar[i])) {
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

      emxFree_int32_T(&b_ia);
      emxFree_int16_T(&b_Lanes2Search);
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

    /*  进入掉头路径前决策 */
    if ((TurnAroundState == 0) || ((TurnAroundState == 1) &&
         (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s >= 0.0))) {
      b_i = TargetLaneIndexOpposite;
      if (b_i - 1 >= 0) {
        WidthOfLanesOpposite_size[0] = 1;
      }

      for (i = 0; i < b_i; i++) {
        if (i < 1) {
          loop_ub = 0;
        } else {
          loop_ub = i;
        }

        WidthOfLanesOpposite_size[1] = loop_ub;
        if (loop_ub - 1 >= 0) {
          memcpy(&posOfLaneCenterline[0], &WidthOfLanesOpposite[0], (unsigned
                  int)loop_ub * sizeof(double));
        }

        d = (0.5 * jerk + WidthOfGap) + 0.5 * WidthOfLanesOpposite[i];
        if (d + sum(posOfLaneCenterline, WidthOfLanesOpposite_size) <
            TurningRadius) {
          if (i < 1) {
            loop_ub = 0;
          } else {
            loop_ub = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                   (unsigned int)loop_ub * sizeof(double));
          }

          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
            TurningRadius * acos((TurningRadius - (d + sum
            (WidthOfLanesOpposite_data, b_WidthOfLanesOpposite_size))) /
            TurningRadius);
        } else {
          if (i < 1) {
            loop_ub = 0;
          } else {
            loop_ub = i;
          }

          b_WidthOfLanesOpposite_size[0] = 1;
          b_WidthOfLanesOpposite_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            memcpy(&WidthOfLanesOpposite_data[0], &WidthOfLanesOpposite[0],
                   (unsigned int)loop_ub * sizeof(double));
          }

          d_veh2cross[i] = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
            (TurningRadius * asin(((d + sum(WidthOfLanesOpposite_data,
                 b_WidthOfLanesOpposite_size)) - TurningRadius) / TurningRadius)
             + TurningRadius * 3.1415926535897931 / 2.0);

          /* , */
        }
      }

      /*  策略模式判断 */
      if (dec_trunAround == 0) {
        d = GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s;
        if ((d <= (0.0 - speed * speed) / (2.0 *
              CalibrationVars->TrajPlanTurnAround.a_min) + 2.0 *
             Parameters.l_veh) && (d > 0.0) && (pos_l <
             GlobVars->TrajPlanTurnAround.posCircle[1])) {
          dec_trunAround = 1;
        }
      } else if (((GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s <= 0.0) ||
                  (wait_turnAround == 1)) && (pos_l <
                  GlobVars->TrajPlanTurnAround.posCircle[1])) {
        dec_trunAround = 0;
      }

      /*  停车决策 */
      if (dec_trunAround == 1) {
        j = 0;
        exitg1 = false;
        while ((!exitg1) && (j < 6)) {
          if ((IndexOfLaneOppositeCarFront[j] <= TargetLaneIndexOpposite) &&
              (IndexOfLaneOppositeCarFront[j] > 0)) {
            b_speed[0] = speed;
            b_speed[1] = 1.0E-5;
            d = maximum(b_speed);
            b_speed[0] = ((d_veh2cross[IndexOfLaneOppositeCarFront[j] - 1] +
                           l_veh * 0.5) / d * SpeedOppositeCarFront[j] + 0.5 *
                          w_veh) + D_safe2;
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
                           CalibrationVars->ACC.d_wait2faultyCar,
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
           GlobVars->TrajPlanTurnAround.posCircle[1],
           GlobVars->TrajPlanTurnAround.pos_mid1[0] -
           GlobVars->TrajPlanTurnAround.posCircle[0]) - rt_atan2d_snf(pos_l -
          GlobVars->TrajPlanTurnAround.posCircle[1], pos_s -
          GlobVars->TrajPlanTurnAround.posCircle[0])) * Parameters.turningRadius
                           + CalibrationVars->ACC.d_wait, speed, 0,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.d_wait2faultyCar,
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
              (IndexOfLaneOppositeCarFront[j] > 0)) {
            IsStopSpeedPlan = IndexOfLaneOppositeCarFront[j] - 1;
            b_speed[0] = ACC(13.888888888888889,
                             SpeedOppositeCarRear[IsStopSpeedPlan],
                             PosSOppositeCarFront[j] -
                             PosSOppositeCarRear[IsStopSpeedPlan],
                             SpeedOppositeCarFront[j], 0.0,
                             CalibrationVars->ACC.a_max,
                             CalibrationVars->ACC.a_min,
                             CalibrationVars->ACC.d_wait2faultyCar,
                             CalibrationVars->ACC.tau_v_com,
                             CalibrationVars->ACC.tau_v,
                             CalibrationVars->ACC.tau_d,
                             CalibrationVars->ACC.tau_v_bre,
                             CalibrationVars->ACC.tau_v_emg,
                             CalibrationVars->ACC.tau_d_emg,
                             CalibrationVars->ACC.t_acc,
                             CalibrationVars->ACC.d_wait);
            b_speed[1] = 0.0;
            disOppositeCar2circle2 = maximum(b_speed);
            if (SpeedOppositeCarFront[j] <= 0.01) {
              disOppositeCar2circle2 = 0.0;
            }

            /*                      [timeGap,~,~] = fzero(@(t)(max([0.00001 SpeedOppositeCarFront(j)])*t+0.5*a_OppositeCarFront*t.^2-max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])),..., */
            /*                          [0-0.01 0.01+max([0 (PosSOppositeCarFront(j)-0.5*w_veh-D_safe2)])/max([0.00001 SpeedOppositeCarFront(j)])]); */
            /*                      % timeGap=max([0 (PosSOppositeCarFront(j)-0.5*w_veh-l_veh)/max([SpeedOppositeCarFront(j) 0.00001])]); */
            /*                      s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            b_speed[0] = 0.0;
            b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) - D_safe2;
            dv[0] = 1.0E-5;
            dv[1] = fmin(SpeedOppositeCarFront[j], v_max);
            d = maximum(b_speed);
            b_speed[0] = -0.01;
            b_speed[1] = d / maximum(dv) + 0.01;
            e_fzero(SpeedOppositeCarFront, (double)j + 1.0, fmin
                    (disOppositeCar2circle2, 1.5), v_max, PosSOppositeCarFront,
                    w_veh, D_safe2, b_speed, &timeGap, &d_cur2pot_tar,
                    &pos_l_TargetLane);

            /* , */
            /*  s_max=0.5*(min([speed+min([a_predict a_max_com])*timeGap v_max_turnAround])+speed)*timeGap; */
            b_speed[0] = a_predict;
            b_speed[1] = a_max_com;
            a_predict = c_minimum(b_speed);
            if (a_predict > 0.0) {
              d_cur2pot_tar = speed + a_predict * timeGap;
              targetSpeed = fmax(d_cur2pot_tar - v_max_turnAround_tmp_tmp, 0.0);
              pos_l_TargetLane = 0.5 * (d_cur2pot_tar + speed) * timeGap -
                targetSpeed * targetSpeed / (2.0 * a_predict +
                2.2204460492503131E-16);
            } else {
              dv1[0] = 0.0;
              dv1[1] = speed + a_predict * timeGap;
              dv1[2] = v_max_turnAround_tmp_tmp;
              pos_l_TargetLane = 0.5 * (median(dv1) + speed) * timeGap;
            }

            if (pos_l_TargetLane <= d_veh2cross[IsStopSpeedPlan] + l_veh * 0.5)
            {
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

  emxFree_real_T(&Lanes2Search);
  emxFree_real_T(&OccupiedLanesPosMid2);

  /*  ACC速度规划 */
  *a_soll_TrajPlanTurnAround = 0.0;

  /* 20220322 */
  if (TypeOfTurnAround == 1) {
    /*  一次顺车掉头速度规划 */
    if (((wait_turnAround == 1) && (GlobVars->TrajPlanTurnAround.posCircle[0] >
          pos_s)) || (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
      /*  a_soll=min([ACC(v_max_int,v_b,s_b,speed,wait_avoidOncomingVehicle) ACC(v_max_int,0,max([0 d_veh2waitingArea+2+l_veh]),speed,wait_avoidOncomingVehicle)]); */
      b_speed[0] = 0.0;
      b_speed[1] = ((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
                    CalibrationVars->ACC.d_wait) -
        CalibrationVars->TrajPlanTurnAround.d_gap2stop;
      d = maximum(b_speed);
      b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                         wait_turnAround, CalibrationVars->ACC.a_max,
                         CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.d_wait2faultyCar,
                         CalibrationVars->ACC.tau_v_com,
                         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
                         CalibrationVars->ACC.tau_v_bre,
                         CalibrationVars->ACC.tau_v_emg,
                         CalibrationVars->ACC.tau_d_emg,
                         CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
      b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                         0.0, d, speed, wait_turnAround,
                         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                         CalibrationVars->ACC.d_wait2faultyCar,
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
         CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
         CalibrationVars->ACC.d_wait);
    } else if (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s > 2.0 *
               Parameters.l_veh) {
      *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
        CurrentLaneFrontDis, speed, wait_turnAround, CalibrationVars->ACC.a_max,
        CalibrationVars->ACC.a_min, CalibrationVars->ACC.d_wait2faultyCar,
        CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
        CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
        CalibrationVars->ACC.tau_v_emg, CalibrationVars->ACC.tau_d_emg,
        CalibrationVars->ACC.t_acc, CalibrationVars->ACC.d_wait);
    } else {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
         CalibrationVars->ACC.d_wait);
    }

    b_speed[0] = *a_soll_TrajPlanTurnAround;
    b_speed[1] = a_soll;
    *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
  } else if (TypeOfTurnAround == 2) {
    /*  二次顺车掉头速度规划 */
    /*      if TurnAroundState==0 */
    if (TurnAroundState == 0) {
      if ((wait_turnAround == 1) ||
          (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
        b_speed[0] = 0.0;
        b_speed[1] = ((GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s) +
                      CalibrationVars->ACC.d_wait) -
          CalibrationVars->TrajPlanTurnAround.d_gap2stop;
        d = maximum(b_speed);
        b_speed[0] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
                           wait_turnAround, CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.d_wait2faultyCar,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
        b_speed[1] = b_ACC(CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
                           0.0, d, speed, wait_turnAround,
                           CalibrationVars->ACC.a_max,
                           CalibrationVars->ACC.a_min,
                           CalibrationVars->ACC.d_wait2faultyCar,
                           CalibrationVars->ACC.tau_v_com,
                           CalibrationVars->ACC.tau_v,
                           CalibrationVars->ACC.tau_d,
                           CalibrationVars->ACC.tau_v_bre,
                           CalibrationVars->ACC.tau_v_emg,
                           CalibrationVars->ACC.tau_d_emg,
                           CalibrationVars->ACC.t_acc,
                           CalibrationVars->ACC.d_wait);
        *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
      } else if (dec_trunAround == 1) {
        *a_soll_TrajPlanTurnAround = fmin(ACC(v_max, 0.0, (((rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
           GlobVars->TrajPlanTurnAround.posCircle[1],
           GlobVars->TrajPlanTurnAround.pos_mid1[0] -
           GlobVars->TrajPlanTurnAround.posCircle[0]) - rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_start[1] -
           GlobVars->TrajPlanTurnAround.posCircle[1],
           GlobVars->TrajPlanTurnAround.pos_start[0] -
           GlobVars->TrajPlanTurnAround.posCircle[0])) *
          Parameters.turningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
          - pos_s) + CalibrationVars->ACC.d_wait, speed, 0.0,
          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
          CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
          CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
          CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
          CalibrationVars->ACC.d_wait), b_ACC
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
           CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
           CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
           CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
           CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
           CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
           CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
           CalibrationVars->ACC.d_wait));
      } else if (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s > 2.0 *
                 Parameters.l_veh) {
        *a_soll_TrajPlanTurnAround = b_ACC(v_max, CurrentLaneFrontVel,
          CurrentLaneFrontDis, speed, wait_turnAround,
          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
          CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
          CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
          CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
          CalibrationVars->ACC.d_wait);
      } else {
        *a_soll_TrajPlanTurnAround = fmin(ACC(v_max, 0.0, (((rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_mid1[1] -
           GlobVars->TrajPlanTurnAround.posCircle[1],
           GlobVars->TrajPlanTurnAround.pos_mid1[0] -
           GlobVars->TrajPlanTurnAround.posCircle[0]) - rt_atan2d_snf
          (GlobVars->TrajPlanTurnAround.pos_start[1] -
           GlobVars->TrajPlanTurnAround.posCircle[1],
           GlobVars->TrajPlanTurnAround.pos_start[0] -
           GlobVars->TrajPlanTurnAround.posCircle[0])) *
          Parameters.turningRadius + GlobVars->TrajPlanTurnAround.pos_start[0])
          - pos_s) + CalibrationVars->ACC.d_wait, speed, 0.0,
          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
          CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
          CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
          CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
          CalibrationVars->ACC.d_wait), b_ACC
          (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
           CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
           CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
           CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
           CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
           CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
           CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
           CalibrationVars->ACC.d_wait));
      }
    } else if (TurnAroundState == 1) {
      b_speed[0] = ACClowSpeed
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
         CalibrationVars->ACClowSpeed.a_max, CalibrationVars->ACClowSpeed.a_min,
         CalibrationVars->ACClowSpeed.a_min_com,
         CalibrationVars->ACClowSpeed.tau_v_com,
         CalibrationVars->ACClowSpeed.tau_v, CalibrationVars->ACClowSpeed.tau_d,
         CalibrationVars->ACClowSpeed.tau_v_bre,
         CalibrationVars->ACClowSpeed.tau_v_emg,
         CalibrationVars->ACClowSpeed.tau_d_emg,
         CalibrationVars->ACClowSpeed.tau_d_lowspeed,
         CalibrationVars->ACClowSpeed.t_acc, CalibrationVars->ACClowSpeed.d_wait);
      b_speed[1] = ACClowSpeed
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0,
         (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid1[1] -
                        GlobVars->TrajPlanTurnAround.posCircle[1],
                        GlobVars->TrajPlanTurnAround.pos_mid1[0] -
                        GlobVars->TrajPlanTurnAround.posCircle[0]) -
          rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.posCircle[1], pos_s
                        - GlobVars->TrajPlanTurnAround.posCircle[0])) *
         Parameters.turningRadius + CalibrationVars->ACClowSpeed.d_wait, speed,
         CalibrationVars->ACClowSpeed.a_max, CalibrationVars->ACClowSpeed.a_min,
         CalibrationVars->ACClowSpeed.a_min_com,
         CalibrationVars->ACClowSpeed.tau_v_com,
         CalibrationVars->ACClowSpeed.tau_v, CalibrationVars->ACClowSpeed.tau_d,
         CalibrationVars->ACClowSpeed.tau_v_bre,
         CalibrationVars->ACClowSpeed.tau_v_emg,
         CalibrationVars->ACClowSpeed.tau_d_emg,
         CalibrationVars->ACClowSpeed.tau_d_lowspeed,
         CalibrationVars->ACClowSpeed.t_acc, CalibrationVars->ACClowSpeed.d_wait);
      *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
      targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid1[0] - pos_s;
      d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid1[1] - pos_l;
      if (sqrt(targetSpeed * targetSpeed + d_cur2pot_tar * d_cur2pot_tar) < 0.15)
      {
        if (rtIsNaN(speed)) {
          d = rtNaN;
        } else if (speed < 0.0) {
          d = -1.0;
        } else {
          d = (speed > 0.0);
        }

        *a_soll_TrajPlanTurnAround = -4.0 * d;
      }
    } else if (TurnAroundState == 2) {
      dis2pos_mid2 = b_mod(rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid2[1]
        - GlobVars->TrajPlanTurnAround.posCircle2[1],
        GlobVars->TrajPlanTurnAround.pos_mid2[0] -
        GlobVars->TrajPlanTurnAround.posCircle2[0]) - rt_atan2d_snf(pos_l -
        GlobVars->TrajPlanTurnAround.posCircle2[1], pos_s -
        GlobVars->TrajPlanTurnAround.posCircle2[0])) * Parameters.turningRadius;
      b_speed[0] = ACClowSpeed
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed,
         CalibrationVars->ACClowSpeed.a_max, CalibrationVars->ACClowSpeed.a_min,
         CalibrationVars->ACClowSpeed.a_min_com,
         CalibrationVars->ACClowSpeed.tau_v_com,
         CalibrationVars->ACClowSpeed.tau_v, CalibrationVars->ACClowSpeed.tau_d,
         CalibrationVars->ACClowSpeed.tau_v_bre,
         CalibrationVars->ACClowSpeed.tau_v_emg,
         CalibrationVars->ACClowSpeed.tau_d_emg,
         CalibrationVars->ACClowSpeed.tau_d_lowspeed,
         CalibrationVars->ACClowSpeed.t_acc, CalibrationVars->ACClowSpeed.d_wait);
      b_speed[1] = ACClowSpeed
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround, 0.0, dis2pos_mid2
         + CalibrationVars->ACClowSpeed.d_wait, speed,
         CalibrationVars->ACClowSpeed.a_max, CalibrationVars->ACClowSpeed.a_min,
         CalibrationVars->ACClowSpeed.a_min_com,
         CalibrationVars->ACClowSpeed.tau_v_com,
         CalibrationVars->ACClowSpeed.tau_v, CalibrationVars->ACClowSpeed.tau_d,
         CalibrationVars->ACClowSpeed.tau_v_bre,
         CalibrationVars->ACClowSpeed.tau_v_emg,
         CalibrationVars->ACClowSpeed.tau_d_emg,
         CalibrationVars->ACClowSpeed.tau_d_lowspeed,
         CalibrationVars->ACClowSpeed.t_acc, CalibrationVars->ACClowSpeed.d_wait);
      *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
      targetSpeed = GlobVars->TrajPlanTurnAround.pos_mid2[0] - pos_s;
      d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid2[1] - pos_l;
      if (sqrt(targetSpeed * targetSpeed + d_cur2pot_tar * d_cur2pot_tar) < 0.15)
      {
        if (rtIsNaN(speed)) {
          d = rtNaN;
        } else if (speed < 0.0) {
          d = -1.0;
        } else {
          d = (speed > 0.0);
        }

        *a_soll_TrajPlanTurnAround = -4.0 * d;
      }
    } else if (TurnAroundState == 3) {
      *a_soll_TrajPlanTurnAround = b_ACC
        (CalibrationVars->TrajPlanTurnAround.v_max_turnAround,
         CurrentLaneFrontVel, CurrentLaneFrontDis, speed, wait_turnAround,
         CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
         CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
         CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
         CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
         CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
         CalibrationVars->ACC.d_wait);
    }

    b_speed[0] = *a_soll_TrajPlanTurnAround;
    b_speed[1] = a_soll;
    *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
  }

  /*  AEB决策 */
  if (*AEBactive == 0) {
    if ((TypeOfTurnAround == 1) && (GlobVars->TrajPlanTurnAround.posCircle[0] -
         pos_s <= 0.0)) {
      boolean_T exitg4;
      j = 0;
      exitg4 = false;
      while ((!exitg4) && (j < 20)) {
        /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离) */
        if (IndexOfLaneOppositeCar[j] != 0) {
          d = GlobVars->
            TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[j] - 1];
          a_predict = d - (GlobVars->TrajPlanTurnAround.posCircle[1] -
                           TurningRadius);
          if (a_predict < TurningRadius) {
            d_cur2pot_tar = GlobVars->TrajPlanTurnAround.posCircle[0] +
              TurningRadius * acos((TurningRadius - a_predict) / TurningRadius);
          } else if (a_predict > (TurningRadius +
                                  GlobVars->TrajPlanTurnAround.posCircle2[1]) -
                     GlobVars->TrajPlanTurnAround.posCircle[1]) {
            d_cur2pot_tar = GlobVars->TrajPlanTurnAround.posCircle[0] +
              (((TurningRadius * asin((((a_predict - TurningRadius) -
                    GlobVars->TrajPlanTurnAround.posCircle2[1]) +
                   GlobVars->TrajPlanTurnAround.posCircle[1]) / TurningRadius) +
                 GlobVars->TrajPlanTurnAround.posCircle2[1]) -
                GlobVars->TrajPlanTurnAround.posCircle[1]) + TurningRadius *
               3.1415926535897931 / 2.0);

            /* , */
          } else {
            d_cur2pot_tar = ((GlobVars->TrajPlanTurnAround.posCircle[0] +
                              TurningRadius * 3.1415926535897931 / 2.0) +
                             a_predict) - TurningRadius;
          }

          if ((pos_l < GlobVars->TrajPlanTurnAround.posCircle[1]) && (pos_s <=
               GlobVars->TrajPlanTurnAround.posCircle[0])) {
            pos_l_TargetLane = pos_s;
          } else if ((pos_l < GlobVars->TrajPlanTurnAround.posCircle[1]) &&
                     (pos_s > GlobVars->TrajPlanTurnAround.posCircle[0])) {
            pos_l_TargetLane = GlobVars->TrajPlanTurnAround.posCircle[0] + atan
              ((pos_s - GlobVars->TrajPlanTurnAround.posCircle[0]) /
               ((GlobVars->TrajPlanTurnAround.posCircle[1] +
                 2.2204460492503131E-16) - pos_l)) * TurningRadius;
          } else if ((pos_l >= GlobVars->TrajPlanTurnAround.posCircle[1]) &&
                     (pos_l <= GlobVars->TrajPlanTurnAround.posCircle2[1])) {
            pos_l_TargetLane = ((GlobVars->TrajPlanTurnAround.posCircle[0] +
                                 1.5707963267948966 * TurningRadius) + pos_l) -
              GlobVars->TrajPlanTurnAround.posCircle[1];
          } else if ((pos_l > GlobVars->TrajPlanTurnAround.posCircle2[1]) &&
                     (pos_s > GlobVars->TrajPlanTurnAround.posCircle2[0])) {
            pos_l_TargetLane = (((GlobVars->TrajPlanTurnAround.posCircle[0] +
                                  1.5707963267948966 * TurningRadius) +
                                 GlobVars->TrajPlanTurnAround.posCircle2[1]) -
                                GlobVars->TrajPlanTurnAround.posCircle[1]) +
              atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle2[1]) /
                   ((pos_s + 2.2204460492503131E-16) -
                    GlobVars->TrajPlanTurnAround.posCircle2[0])) * TurningRadius;
          } else {
            pos_l_TargetLane = (((((GlobVars->TrajPlanTurnAround.posCircle[0] +
              1.5707963267948966 * TurningRadius) +
              GlobVars->TrajPlanTurnAround.posCircle2[1]) -
                                  GlobVars->TrajPlanTurnAround.posCircle[1]) +
                                 1.5707963267948966 * TurningRadius) +
                                GlobVars->TrajPlanTurnAround.posCircle2[0]) -
              pos_s;
          }

          d_cur2pot_tar -= pos_l_TargetLane;

          /*  用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2 */
          if (d < GlobVars->TrajPlanTurnAround.posCircle[1]) {
            targetSpeed = d - GlobVars->TrajPlanTurnAround.posCircle[1];
            disOppositeCar2circle2 = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.posCircle[0] + sqrt(fabs
                (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
          } else if ((d >= GlobVars->TrajPlanTurnAround.posCircle[1]) && (d <=
                      GlobVars->TrajPlanTurnAround.posCircle2[1])) {
            disOppositeCar2circle2 = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.posCircle[0] + TurningRadius);
          } else {
            targetSpeed = d - GlobVars->TrajPlanTurnAround.posCircle2[1];
            disOppositeCar2circle2 = PosSOppositeCar[j] -
              (GlobVars->TrajPlanTurnAround.posCircle2[0] + sqrt(fabs
                (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
          }

          if ((IndexOfLaneOppositeCar[j] <= TargetLaneIndexOpposite) &&
              (IndexOfLaneOppositeCar[j] > 0) && (d_cur2pot_tar > 0.0)) {
            if (disOppositeCar2circle2 > 0.0) {
              /*                      if IndexOfLaneOppositeCar(j)<=TargetLaneIndexOpposite && IndexOfLaneOppositeCar(j)>0 && d_veh2cross_strich>0 */
              b_speed[0] = SpeedOppositeCarFront[j];
              b_speed[1] = 1.0E-5;
              d = maximum(b_speed);
              b_speed[0] = 0.0;
              b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) / d;
              timeGap = maximum(b_speed);
              b_speed[0] = speed + a_max_com * timeGap;
              b_speed[1] = v_max_turnAround_tmp_tmp;
              if (0.5 * (c_minimum(b_speed) + speed) * timeGap <= d_cur2pot_tar
                  + l_veh * 0.5) {
                *AEBactive = 5;
                exitg4 = true;
              } else {
                j++;
              }
            } else if ((disOppositeCar2circle2 <= 0.0) &&
                       (disOppositeCar2circle2 > -LengthOppositeCar[j])) {
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
    } else if (TypeOfTurnAround == 2) {
      boolean_T guard1 = false;
      guard1 = false;
      if (TurnAroundState == 1) {
        d = GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s;
        if (d <= 0.0) {
          boolean_T exitg3;
          j = 0;
          exitg3 = false;
          while ((!exitg3) && (j < 20)) {
            if (IndexOfLaneOppositeCar[j] != 0) {
              /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、PosCircle1、TurningRadius求d_veh2cross_strich(注意是车头到车道中心线的距离) */
              if (pos_s < GlobVars->TrajPlanTurnAround.posCircle[0]) {
                a_predict = GlobVars->
                  TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[j] -
                  1] - (GlobVars->TrajPlanTurnAround.posCircle[1] -
                        TurningRadius);
                if (a_predict < TurningRadius) {
                  d_cur2pot_tar = d + TurningRadius * acos((TurningRadius -
                    a_predict) / TurningRadius);
                } else {
                  d_cur2pot_tar = d + (TurningRadius * asin((a_predict -
                    TurningRadius) / TurningRadius) + TurningRadius *
                                       3.1415926535897931 / 2.0);
                }
              } else {
                d_cur2pot_tar = (asin
                                 ((GlobVars->
                                   TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar
                                   [j] - 1] -
                                   GlobVars->TrajPlanTurnAround.posCircle[1]) /
                                  TurningRadius) - rt_atan2d_snf(pos_l -
                  GlobVars->TrajPlanTurnAround.posCircle[1], pos_s -
                  GlobVars->TrajPlanTurnAround.posCircle[0])) * TurningRadius;
              }

              /*  用PosCircle1、TurningRadius、LaneCenterline、PosSOppositeCar(j)求disOppositeCar2circle2 */
              targetSpeed = GlobVars->
                TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[j] - 1]
                - GlobVars->TrajPlanTurnAround.posCircle[1];
              disOppositeCar2circle2 = PosSOppositeCar[j] -
                (GlobVars->TrajPlanTurnAround.posCircle[0] + sqrt(fabs
                  (TurningRadius * TurningRadius - targetSpeed * targetSpeed)));
              if ((IndexOfLaneOppositeCar[j] <= TargetLaneIndexOpposite) &&
                  (IndexOfLaneOppositeCar[j] > 0) && (d_cur2pot_tar > 0.0)) {
                if (disOppositeCar2circle2 > 0.0) {
                  b_speed[0] = SpeedOppositeCarFront[j];
                  b_speed[1] = 1.0E-5;
                  a_predict = maximum(b_speed);
                  b_speed[0] = 0.0;
                  b_speed[1] = (PosSOppositeCarFront[j] - 0.5 * w_veh) /
                    a_predict;
                  timeGap = maximum(b_speed);
                  b_speed[0] = speed + a_max_com * timeGap;
                  b_speed[1] = v_max_turnAround_tmp_tmp;
                  if (0.5 * (c_minimum(b_speed) + speed) * timeGap <=
                      d_cur2pot_tar + 0.5 * l_veh * (double)
                      (IndexOfLaneOppositeCar[j] != TargetLaneIndexOpposite)) {
                    *AEBactive = 5;
                    exitg3 = true;
                  } else {
                    j++;
                  }
                } else if ((disOppositeCar2circle2 <= 0.0) &&
                           (disOppositeCar2circle2 > -LengthOppositeCar[j])) {
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
          boolean_T exitg5;
          d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] -
            GlobVars->TrajPlanTurnAround.pos_mid2_rear[0];
          k = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[1] -
               GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]) / d_cur2pot_tar;
          b = (GlobVars->TrajPlanTurnAround.pos_mid1_rear[0] *
               GlobVars->TrajPlanTurnAround.pos_mid2_rear[1] -
               GlobVars->TrajPlanTurnAround.pos_mid2_rear[0] *
               GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]) / d_cur2pot_tar;
          i = 0;
          exitg5 = false;
          while ((!exitg5) && (i < 20)) {
            if (IndexOfLaneOppositeCar[i] != 0) {
              /*  用pos_s、Pos_circle2、pos_l求pos_s_rear、pos_l_rear */
              pos_l_TargetLane = atan((pos_l -
                GlobVars->TrajPlanTurnAround.posCircle2[1]) / (pos_s -
                GlobVars->TrajPlanTurnAround.posCircle2[0]));

              /*  用IndexOfLaneOppositeCar(i)、pos_s_rear、pos_l_rear、LaneCenterline、k、b、求d_veh2cross_strich(注意是车尾到车道中心线的距离) */
              d_cur2pot_tar = GlobVars->
                TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[i] - 1];
              disOppositeCar2circle2 = (d_cur2pot_tar - b) / k;
              targetSpeed = (pos_s + sin(pos_l_TargetLane) * l_veh * 0.5) -
                disOppositeCar2circle2;
              d_cur2pot_tar = (pos_l - cos(pos_l_TargetLane) * l_veh * 0.5) -
                d_cur2pot_tar;
              if (rtIsNaN(d_cur2pot_tar)) {
                d = rtNaN;
              } else if (d_cur2pot_tar < 0.0) {
                d = -1.0;
              } else {
                d = (d_cur2pot_tar > 0.0);
              }

              d_cur2pot_tar = sqrt(targetSpeed * targetSpeed + d_cur2pot_tar *
                                   d_cur2pot_tar) * d;
              if (IndexOfLaneOppositeCar[i] <= TargetLaneIndexOpposite) {
                b_speed[0] = 1.0;
                b_speed[1] = GlobVars->TrajPlanTurnAround.pos_mid2_rear[3];
                if ((IndexOfLaneOppositeCar[i] >= maximum(b_speed)) &&
                    (d_cur2pot_tar > 0.0)) {
                  disOppositeCar2circle2 = PosSOppositeCar[i] -
                    disOppositeCar2circle2;
                  if (disOppositeCar2circle2 > 0.0) {
                    b_speed[0] = SpeedOppositeCar[i];
                    b_speed[1] = 1.0E-5;
                    d = maximum(b_speed);
                    b_speed[0] = 0.0;
                    b_speed[1] = (disOppositeCar2circle2 - 0.5 * w_veh) / d;
                    timeGap = maximum(b_speed);
                    b_speed[0] = speed + a_max_com * timeGap;
                    b_speed[1] = v_max_turnAround_tmp_tmp;
                    if (0.5 * (c_minimum(b_speed) + speed) * timeGap <=
                        d_cur2pot_tar + 0.5 * l_veh * (double)
                        (IndexOfLaneOppositeCar[i] != TargetLaneIndexOpposite))
                    {
                      *AEBactive = 5;
                      exitg5 = true;
                    } else {
                      i++;
                    }
                  } else if ((disOppositeCar2circle2 <= 0.0) &&
                             (disOppositeCar2circle2 > -LengthOppositeCar[i])) {
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
              pos_l_TargetLane = atan((pos_l -
                GlobVars->TrajPlanTurnAround.posCircle2[1]) / (pos_s -
                GlobVars->TrajPlanTurnAround.posCircle2[0]));
              d_cur2pot_tar = pos_s + sin(pos_l_TargetLane) * l_veh * 0.5;
              pos_l_TargetLane = pos_l - cos(pos_l_TargetLane) * l_veh * 0.5;

              /*  IndexOfLaneCodirectCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(默认同向第二条车道的宽度为3.2)(注意是车尾到车道中心线的距离) */
              if (IndexOfLaneCodirectCar[j] == -1) {
                targetSpeed = d_cur2pot_tar -
                  (GlobVars->TrajPlanTurnAround.laneCenterline[6] - b) / k;
                d_cur2pot_tar = pos_l_TargetLane -
                  GlobVars->TrajPlanTurnAround.laneCenterline[6];
                if (rtIsNaN(d_cur2pot_tar)) {
                  d = rtNaN;
                } else if (d_cur2pot_tar < 0.0) {
                  d = -1.0;
                } else {
                  d = (d_cur2pot_tar > 0.0);
                }

                d_cur2pot_tar = sqrt(targetSpeed * targetSpeed + d_cur2pot_tar *
                                     d_cur2pot_tar) * d;
              } else {
                targetSpeed = d_cur2pot_tar -
                  ((GlobVars->TrajPlanTurnAround.laneCenterline[6] - 3.2) - b) /
                  k;
                d_cur2pot_tar = pos_l_TargetLane -
                  (GlobVars->TrajPlanTurnAround.laneCenterline[6] - 3.2);
                if (rtIsNaN(d_cur2pot_tar)) {
                  d = rtNaN;
                } else if (d_cur2pot_tar < 0.0) {
                  d = -1.0;
                } else {
                  d = (d_cur2pot_tar > 0.0);
                }

                d_cur2pot_tar = sqrt(targetSpeed * targetSpeed + d_cur2pot_tar *
                                     d_cur2pot_tar) * d;
              }

              if ((IndexOfLaneCodirectCar[j] >=
                   GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]) &&
                  (d_cur2pot_tar > 0.0)) {
                if (IndexOfLaneCodirectCar[j] == -1) {
                  disOppositeCar2circle2 =
                    (GlobVars->TrajPlanTurnAround.laneCenterline[6] - b) / k -
                    PosSCodirectCar[j];
                } else {
                  disOppositeCar2circle2 =
                    ((GlobVars->TrajPlanTurnAround.laneCenterline[6] - 3.2) - b)
                    / k - PosSCodirectCar[j];
                }

                if (disOppositeCar2circle2 > 0.0) {
                  b_speed[0] = SpeedCodirectCar[j];
                  b_speed[1] = 1.0E-5;
                  d = maximum(b_speed);
                  b_speed[0] = 0.0;
                  b_speed[1] = (disOppositeCar2circle2 - 0.5 * w_veh) / d;
                  timeGap = maximum(b_speed);
                  b_speed[0] = speed + a_max_com * timeGap;
                  b_speed[1] = v_max_turnAround_tmp_tmp;
                  if (0.5 * (c_minimum(b_speed) + speed) * timeGap <=
                      d_cur2pot_tar + 0.5 * l_veh * (double)
                      (IndexOfLaneCodirectCar[j] !=
                       GlobVars->TrajPlanTurnAround.pos_mid2_rear[3])) {
                    *AEBactive = 5;
                    exitg2 = true;
                  } else {
                    j++;
                  }
                } else if ((disOppositeCar2circle2 <= 0.0) &&
                           (disOppositeCar2circle2 > -LengthCodirectCar[j])) {
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
        } else if (TurnAroundState == 3) {
          d_cur2pot_tar = GlobVars->TrajPlanTurnAround.pos_mid2[0] -
            GlobVars->TrajPlanTurnAround.pos_end[0];
          k = (GlobVars->TrajPlanTurnAround.pos_mid2[1] -
               GlobVars->TrajPlanTurnAround.pos_end[1]) / d_cur2pot_tar;
          b = (GlobVars->TrajPlanTurnAround.pos_mid2[0] *
               GlobVars->TrajPlanTurnAround.pos_end[1] -
               GlobVars->TrajPlanTurnAround.pos_end[0] *
               GlobVars->TrajPlanTurnAround.pos_mid2[1]) / d_cur2pot_tar;
          i = 0;
          exitg1 = false;
          while ((!exitg1) && (i < 20)) {
            if (IndexOfLaneOppositeCar[i] != 0) {
              /*  用IndexOfLaneOppositeCar(i)、pos_s、pos_l、LaneCenterline、k、b、求d_veh2cross_strich(注意是车头到车道中心线的距离) */
              d_cur2pot_tar = GlobVars->
                TrajPlanTurnAround.laneCenterline[IndexOfLaneOppositeCar[i] - 1];
              disOppositeCar2circle2 = (d_cur2pot_tar - b) / k;
              targetSpeed = disOppositeCar2circle2 - pos_s;
              d_cur2pot_tar -= pos_l;
              if (rtIsNaN(d_cur2pot_tar)) {
                d = rtNaN;
              } else if (d_cur2pot_tar < 0.0) {
                d = -1.0;
              } else {
                d = (d_cur2pot_tar > 0.0);
              }

              d_cur2pot_tar = sqrt(targetSpeed * targetSpeed + d_cur2pot_tar *
                                   d_cur2pot_tar) * d;
              if ((IndexOfLaneOppositeCar[i] <= TargetLaneIndexOpposite) &&
                  (IndexOfLaneOppositeCar[i] > 0) && (d_cur2pot_tar > 0.0)) {
                disOppositeCar2circle2 = PosSOppositeCar[i] -
                  disOppositeCar2circle2;
                if (disOppositeCar2circle2 > 0.0) {
                  /*                          timeGap=max([0 (disOppositeCar2circle2-0.5*w_veh-l_veh)/max([SpeedOppositeCar(i) 0.00001])]); */
                  b_speed[0] = SpeedOppositeCar[i];
                  b_speed[1] = 1.0E-5;
                  d = maximum(b_speed);
                  b_speed[0] = 0.0;
                  b_speed[1] = (disOppositeCar2circle2 - 0.5 * w_veh) / d;
                  timeGap = maximum(b_speed);

                  /* 20220214 */
                  b_speed[0] = speed + a_max_com * timeGap;
                  b_speed[1] = v_max_turnAround_tmp_tmp;
                  if (0.5 * (c_minimum(b_speed) + speed) * timeGap <=
                      d_cur2pot_tar + l_veh * 0.5) {
                    *AEBactive = 5;
                    exitg1 = true;
                  } else {
                    i++;
                  }
                } else if ((disOppositeCar2circle2 <= 0.0) &&
                           (disOppositeCar2circle2 > -LengthOppositeCar[i])) {
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

  /*  输出a_soll_TrajPlanTurnAround（-4或者不变）和AEBactive=5 */
  if (*AEBactive == 5) {
    if (rtIsNaN(speed)) {
      d = rtNaN;
    } else if (speed < 0.0) {
      d = -1.0;
    } else {
      d = (speed > 0.0);
    }

    b_speed[0] = -4.0 * d;
    b_speed[1] = *a_soll_TrajPlanTurnAround;
    *a_soll_TrajPlanTurnAround = c_minimum(b_speed);
  }

  *a_sollTurnAround2Decider = *a_soll_TrajPlanTurnAround;

  /*  一次顺车掉头轨迹生成 */
  if (TypeOfTurnAround == 1) {
    /* ------------------------------------------------------------------------------------------------------------------------------------------ */
    if ((wait_turnAround == 1) ||
        (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
      /* 停车点计算 */
      stopdistance = fmin((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) -
                          CalibrationVars->TrajPlanTurnAround.d_gap2stop,
                          stopdistance);
      D_safe2 = 0.0;
    } else if (dec_trunAround == 1) {
      stopdistance = fmin((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) -
                          CalibrationVars->TrajPlanTurnAround.d_gap2stop,
                          stopdistance);
      D_safe2 = CalibrationVars->TrajPlanTurnAround.v_max_turnAround;
    } else {
      D_safe2 = v_max;
    }

    IsStopSpeedPlan = 0;
    if ((D_safe2 < speed) && (stopdistance < 200.0) && ((D_safe2 * D_safe2 -
          speed * speed) / -8.0 <= stopdistance) && (*AEBactive == 0)) {
      d_cur2pot_tar = 0.66666666666666663 * speed + 0.33333333333333331 *
        D_safe2;
      pos_l_TargetLane = d_cur2pot_tar * d_cur2pot_tar;
      d_cur2pot_tar = -pos_l_TargetLane / (0.66666666666666663 * stopdistance);
      *a_soll_TrajPlanTurnAround = fmax(*a_soll_TrajPlanTurnAround,
        d_cur2pot_tar);
      JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                a_soll_TrajPlanTurnAround,
                CalibrationVars->UrbanPlanner.jerkLimit);
      if (*a_soll_TrajPlanTurnAround >= d_cur2pot_tar) {
        k = ((sqrt(fmax(0.0, pos_l_TargetLane + 0.66666666666666663 *
                        *a_soll_TrajPlanTurnAround * stopdistance)) -
              0.66666666666666663 * speed) - 0.33333333333333331 * D_safe2) /
          (0.33333333333333331 * *a_soll_TrajPlanTurnAround +
           2.2204460492503131E-16);
        pos_l_TargetLane = D_safe2 - speed;
        jerk = (pos_l_TargetLane - *a_soll_TrajPlanTurnAround * k) / (0.5 * (k *
          k));
        d = -*a_soll_TrajPlanTurnAround / jerk;
        if ((d < k) && (d > 0.0) && (jerk > 0.0)) {
          d_cur2pot_tar = speed + 2.0 * D_safe2;
          k = 3.0 * stopdistance / d_cur2pot_tar;
          pos_l_TargetLane *= 2.0;
          jerk = -(pos_l_TargetLane * (d_cur2pot_tar * d_cur2pot_tar)) / (9.0 *
            (stopdistance * stopdistance));
          d_cur2pot_tar = pos_l_TargetLane * d_cur2pot_tar / (3.0 * stopdistance);
          d = d_cur2pot_tar;
          JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime, &d,
                    CalibrationVars->UrbanPlanner.jerkLimit);
          if (d == d_cur2pot_tar) {
            *a_soll_TrajPlanTurnAround = d_cur2pot_tar;
            if ((d_cur2pot_tar <= a_soll_ACC) || (CurrentLaneFrontVel < 0.2)) {
              IsStopSpeedPlan = 1;
            }
          }
        } else if ((*a_soll_TrajPlanTurnAround <= a_soll_ACC) ||
                   (CurrentLaneFrontVel < 0.2)) {
          IsStopSpeedPlan = 1;
        }

        if (IsStopSpeedPlan == 1) {
          b = pos_s - GlobVars->TrajPlanTurnAround.posCircle[0];
          if ((b > 0.0) && (pos_l < GlobVars->TrajPlanTurnAround.posCircle[1]))
          {
            b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) /
                      (pos_s - GlobVars->TrajPlanTurnAround.posCircle[0])) +
                 1.5707963267948966) * Parameters.turningRadius;
          } else if ((pos_l <= GlobVars->TrajPlanTurnAround.posCircle2[1]) &&
                     (pos_l >= GlobVars->TrajPlanTurnAround.posCircle[1])) {
            b = (pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) +
              Parameters.turningRadius * 3.1415926535897931 / 2.0;
          } else if (pos_l > GlobVars->TrajPlanTurnAround.posCircle2[1]) {
            pos_l_TargetLane = atan((pos_l -
              GlobVars->TrajPlanTurnAround.posCircle2[1]) / (pos_s -
              GlobVars->TrajPlanTurnAround.posCircle2[0]));
            if (pos_l_TargetLane < 0.0) {
              b = (((GlobVars->TrajPlanTurnAround.posCircle2[1] -
                     GlobVars->TrajPlanTurnAround.posCircle[1]) +
                    Parameters.turningRadius * 3.1415926535897931) +
                   GlobVars->TrajPlanTurnAround.posCircle2[0]) - pos_s;
            } else {
              b = ((pos_l_TargetLane * Parameters.turningRadius +
                    GlobVars->TrajPlanTurnAround.posCircle2[1]) -
                   GlobVars->TrajPlanTurnAround.posCircle[1]) +
                Parameters.turningRadius * 3.1415926535897931 / 2.0;
            }
          }

          d = 3.1415926535897931 * TurningRadius / 2.0;
          for (loop_ub = 0; loop_ub < 80; loop_ub++) {
            d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
            if (d_cur2pot_tar <= k) {
              pos_l_TargetLane = d_cur2pot_tar * d_cur2pot_tar;
              targetSpeed = (speed + *a_soll_TrajPlanTurnAround * d_cur2pot_tar)
                + 0.5 * jerk * pos_l_TargetLane;
              d_cur2pot_tar = (speed * d_cur2pot_tar + 0.5 *
                               *a_soll_TrajPlanTurnAround * pos_l_TargetLane) +
                0.16666666666666666 * jerk * rt_powd_snf(d_cur2pot_tar, 3.0);
            } else {
              targetSpeed = D_safe2;
              d_cur2pot_tar = stopdistance + (d_cur2pot_tar - k) * D_safe2;
            }

            pos_l_TargetLane = d_cur2pot_tar + b;
            if (pos_l_TargetLane < d) {
              disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
                1.5707963267948966;
            } else {
              a_predict = GlobVars->TrajPlanTurnAround.posCircle2[1];
              timeGap = GlobVars->TrajPlanTurnAround.posCircle[1];
              if ((pos_l_TargetLane <= (d + a_predict) - timeGap) &&
                  (pos_l_TargetLane >= d)) {
                disOppositeCar2circle2 = 0.0;
              } else {
                disOppositeCar2circle2 = (pos_l_TargetLane - (a_predict -
                  timeGap)) / TurningRadius - 1.5707963267948966;
              }
            }

            if (disOppositeCar2circle2 <= -1.5707963267948966) {
              traj_s[loop_ub] = pos_s + d_cur2pot_tar;
              traj_l[loop_ub] = pos_l_CurrentLane;
              traj_psi[loop_ub] = 90.0;
              traj_vs[loop_ub] = targetSpeed;
              traj_vl[loop_ub] = 0.0;
              traj_omega[loop_ub] = 0.0;
            } else if (disOppositeCar2circle2 < 0.0) {
              /* PI() */
              a_predict = cos(disOppositeCar2circle2);
              traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] +
                a_predict * TurningRadius;
              timeGap = sin(disOppositeCar2circle2);
              traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] +
                timeGap * TurningRadius;
              traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
                3.1415926535897931;
              traj_vs[loop_ub] = -targetSpeed * timeGap;
              traj_vl[loop_ub] = targetSpeed * a_predict;
              traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
                3.1415926535897931;
            } else if (disOppositeCar2circle2 == 0.0) {
              /* PI() */
              traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] +
                TurningRadius;
              traj_l[loop_ub] = (GlobVars->TrajPlanTurnAround.posCircle[1] +
                                 pos_l_TargetLane) - d;
              traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
                3.1415926535897931;
              traj_vs[loop_ub] = 0.0;
              traj_vl[loop_ub] = targetSpeed;
              traj_omega[loop_ub] = targetSpeed;
            } else if (disOppositeCar2circle2 <= 1.5707963267948966) {
              /* PI() */
              traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] + cos
                (disOppositeCar2circle2) * TurningRadius;
              pos_l_TargetLane = sin(disOppositeCar2circle2);
              traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] +
                pos_l_TargetLane * TurningRadius;
              traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
                3.1415926535897931;
              traj_vs[loop_ub] = -targetSpeed * pos_l_TargetLane;
              traj_vl[loop_ub] = targetSpeed * cos(disOppositeCar2circle2);
              traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
                3.1415926535897931;
            } else {
              a_predict = GlobVars->TrajPlanTurnAround.posCircle2[1];
              traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] -
                (((pos_l_TargetLane - TurningRadius * 3.1415926535897931) -
                  a_predict) + GlobVars->TrajPlanTurnAround.posCircle[1]);
              traj_l[loop_ub] = a_predict + TurningRadius;
              traj_psi[loop_ub] = -90.0;
              traj_vs[loop_ub] = -targetSpeed;
              traj_vl[loop_ub] = 0.0;
              traj_omega[loop_ub] = 0.0;
            }
          }
        }
      }
    }

    if (IsStopSpeedPlan == 0) {
      JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                a_soll_TrajPlanTurnAround,
                CalibrationVars->UrbanPlanner.jerkLimit);

      /* ------------------------------------------------------------------------------------------------------------ */
      if (*a_soll_TrajPlanTurnAround < 0.0) {
        D_safe2 = 0.0;

        /*  tend=(0-speed)/(a_soll+eps); */
      } else if (*a_soll_TrajPlanTurnAround > 0.0) {
        D_safe2 = v_max;

        /*  tend=(0-speed)/(a_soll+eps); */
      } else {
        D_safe2 = speed;

        /*  tend=0; */
      }

      k = (D_safe2 - speed) / (*a_soll_TrajPlanTurnAround +
        2.2204460492503131E-16);

      /* ------------------------------------------------------------------------------------------------------------ */
      b = pos_s - GlobVars->TrajPlanTurnAround.posCircle[0];
      if ((b > 0.0) && (pos_l < GlobVars->TrajPlanTurnAround.posCircle[1])) {
        b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) / b) +
             1.5707963267948966) * Parameters.turningRadius;
      } else if ((pos_l <= GlobVars->TrajPlanTurnAround.posCircle2[1]) && (pos_l
                  >= GlobVars->TrajPlanTurnAround.posCircle[1])) {
        b = (pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) +
          Parameters.turningRadius * 3.1415926535897931 / 2.0;
      } else if (pos_l > GlobVars->TrajPlanTurnAround.posCircle2[1]) {
        pos_l_TargetLane = atan((pos_l - GlobVars->
          TrajPlanTurnAround.posCircle2[1]) / (pos_s -
          GlobVars->TrajPlanTurnAround.posCircle2[0]));
        if (pos_l_TargetLane < 0.0) {
          b = (((GlobVars->TrajPlanTurnAround.posCircle2[1] -
                 GlobVars->TrajPlanTurnAround.posCircle[1]) +
                Parameters.turningRadius * 3.1415926535897931) +
               GlobVars->TrajPlanTurnAround.posCircle2[0]) - pos_s;
        } else {
          b = ((pos_l_TargetLane * Parameters.turningRadius +
                GlobVars->TrajPlanTurnAround.posCircle2[1]) -
               GlobVars->TrajPlanTurnAround.posCircle[1]) +
            Parameters.turningRadius * 3.1415926535897931 / 2.0;
        }
      }

      d = 3.1415926535897931 * TurningRadius / 2.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
        if (d_cur2pot_tar <= k) {
          targetSpeed = speed + *a_soll_TrajPlanTurnAround * d_cur2pot_tar;
          d_cur2pot_tar = speed * d_cur2pot_tar + 0.5 *
            *a_soll_TrajPlanTurnAround * (d_cur2pot_tar * d_cur2pot_tar);
        } else {
          targetSpeed = D_safe2;
          d_cur2pot_tar = (D_safe2 * D_safe2 - speed * speed) / (2.0 *
            *a_soll_TrajPlanTurnAround + 2.2204460492503131E-16) +
            (d_cur2pot_tar - k) * D_safe2;
        }

        pos_l_TargetLane = d_cur2pot_tar + b;
        if (pos_l_TargetLane < d) {
          disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
            1.5707963267948966;
        } else {
          a_predict = GlobVars->TrajPlanTurnAround.posCircle2[1];
          timeGap = GlobVars->TrajPlanTurnAround.posCircle[1];
          if ((pos_l_TargetLane <= (d + a_predict) - timeGap) &&
              (pos_l_TargetLane >= d)) {
            disOppositeCar2circle2 = 0.0;
          } else {
            disOppositeCar2circle2 = (pos_l_TargetLane - (a_predict - timeGap)) /
              TurningRadius - 1.5707963267948966;
          }
        }

        if (disOppositeCar2circle2 <= -1.5707963267948966) {
          traj_s[loop_ub] = pos_s + d_cur2pot_tar;
          traj_l[loop_ub] = pos_l_CurrentLane;
          traj_psi[loop_ub] = 90.0;
          traj_vs[loop_ub] = targetSpeed;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        } else if (disOppositeCar2circle2 < 0.0) {
          /* PI() */
          /*          targetAngle=targetAngle-pi/2; */
          a_predict = cos(disOppositeCar2circle2);
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] +
            a_predict * TurningRadius;
          timeGap = sin(disOppositeCar2circle2);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] + timeGap *
            TurningRadius;
          traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
            3.1415926535897931;
          traj_vs[loop_ub] = -targetSpeed * timeGap;
          traj_vl[loop_ub] = targetSpeed * a_predict;
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else if (disOppositeCar2circle2 == 0.0) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] +
            TurningRadius;
          traj_l[loop_ub] = (GlobVars->TrajPlanTurnAround.posCircle[1] +
                             pos_l_TargetLane) - d;
          traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
            3.1415926535897931;
          traj_vs[loop_ub] = 0.0;
          traj_vl[loop_ub] = targetSpeed;
          traj_omega[loop_ub] = targetSpeed;
        } else if (disOppositeCar2circle2 <= 1.5707963267948966) {
          /* PI() */
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] + cos
            (disOppositeCar2circle2) * TurningRadius;
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] + sin
            (disOppositeCar2circle2) * TurningRadius;
          traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
            3.1415926535897931;
          traj_vs[loop_ub] = -targetSpeed * sin(disOppositeCar2circle2);
          traj_vl[loop_ub] = targetSpeed * cos(disOppositeCar2circle2);
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else {
          a_predict = GlobVars->TrajPlanTurnAround.posCircle2[1];
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] -
            (((pos_l_TargetLane - TurningRadius * 3.1415926535897931) -
              a_predict) + GlobVars->TrajPlanTurnAround.posCircle[1]);
          traj_l[loop_ub] = a_predict + TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -targetSpeed;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    }

    /* ------------------------------------------------------------------------------------------------------------------------------------------ */
    /*      if wait_turnAround==1||wait_TrafficLight==1%停车点计算 */
    /*          stopdistance=min(PosCircle1(1)-pos_s-d_gap2stop,stopdistance); */
    /*      end */
    /*      IsStopSpeedPlan=0; */
    /*      if stopdistance<200&&speed.^2/8<=stopdistance && (-((4/9)*speed.^2/(2/3*stopdistance))<=a_soll_ACC || CurrentLaneFrontVel<0.2)&&AEBactive==0 */
    /*          a_soll_TrajPlanTurnAround=max(a_soll_TrajPlanTurnAround,-((4/9)*speed.^2/(2/3*stopdistance))); */
    /*          [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround); */
    /*          if a_soll_TrajPlanTurnAround>=-((4/9)*speed.^2/(2/3*stopdistance)) */
    /*              tend=(3*sqrt(max(0,(4/9)*speed.^2+(2/3)*a_soll_TrajPlanTurnAround*stopdistance))-2*speed)/(a_soll_TrajPlanTurnAround+eps); */
    /*              jerk=-2*(speed+a_soll_TrajPlanTurnAround*tend)/(tend.^2); */
    /*              for count_1=1:1:80 */
    /*                  t_count_1=0.05*count_1; */
    /*                  if t_count_1<=tend */
    /*                      traj_vs(count_1)= speed+a_soll_TrajPlanTurnAround*t_count_1+0.5*jerk*t_count_1.^2; */
    /*                      traj_s(count_1)=pos_s+speed*t_count_1+0.5*a_soll_TrajPlanTurnAround*t_count_1.^2+(1/6)*jerk*t_count_1.^3; */
    /*                  else */
    /*                      traj_vs(count_1)=0; */
    /*                      traj_s(count_1)=pos_s+stopdistance; */
    /*                  end */
    /*                  traj_vl(count_1)=0; */
    /*                  traj_omega(count_1)=0; */
    /*                  traj_l(count_1)=pos_l_CurrentLane; */
    /*                  traj_psi(count_1)=90; */
    /*                  IsStopSpeedPlan=1; */
    /*              end */
    /*          end */
    /*      end */
    /*      if IsStopSpeedPlan==0 */
    /*          [a_soll_TrajPlanTurnAround]=JerkLimit(GlobVars.Decider.a_sollpre2traj,SampleTime,a_soll_TrajPlanTurnAround); */
    /*          if pos_s-PosCircle1(1)>0&&pos_l<PosCircle1(2) */
    /*              passedAngle=atan((pos_l-PosCircle1(2))/(pos_s-PosCircle1(1))); */
    /*              passedPerimeter=(pi/2+passedAngle)*TurningRadius; */
    /*          elseif pos_l<=PosCircle2(2)&&pos_l>=PosCircle1(2) */
    /*              passedPerimeter=pos_l-PosCircle1(2)+TurningRadius*pi/2; */
    /*          elseif pos_l>PosCircle2(2) */
    /*              passedAngle=atan((pos_l-PosCircle2(2))/(pos_s-PosCircle2(1))); */
    /*              if passedAngle<0 */
    /*                  passedPerimeter=PosCircle2(2)-PosCircle1(2)+TurningRadius*pi+PosCircle2(1)-pos_s; */
    /*              else */
    /*                  passedPerimeter=passedAngle*TurningRadius+PosCircle2(2)-PosCircle1(2)+TurningRadius*pi/2; */
    /*              end */
    /*          else */
    /*              passedPerimeter=pos_s-PosCircle1(1); */
    /*          end */
    /*          for count_1=1:1:80 */
    /*              t_count_1=0.05*count_1; */
    /*              targetSpeed=max([0 speed+a_soll_TrajPlanTurnAround*t_count_1]); */
    /*              if targetSpeed==0 */
    /*                  adavancedPerimeter=(0-speed.^2)/(2*a_soll_TrajPlanTurnAround+eps); */
    /*              else */
    /*                  adavancedPerimeter=(targetSpeed+speed)*t_count_1/2; */
    /*              end */
    /*              targetPerimeter=adavancedPerimeter+passedPerimeter; */
    /*              if targetPerimeter<pi*TurningRadius/2 */
    /*                  targetAngle=targetPerimeter/TurningRadius-pi/2; */
    /*              elseif targetPerimeter<=pi*TurningRadius/2+PosCircle2(2)-PosCircle1(2)&&targetPerimeter>=pi*TurningRadius/2 */
    /*                  targetAngle=0; */
    /*              else */
    /*                  targetAngle=(targetPerimeter-(PosCircle2(2)-PosCircle1(2)))/TurningRadius-pi/2; */
    /*              end */
    /*              if targetAngle<=-pi/2 */
    /*                  traj_s(count_1)=pos_s+adavancedPerimeter; */
    /*                  traj_l(count_1)=pos_l_CurrentLane; */
    /*                  traj_psi(count_1)=90; */
    /*                  traj_vs(count_1)=targetSpeed; */
    /*                  traj_vl(count_1)=0; */
    /*                  traj_omega(count_1)=0; */
    /*              elseif targetAngle<0%PI() */
    /*                  %         targetAngle=targetAngle-pi/2; */
    /*                  traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius; */
    /*                  traj_l(count_1)=PosCircle1(2)+sin(targetAngle)*TurningRadius; */
    /*                  traj_psi(count_1)=-targetAngle*180/pi; */
    /*                  traj_vs(count_1)=-targetSpeed*sin(targetAngle); */
    /*                  traj_vl(count_1)=targetSpeed*cos(targetAngle); */
    /*                  traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi; */
    /*              elseif targetAngle==0%PI() */
    /*                  traj_s(count_1)=PosCircle1(1)+TurningRadius; */
    /*                  traj_l(count_1)=PosCircle1(2)+targetPerimeter-pi*TurningRadius/2; */
    /*                  traj_psi(count_1)=-targetAngle*180/pi; */
    /*                  traj_vs(count_1)=0; */
    /*                  traj_vl(count_1)=targetSpeed; */
    /*                  traj_omega(count_1)=targetSpeed; */
    /*              elseif targetAngle<=pi/2%PI() */
    /*                  traj_s(count_1)=PosCircle1(1)+cos(targetAngle)*TurningRadius; */
    /*                  traj_l(count_1)=PosCircle2(2)+sin(targetAngle)*TurningRadius; */
    /*                  traj_psi(count_1)=-targetAngle*180/pi; */
    /*                  traj_vs(count_1)=-targetSpeed*sin(targetAngle); */
    /*                  traj_vl(count_1)=targetSpeed*cos(targetAngle); */
    /*                  traj_omega(count_1)=-targetSpeed/TurningRadius*180/pi; */
    /*              else */
    /*                  traj_s(count_1)=PosCircle1(1)-(targetPerimeter-TurningRadius*pi-PosCircle2(2)+PosCircle1(2)); */
    /*                  traj_l(count_1)=PosCircle2(2)+TurningRadius; */
    /*                  traj_psi(count_1)=-90; */
    /*                  traj_vs(count_1)=-targetSpeed; */
    /*                  traj_vl(count_1)=0; */
    /*                  traj_omega(count_1)=0; */
    /*              end */
    /*          end */
    /*      end */
    /* ------------------------------------------------------------------------------------------------------------------------------------------ */
    /* 一次掉头结束判断 */
    if ((traj_psi[0] == -90.0) && (pos_s <
         GlobVars->TrajPlanTurnAround.posCircle[0] - Parameters.turningRadius /
         2.0)) {
      *TurnAroundActive = 0;
    }

    *a_sollTurnAround2Decider = *a_soll_TrajPlanTurnAround;
  }

  /*  二次顺车掉头轨迹生成 */
  b = 0.0;

  /* 20220324 */
  if (TypeOfTurnAround == 2) {
    /* --------------------------------------------------------------------------------------------------------------------------------------------- */
    if ((TurnAroundState == 1) || (TurnAroundState == 0)) {
      if (TurnAroundState == 0) {
        if ((wait_turnAround == 1) ||
            (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1)) {
          /* 停车点计算 */
          stopdistance = fmin((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s)
                              - CalibrationVars->TrajPlanTurnAround.d_gap2stop,
                              stopdistance);
          D_safe2 = 0.0;
        } else if (dec_trunAround == 1) {
          /* 减速 */
          stopdistance = fmin((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s)
                              - CalibrationVars->TrajPlanTurnAround.d_gap2stop,
                              stopdistance);
          D_safe2 = CalibrationVars->TrajPlanTurnAround.v_max_turnAround;
        } else {
          stopdistance = fmin((GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s)
                              - CalibrationVars->TrajPlanTurnAround.d_gap2stop,
                              stopdistance);
          D_safe2 = v_max;
        }
      } else {
        /*  TurnAroundState==1%停车点计算 */
        if (GlobVars->TrajPlanTurnAround.pos_start[0] - pos_s > 0.0) {
          d = ((rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid1[1] -
                              GlobVars->TrajPlanTurnAround.posCircle[1],
                              GlobVars->TrajPlanTurnAround.pos_mid1[0] -
                              GlobVars->TrajPlanTurnAround.posCircle[0]) -
                rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_start[1] -
                              GlobVars->TrajPlanTurnAround.posCircle[1],
                              GlobVars->TrajPlanTurnAround.pos_start[0] -
                              GlobVars->TrajPlanTurnAround.posCircle[0])) *
               Parameters.turningRadius + GlobVars->
               TrajPlanTurnAround.pos_start[0]) - pos_s;
        } else {
          d = (rt_atan2d_snf(GlobVars->TrajPlanTurnAround.pos_mid1[1] -
                             GlobVars->TrajPlanTurnAround.posCircle[1],
                             GlobVars->TrajPlanTurnAround.pos_mid1[0] -
                             GlobVars->TrajPlanTurnAround.posCircle[0]) -
               rt_atan2d_snf(pos_l - GlobVars->TrajPlanTurnAround.posCircle[1],
                             pos_s - GlobVars->TrajPlanTurnAround.posCircle[0]))
            * Parameters.turningRadius;
        }

        stopdistance = fmin(d, stopdistance);
        D_safe2 = 0.0;
      }

      IsStopSpeedPlan = 0;
      if ((D_safe2 < speed) && (stopdistance < 200.0) && ((D_safe2 * D_safe2 -
            speed * speed) / -8.0 <= stopdistance) && (*AEBactive == 0)) {
        d_cur2pot_tar = 0.66666666666666663 * speed + 0.33333333333333331 *
          D_safe2;
        pos_l_TargetLane = d_cur2pot_tar * d_cur2pot_tar;
        d_cur2pot_tar = -pos_l_TargetLane / (0.66666666666666663 * stopdistance);
        *a_soll_TrajPlanTurnAround = fmax(*a_soll_TrajPlanTurnAround,
          d_cur2pot_tar);
        JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                  a_soll_TrajPlanTurnAround,
                  CalibrationVars->UrbanPlanner.jerkLimit);
        if (*a_soll_TrajPlanTurnAround >= d_cur2pot_tar) {
          k = ((sqrt(fmax(0.0, pos_l_TargetLane + 0.66666666666666663 *
                          *a_soll_TrajPlanTurnAround * stopdistance)) -
                0.66666666666666663 * speed) - 0.33333333333333331 * D_safe2) /
            (0.33333333333333331 * *a_soll_TrajPlanTurnAround +
             2.2204460492503131E-16);
          pos_l_TargetLane = D_safe2 - speed;
          jerk = (pos_l_TargetLane - *a_soll_TrajPlanTurnAround * k) / (0.5 * (k
            * k));
          d = -*a_soll_TrajPlanTurnAround / jerk;
          if ((d < k) && (d > 0.0) && (jerk > 0.0)) {
            d_cur2pot_tar = speed + 2.0 * D_safe2;
            k = 3.0 * stopdistance / d_cur2pot_tar;
            pos_l_TargetLane *= 2.0;
            jerk = -(pos_l_TargetLane * (d_cur2pot_tar * d_cur2pot_tar)) / (9.0 *
              (stopdistance * stopdistance));
            d_cur2pot_tar = pos_l_TargetLane * d_cur2pot_tar / (3.0 *
              stopdistance);
            d = d_cur2pot_tar;
            JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime, &d,
                      CalibrationVars->UrbanPlanner.jerkLimit);
            if (d == d_cur2pot_tar) {
              *a_soll_TrajPlanTurnAround = d_cur2pot_tar;
              if ((d_cur2pot_tar <= a_soll_ACC) || (CurrentLaneFrontVel < 0.2))
              {
                IsStopSpeedPlan = 1;
              }
            }
          } else if ((*a_soll_TrajPlanTurnAround <= a_soll_ACC) ||
                     (CurrentLaneFrontVel < 0.2)) {
            IsStopSpeedPlan = 1;
          }

          if (IsStopSpeedPlan == 1) {
            b = pos_s - GlobVars->TrajPlanTurnAround.posCircle[0];
            if (b > 0.0) {
              b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) /
                        (pos_s - GlobVars->TrajPlanTurnAround.posCircle[0])) +
                   1.5707963267948966) * Parameters.turningRadius;
            } else if ((b < 0.0) && (pos_l >
                        GlobVars->TrajPlanTurnAround.posCircle[1])) {
              b = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
                Parameters.turningRadius * 3.1415926535897931;
            }

            for (loop_ub = 0; loop_ub < 80; loop_ub++) {
              d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
              if (d_cur2pot_tar <= k) {
                pos_l_TargetLane = d_cur2pot_tar * d_cur2pot_tar;
                targetSpeed = (speed + *a_soll_TrajPlanTurnAround *
                               d_cur2pot_tar) + 0.5 * jerk * pos_l_TargetLane;
                d_cur2pot_tar = (speed * d_cur2pot_tar + 0.5 *
                                 *a_soll_TrajPlanTurnAround * pos_l_TargetLane)
                  + 0.16666666666666666 * jerk * rt_powd_snf(d_cur2pot_tar, 3.0);
              } else {
                targetSpeed = D_safe2;
                d_cur2pot_tar = stopdistance + (d_cur2pot_tar - k) * D_safe2;
              }

              pos_l_TargetLane = d_cur2pot_tar + b;
              disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
                1.5707963267948966;
              if (disOppositeCar2circle2 <= -1.5707963267948966) {
                traj_s[loop_ub] = pos_s + d_cur2pot_tar;
                traj_l[loop_ub] = pos_l;
                traj_psi[loop_ub] = 90.0;
                traj_vs[loop_ub] = targetSpeed;
                traj_vl[loop_ub] = 0.0;
                traj_omega[loop_ub] = 0.0;
              } else if (disOppositeCar2circle2 < 1.5707963267948966) {
                d_cur2pot_tar = cos(disOppositeCar2circle2);
                traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] +
                  d_cur2pot_tar * TurningRadius;
                pos_l_TargetLane = sin(disOppositeCar2circle2);
                traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] +
                  pos_l_TargetLane * TurningRadius;
                traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
                  3.1415926535897931;
                traj_vs[loop_ub] = -targetSpeed * pos_l_TargetLane;
                traj_vl[loop_ub] = targetSpeed * d_cur2pot_tar;
                traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
                  3.1415926535897931;
              } else {
                traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] -
                  (pos_l_TargetLane - TurningRadius * 3.1415926535897931);
                traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] +
                  TurningRadius;
                traj_psi[loop_ub] = -90.0;
                traj_vs[loop_ub] = -targetSpeed;
                traj_vl[loop_ub] = 0.0;
                traj_omega[loop_ub] = 0.0;
              }
            }
          }
        }
      }

      if (IsStopSpeedPlan == 0) {
        JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                  a_soll_TrajPlanTurnAround,
                  CalibrationVars->UrbanPlanner.jerkLimit);

        /* ------------------------------------------------------------------------------------------------------------ */
        if (*a_soll_TrajPlanTurnAround < 0.0) {
          D_safe2 = 0.0;
        } else if (*a_soll_TrajPlanTurnAround > 0.0) {
          D_safe2 = v_max;
        } else {
          D_safe2 = speed;
        }

        k = (D_safe2 - speed) / (*a_soll_TrajPlanTurnAround +
          2.2204460492503131E-16);

        /* ------------------------------------------------------------------------------------------------------------ */
        b = pos_s - GlobVars->TrajPlanTurnAround.posCircle[0];
        if (b > 0.0) {
          b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle[1]) / (pos_s
                 - GlobVars->TrajPlanTurnAround.posCircle[0])) +
               1.5707963267948966) * Parameters.turningRadius;
        } else if ((b < 0.0) && (pos_l > GlobVars->TrajPlanTurnAround.posCircle
                    [1])) {
          b = (GlobVars->TrajPlanTurnAround.posCircle[0] - pos_s) +
            Parameters.turningRadius * 3.1415926535897931;
        }

        for (loop_ub = 0; loop_ub < 80; loop_ub++) {
          d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
          if (d_cur2pot_tar <= k) {
            targetSpeed = speed + *a_soll_TrajPlanTurnAround * d_cur2pot_tar;
            d_cur2pot_tar = speed * d_cur2pot_tar + 0.5 *
              *a_soll_TrajPlanTurnAround * (d_cur2pot_tar * d_cur2pot_tar);
          } else {
            targetSpeed = D_safe2;
            d_cur2pot_tar = (D_safe2 * D_safe2 - speed * speed) / (2.0 *
              *a_soll_TrajPlanTurnAround + 2.2204460492503131E-16) +
              (d_cur2pot_tar - k) * D_safe2;
          }

          pos_l_TargetLane = d_cur2pot_tar + b;
          disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
            1.5707963267948966;
          if (disOppositeCar2circle2 <= -1.5707963267948966) {
            traj_s[loop_ub] = pos_s + d_cur2pot_tar;
            traj_l[loop_ub] = pos_l;
            traj_psi[loop_ub] = 90.0;
            traj_vs[loop_ub] = targetSpeed;
            traj_vl[loop_ub] = 0.0;
            traj_omega[loop_ub] = 0.0;
          } else if (disOppositeCar2circle2 < 1.5707963267948966) {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] + cos
              (disOppositeCar2circle2) * TurningRadius;
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] + sin
              (disOppositeCar2circle2) * TurningRadius;
            traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
              3.1415926535897931;
            traj_vs[loop_ub] = -targetSpeed * sin(disOppositeCar2circle2);
            traj_vl[loop_ub] = targetSpeed * cos(disOppositeCar2circle2);
            traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
              3.1415926535897931;
          } else {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[0] -
              (pos_l_TargetLane - TurningRadius * 3.1415926535897931);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle[1] +
              TurningRadius;
            traj_psi[loop_ub] = -90.0;
            traj_vs[loop_ub] = -targetSpeed;
            traj_vl[loop_ub] = 0.0;
            traj_omega[loop_ub] = 0.0;
          }
        }
      }
    } else if (TurnAroundState == 2) {
      boolean_T b_guard1 = false;

      /* 停车点计算   */
      stopdistance = fmin(dis2pos_mid2, stopdistance);
      b_guard1 = false;
      if (stopdistance < 200.0) {
        d = speed * speed;
        if (d / 8.0 <= stopdistance) {
          d *= 0.44444444444444442;
          a_predict = -(d / (0.66666666666666663 * stopdistance));
          if (((a_predict <= a_soll_ACC) || (CurrentLaneFrontVel < 0.2)) &&
              (*AEBactive == 0)) {
            *a_soll_TrajPlanTurnAround = fmax(*a_soll_TrajPlanTurnAround,
              a_predict);
            JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                      a_soll_TrajPlanTurnAround,
                      CalibrationVars->UrbanPlanner.jerkLimit);
            if (*a_soll_TrajPlanTurnAround >= -(d / (0.66666666666666663 *
                  stopdistance))) {
              k = (3.0 * sqrt(fmax(0.0, d + 0.66666666666666663 *
                                   *a_soll_TrajPlanTurnAround * stopdistance)) -
                   2.0 * speed) / (*a_soll_TrajPlanTurnAround +
                                   2.2204460492503131E-16);
              jerk = -2.0 * (speed + *a_soll_TrajPlanTurnAround * k) / (k * k);
              if (pos_s - GlobVars->TrajPlanTurnAround.posCircle2[0] < 0.0) {
                b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle2[1]) /
                          (pos_s - GlobVars->TrajPlanTurnAround.posCircle2[0]))
                     + 1.5707963267948966) * Parameters.turningRadius;
              } else {
                b = (pos_s - GlobVars->TrajPlanTurnAround.posCircle2[0]) +
                  Parameters.turningRadius * 3.1415926535897931;
              }

              for (loop_ub = 0; loop_ub < 80; loop_ub++) {
                d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
                if (d_cur2pot_tar <= k) {
                  pos_l_TargetLane = d_cur2pot_tar * d_cur2pot_tar;
                  targetSpeed = (speed + *a_soll_TrajPlanTurnAround *
                                 d_cur2pot_tar) + 0.5 * jerk * pos_l_TargetLane;
                  d_cur2pot_tar = (speed * d_cur2pot_tar + 0.5 *
                                   *a_soll_TrajPlanTurnAround * pos_l_TargetLane)
                    + 0.16666666666666666 * jerk * rt_powd_snf(d_cur2pot_tar,
                    3.0);
                } else {
                  targetSpeed = 0.0;
                  d_cur2pot_tar = stopdistance;
                }

                pos_l_TargetLane = d_cur2pot_tar + b;
                disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
                  1.5707963267948966;
                if (disOppositeCar2circle2 < 1.5707963267948966) {
                  traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[0] -
                    cos(disOppositeCar2circle2) * TurningRadius;
                  traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] -
                    sin(disOppositeCar2circle2) * TurningRadius;
                  traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
                    3.1415926535897931;
                  traj_vs[loop_ub] = targetSpeed * sin(disOppositeCar2circle2);
                  traj_vl[loop_ub] = -targetSpeed * cos(disOppositeCar2circle2);
                  traj_omega[loop_ub] = targetSpeed / TurningRadius * 180.0 /
                    3.1415926535897931;
                } else {
                  traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[0] +
                    (pos_l_TargetLane - TurningRadius * 3.1415926535897931);
                  traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] -
                    TurningRadius;
                  traj_psi[loop_ub] = -90.0;
                  traj_vs[loop_ub] = targetSpeed;
                  traj_vl[loop_ub] = 0.0;
                  traj_omega[loop_ub] = 0.0;
                }
              }
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
        } else {
          b_guard1 = true;
        }
      } else {
        b_guard1 = true;
      }

      if (b_guard1) {
        JerkLimit(GlobVars->Decider.a_sollpre2traj, SampleTime,
                  a_soll_TrajPlanTurnAround,
                  CalibrationVars->UrbanPlanner.jerkLimit);
        d = pos_s - GlobVars->TrajPlanTurnAround.posCircle2[0];
        if (d < 0.0) {
          b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle2[1]) /
                    (pos_s - GlobVars->TrajPlanTurnAround.posCircle2[0])) +
               1.5707963267948966) * Parameters.turningRadius;
        } else {
          b = d + Parameters.turningRadius * 3.1415926535897931;
        }

        b_speed[0] = 0.0;
        for (loop_ub = 0; loop_ub < 80; loop_ub++) {
          d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
          b_speed[1] = speed + *a_soll_TrajPlanTurnAround * d_cur2pot_tar;
          targetSpeed = maximum(b_speed);
          if (targetSpeed == 0.0) {
            d_cur2pot_tar = (0.0 - speed * speed) / (2.0 *
              *a_soll_TrajPlanTurnAround + 2.2204460492503131E-16);
          } else {
            d_cur2pot_tar = (targetSpeed + speed) * d_cur2pot_tar / 2.0;
          }

          pos_l_TargetLane = d_cur2pot_tar + b;
          disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
            1.5707963267948966;
          if (disOppositeCar2circle2 < 1.5707963267948966) {
            d_cur2pot_tar = cos(disOppositeCar2circle2);
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[0] -
              d_cur2pot_tar * TurningRadius;
            pos_l_TargetLane = sin(disOppositeCar2circle2);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] -
              pos_l_TargetLane * TurningRadius;
            traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
              3.1415926535897931;
            traj_vs[loop_ub] = targetSpeed * pos_l_TargetLane;
            traj_vl[loop_ub] = -targetSpeed * d_cur2pot_tar;
            traj_omega[loop_ub] = targetSpeed / TurningRadius * 180.0 /
              3.1415926535897931;
          } else {
            traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[0] +
              (pos_l_TargetLane - TurningRadius * 3.1415926535897931);
            traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle2[1] -
              TurningRadius;
            traj_psi[loop_ub] = -90.0;
            traj_vs[loop_ub] = targetSpeed;
            traj_vl[loop_ub] = 0.0;
            traj_omega[loop_ub] = 0.0;
          }
        }
      }
    } else if (TurnAroundState == 3) {
      d = pos_s - GlobVars->TrajPlanTurnAround.posCircle3[0];
      if (d > 0.0) {
        b = (atan((pos_l - GlobVars->TrajPlanTurnAround.posCircle3[1]) / d) +
             1.5707963267948966) * Parameters.turningRadius;
      } else if ((d < 0.0) && (pos_l > GlobVars->TrajPlanTurnAround.posCircle3[1]))
      {
        b = (GlobVars->TrajPlanTurnAround.posCircle3[0] - pos_s) +
          Parameters.turningRadius * 3.1415926535897931;
      }

      b_speed[0] = 0.0;
      for (loop_ub = 0; loop_ub < 80; loop_ub++) {
        d_cur2pot_tar = 0.05 * ((double)loop_ub + 1.0);
        b_speed[1] = speed + *a_soll_TrajPlanTurnAround * d_cur2pot_tar;
        targetSpeed = maximum(b_speed);
        if (targetSpeed == 0.0) {
          d_cur2pot_tar = (0.0 - speed * speed) / (2.0 *
            *a_soll_TrajPlanTurnAround + 2.2204460492503131E-16);
        } else {
          d_cur2pot_tar = (targetSpeed + speed) * d_cur2pot_tar / 2.0;
        }

        pos_l_TargetLane = d_cur2pot_tar + b;
        disOppositeCar2circle2 = pos_l_TargetLane / TurningRadius -
          1.5707963267948966;
        if (disOppositeCar2circle2 < 1.5707963267948966) {
          d = cos(disOppositeCar2circle2);
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle3[0] + d *
            TurningRadius;
          a_predict = sin(disOppositeCar2circle2);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle3[1] +
            a_predict * TurningRadius;
          traj_psi[loop_ub] = -disOppositeCar2circle2 * 180.0 /
            3.1415926535897931;
          traj_vs[loop_ub] = -targetSpeed * a_predict;
          traj_vl[loop_ub] = targetSpeed * d;
          traj_omega[loop_ub] = -targetSpeed / TurningRadius * 180.0 /
            3.1415926535897931;
        } else {
          traj_s[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle3[0] -
            (pos_l_TargetLane - TurningRadius * 3.1415926535897931);
          traj_l[loop_ub] = GlobVars->TrajPlanTurnAround.posCircle3[1] +
            TurningRadius;
          traj_psi[loop_ub] = -90.0;
          traj_vs[loop_ub] = -targetSpeed;
          traj_vl[loop_ub] = 0.0;
          traj_omega[loop_ub] = 0.0;
        }
      }
    }

    /* --------------------------------------------------------------------------------------------------------------------------------------------- */
    *a_sollTurnAround2Decider = *a_soll_TrajPlanTurnAround;
  }

  GlobVars->Decider.a_sollpre2traj = *a_soll_TrajPlanTurnAround;
  GlobVars->TrajPlanTurnAround.dec_trunAround = dec_trunAround;
  GlobVars->TrajPlanTurnAround.wait_turnAround = wait_turnAround;
  GlobVars->TrajPlanTurnAround.typeOfTurnAround = TypeOfTurnAround;
  GlobVars->TrajPlanTurnAround.turnAroundState = TurnAroundState;
  GlobVars->TrajPlanTurnAround.targetLaneIndexOpposite = TargetLaneIndexOpposite;
}

/*
 * Arguments    : double v_max
 *                double v_soll
 *                double d_ist
 *                double speed
 *                short b_wait
 *                double CalibrationVars_ACC_a_max
 *                double CalibrationVars_ACC_a_min
 *                double c_CalibrationVars_ACC_d_wait2fa
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
                    c_CalibrationVars_ACC_d_wait2fa, double
                    CalibrationVars_ACC_tau_v_com, double
                    CalibrationVars_ACC_tau_v, double CalibrationVars_ACC_tau_d,
                    double CalibrationVars_ACC_tau_v_bre, double
                    CalibrationVars_ACC_tau_v_emg, double
                    CalibrationVars_ACC_tau_d_emg, double
                    CalibrationVars_ACC_t_acc, double CalibrationVars_ACC_d_wait)
{
  double accel;
  double d_soll;

  /* 2.5; */
  /* -4; */
  /* 13; */
  /* 4; */
  /* 2; */
  /* 5; */
  /* 1; */
  /* 0.5; */
  /* 2; */
  /* 4 */
  accel = 100.0;
  if (d_ist < 100.0) {
    double b_speed[3];

    /*      if wait==-1 % 停车距离远一些，避免停在故障车后面停得过近，无法换道 */
    /*          d_soll=max([speed*t_acc 17 (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*      else */
    /*  %         d_soll=max([speed*t_acc 9 (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*          d_soll=max([speed*t_acc d_wait (v_soll.^2-speed.^2)/(2*a_min) ]); */
    /*      end */
    if (b_wait == -1) {
      /*  停车距离远一些，避免停在故障车后面停得过近，无法换道 */
      d_ist = fmax(d_ist - c_CalibrationVars_ACC_d_wait2fa, 0.0);
    }

    b_speed[0] = speed * CalibrationVars_ACC_t_acc;
    b_speed[1] = CalibrationVars_ACC_d_wait;
    b_speed[2] = (v_soll * v_soll - speed * speed) / (2.0 *
      CalibrationVars_ACC_a_min);
    d_soll = b_maximum(b_speed);
    if ((v_soll == 0.0) && (d_ist <= CalibrationVars_ACC_d_wait + 0.15)) {
      if (rtIsNaN(speed)) {
        d_soll = rtNaN;
      } else if (speed < 0.0) {
        d_soll = -1.0;
      } else {
        d_soll = (speed > 0.0);
      }

      accel = -4.0 * d_soll;
    } else {
      if (v_soll == 0.0) {
        v_soll = 0.6;
      }

      if (d_ist < CalibrationVars_ACC_d_wait) {
        accel = ((v_soll - speed) + (d_ist - d_soll) /
                 CalibrationVars_ACC_tau_d_emg) / CalibrationVars_ACC_tau_v_emg;
      } else if ((d_ist < CalibrationVars_ACC_d_wait + 3.0) && (b_wait != 1)) {
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

  d_soll = (v_max - speed) / CalibrationVars_ACC_tau_v;
  if (!(d_soll > -2.5)) {
    d_soll = -2.5;
  }

  if ((accel > d_soll) || rtIsNaN(accel)) {
    accel = d_soll;
  }

  if (accel > CalibrationVars_ACC_a_max) {
    accel = CalibrationVars_ACC_a_max;
  }

  if ((accel < CalibrationVars_ACC_a_min) || (rtIsNaN(accel) && (!rtIsNaN
        (CalibrationVars_ACC_a_min)))) {
    accel = CalibrationVars_ACC_a_min;
  }

  return accel;
}

/*
 * Arguments    : double *x
 * Return Type  : void
 */
static void b_cosd(double *x)
{
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = rtNaN;
  } else {
    double absx;
    signed char n;
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
 * Arguments    : double *x
 * Return Type  : void
 */
static void b_cotd(double *x)
{
  double absx;
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    absx = rtNaN;
  } else {
    signed char n;
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

    absx = tan(*x);
    if ((n == 1) || (n == -1)) {
      double z;
      z = 1.0 / absx;
      absx = -(1.0 / absx);
      if (rtIsInf(absx) && (n == 1)) {
        absx = z;
      }
    }
  }

  *x = 1.0 / absx;
}

/*
 * Arguments    : const emxArray_int16_T *a
 *                const emxArray_real_T *b
 *                emxArray_int16_T *c
 *                emxArray_int32_T *ia
 *                int *ib_size
 * Return Type  : void
 */
static void b_do_vectors(const emxArray_int16_T *a, const emxArray_real_T *b,
  emxArray_int16_T *c, emxArray_int32_T *ia, int *ib_size)
{
  const double *b_data;
  int b_ialast;
  int iafirst;
  int ialast;
  int iblast;
  int na;
  int nc;
  int nia;
  int *ia_data;
  const short *a_data;
  short *c_data;
  b_data = b->data;
  a_data = a->data;
  na = a->size[1];
  iafirst = c->size[0] * c->size[1];
  c->size[0] = 1;
  c->size[1] = a->size[1];
  emxEnsureCapacity_int16_T(c, iafirst);
  c_data = c->data;
  iafirst = ia->size[0];
  ia->size[0] = a->size[1];
  emxEnsureCapacity_int32_T(ia, iafirst);
  ia_data = ia->data;
  *ib_size = 0;
  nc = 0;
  nia = 0;
  iafirst = 0;
  ialast = 0;
  iblast = 1;
  while ((ialast + 1 <= na) && (iblast <= b->size[1])) {
    double bk;
    short ak;
    b_ialast = ialast + 1;
    ak = a_data[ialast];
    while ((b_ialast < a->size[1]) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast - 1;
    bk = b_data[iblast - 1];
    while ((iblast < b->size[1]) && (b_data[iblast] == bk)) {
      iblast++;
    }

    if (ak == bk) {
      ialast = b_ialast;
      iafirst = b_ialast;
      iblast++;
    } else if (rtIsNaN(bk) || (ak < bk)) {
      nc++;
      nia++;
      c_data[nc - 1] = ak;
      ia_data[nia - 1] = iafirst + 1;
      ialast = b_ialast;
      iafirst = b_ialast;
    } else {
      iblast++;
    }
  }

  while (ialast + 1 <= na) {
    b_ialast = ialast + 1;
    while ((b_ialast < a->size[1]) && (a_data[b_ialast] == a_data[ialast])) {
      b_ialast++;
    }

    nc++;
    nia++;
    c_data[nc - 1] = a_data[ialast];
    ia_data[nia - 1] = iafirst + 1;
    ialast = b_ialast;
    iafirst = b_ialast;
  }

  if (a->size[1] > 0) {
    iafirst = ia->size[0];
    if (nia < 1) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia;
    }

    emxEnsureCapacity_int32_T(ia, iafirst);
    iafirst = c->size[0] * c->size[1];
    if (nc < 1) {
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
  double *y_data;
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
  y_data = y->data;
  if (n > 0) {
    y_data[0] = a;
    if (n > 1) {
      y_data[n - 1] = apnd;
      nm1d2 = (n - 1) / 2;
      for (k = 0; k <= nm1d2 - 2; k++) {
        y_data[k + 1] = a + ((double)k + 1.0);
        y_data[(n - k) - 2] = apnd - ((double)k + 1.0);
      }

      if (nm1d2 << 1 == n - 1) {
        y_data[nm1d2] = (a + apnd) / 2.0;
      } else {
        y_data[nm1d2] = a + (double)nm1d2;
        y_data[nm1d2 + 1] = apnd - (double)nm1d2;
      }
    }
  }
}

/*
 * Arguments    : const double c_FunFcn_workspace_fun_x_worksp[3]
 *                const double FunFcn_workspace_S_traj[80]
 *                double FunFcn_workspace_i_traj
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void b_fzero(const double c_FunFcn_workspace_fun_x_worksp[3], const
                    double FunFcn_workspace_S_traj[80], double
                    FunFcn_workspace_i_traj, const double x[2], double *b,
                    double *fval, double *exitflag)
{
  double a;
  double fa;
  double savefa;
  double savefb;
  *exitflag = 1.0;
  a = 0.0;
  *b = x[1];
  fa = TrajPlanLaneChange_anonFcn8(c_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_S_traj, FunFcn_workspace_i_traj, 0.0);
  *fval = TrajPlanLaneChange_anonFcn8(c_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_S_traj, FunFcn_workspace_i_traj, x[1]);
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = 0.0;
    *fval = fa;
  } else if (!(*fval == 0.0)) {
    double c;
    double d;
    double e;
    double fc;
    boolean_T exitg1;
    fc = *fval;
    c = x[1];
    e = 0.0;
    d = 0.0;
    exitg1 = false;
    while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
      double m;
      double toler;
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
          double q;
          double s;
          s = *fval / fa;
          if (a == c) {
            fa = 2.0 * m * s;
            q = 1.0 - s;
          } else {
            double r;
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

          if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e *
                q))) {
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

        *fval = TrajPlanLaneChange_anonFcn8(c_FunFcn_workspace_fun_x_worksp,
          FunFcn_workspace_S_traj, FunFcn_workspace_i_traj, *b);
      }
    }

    if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
      *exitflag = -5.0;
    }
  }
}

/*
 * Arguments    : double d1
 *                double d2
 *                double n
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void b_linspace(double d1, double d2, double n, emxArray_real_T *y)
{
  double *y_data;
  int k;
  if (!(n >= 0.0)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    double delta1;
    int i;
    delta1 = floor(n);
    i = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)delta1;
    emxEnsureCapacity_real_T(y, i);
    y_data = y->data;
    if ((int)delta1 >= 1) {
      y_data[(int)delta1 - 1] = d2;
      if (y->size[1] >= 2) {
        y_data[0] = d1;
        if (y->size[1] >= 3) {
          if (d1 == -d2) {
            delta1 = d2 / ((double)y->size[1] - 1.0);
            i = y->size[1] - 1;
            for (k = 2; k <= i; k++) {
              y_data[k - 1] = (double)(((k << 1) - y->size[1]) - 1) * delta1;
            }

            if ((y->size[1] & 1) == 1) {
              y_data[y->size[1] >> 1] = 0.0;
            }
          } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) >
                       8.9884656743115785E+307) || (fabs(d2) >
                       8.9884656743115785E+307))) {
            double delta2;
            delta1 = d1 / ((double)y->size[1] - 1.0);
            delta2 = d2 / ((double)y->size[1] - 1.0);
            i = y->size[1];
            for (k = 0; k <= i - 3; k++) {
              y_data[k + 1] = (d1 + delta2 * ((double)k + 1.0)) - delta1 *
                ((double)k + 1.0);
            }
          } else {
            delta1 = (d2 - d1) / ((double)y->size[1] - 1.0);
            i = y->size[1];
            for (k = 0; k <= i - 3; k++) {
              y_data[k + 1] = d1 + ((double)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double b_maximum(const double x[3])
{
  double ex;
  int idx;
  int k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
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
      double d;
      d = x[k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : int idx[132]
 *                double x[132]
 *                int offset
 *                int np
 *                int nq
 *                int iwork[132]
 *                double xwork[132]
 * Return Type  : void
 */
static void b_merge(int idx[132], double x[132], int offset, int np, int nq, int
                    iwork[132], double xwork[132])
{
  int j;
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork[j] = idx[iout];
      xwork[j] = x[iout];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[q]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx[iout] = iwork[j - 1];
            x[iout] = xwork[j - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
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
  int k;
  int last;
  last = x_size[1];
  if (x_size[1] <= 2) {
    if (x_size[1] == 1) {
      *ex = x_data[0];
      *idx = 1;
    } else {
      *ex = x_data[x_size[1] - 1];
      if ((x_data[0] > *ex) || (rtIsNaN(x_data[0]) && (!rtIsNaN(*ex)))) {
        *idx = x_size[1];
      } else {
        *ex = x_data[0];
        *idx = 1;
      }
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      *idx = 1;
    } else {
      boolean_T exitg1;
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
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
      int i;
      *ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= last; k++) {
        double d;
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
 * Arguments    : const double A[16]
 *                double B[4]
 * Return Type  : void
 */
static void b_mldivide(const double A[16], double B[4])
{
  double b_A[16];
  double smax;
  int a;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  signed char ipiv[4];
  memcpy(&b_A[0], &A[0], 16U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    int b_tmp;
    int mmj_tmp;
    signed char i1;
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jA = 4 - j;
    a = 0;
    smax = fabs(b_A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }

    if (b_A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = (signed char)(jA + 1);
        smax = b_A[j];
        b_A[j] = b_A[jA];
        b_A[jA] = smax;
        smax = b_A[j + 4];
        b_A[j + 4] = b_A[jA + 4];
        b_A[jA + 4] = smax;
        smax = b_A[j + 8];
        b_A[j + 8] = b_A[jA + 8];
        b_A[jA + 8] = smax;
        smax = b_A[j + 12];
        b_A[j + 12] = b_A[jA + 12];
        b_A[jA + 12] = smax;
      }

      i = (b_tmp - j) + 4;
      for (a = jp1j; a <= i; a++) {
        b_A[a - 1] /= b_A[b_tmp];
      }
    }

    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = b_A[(b_tmp + (jp1j << 2)) + 4];
      if (smax != 0.0) {
        i = jA + 6;
        a = (jA - j) + 8;
        for (k = i; k <= a; k++) {
          b_A[k - 1] += b_A[((b_tmp + k) - jA) - 5] * -smax;
        }
      }

      jA += 4;
    }

    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }

  for (k = 0; k < 4; k++) {
    jA = k << 2;
    if (B[k] != 0.0) {
      i = k + 2;
      for (a = i; a < 5; a++) {
        B[a - 1] -= B[k] * b_A[(a + jA) - 1];
      }
    }
  }

  for (k = 3; k >= 0; k--) {
    jA = k << 2;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      for (a = 0; a < k; a++) {
        B[a] -= B[k] * b_A[a + jA];
      }
    }
  }
}

/*
 * Arguments    : double x
 * Return Type  : double
 */
static double b_mod(double x)
{
  double r;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    boolean_T rEQ0;
    r = fmod(x, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      double q;
      q = fabs(x / 6.2831853071795862);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 6.2831853071795862;
    }
  }

  return r;
}

/*
 * Arguments    : const double x[4]
 *                const double y[4]
 *                double v_breaks[4]
 *                double v_coefs[12]
 * Return Type  : void
 */
static void b_pchip(const double x[4], const double y[4], double v_breaks[4],
                    double v_coefs[12])
{
  double d;
  double d1;
  double del_idx_0;
  double del_idx_1;
  double del_idx_2;
  double dzzdx;
  double h_idx_0;
  double h_idx_1;
  double signd1;
  double slopes_idx_1;
  double slopes_idx_2;
  double w2;
  d = x[1] - x[0];
  h_idx_0 = d;
  del_idx_0 = (y[1] - y[0]) / d;
  d = x[2] - x[1];
  h_idx_1 = d;
  del_idx_1 = (y[2] - y[1]) / d;
  d = x[3] - x[2];
  del_idx_2 = (y[3] - y[2]) / d;
  signd1 = 2.0 * h_idx_1 + h_idx_0;
  w2 = h_idx_1 + 2.0 * h_idx_0;
  slopes_idx_1 = 0.0;
  dzzdx = del_idx_0 * del_idx_1;
  if (!rtIsNaN(dzzdx)) {
    if (dzzdx < 0.0) {
      dzzdx = -1.0;
    } else {
      dzzdx = (dzzdx > 0.0);
    }
  }

  if (dzzdx > 0.0) {
    slopes_idx_1 = (signd1 + w2) / (signd1 / del_idx_0 + w2 / del_idx_1);
  }

  signd1 = 2.0 * d + h_idx_1;
  w2 = d + 2.0 * h_idx_1;
  slopes_idx_2 = 0.0;
  dzzdx = del_idx_1 * del_idx_2;
  if (!rtIsNaN(dzzdx)) {
    if (dzzdx < 0.0) {
      dzzdx = -1.0;
    } else {
      dzzdx = (dzzdx > 0.0);
    }
  }

  if (dzzdx > 0.0) {
    slopes_idx_2 = (signd1 + w2) / (signd1 / del_idx_1 + w2 / del_idx_2);
  }

  w2 = ((2.0 * h_idx_0 + h_idx_1) * del_idx_0 - h_idx_0 * del_idx_1) / (h_idx_0
    + h_idx_1);
  if (rtIsNaN(del_idx_0)) {
    signd1 = rtNaN;
  } else if (del_idx_0 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_0 > 0.0);
  }

  if (rtIsNaN(w2)) {
    d1 = rtNaN;
  } else if (w2 < 0.0) {
    d1 = -1.0;
  } else {
    d1 = (w2 > 0.0);
  }

  if (d1 != signd1) {
    w2 = 0.0;
  } else {
    if (rtIsNaN(del_idx_1)) {
      d1 = rtNaN;
    } else if (del_idx_1 < 0.0) {
      d1 = -1.0;
    } else {
      d1 = (del_idx_1 > 0.0);
    }

    if ((signd1 != d1) && (fabs(w2) > fabs(3.0 * del_idx_0))) {
      w2 = 3.0 * del_idx_0;
    }
  }

  v_breaks[0] = x[0];
  v_breaks[1] = x[1];
  v_breaks[2] = x[2];
  v_breaks[3] = x[3];
  dzzdx = (del_idx_0 - w2) / h_idx_0;
  signd1 = (slopes_idx_1 - del_idx_0) / h_idx_0;
  v_coefs[0] = (signd1 - dzzdx) / h_idx_0;
  v_coefs[3] = 2.0 * dzzdx - signd1;
  v_coefs[6] = w2;
  v_coefs[9] = y[0];
  dzzdx = (del_idx_1 - slopes_idx_1) / h_idx_1;
  signd1 = (slopes_idx_2 - del_idx_1) / h_idx_1;
  v_coefs[1] = (signd1 - dzzdx) / h_idx_1;
  v_coefs[4] = 2.0 * dzzdx - signd1;
  v_coefs[7] = slopes_idx_1;
  v_coefs[10] = y[1];
  dzzdx = (del_idx_2 - slopes_idx_2) / d;
  w2 = ((2.0 * d + h_idx_1) * del_idx_2 - d * del_idx_1) / (d + h_idx_1);
  if (rtIsNaN(del_idx_2)) {
    signd1 = rtNaN;
  } else if (del_idx_2 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_2 > 0.0);
  }

  if (rtIsNaN(w2)) {
    d1 = rtNaN;
  } else if (w2 < 0.0) {
    d1 = -1.0;
  } else {
    d1 = (w2 > 0.0);
  }

  if (d1 != signd1) {
    w2 = 0.0;
  } else {
    if (rtIsNaN(del_idx_1)) {
      d1 = rtNaN;
    } else if (del_idx_1 < 0.0) {
      d1 = -1.0;
    } else {
      d1 = (del_idx_1 > 0.0);
    }

    if ((signd1 != d1) && (fabs(w2) > fabs(3.0 * del_idx_2))) {
      w2 = 3.0 * del_idx_2;
    }
  }

  signd1 = (w2 - del_idx_2) / d;
  v_coefs[2] = (signd1 - dzzdx) / d;
  v_coefs[5] = 2.0 * dzzdx - signd1;
  v_coefs[8] = slopes_idx_2;
  v_coefs[11] = y[2];
}

/*
 * Arguments    : const double pp_breaks[5]
 *                const double pp_coefs[16]
 *                double x
 * Return Type  : double
 */
static double b_ppval(const double pp_breaks[5], const double pp_coefs[16],
                      double x)
{
  double v;
  if (rtIsNaN(x)) {
    v = rtNaN;
  } else {
    int high_i;
    int low_i;
    int low_ip1;
    low_i = 0;
    low_ip1 = 2;
    high_i = 5;
    while (high_i > low_ip1) {
      int mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    double xloc;
    xloc = x - pp_breaks[low_i];
    v = xloc * (xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 4]) +
                pp_coefs[low_i + 8]) + pp_coefs[low_i + 12];
  }

  return v;
}

/*
 * Arguments    : double *x
 * Return Type  : void
 */
static void b_sind(double *x)
{
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = rtNaN;
  } else {
    double absx;
    signed char n;
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
 * Arguments    : double x[132]
 *                int idx[132]
 * Return Type  : void
 */
static void b_sort(double x[132], int idx[132])
{
  double xwork[132];
  double x4[4];
  int iwork[132];
  int i1;
  int i3;
  int ib;
  int k;
  int nNaNs;
  int quartetOffset;
  unsigned char idx4[4];
  x4[0] = 0.0;
  idx4[0] = 0U;
  x4[1] = 0.0;
  idx4[1] = 0U;
  x4[2] = 0.0;
  idx4[2] = 0U;
  x4[3] = 0.0;
  idx4[3] = 0U;
  memset(&idx[0], 0, 132U * sizeof(int));
  memset(&xwork[0], 0, 132U * sizeof(double));
  nNaNs = 0;
  ib = 0;
  for (k = 0; k < 132; k++) {
    if (rtIsNaN(x[k])) {
      idx[131 - nNaNs] = k + 1;
      xwork[131 - nNaNs] = x[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = (unsigned char)(k + 1);
      x4[ib - 1] = x[k];
      if (ib == 4) {
        double d;
        double d1;
        int i4;
        signed char b_i1;
        signed char b_i3;
        signed char i;
        signed char i2;
        quartetOffset = k - nNaNs;
        if (x4[0] <= x4[1]) {
          i1 = 1;
          ib = 2;
        } else {
          i1 = 2;
          ib = 1;
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
          d = x4[ib - 1];
          if (d <= d1) {
            i = (signed char)i1;
            b_i1 = (signed char)ib;
            i2 = (signed char)i3;
            b_i3 = (signed char)i4;
          } else if (d <= x4[i4 - 1]) {
            i = (signed char)i1;
            b_i1 = (signed char)i3;
            i2 = (signed char)ib;
            b_i3 = (signed char)i4;
          } else {
            i = (signed char)i1;
            b_i1 = (signed char)i3;
            i2 = (signed char)i4;
            b_i3 = (signed char)ib;
          }
        } else {
          d1 = x4[i4 - 1];
          if (d <= d1) {
            if (x4[ib - 1] <= d1) {
              i = (signed char)i3;
              b_i1 = (signed char)i1;
              i2 = (signed char)ib;
              b_i3 = (signed char)i4;
            } else {
              i = (signed char)i3;
              b_i1 = (signed char)i1;
              i2 = (signed char)i4;
              b_i3 = (signed char)ib;
            }
          } else {
            i = (signed char)i3;
            b_i1 = (signed char)i4;
            i2 = (signed char)i1;
            b_i3 = (signed char)ib;
          }
        }

        idx[quartetOffset - 3] = idx4[i - 1];
        idx[quartetOffset - 2] = idx4[b_i1 - 1];
        idx[quartetOffset - 1] = idx4[i2 - 1];
        idx[quartetOffset] = idx4[b_i3 - 1];
        x[quartetOffset - 3] = x4[i - 1];
        x[quartetOffset - 2] = x4[b_i1 - 1];
        x[quartetOffset - 1] = x4[i2 - 1];
        x[quartetOffset] = x4[b_i3 - 1];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    signed char perm[4];
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
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

    quartetOffset = (unsigned char)ib;
    for (k = 0; k < quartetOffset; k++) {
      i1 = perm[k] - 1;
      i3 = ((k - nNaNs) - ib) + 132;
      idx[i3] = idx4[i1];
      x[i3] = x4[i1];
    }
  }

  ib = (nNaNs >> 1) + 132;
  for (k = 0; k <= ib - 133; k++) {
    i1 = (k - nNaNs) + 132;
    quartetOffset = idx[i1];
    idx[i1] = idx[131 - k];
    idx[131 - k] = quartetOffset;
    x[i1] = xwork[131 - k];
    x[131 - k] = xwork[i1];
  }

  if ((nNaNs & 1) != 0) {
    quartetOffset = ib - nNaNs;
    x[quartetOffset] = xwork[quartetOffset];
  }

  if (132 - nNaNs > 1) {
    memset(&iwork[0], 0, 132U * sizeof(int));
    i3 = (132 - nNaNs) >> 2;
    quartetOffset = 4;
    while (i3 > 1) {
      if ((i3 & 1) != 0) {
        i3--;
        ib = quartetOffset * i3;
        i1 = 132 - (nNaNs + ib);
        if (i1 > quartetOffset) {
          b_merge(idx, x, ib, quartetOffset, i1 - quartetOffset, iwork, xwork);
        }
      }

      ib = quartetOffset << 1;
      i3 >>= 1;
      for (k = 0; k < i3; k++) {
        b_merge(idx, x, k * ib, quartetOffset, quartetOffset, iwork, xwork);
      }

      quartetOffset = ib;
    }

    if (132 - nNaNs > quartetOffset) {
      b_merge(idx, x, 0, quartetOffset, 132 - (nNaNs + quartetOffset), iwork,
              xwork);
    }
  }
}

/*
 * Arguments    : const double x[50]
 *                const double y[50]
 * Return Type  : double
 */
static double b_trapz(const double x[50], const double y[50])
{
  double c[50];
  double z;
  int ia;
  int iac;
  int ix;
  c[0] = 0.5 * (x[1] - x[0]);
  for (ix = 0; ix < 48; ix++) {
    c[ix + 1] = 0.5 * (x[ix + 2] - x[ix]);
  }

  c[49] = 0.5 * (x[49] - x[48]);
  z = 0.0;
  ix = 0;
  for (iac = 0; iac < 50; iac++) {
    int b_iac;
    b_iac = iac + 1;
    for (ia = b_iac; ia <= b_iac; ia++) {
      z += y[ia - 1] * c[ix];
    }

    ix++;
  }

  return z;
}

/*
 * Arguments    : const TypeAvoMainRoVehInfo *in1
 *                const signed char in2_data[]
 *                const int in2_size[2]
 *                const signed char in3_data[]
 *                const int in3_size[2]
 *                double *out1
 *                int *out2
 * Return Type  : void
 */
static void binary_expand_op(const TypeAvoMainRoVehInfo *in1, const signed char
  in2_data[], const int in2_size[2], const signed char in3_data[], const int
  in3_size[2], double *out1, int *out2)
{
  double in1_data[4];
  int in1_size[2];
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_size[0] = 1;
  if (in3_size[1] == 1) {
    in1_size[1] = in2_size[1];
  } else {
    in1_size[1] = in3_size[1];
  }

  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  if (in3_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in3_size[1];
  }

  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in1->d_veh2converge - in1->
                   targetLaneBehindDisAvoidVehicle[in2_data[i * stride_0_1] - 1])
      / (in1->targetLaneBehindVelAvoidVehicle[in3_data[i * stride_1_1] - 1] +
         2.2204460492503131E-16);
  }

  b_minimum(in1_data, in1_size, out1, out2);
}

/*
 * Arguments    : double c_FunFcn_workspace_fun_x_worksp
 *                const double d_FunFcn_workspace_fun_x_worksp[5]
 *                const double e_FunFcn_workspace_fun_x_worksp[12]
 *                double FunFcn_workspace_length
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void c_fzero(double c_FunFcn_workspace_fun_x_worksp, const double
                    d_FunFcn_workspace_fun_x_worksp[5], const double
                    e_FunFcn_workspace_fun_x_worksp[12], double
                    FunFcn_workspace_length, const double x[2], double *b,
                    double *fval, double *exitflag)
{
  double a;
  double fa;
  double savefa;
  double savefb;
  *exitflag = 1.0;
  a = x[0];
  *b = x[1];
  fa = ReplanTrajPosCalc4_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
    d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_length, x[0]);
  *fval = ReplanTrajPosCalc4_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
    d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_length, x[1]);
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = x[0];
    *fval = fa;
  } else if (!(*fval == 0.0)) {
    double c;
    double d;
    double e;
    double fc;
    boolean_T exitg1;
    fc = *fval;
    c = x[1];
    e = 0.0;
    d = 0.0;
    exitg1 = false;
    while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
      double m;
      double toler;
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
          double q;
          double s;
          s = *fval / fa;
          if (a == c) {
            fa = 2.0 * m * s;
            q = 1.0 - s;
          } else {
            double r;
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

          if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e *
                q))) {
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

        *fval = ReplanTrajPosCalc4_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
          d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
          FunFcn_workspace_length, *b);
      }
    }

    if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
      *exitflag = -5.0;
    }
  }
}

/*
 * Arguments    : double d1
 *                double d2
 *                double y[5]
 * Return Type  : void
 */
static void c_linspace(double d1, double d2, double y[5])
{
  y[4] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    double delta1;
    delta1 = d2 / 4.0;
    y[1] = -2.0 * delta1;
    y[3] = 2.0 * delta1;
    y[2] = 0.0;
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307)
              || (fabs(d2) > 8.9884656743115785E+307))) {
    double delta1;
    double delta2;
    delta1 = d1 / 4.0;
    delta2 = d2 / 4.0;
    y[1] = (d1 + delta2) - delta1;
    y[2] = (d1 + delta2 * 2.0) - delta1 * 2.0;
    y[3] = (d1 + delta2 * 3.0) - delta1 * 3.0;
  } else {
    double delta1;
    delta1 = (d2 - d1) / 4.0;
    y[1] = d1 + delta1;
    y[2] = d1 + 2.0 * delta1;
    y[3] = d1 + 3.0 * delta1;
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
 * Arguments    : const double pp_breaks[5]
 *                const double pp_coefs[12]
 *                double x
 * Return Type  : double
 */
static double c_ppval(const double pp_breaks[5], const double pp_coefs[12],
                      double x)
{
  double v;
  if (rtIsNaN(x)) {
    v = rtNaN;
  } else {
    int high_i;
    int low_i;
    int low_ip1;
    low_i = 0;
    low_ip1 = 2;
    high_i = 5;
    while (high_i > low_ip1) {
      int mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    double xloc;
    xloc = x - pp_breaks[low_i];
    v = xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 4]) + pp_coefs[low_i +
      8];
  }

  return v;
}

/*
 * Arguments    : double c_FunFcn_workspace_fun_x_worksp
 *                const double d_FunFcn_workspace_fun_x_worksp[4]
 *                const double e_FunFcn_workspace_fun_x_worksp[9]
 *                double FunFcn_workspace_length
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void d_fzero(double c_FunFcn_workspace_fun_x_worksp, const double
                    d_FunFcn_workspace_fun_x_worksp[4], const double
                    e_FunFcn_workspace_fun_x_worksp[9], double
                    FunFcn_workspace_length, const double x[2], double *b,
                    double *fval, double *exitflag)
{
  double a;
  double fa;
  double savefa;
  double savefb;
  *exitflag = 1.0;
  a = x[0];
  *b = x[1];
  fa = ReplanTrajPosCalc3_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
    d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_length, x[0]);
  *fval = ReplanTrajPosCalc3_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
    d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
    FunFcn_workspace_length, x[1]);
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = x[0];
    *fval = fa;
  } else if (!(*fval == 0.0)) {
    double c;
    double d;
    double e;
    double fc;
    boolean_T exitg1;
    fc = *fval;
    c = x[1];
    e = 0.0;
    d = 0.0;
    exitg1 = false;
    while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
      double m;
      double toler;
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
          double q;
          double s;
          s = *fval / fa;
          if (a == c) {
            fa = 2.0 * m * s;
            q = 1.0 - s;
          } else {
            double r;
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

          if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e *
                q))) {
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

        *fval = ReplanTrajPosCalc3_anonFcn3(c_FunFcn_workspace_fun_x_worksp,
          d_FunFcn_workspace_fun_x_worksp, e_FunFcn_workspace_fun_x_worksp,
          FunFcn_workspace_length, *b);
      }
    }

    if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
      *exitflag = -5.0;
    }
  }
}

/*
 * Arguments    : double d2
 *                double y[100]
 * Return Type  : void
 */
static void d_linspace(double d2, double y[100])
{
  int k;
  y[99] = d2;
  y[0] = 0.0;
  if (-d2 == 0.0) {
    double delta1;
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = (2.0 * ((double)k + 2.0) - 101.0) * delta1;
    }
  } else if ((d2 < 0.0) && (fabs(d2) > 8.9884656743115785E+307)) {
    double delta1;
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = delta1 * ((double)k + 1.0);
    }
  } else {
    double delta1;
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = ((double)k + 1.0) * delta1;
    }
  }
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
static double d_minimum(const double x[3])
{
  double ex;
  int idx;
  int k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
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
      double d;
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double pp_breaks[4]
 *                const double pp_coefs[9]
 *                const double x[50]
 *                double v[50]
 * Return Type  : void
 */
static void d_ppval(const double pp_breaks[4], const double pp_coefs[9], const
                    double x[50], double v[50])
{
  int ix;
  for (ix = 0; ix < 50; ix++) {
    double xloc;
    if (rtIsNaN(x[ix])) {
      xloc = rtNaN;
    } else {
      int high_i;
      int low_i;
      int low_ip1;
      low_i = 0;
      low_ip1 = 2;
      high_i = 4;
      while (high_i > low_ip1) {
        int mid_i;
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[ix] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[ix] - pp_breaks[low_i];
      xloc = xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 3]) +
        pp_coefs[low_i + 6];
    }

    v[ix] = xloc;
  }
}

/*
 * Arguments    : const emxArray_real_T *a
 *                const emxArray_real_T *b
 *                emxArray_real_T *c
 *                emxArray_int32_T *ia
 *                int *ib_size
 * Return Type  : void
 */
static void do_vectors(const emxArray_real_T *a, const emxArray_real_T *b,
  emxArray_real_T *c, emxArray_int32_T *ia, int *ib_size)
{
  const double *a_data;
  const double *b_data;
  double *c_data;
  int b_ialast;
  int iafirst;
  int ialast;
  int iblast;
  int na;
  int nc;
  int nia;
  int *ia_data;
  b_data = b->data;
  a_data = a->data;
  na = a->size[1];
  iafirst = c->size[0] * c->size[1];
  c->size[0] = 1;
  c->size[1] = a->size[1];
  emxEnsureCapacity_real_T(c, iafirst);
  c_data = c->data;
  iafirst = ia->size[0];
  ia->size[0] = a->size[1];
  emxEnsureCapacity_int32_T(ia, iafirst);
  ia_data = ia->data;
  *ib_size = 0;
  nc = 0;
  nia = 0;
  iafirst = 0;
  ialast = 0;
  iblast = 1;
  while ((ialast + 1 <= na) && (iblast <= b->size[1])) {
    double ak;
    double bk;
    b_ialast = ialast + 1;
    ak = a_data[ialast];
    while ((b_ialast < a->size[1]) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }

    ialast = b_ialast - 1;
    bk = b_data[iblast - 1];
    while ((iblast < b->size[1]) && (b_data[iblast] == bk)) {
      iblast++;
    }

    if (ak == bk) {
      ialast = b_ialast;
      iafirst = b_ialast;
      iblast++;
    } else {
      boolean_T p;
      if (rtIsNaN(bk)) {
        p = !rtIsNaN(ak);
      } else if (rtIsNaN(ak)) {
        p = false;
      } else {
        p = (ak < bk);
      }

      if (p) {
        nc++;
        nia++;
        c_data[nc - 1] = ak;
        ia_data[nia - 1] = iafirst + 1;
        ialast = b_ialast;
        iafirst = b_ialast;
      } else {
        iblast++;
      }
    }
  }

  while (ialast + 1 <= na) {
    b_ialast = ialast + 1;
    while ((b_ialast < a->size[1]) && (a_data[b_ialast] == a_data[ialast])) {
      b_ialast++;
    }

    nc++;
    nia++;
    c_data[nc - 1] = a_data[ialast];
    ia_data[nia - 1] = iafirst + 1;
    ialast = b_ialast;
    iafirst = b_ialast;
  }

  if (a->size[1] > 0) {
    iafirst = ia->size[0];
    if (nia < 1) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia;
    }

    emxEnsureCapacity_int32_T(ia, iafirst);
    iafirst = c->size[0] * c->size[1];
    if (nc < 1) {
      c->size[1] = 0;
    } else {
      c->size[1] = nc;
    }

    emxEnsureCapacity_real_T(c, iafirst);
  }
}

/*
 * Arguments    : const double c_FunFcn_workspace_SpeedOpposit[6]
 *                double FunFcn_workspace_j
 *                double c_FunFcn_workspace_a_OppositeCa
 *                double FunFcn_workspace_v_max
 *                const double c_FunFcn_workspace_PosSOpposite[6]
 *                double FunFcn_workspace_w_veh
 *                double FunFcn_workspace_D_safe2
 *                const double x[2]
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void e_fzero(const double c_FunFcn_workspace_SpeedOpposit[6], double
                    FunFcn_workspace_j, double c_FunFcn_workspace_a_OppositeCa,
                    double FunFcn_workspace_v_max, const double
                    c_FunFcn_workspace_PosSOpposite[6], double
                    FunFcn_workspace_w_veh, double FunFcn_workspace_D_safe2,
                    const double x[2], double *b, double *fval, double *exitflag)
{
  double a;
  double fa;
  double fa_tmp;
  double p;
  double q;
  double savefa;
  double savefb;
  double x_idx_1_tmp;
  *exitflag = 1.0;
  a = -0.01;
  *b = x[1];
  x_idx_1_tmp = (c_FunFcn_workspace_PosSOpposite[(int)FunFcn_workspace_j - 1] -
                 0.5 * FunFcn_workspace_w_veh) - FunFcn_workspace_D_safe2;
  if (x_idx_1_tmp > 0.0) {
    q = x_idx_1_tmp;
    p = x_idx_1_tmp;
  } else {
    q = 0.0;
    p = 0.0;
  }

  fa_tmp = c_FunFcn_workspace_SpeedOpposit[(int)FunFcn_workspace_j - 1];
  fa = fmax((fa_tmp + fmin(fa_tmp + c_FunFcn_workspace_a_OppositeCa * -0.01,
              FunFcn_workspace_v_max)) / 2.0, 0.0001) * -0.01 - q;

  /* , */
  *fval = fmax((fa_tmp + fmin(fa_tmp + c_FunFcn_workspace_a_OppositeCa * x[1],
    FunFcn_workspace_v_max)) / 2.0, 0.0001) * x[1] - p;

  /* , */
  savefa = fa;
  savefb = *fval;
  if (!(*fval == 0.0)) {
    double c;
    double d;
    double e;
    double fc;
    boolean_T exitg1;
    fc = *fval;
    c = x[1];
    e = 0.0;
    d = 0.0;
    exitg1 = false;
    while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
      double m;
      double toler;
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
          double s;
          s = *fval / fa;
          if (a == c) {
            p = 2.0 * m * s;
            q = 1.0 - s;
          } else {
            q = fa / fc;
            fa = *fval / fc;
            p = s * (2.0 * m * q * (q - fa) - (*b - a) * (fa - 1.0));
            q = (q - 1.0) * (fa - 1.0) * (s - 1.0);
          }

          if (p > 0.0) {
            q = -q;
          } else {
            p = -p;
          }

          if ((2.0 * p < 3.0 * m * q - fabs(toler * q)) && (p < fabs(0.5 * e * q)))
          {
            e = d;
            d = p / q;
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

        if (x_idx_1_tmp > 0.0) {
          q = x_idx_1_tmp;
        } else {
          q = 0.0;
        }

        *fval = fmax((fa_tmp + fmin(fa_tmp + c_FunFcn_workspace_a_OppositeCa *
          *b, FunFcn_workspace_v_max)) / 2.0, 0.0001) * *b - q;

        /* , */
      }
    }

    if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
      *exitflag = -5.0;
    }
  }
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
static double e_minimum(const double x_data[], const int x_size[2])
{
  double ex;
  int k;
  int last;
  last = x_size[1];
  if (x_size[1] <= 2) {
    if (x_size[1] == 1) {
      ex = x_data[0];
    } else {
      ex = x_data[x_size[1] - 1];
      if ((!(x_data[0] > ex)) && ((!rtIsNaN(x_data[0])) || rtIsNaN(ex))) {
        ex = x_data[0];
      }
    }
  } else {
    int idx;
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      boolean_T exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
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
      for (k = idx; k <= last; k++) {
        double d;
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
 * Arguments    : const double pp_breaks[4]
 *                const double pp_coefs[12]
 *                double x
 * Return Type  : double
 */
static double e_ppval(const double pp_breaks[4], const double pp_coefs[12],
                      double x)
{
  double v;
  if (rtIsNaN(x)) {
    v = rtNaN;
  } else {
    int high_i;
    int low_i;
    int low_ip1;
    low_i = 0;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      int mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    double xloc;
    xloc = x - pp_breaks[low_i];
    v = xloc * (xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 3]) +
                pp_coefs[low_i + 6]) + pp_coefs[low_i + 9];
  }

  return v;
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
  double *y_data;
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
  y_data = y->data;
  if (n > 0) {
    y_data[0] = a;
    if (n > 1) {
      y_data[n - 1] = apnd;
      nm1d2 = (n - 1) / 2;
      for (k = 0; k <= nm1d2 - 2; k++) {
        ndbl = ((double)k + 1.0) * d;
        y_data[k + 1] = a + ndbl;
        y_data[(n - k) - 2] = apnd - ndbl;
      }

      if (nm1d2 << 1 == n - 1) {
        y_data[nm1d2] = (a + apnd) / 2.0;
      } else {
        ndbl = (double)nm1d2 * d;
        y_data[nm1d2] = a + ndbl;
        y_data[nm1d2 + 1] = apnd - ndbl;
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
  short *y_data;
  if (b < (short)a) {
    n = 0;
  } else {
    unsigned short b_a;
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
  y_data = y->data;
  if (n > 0) {
    short yk;
    y_data[0] = (short)a;
    yk = (short)a;
    for (k = 2; k <= n; k++) {
      yk++;
      y_data[k - 1] = yk;
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
 * Arguments    : const double pp_breaks[4]
 *                const double pp_coefs[9]
 *                double x
 * Return Type  : double
 */
static double f_ppval(const double pp_breaks[4], const double pp_coefs[9],
                      double x)
{
  double v;
  if (rtIsNaN(x)) {
    v = rtNaN;
  } else {
    int high_i;
    int low_i;
    int low_ip1;
    low_i = 0;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      int mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    double xloc;
    xloc = x - pp_breaks[low_i];
    v = xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 3]) + pp_coefs[low_i +
      6];
  }

  return v;
}

/*
 * Arguments    : double c_FunFcn_workspace_fun_t_worksp
 *                double FunFcn_workspace_S_end
 *                double FunFcn_workspace_w_lane
 *                double *b
 *                double *fval
 *                double *exitflag
 * Return Type  : void
 */
static void fzero(double c_FunFcn_workspace_fun_t_worksp, double
                  FunFcn_workspace_S_end, double FunFcn_workspace_w_lane, double
                  *b, double *fval, double *exitflag)
{
  double a;
  double fa;
  double fa_tmp;
  double savefa;
  double savefb;
  *exitflag = 1.0;
  a = 1.0;
  *b = 4.0;
  fa_tmp = sqrt(FunFcn_workspace_S_end * FunFcn_workspace_S_end +
                FunFcn_workspace_w_lane * FunFcn_workspace_w_lane);
  fa = (c_FunFcn_workspace_fun_t_worksp + 1.0) - fa_tmp;
  *fval = (c_FunFcn_workspace_fun_t_worksp * 4.0 + 16.0) - fa_tmp;
  savefa = fa;
  savefb = *fval;
  if (fa == 0.0) {
    *b = 1.0;
    *fval = fa;
  } else if (!(*fval == 0.0)) {
    double c;
    double d;
    double e;
    double fc;
    boolean_T exitg1;
    fc = *fval;
    c = 4.0;
    e = 0.0;
    d = 0.0;
    exitg1 = false;
    while ((!exitg1) && ((*fval != 0.0) && (a != *b))) {
      double m;
      double toler;
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
          double q;
          double s;
          s = *fval / fa;
          if (a == c) {
            fa = 2.0 * m * s;
            q = 1.0 - s;
          } else {
            double r;
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

          if ((2.0 * fa < 3.0 * m * q - fabs(toler * q)) && (fa < fabs(0.5 * e *
                q))) {
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

        *fval = (c_FunFcn_workspace_fun_t_worksp * *b + *b * *b) - fa_tmp;
      }
    }

    if (!(fabs(*fval) <= fmax(fabs(savefa), fabs(savefb)))) {
      *exitflag = -5.0;
    }
  }
}

/*
 * Arguments    : const double x[8]
 * Return Type  : double
 */
static double g_minimum(const double x[8])
{
  double ex;
  int idx;
  int k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
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
      double d;
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double x[9]
 * Return Type  : double
 */
static double h_minimum(const double x[9])
{
  double ex;
  int idx;
  int k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 9)) {
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
    for (k = idx; k < 10; k++) {
      double d;
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }

  return ex;
}

/*
 * Arguments    : const double s[6]
 * Return Type  : int
 */
static int intnnz(const double s[6])
{
  int k;
  int n;
  n = 0;
  for (k = 0; k < 6; k++) {
    if (s[k] != 0.0) {
      n++;
    }
  }

  return n;
}

/*
 * Arguments    : short a
 *                const emxArray_int16_T *s
 * Return Type  : boolean_T
 */
static boolean_T isMember(short a, const emxArray_int16_T *s)
{
  int k;
  const short *s_data;
  boolean_T exitg1;
  boolean_T tf;
  s_data = s->data;
  tf = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= s->size[1] - 1)) {
    if (a == s_data[k]) {
      tf = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return tf;
}

/*
 * Arguments    : double d1
 *                double d2
 *                double y[50]
 * Return Type  : void
 */
static void linspace(double d1, double d2, double y[50])
{
  int k;
  y[49] = d2;
  y[0] = d1;
  if (d1 == -d2) {
    double delta1;
    delta1 = d2 / 49.0;
    for (k = 0; k < 48; k++) {
      y[k + 1] = (2.0 * ((double)k + 2.0) - 51.0) * delta1;
    }
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307)
              || (fabs(d2) > 8.9884656743115785E+307))) {
    double delta1;
    double delta2;
    delta1 = d1 / 49.0;
    delta2 = d2 / 49.0;
    for (k = 0; k < 48; k++) {
      y[k + 1] = (d1 + delta2 * ((double)k + 1.0)) - delta1 * ((double)k + 1.0);
    }
  } else {
    double delta1;
    delta1 = (d2 - d1) / 49.0;
    for (k = 0; k < 48; k++) {
      y[k + 1] = d1 + ((double)k + 1.0) * delta1;
    }
  }
}

/*
 * ,
 *
 * Arguments    : double BasicsInfo_currentLaneFrontDis
 *                double BasicsInfo_currentLaneFrontVel
 *                double BasicsInfo_currentLaneFrontLen
 *                double BasicsInfo_pos_s
 *                double BasicsInfo_pos_l
 *                double BasicsInfo_pos_psi
 *                double BasicsInfo_pos_l_CurrentLane
 *                short BasicsInfo_currentLaneIndex
 *                const double BasicsInfo_widthOfLanes[6]
 *                short BasicsInfo_targetLaneIndex
 *                double BasicsInfo_v_max
 *                short BasicsInfo_goalLaneIndex
 *                double BasicsInfo_d_veh2goal
 *                double BasicsInfo_sampleTime
 *                double ChassisInfo_speed
 *                short ChassisInfo_currentGear
 *                const TypeLaneChangeInfo *LaneChangeInfo
 *                const TypeAvoMainRoVehInfo *AvoMainRoVehInfo
 *                const TypeAvoPedInfo *AvoPedInfo
 *                const TypeTrafficLightInfo *TrafficLightInfo
 *                const TypeAvoOncomingVehInfo *AvoOncomingVehInfo
 *                const TypeAvoFailVehInfo *AvoFailVehInfo
 *                const TypeTurnAroundInfo *TurnAroundInfo
 *                const TypeStopSignInfo StopSignInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short TurnAroundActive
 *                short GlosaActive
 *                short PlannerLevel
 *                const TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters Parameters
 * Return Type  : void
 */
static void logInput(double BasicsInfo_currentLaneFrontDis, double
                     BasicsInfo_currentLaneFrontVel, double
                     BasicsInfo_currentLaneFrontLen, double BasicsInfo_pos_s,
                     double BasicsInfo_pos_l, double BasicsInfo_pos_psi, double
                     BasicsInfo_pos_l_CurrentLane, short
                     BasicsInfo_currentLaneIndex, const double
                     BasicsInfo_widthOfLanes[6], short
                     BasicsInfo_targetLaneIndex, double BasicsInfo_v_max, short
                     BasicsInfo_goalLaneIndex, double BasicsInfo_d_veh2goal,
                     double BasicsInfo_sampleTime, double ChassisInfo_speed,
                     short ChassisInfo_currentGear, const TypeLaneChangeInfo
                     *LaneChangeInfo, const TypeAvoMainRoVehInfo
                     *AvoMainRoVehInfo, const TypeAvoPedInfo *AvoPedInfo, const
                     TypeTrafficLightInfo *TrafficLightInfo, const
                     TypeAvoOncomingVehInfo *AvoOncomingVehInfo, const
                     TypeAvoFailVehInfo *AvoFailVehInfo, const
                     TypeTurnAroundInfo *TurnAroundInfo, const TypeStopSignInfo
                     StopSignInfo, short LaneChangeActive, short
                     PedestrianActive, short TrafficLightActive, short
                     VehicleCrossingActive, short VehicleOncomingActive, short
                     TurnAroundActive, short GlosaActive, short PlannerLevel,
                     const TypeGlobVars *GlobVars, const TypeCalibrationVars
                     *CalibrationVars, const TypeParameters Parameters)
{
  int i;

  /* 打印入参 */
  if (CalibrationVars->UrbanPlanner.logTrigger[0] == 1) {
    printf("AvoFailVehInfo.lanesWithFail= \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%d\n", AvoFailVehInfo->lanesWithFail[5]);
        fflush(stdout);
      } else {
        printf("%d\t", AvoFailVehInfo->lanesWithFail[i]);
        fflush(stdout);
      }
    }

    printf("AvoFailVehInfo.failLaneindex= \t");
    fflush(stdout);
    for (i = 0; i < 5; i++) {
      if (i + 1 == 5) {
        printf("%d\n", AvoFailVehInfo->failLaneindex[4]);
        fflush(stdout);
      } else {
        printf("%d\t", AvoFailVehInfo->failLaneindex[i]);
        fflush(stdout);
      }
    }

    printf("AvoFailVehInfo.failLaneFrontDis = \t");
    fflush(stdout);
    for (i = 0; i < 5; i++) {
      if (i + 1 == 5) {
        printf("%f\n", AvoFailVehInfo->failLaneFrontDis[4]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoFailVehInfo->failLaneFrontDis[i]);
        fflush(stdout);
      }
    }

    printf("AvoFailVehInfo.failLaneFrontVel = \t");
    fflush(stdout);
    for (i = 0; i < 5; i++) {
      if (i + 1 == 5) {
        printf("%f\n", AvoFailVehInfo->failLaneFrontVel[4]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoFailVehInfo->failLaneFrontVel[i]);
        fflush(stdout);
      }
    }

    printf("AvoFailVehInfo.failLaneFrontLen = \t");
    fflush(stdout);
    for (i = 0; i < 5; i++) {
      if (i + 1 == 5) {
        printf("%f\n", AvoFailVehInfo->failLaneFrontLen[4]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoFailVehInfo->failLaneFrontLen[i]);
        fflush(stdout);
      }
    }
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[1] == 1) {
    printf("AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindVelAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindVelAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindVelAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneBehindVelAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindLenAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindLenAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneBehindLenAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneBehindLenAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle = \t");
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[0]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[1]);
    fflush(stdout);
    printf("%f\t", AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[2]);
    fflush(stdout);
    printf("%f\n", AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[3]);
    fflush(stdout);
    printf("AvoMainRoVehInfo.d_veh2converge = %f\n",
           AvoMainRoVehInfo->d_veh2converge);
    fflush(stdout);
    printf("AvoMainRoVehInfo.d_veh2stopline = %f\n",
           AvoMainRoVehInfo->d_veh2stopline);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[2] == 1) {
    printf("AvoOncomingVehInfo.d_veh2waitingArea = %f\n",
           AvoOncomingVehInfo->d_veh2waitingArea);
    fflush(stdout);
    printf("AvoOncomingVehInfo.s_veh = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->s_veh[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->s_veh[i]);
        fflush(stdout);
      }
    }

    printf("AvoOncomingVehInfo.v_veh = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->v_veh[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->v_veh[i]);
        fflush(stdout);
      }
    }

    printf("AvoOncomingVehInfo.l_veh = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->l_veh[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->l_veh[i]);
        fflush(stdout);
      }
    }

    printf("AvoOncomingVehInfo.d_veh2conflict = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->d_veh2conflict[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->d_veh2conflict[i]);
        fflush(stdout);
      }
    }

    printf("AvoOncomingVehInfo.s_vehapostrophe = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->s_vehapostrophe[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->s_vehapostrophe[i]);
        fflush(stdout);
      }
    }

    printf("AvoOncomingVehInfo.l_vehapostrophe = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", AvoOncomingVehInfo->l_vehapostrophe[5]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoOncomingVehInfo->l_vehapostrophe[i]);
        fflush(stdout);
      }
    }
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[3] == 1) {
    printf("AvoPedInfo.d_veh2cross = %f\n", AvoPedInfo->d_veh2cross);
    fflush(stdout);
    printf("AvoPedInfo.w_cross = %f\n", AvoPedInfo->w_cross);
    fflush(stdout);
    printf("AvoPedInfo.s_ped = \t");
    fflush(stdout);
    for (i = 0; i < 40; i++) {
      if (i + 1 == 40) {
        printf("%f\n", AvoPedInfo->s_ped[39]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoPedInfo->s_ped[i]);
        fflush(stdout);
      }
    }

    printf("AvoPedInfo.v_ped = \t");
    fflush(stdout);
    for (i = 0; i < 40; i++) {
      if (i + 1 == 40) {
        printf("%f\n", AvoPedInfo->v_ped[39]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoPedInfo->v_ped[i]);
        fflush(stdout);
      }
    }

    printf("AvoPedInfo.l_ped = \t");
    fflush(stdout);
    for (i = 0; i < 40; i++) {
      if (i + 1 == 40) {
        printf("%f\n", AvoPedInfo->l_ped[39]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoPedInfo->l_ped[i]);
        fflush(stdout);
      }
    }

    printf("AvoPedInfo.psi_ped = \t");
    fflush(stdout);
    for (i = 0; i < 40; i++) {
      if (i + 1 == 40) {
        printf("%f\n", AvoPedInfo->psi_ped[39]);
        fflush(stdout);
      } else {
        printf("%f\t", AvoPedInfo->psi_ped[i]);
        fflush(stdout);
      }
    }
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[4] == 1) {
    printf("BasicsInfo.currentLaneFrontDis = %f\n",
           BasicsInfo_currentLaneFrontDis);
    fflush(stdout);
    printf("BasicsInfo.currentLaneFrontVel = %f\n",
           BasicsInfo_currentLaneFrontVel);
    fflush(stdout);
    printf("BasicsInfo.currentLaneFrontLen = %f\n",
           BasicsInfo_currentLaneFrontLen);
    fflush(stdout);
    printf("BasicsInfo.pos_s = %f\n", BasicsInfo_pos_s);
    fflush(stdout);
    printf("BasicsInfo.pos_l = %f\n", BasicsInfo_pos_l);
    fflush(stdout);
    printf("BasicsInfo.pos_psi = %f\n", BasicsInfo_pos_psi);
    fflush(stdout);
    printf("BasicsInfo.pos_l_CurrentLane = %f\n", BasicsInfo_pos_l_CurrentLane);
    fflush(stdout);
    printf("BasicsInfo.currentLaneIndex = %d\n", BasicsInfo_currentLaneIndex);
    fflush(stdout);
    printf("BasicsInfo.widthOfLanes = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", BasicsInfo_widthOfLanes[5]);
        fflush(stdout);
      } else {
        printf("%f\t", BasicsInfo_widthOfLanes[i]);
        fflush(stdout);
      }
    }

    printf("BasicsInfo.targetLaneIndex = %d\n", BasicsInfo_targetLaneIndex);
    fflush(stdout);
    printf("BasicsInfo.v_max = %f\n", BasicsInfo_v_max);
    fflush(stdout);
    printf("BasicsInfo.goalLaneIndex = %d\n", BasicsInfo_goalLaneIndex);
    fflush(stdout);
    printf("BasicsInfo.d_veh2goal = %f\n", BasicsInfo_d_veh2goal);
    fflush(stdout);
    printf("BasicsInfo.sampleTime = %f\n", BasicsInfo_sampleTime);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[5] == 1) {
    printf("CalibrationVars.TrajPlanTurnAround.d_safe1 = %f\n",
           CalibrationVars->TrajPlanTurnAround.d_safe1);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.d_safe2 = %f\n",
           CalibrationVars->TrajPlanTurnAround.d_safe2);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.dec2line = %f\n",
           CalibrationVars->TrajPlanTurnAround.dec2line);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.a_min = %f\n",
           CalibrationVars->TrajPlanTurnAround.a_min);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.a_max_com = %f\n",
           CalibrationVars->TrajPlanTurnAround.a_max_com);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.v_max_turnAround = %f\n",
           CalibrationVars->TrajPlanTurnAround.v_max_turnAround);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanTurnAround.d_gap2stop = %f\n",
           CalibrationVars->TrajPlanTurnAround.d_gap2stop);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[6] == 1) {
    printf("CalibrationVars.SpeedPlanAvoidPedestrian.a_max = %f\n",
           CalibrationVars->SpeedPlanAvoidPedestrian.a_max);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidPedestrian.a_min = %f\n",
           CalibrationVars->SpeedPlanAvoidPedestrian.a_min);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int = %f\n",
           CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg = %f\n",
           CalibrationVars->SpeedPlanAvoidPedestrian.v_max_int_emg);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped = %f\n",
           CalibrationVars->SpeedPlanAvoidPedestrian.d_gap2ped);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[7] == 1) {
    printf("CalibrationVars.SpeedPlanTrafficLight.a_min_com = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.a_min_com);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanTrafficLight.a_max = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.a_max);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanTrafficLight.a_min = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.a_min);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanTrafficLight.v_max_int = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.v_max_int);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanTrafficLight.t_acc = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.t_acc);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline = %f\n",
           CalibrationVars->SpeedPlanTrafficLight.d_gap2stopline);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[8] == 1) {
    printf("CalibrationVars.SpeedPlanAvoidVehicle.a_min_com = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.a_min_com);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidVehicle.a_max = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.a_max);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidVehicle.a_min = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.a_min);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidVehicle.v_max = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.v_max);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidVehicle.t_re = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.t_re);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidVehicle.gapIndex = %f\n",
           CalibrationVars->SpeedPlanAvoidVehicle.gapIndex);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[9] == 1) {
    printf("CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com = %f\n",
           CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_max_com);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min = %f\n",
           CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_min);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int = %f\n",
           CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int);
    fflush(stdout);
    printf("CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe = %f\n",
           CalibrationVars->SpeedPlanAvoidOncomingVehicle.d_safe);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[10] == 1) {
    printf("CalibrationVars.TrajPlanLaneChange.v_max_int = %f\n",
           CalibrationVars->TrajPlanLaneChange.v_max_int);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int = %f\n",
           CalibrationVars->TrajPlanLaneChange.indexAfterLaneChangeDis2Int);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.t_permit = %f\n",
           CalibrationVars->TrajPlanLaneChange.t_permit);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.t_re = %f\n",
           CalibrationVars->TrajPlanLaneChange.t_re);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.index_accel = %f\n",
           CalibrationVars->TrajPlanLaneChange.index_accel);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.a_max_comfort = %f\n",
           CalibrationVars->TrajPlanLaneChange.a_max_comfort);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.a_min = %f\n",
           CalibrationVars->TrajPlanLaneChange.a_min);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.a_max = %f\n",
           CalibrationVars->TrajPlanLaneChange.a_max);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.a_min_comfort = %f\n",
           CalibrationVars->TrajPlanLaneChange.a_min_comfort);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange.a_lateral = %f\n",
           CalibrationVars->TrajPlanLaneChange.a_lateral);
    fflush(stdout);
    printf("CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit = %f\n",
           CalibrationVars->TrajPlanLaneChange_RePlan.frontWheelAnglelLimit);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[11] == 1) {
    printf("CalibrationVars.ACC.a_max = %f\n", CalibrationVars->ACC.a_max);
    fflush(stdout);
    printf("CalibrationVars.ACC.a_min = %f\n", CalibrationVars->ACC.a_min);
    fflush(stdout);
    printf("CalibrationVars.ACC.d_wait2faultyCar = %f\n",
           CalibrationVars->ACC.d_wait2faultyCar);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_v_com = %f\n",
           CalibrationVars->ACC.tau_v_com);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_v = %f\n", CalibrationVars->ACC.tau_v);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_d = %f\n", CalibrationVars->ACC.tau_d);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_v_bre = %f\n",
           CalibrationVars->ACC.tau_v_bre);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_v_emg = %f\n",
           CalibrationVars->ACC.tau_v_emg);
    fflush(stdout);
    printf("CalibrationVars.ACC.tau_d_emg = %f\n",
           CalibrationVars->ACC.tau_d_emg);
    fflush(stdout);
    printf("CalibrationVars.ACC.t_acc = %f\n", CalibrationVars->ACC.t_acc);
    fflush(stdout);
    printf("CalibrationVars.ACC.d_wait = %f\n", CalibrationVars->ACC.d_wait);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[12] == 1) {
    printf("CalibrationVars.ACClowSpeed.a_max = %f\n",
           CalibrationVars->ACClowSpeed.a_max);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.a_min = %f\n",
           CalibrationVars->ACClowSpeed.a_min);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.a_min_com = %f\n",
           CalibrationVars->ACClowSpeed.a_min_com);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_v_com = %f\n",
           CalibrationVars->ACClowSpeed.tau_v_com);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_v = %f\n",
           CalibrationVars->ACClowSpeed.tau_v);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_d = %f\n",
           CalibrationVars->ACClowSpeed.tau_d);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_v_bre = %f\n",
           CalibrationVars->ACClowSpeed.tau_v_bre);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_v_emg = %f\n",
           CalibrationVars->ACClowSpeed.tau_v_emg);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_d_emg = %f\n",
           CalibrationVars->ACClowSpeed.tau_d_emg);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.tau_d_lowspeed = %f\n",
           CalibrationVars->ACClowSpeed.tau_d_lowspeed);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.t_acc = %f\n",
           CalibrationVars->ACClowSpeed.t_acc);
    fflush(stdout);
    printf("CalibrationVars.ACClowSpeed.d_wait = %f\n",
           CalibrationVars->ACClowSpeed.d_wait);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[13] == 1) {
    printf("CalibrationVars.Decider.a_bre = %f\n",
           CalibrationVars->Decider.a_bre);
    fflush(stdout);
    printf("CalibrationVars.Decider.a_bre_com = %f\n",
           CalibrationVars->Decider.a_bre_com);
    fflush(stdout);
    printf("CalibrationVars.Decider.idle_speed = %f\n",
           CalibrationVars->Decider.idle_speed);
    fflush(stdout);
    printf("CalibrationVars.Decider.dist_wait2pilot = %f\n",
           CalibrationVars->Decider.dist_wait2pilot);
    fflush(stdout);
    printf("CalibrationVars.Decider.dist_wait2veh = %f\n",
           CalibrationVars->Decider.dist_wait2veh);
    fflush(stdout);
    printf("CalibrationVars.Decider.glosaAdp = %f\n",
           CalibrationVars->Decider.glosaAdp);
    fflush(stdout);
    printf("CalibrationVars.Decider.mrg = %f\n", CalibrationVars->Decider.mrg);
    fflush(stdout);
    printf("CalibrationVars.Decider.desRate = %f\n",
           CalibrationVars->Decider.desRate);
    fflush(stdout);
    printf("CalibrationVars.Decider.dIntxn = %f\n",
           CalibrationVars->Decider.dIntxn);
    fflush(stdout);
    printf("CalibrationVars.Decider.dMin = %f\n", CalibrationVars->Decider.dMin);
    fflush(stdout);
    printf("CalibrationVars.Decider.dec = %f\n", CalibrationVars->Decider.dec);
    fflush(stdout);
    printf("CalibrationVars.Decider.glosaAverageIndex = %f\n",
           CalibrationVars->Decider.glosaAverageIndex);
    fflush(stdout);
    printf("CalibrationVars.Decider.d_veh2endpoint = %f\n",
           CalibrationVars->Decider.d_veh2endpoint);
    fflush(stdout);
    printf("CalibrationVars.Decider.glosaVMin = %f\n",
           CalibrationVars->Decider.glosaVMin);
    fflush(stdout);
    printf("CalibrationVars.UrbanPlanner.jerkLimit = %f\n",
           CalibrationVars->UrbanPlanner.jerkLimit);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[14] == 1) {
    printf("CalibrationVars.AEBDecision.minGapIsTolerated = %f\n",
           CalibrationVars->AEBDecision.minGapIsTolerated);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[15] == 1) {
    printf("ChassisInfo.speed = %f\n", ChassisInfo_speed);
    fflush(stdout);
    printf("ChassisInfo.currentGear = %d\n", ChassisInfo_currentGear);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[16] == 1) {
    printf("GlobVars.AEBDecision.AEBActive = %d\n",
           GlobVars->AEBDecision.AEBActive);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[17] == 1) {
    printf("GlobVars.TrajPlanTurnAround.posCircle = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.posCircle[0]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.posCircle[1]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.posCircle2 = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.posCircle2[0]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.posCircle2[1]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.posCircle3 = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.posCircle3[0]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.posCircle3[1]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_start = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_start[0]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_start[1]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_mid1 = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1[0]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1[1]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1[2]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_mid1[3]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_mid2 = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2[0]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2[1]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2[2]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_mid2[3]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_mid1_rear = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1_rear[0]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1_rear[1]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid1_rear[2]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_mid1_rear[3]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_mid2_rear = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2_rear[0]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2_rear[1]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_mid2_rear[2]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_mid2_rear[3]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.pos_end = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanTurnAround.pos_end[0]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanTurnAround.pos_end[1]);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.laneCenterline = \t");
    fflush(stdout);
    for (i = 0; i < 7; i++) {
      if (i + 1 == 7) {
        printf("%f\n", GlobVars->TrajPlanTurnAround.laneCenterline[6]);
        fflush(stdout);
      } else {
        printf("%f\t", GlobVars->TrajPlanTurnAround.laneCenterline[i]);
        fflush(stdout);
      }
    }

    printf("GlobVars.TrajPlanTurnAround.dec_trunAround = %d\n",
           GlobVars->TrajPlanTurnAround.dec_trunAround);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.wait_turnAround = %d\n",
           GlobVars->TrajPlanTurnAround.wait_turnAround);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.typeOfTurnAround = %d\n",
           GlobVars->TrajPlanTurnAround.typeOfTurnAround);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.turnAroundState = %d\n",
           GlobVars->TrajPlanTurnAround.turnAroundState);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.targetLaneIndexOpposite = %d\n",
           GlobVars->TrajPlanTurnAround.targetLaneIndexOpposite);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.turnAroundActive = %d\n",
           GlobVars->TrajPlanTurnAround.turnAroundActive);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.reflineSend = %f\n",
           GlobVars->TrajPlanTurnAround.reflineSend);
    fflush(stdout);
    printf("GlobVars.TrajPlanTurnAround.reflineLend = %f\n",
           GlobVars->TrajPlanTurnAround.reflineLend);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[18] == 1) {
    printf("GlobVars.SpeedPlanAvoidPedestrian.dec_ped = %d\n",
           GlobVars->SpeedPlanAvoidPedestrian.dec_ped);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidPedestrian.wait_ped = %d\n",
           GlobVars->SpeedPlanAvoidPedestrian.wait_ped);
    fflush(stdout);
    printf("GlobVars.SpeedPlanTrafficLight.dec_fol_TrafficLight = %d\n",
           GlobVars->SpeedPlanTrafficLight.dec_fol_TrafficLight);
    fflush(stdout);
    printf("GlobVars.SpeedPlanTrafficLight.dec_bre_TrafficLight = %d\n",
           GlobVars->SpeedPlanTrafficLight.dec_bre_TrafficLight);
    fflush(stdout);
    printf("GlobVars.SpeedPlanTrafficLight.wait_TrafficLight = %d\n",
           GlobVars->SpeedPlanTrafficLight.wait_TrafficLight);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle = %d\n",
           GlobVars->SpeedPlanAvoidVehicle.dec_fol_AvoidVehicle);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle = %d\n",
           GlobVars->SpeedPlanAvoidVehicle.dec_bre_AvoidVehicle);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidVehicle.wait_AvoidVehicle = %d\n",
           GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle = %d\n",
           GlobVars->SpeedPlanAvoidOncomingVehicle.dec_avoidOncomingVehicle);
    fflush(stdout);
    printf("GlobVars.SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle = %d\n",
           GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[19] == 1) {
    printf("GlobVars.TrajPlanLaneChange.countLaneChange = %d\n",
           GlobVars->TrajPlanLaneChange.countLaneChange);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange.durationLaneChange = %d\n",
           GlobVars->TrajPlanLaneChange.durationLaneChange);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange.laneChangePath = \t");
    fflush(stdout);
    for (i = 0; i < 120; i++) {
      if (i + 1 == 120) {
        printf("%f\n", GlobVars->TrajPlanLaneChange.laneChangePath[119]);
        fflush(stdout);
      } else {
        printf("%f\t", GlobVars->TrajPlanLaneChange.laneChangePath[i]);
        fflush(stdout);
      }
    }

    printf("GlobVars.TrajPlanLaneChange.t_lc_traj = %f\n",
           GlobVars->TrajPlanLaneChange.t_lc_traj);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange.currentTargetLaneIndex = %d\n",
           GlobVars->TrajPlanLaneChange.currentTargetLaneIndex);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[20] == 1) {
    printf("GlobVars.TrajPlanLaneChange_RePlan.durationLaneChange_RePlan = %d\n",
           GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange_RePlan.para = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", GlobVars->TrajPlanLaneChange_RePlan.para[5]);
        fflush(stdout);
      } else {
        printf("%f\t", GlobVars->TrajPlanLaneChange_RePlan.para[i]);
        fflush(stdout);
      }
    }

    printf("GlobVars.TrajPlanLaneChange_RePlan.s_end = %f\n",
           GlobVars->TrajPlanLaneChange_RePlan.s_end);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange_RePlan.l_end = %f\n",
           GlobVars->TrajPlanLaneChange_RePlan.l_end);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange_RePlan.para1 = \t");
    fflush(stdout);
    for (i = 0; i < 5; i++) {
      if (i + 1 == 5) {
        printf("%f\n", GlobVars->TrajPlanLaneChange_RePlan.para1[4]);
        fflush(stdout);
      } else {
        printf("%f\t", GlobVars->TrajPlanLaneChange_RePlan.para1[i]);
        fflush(stdout);
      }
    }

    printf("GlobVars.TrajPlanLaneChange_RePlan.para2 = \t");
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanLaneChange_RePlan.para2[0]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanLaneChange_RePlan.para2[1]);
    fflush(stdout);
    printf("%f\t", GlobVars->TrajPlanLaneChange_RePlan.para2[2]);
    fflush(stdout);
    printf("%f\n", GlobVars->TrajPlanLaneChange_RePlan.para2[3]);
    fflush(stdout);
    printf("GlobVars.TrajPlanLaneChange_RePlan.para3 = %f\n",
           GlobVars->TrajPlanLaneChange_RePlan.para3);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[21] == 1) {
    printf("GlobVars.Decider.dec_start = %d\n", GlobVars->Decider.dec_start);
    fflush(stdout);
    printf("GlobVars.Decider.dir_start = %d\n", GlobVars->Decider.dir_start);
    fflush(stdout);
    printf("GlobVars.Decider.countLaneChangeDecider = %d\n",
           GlobVars->Decider.countLaneChangeDecider);
    fflush(stdout);
    printf("GlobVars.Decider.currentTargetLaneIndexDecider = %d\n",
           GlobVars->Decider.currentTargetLaneIndexDecider);
    fflush(stdout);
    printf("GlobVars.Decider.a_soll_pre = %f\n", GlobVars->Decider.a_soll_pre);
    fflush(stdout);
    printf("GlobVars.Decider.a_sollpre2traj = %f\n",
           GlobVars->Decider.a_sollpre2traj);
    fflush(stdout);
    printf("GlobVars.Decider.wait_pullover = %d\n",
           GlobVars->Decider.wait_pullover);
    fflush(stdout);
    printf("GlobVars.Decider.distBehindGoal = %f\n",
           GlobVars->Decider.distBehindGoal);
    fflush(stdout);
    printf("GlobVars.Decider.dec_follow = %d\n", GlobVars->Decider.dec_follow);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[22] == 1) {
    printf("GlobVars.SpeedPlanStopSign.wait_stopsign = %d\n",
           GlobVars->SpeedPlanStopSign.wait_stopsign);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[28] == 1) {
    printf("GlosaActive = %d\n", GlosaActive);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[23] == 1) {
    printf("LaneChangeActive = %d\n", LaneChangeActive);
    fflush(stdout);
    printf("LaneChangeInfo.d_veh2int = %f\n", LaneChangeInfo->d_veh2int);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneBehindDis = %f\n",
           LaneChangeInfo->leftLaneBehindDis);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneBehindVel = %f\n",
           LaneChangeInfo->leftLaneBehindVel);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneFrontDis = %f\n",
           LaneChangeInfo->leftLaneFrontDis);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneFrontVel = %f\n",
           LaneChangeInfo->leftLaneFrontVel);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneBehindDis = %f\n",
           LaneChangeInfo->rightLaneBehindDis);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneBehindVel = %f\n",
           LaneChangeInfo->rightLaneBehindVel);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneFrontDis = %f\n",
           LaneChangeInfo->rightLaneFrontDis);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneFrontVel = %f\n",
           LaneChangeInfo->rightLaneFrontVel);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneBehindLen = %f\n",
           LaneChangeInfo->leftLaneBehindLen);
    fflush(stdout);
    printf("LaneChangeInfo.leftLaneFrontLen = %f\n",
           LaneChangeInfo->leftLaneFrontLen);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneBehindLen = %f\n",
           LaneChangeInfo->rightLaneBehindLen);
    fflush(stdout);
    printf("LaneChangeInfo.rightLaneFrontLen = %f\n",
           LaneChangeInfo->rightLaneFrontLen);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[24] == 1) {
    printf("Parameters.turningRadius = %f\n", Parameters.turningRadius);
    fflush(stdout);
    printf("Parameters.w_veh = %f\n", Parameters.w_veh);
    fflush(stdout);
    printf("Parameters.l_veh = %f\n", Parameters.l_veh);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[28] == 1) {
    printf("PedestrianActive = %d\n", PedestrianActive);
    fflush(stdout);
    printf("PlannerLevel = %d\n", PlannerLevel);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[25] == 1) {
    printf("StopSignInfo.d_veh2stopline = %f\n", StopSignInfo.d_veh2stopline);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[28] == 1) {
    printf("TrafficLightActive = %d\n", TrafficLightActive);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[26] == 1) {
    printf("TrafficLightInfo.greenLight = %f\n", TrafficLightInfo->greenLight);
    fflush(stdout);
    printf("TrafficLightInfo.d_veh2stopline = %f\n",
           TrafficLightInfo->d_veh2stopline);
    fflush(stdout);
    printf("TrafficLightInfo.phase = \t");
    fflush(stdout);
    for (i = 0; i < 10; i++) {
      if (i + 1 == 10) {
        printf("%f\n", TrafficLightInfo->phase[9]);
        fflush(stdout);
      } else {
        printf("%f\t", TrafficLightInfo->phase[i]);
        fflush(stdout);
      }
    }
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[28] == 1) {
    printf("TurnAroundActive = %d\n", TurnAroundActive);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[27] == 1) {
    printf("TurnAroundInfo.numOfLanesOpposite = %d\n",
           TurnAroundInfo->numOfLanesOpposite);
    fflush(stdout);
    printf("TurnAroundInfo.widthOfLanesOpposite = \t");
    fflush(stdout);
    for (i = 0; i < 6; i++) {
      if (i + 1 == 6) {
        printf("%f\n", TurnAroundInfo->widthOfLanesOpposite[5]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->widthOfLanesOpposite[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.widthOfGap = %f\n", TurnAroundInfo->widthOfGap);
    fflush(stdout);
    printf("TurnAroundInfo.s_turnaround_border = %f\n",
           TurnAroundInfo->s_turnaround_border);
    fflush(stdout);
    printf("TurnAroundInfo.indexOfLaneOppositeCar= \t");
    fflush(stdout);
    for (i = 0; i < 20; i++) {
      if (i + 1 == 20) {
        printf("%d\n", TurnAroundInfo->indexOfLaneOppositeCar[19]);
        fflush(stdout);
      } else {
        printf("%d\t", TurnAroundInfo->indexOfLaneOppositeCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.speedOppositeCar = \t");
    fflush(stdout);
    for (i = 0; i < 20; i++) {
      if (i + 1 == 20) {
        printf("%f\n", TurnAroundInfo->speedOppositeCar[19]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->speedOppositeCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.posSOppositeCar = \t");
    fflush(stdout);
    for (i = 0; i < 20; i++) {
      if (i + 1 == 20) {
        printf("%f\n", TurnAroundInfo->posSOppositeCar[19]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->posSOppositeCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.indexOfLaneCodirectCar= \t");
    fflush(stdout);
    for (i = 0; i < 10; i++) {
      if (i + 1 == 10) {
        printf("%d\n", TurnAroundInfo->indexOfLaneCodirectCar[9]);
        fflush(stdout);
      } else {
        printf("%d\t", TurnAroundInfo->indexOfLaneCodirectCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.speedCodirectCar = \t");
    fflush(stdout);
    for (i = 0; i < 10; i++) {
      if (i + 1 == 10) {
        printf("%f\n", TurnAroundInfo->speedCodirectCar[9]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->speedCodirectCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.posSCodirectCar = \t");
    fflush(stdout);
    for (i = 0; i < 10; i++) {
      if (i + 1 == 10) {
        printf("%f\n", TurnAroundInfo->posSCodirectCar[9]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->posSCodirectCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.lengthOppositeCar = \t");
    fflush(stdout);
    for (i = 0; i < 20; i++) {
      if (i + 1 == 20) {
        printf("%f\n", TurnAroundInfo->lengthOppositeCar[19]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->lengthOppositeCar[i]);
        fflush(stdout);
      }
    }

    printf("TurnAroundInfo.lengthCodirectCar = \t");
    fflush(stdout);
    for (i = 0; i < 10; i++) {
      if (i + 1 == 10) {
        printf("%f\n", TurnAroundInfo->lengthCodirectCar[9]);
        fflush(stdout);
      } else {
        printf("%f\t", TurnAroundInfo->lengthCodirectCar[i]);
        fflush(stdout);
      }
    }
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[28] == 1) {
    printf("VehicleCrossingActive = %d\n", VehicleCrossingActive);
    fflush(stdout);
    printf("VehicleOncomingActive = %d\n", VehicleOncomingActive);
    fflush(stdout);
  }
}

/*
 * Arguments    : const double Trajectory_traj_s[80]
 *                const double Trajectory_traj_l[80]
 *                const double Trajectory_traj_psi[80]
 *                const double Trajectory_traj_vs[80]
 *                const double Trajectory_traj_vl[80]
 *                const double Trajectory_traj_omega[80]
 *                short Trajectory_planning_states
 *                const struct1_T *Decision
 *                const struct2_T *Refline
 *                const TypeCalibrationVars *CalibrationVars
 * Return Type  : void
 */
static void logOutput(const double Trajectory_traj_s[80], const double
                      Trajectory_traj_l[80], const double Trajectory_traj_psi[80],
                      const double Trajectory_traj_vs[80], const double
                      Trajectory_traj_vl[80], const double
                      Trajectory_traj_omega[80], short
                      Trajectory_planning_states, const struct1_T *Decision,
                      const struct2_T *Refline, const TypeCalibrationVars
                      *CalibrationVars)
{
  int i;
  if (CalibrationVars->UrbanPlanner.logTrigger[29] == 1) {
    printf("Decision.AEBactive = %d\n", Decision->AEBactive);
    fflush(stdout);
    printf("Decision.TargetGear = %d\n", Decision->TargetGear);
    fflush(stdout);
    printf("Decision.states = %d\n", Decision->states);
    fflush(stdout);
    printf("Decision.LaneChange = %d\n", Decision->LaneChange);
    fflush(stdout);
    printf("Decision.SlowDown = %d\n", Decision->SlowDown);
    fflush(stdout);
    printf("Decision.TargetSpeed = %f\n", Decision->TargetSpeed);
    fflush(stdout);
    printf("Decision.Wait = %d\n", Decision->Wait);
    fflush(stdout);
    printf("Decision.WaitDistance = %f\n", Decision->WaitDistance);
    fflush(stdout);
    printf("Decision.Start = %d\n", Decision->Start);
    fflush(stdout);
    printf("Decision.a_soll = %f\n", Decision->a_soll);
    fflush(stdout);
    printf("Decision.PedestrianState = %d\n", Decision->PedestrianState);
    fflush(stdout);
    printf("Decision.TrafficLightState = %d\n", Decision->TrafficLightState);
    fflush(stdout);
    printf("Decision.VehicleCrossingState = %d\n",
           Decision->VehicleCrossingState);
    fflush(stdout);
    printf("Decision.VehicleOncomingState = %d\n",
           Decision->VehicleOncomingState);
    fflush(stdout);
    printf("Decision.StopSignState = %d\n", Decision->StopSignState);
    fflush(stdout);
    printf("Decision.FollowState = %d\n", Decision->FollowState);
    fflush(stdout);
    printf("Decision.PullOverState = %d\n", Decision->PullOverState);
    fflush(stdout);
    printf("Decision.TurnAroundState = %d\n", Decision->TurnAroundState);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[30] == 1) {
    printf("Refline.NumRefLaneTurnAround = %d\n", Refline->NumRefLaneTurnAround);
    fflush(stdout);
    printf("Refline.SRefLaneTurnAround = \t");
    fflush(stdout);
    for (i = 0; i < 100; i++) {
      if (i + 1 == 100) {
        printf("%f\n", Refline->SRefLaneTurnAround[99]);
        fflush(stdout);
      } else {
        printf("%f\t", Refline->SRefLaneTurnAround[i]);
        fflush(stdout);
      }
    }

    printf("Refline.LRefLaneTurnAround = \t");
    fflush(stdout);
    for (i = 0; i < 100; i++) {
      if (i + 1 == 100) {
        printf("%f\n", Refline->LRefLaneTurnAround[99]);
        fflush(stdout);
      } else {
        printf("%f\t", Refline->LRefLaneTurnAround[i]);
        fflush(stdout);
      }
    }

    printf("Refline.TurnAroundReflineState = %d\n",
           Refline->TurnAroundReflineState);
    fflush(stdout);
  }

  if (CalibrationVars->UrbanPlanner.logTrigger[31] == 1) {
    printf("Trajectory.traj_s = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_s[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_s[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.traj_l = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_l[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_l[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.traj_psi = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_psi[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_psi[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.traj_vs = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_vs[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_vs[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.traj_vl = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_vl[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_vl[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.traj_omega = \t");
    fflush(stdout);
    for (i = 0; i < 80; i++) {
      if (i + 1 == 80) {
        printf("%f\n", Trajectory_traj_omega[79]);
        fflush(stdout);
      } else {
        printf("%f\t", Trajectory_traj_omega[i]);
        fflush(stdout);
      }
    }

    printf("Trajectory.planning_states = %d\n", Trajectory_planning_states);
    fflush(stdout);
  }
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
  int k;
  k = 0;
  int exitg1;
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
  double *x_data;
  double *xwork_data;
  int j;
  int *idx_data;
  int *iwork_data;
  xwork_data = xwork->data;
  iwork_data = iwork->data;
  x_data = x->data;
  idx_data = idx->data;
  if (nq != 0) {
    int iout;
    int n_tmp;
    int p;
    int q;
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    int exitg1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
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
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    int tailOffset;
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
  int k;
  if (!rtIsNaN(x[0])) {
    *idx = 1;
  } else {
    boolean_T exitg1;
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
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 5; k++) {
      double d;
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
    int rtemp;
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
 * Arguments    : const double x[5]
 *                const double y[5]
 *                double v_breaks[5]
 *                double v_coefs[16]
 * Return Type  : void
 */
static void pchip(const double x[5], const double y[5], double v_breaks[5],
                  double v_coefs[16])
{
  double d;
  double d1;
  double del_idx_0;
  double del_idx_1;
  double del_idx_2;
  double del_idx_3;
  double dzzdx;
  double h_idx_0;
  double h_idx_1;
  double h_idx_2;
  double signd1;
  double slopes_idx_1;
  double slopes_idx_2;
  double slopes_idx_3;
  double w2;
  int i;
  d = x[1] - x[0];
  h_idx_0 = d;
  del_idx_0 = (y[1] - y[0]) / d;
  d = x[2] - x[1];
  h_idx_1 = d;
  del_idx_1 = (y[2] - y[1]) / d;
  d = x[3] - x[2];
  h_idx_2 = d;
  del_idx_2 = (y[3] - y[2]) / d;
  d = x[4] - x[3];
  del_idx_3 = (y[4] - y[3]) / d;
  signd1 = 2.0 * h_idx_1 + h_idx_0;
  w2 = h_idx_1 + 2.0 * h_idx_0;
  slopes_idx_1 = 0.0;
  dzzdx = del_idx_0 * del_idx_1;
  if (!rtIsNaN(dzzdx)) {
    if (dzzdx < 0.0) {
      dzzdx = -1.0;
    } else {
      dzzdx = (dzzdx > 0.0);
    }
  }

  if (dzzdx > 0.0) {
    slopes_idx_1 = (signd1 + w2) / (signd1 / del_idx_0 + w2 / del_idx_1);
  }

  signd1 = 2.0 * h_idx_2 + h_idx_1;
  w2 = h_idx_2 + 2.0 * h_idx_1;
  slopes_idx_2 = 0.0;
  dzzdx = del_idx_1 * del_idx_2;
  if (!rtIsNaN(dzzdx)) {
    if (dzzdx < 0.0) {
      dzzdx = -1.0;
    } else {
      dzzdx = (dzzdx > 0.0);
    }
  }

  if (dzzdx > 0.0) {
    slopes_idx_2 = (signd1 + w2) / (signd1 / del_idx_1 + w2 / del_idx_2);
  }

  signd1 = 2.0 * d + h_idx_2;
  w2 = d + 2.0 * h_idx_2;
  slopes_idx_3 = 0.0;
  dzzdx = del_idx_2 * del_idx_3;
  if (!rtIsNaN(dzzdx)) {
    if (dzzdx < 0.0) {
      dzzdx = -1.0;
    } else {
      dzzdx = (dzzdx > 0.0);
    }
  }

  if (dzzdx > 0.0) {
    slopes_idx_3 = (signd1 + w2) / (signd1 / del_idx_2 + w2 / del_idx_3);
  }

  w2 = ((2.0 * h_idx_0 + h_idx_1) * del_idx_0 - h_idx_0 * del_idx_1) / (h_idx_0
    + h_idx_1);
  if (rtIsNaN(del_idx_0)) {
    signd1 = rtNaN;
  } else if (del_idx_0 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_0 > 0.0);
  }

  if (rtIsNaN(w2)) {
    d1 = rtNaN;
  } else if (w2 < 0.0) {
    d1 = -1.0;
  } else {
    d1 = (w2 > 0.0);
  }

  if (d1 != signd1) {
    w2 = 0.0;
  } else {
    if (rtIsNaN(del_idx_1)) {
      d1 = rtNaN;
    } else if (del_idx_1 < 0.0) {
      d1 = -1.0;
    } else {
      d1 = (del_idx_1 > 0.0);
    }

    if ((signd1 != d1) && (fabs(w2) > fabs(3.0 * del_idx_0))) {
      w2 = 3.0 * del_idx_0;
    }
  }

  for (i = 0; i < 5; i++) {
    v_breaks[i] = x[i];
  }

  dzzdx = (del_idx_0 - w2) / h_idx_0;
  signd1 = (slopes_idx_1 - del_idx_0) / h_idx_0;
  v_coefs[0] = (signd1 - dzzdx) / h_idx_0;
  v_coefs[4] = 2.0 * dzzdx - signd1;
  v_coefs[8] = w2;
  v_coefs[12] = y[0];
  dzzdx = (del_idx_1 - slopes_idx_1) / h_idx_1;
  signd1 = (slopes_idx_2 - del_idx_1) / h_idx_1;
  v_coefs[1] = (signd1 - dzzdx) / h_idx_1;
  v_coefs[5] = 2.0 * dzzdx - signd1;
  v_coefs[9] = slopes_idx_1;
  v_coefs[13] = y[1];
  dzzdx = (del_idx_2 - slopes_idx_2) / h_idx_2;
  signd1 = (slopes_idx_3 - del_idx_2) / h_idx_2;
  v_coefs[2] = (signd1 - dzzdx) / h_idx_2;
  v_coefs[6] = 2.0 * dzzdx - signd1;
  v_coefs[10] = slopes_idx_2;
  v_coefs[14] = y[2];
  dzzdx = (del_idx_3 - slopes_idx_3) / d;
  w2 = ((2.0 * d + h_idx_2) * del_idx_3 - d * del_idx_2) / (d + h_idx_2);
  if (rtIsNaN(del_idx_3)) {
    signd1 = rtNaN;
  } else if (del_idx_3 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_3 > 0.0);
  }

  if (rtIsNaN(w2)) {
    d1 = rtNaN;
  } else if (w2 < 0.0) {
    d1 = -1.0;
  } else {
    d1 = (w2 > 0.0);
  }

  if (d1 != signd1) {
    w2 = 0.0;
  } else {
    if (rtIsNaN(del_idx_2)) {
      d1 = rtNaN;
    } else if (del_idx_2 < 0.0) {
      d1 = -1.0;
    } else {
      d1 = (del_idx_2 > 0.0);
    }

    if ((signd1 != d1) && (fabs(w2) > fabs(3.0 * del_idx_3))) {
      w2 = 3.0 * del_idx_3;
    }
  }

  signd1 = (w2 - del_idx_3) / d;
  v_coefs[3] = (signd1 - dzzdx) / d;
  v_coefs[7] = 2.0 * dzzdx - signd1;
  v_coefs[11] = slopes_idx_3;
  v_coefs[15] = y[3];
}

/*
 * Arguments    : const double pp_breaks[5]
 *                const double pp_coefs[12]
 *                const double x[50]
 *                double v[50]
 * Return Type  : void
 */
static void ppval(const double pp_breaks[5], const double pp_coefs[12], const
                  double x[50], double v[50])
{
  int ix;
  for (ix = 0; ix < 50; ix++) {
    double xloc;
    if (rtIsNaN(x[ix])) {
      xloc = rtNaN;
    } else {
      int high_i;
      int low_i;
      int low_ip1;
      low_i = 0;
      low_ip1 = 2;
      high_i = 5;
      while (high_i > low_ip1) {
        int mid_i;
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[ix] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      xloc = x[ix] - pp_breaks[low_i];
      xloc = xloc * (xloc * pp_coefs[low_i] + pp_coefs[low_i + 4]) +
        pp_coefs[low_i + 8];
    }

    v[ix] = xloc;
  }
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }

    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }

    y = atan2(i, i1);
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
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
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
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = rtNaN;
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    double q;
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
 * Arguments    : double d
 *                double v0
 *                const double tl[10]
 *                double vMax
 *                double vMin
 *                double a0
 *                double dec
 *                double mrg
 *                double desRate
 *                double dMin
 *                double *vOpt
 *                double *vgMin
 *                double *vgMax
 * Return Type  : void
 */
static void scen_glosa(double d, double v0, const double tl[10], double vMax,
  double vMin, double a0, double dec, double mrg, double desRate, double dMin,
  double *vOpt, double *vgMin, double *vgMax)
{
  double tp[132];
  double a;
  double s;
  double ts;
  double v1;
  int iidx[132];
  int b_i;
  int exitg2;
  int i;
  signed char vList[132];
  boolean_T exitg1;
  boolean_T guard1 = false;
  *vgMin = -1.0;
  *vgMax = -1.0;

  /*  限速33m/s, 离散间隔0.25m/s */
  ts = v0 * v0;
  for (i = 0; i < 132; i++) {
    vList[i] = 0;
    v1 = ((double)i + 1.0) * 0.25;
    a = a0;
    if (v1 < v0) {
      a = -a0;
    }

    s = (v1 * v1 - ts) / a / 2.0;
    if (s <= d) {
      tp[i] = (v1 - v0) / a + (d - s) / v1;
    } else {
      tp[i] = (sqrt(ts + 2.0 * a * d) - v0) / a;
    }
  }

  b_sort(tp, iidx);
  ts = 0.0;
  s = 1.0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 132)) {
    do {
      exitg2 = 0;
      v1 = tl[(int)s - 1];
      if (v1 != 0.0) {
        a = ts + fabs(v1);
        if (a <= tp[i]) {
          ts = a;
          s++;
        } else {
          exitg2 = 1;
        }
      } else {
        exitg2 = 1;
      }
    } while (exitg2 == 0);

    if (v1 == 0.0) {
      exitg1 = true;
    } else {
      if (v1 > 0.0) {
        vList[iidx[i] - 1] = 1;
      }

      i++;
    }
  }

  /*  去掉限速外车速 */
  b_i = (int)floor(vMin / 0.25);
  if (b_i - 1 >= 0) {
    memset(&vList[0], 0, (unsigned int)b_i * sizeof(signed char));
  }

  ts = floor(vMax / 0.25);
  b_i = (int)((1.0 - (ts + 1.0)) + 132.0);
  for (i = 0; i < b_i; i++) {
    vList[(int)((ts + 1.0) + (double)i) - 1] = 0;
  }

  /*      close;plot([1:vn]*dv, vList, 'linewidth', 2); */
  /*  ----------------------- 找最佳车速与区间 ---------------------------- */
  /*  可行车速需要可行车速区间大小超阈值，设0为不启用 */
  /*  情况1，如果当前车速在可行区间 */
  s = floor(v0 / 0.25);
  guard1 = false;
  if ((s > 0.0) && (vList[(int)s - 1] == 1)) {
    int j;
    i = (int)s;
    while ((i > 1) && (vList[i - 2] == 1)) {
      i--;
    }

    j = (int)s;
    while ((j < 132) && (vList[j] == 1)) {
      j++;
    }

    /*          if k - i >= mrg && j - k >= mrg */
    /*              vOpt = v0; */
    /*              vgMin = i*dv; */
    /*              vgMax = j*dv; */
    /*              return */
    /*          else */
    b_i = j - i;
    if (b_i >= mrg) {
      *vgMin = (double)i * 0.25;
      *vOpt = *vgMin + (double)b_i * desRate * 0.25;
      *vgMax = (double)j * 0.25;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    /*  情况2，另找可行区间 */
    s = fmin(132.0, ts);
    do {
      exitg2 = 0;
      if (s >= 1.0) {
        if (vList[(int)s - 1] == 0) {
          s--;
        } else {
          i = (int)s;
          while ((i > 1) && (vList[i - 2] == 1)) {
            i--;
          }

          b_i = (int)s - i;
          if (b_i >= mrg) {
            *vgMin = (double)i * 0.25;
            *vOpt = *vgMin + (double)b_i * desRate * 0.25;
            *vgMax = s * 0.25;
            exitg2 = 1;
          } else {
            s--;
          }
        }
      } else {
        /*  不能通行的速度规划 */
        if (d > dMin) {
          ts = fmin(sqrt(2.0 * dec * (d - dMin)), vMax);
          if (v0 >= ts) {
            *vOpt = ts;
          } else {
            *vOpt = -1.0;
          }
        } else {
          *vOpt = 0.0;
        }

        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
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
  double *vwork_data;
  double *x_data;
  double *xwork_data;
  int b;
  int b_b;
  int dim;
  int i;
  int i1;
  int j;
  int k;
  int vlen;
  int vstride;
  int *iidx_data;
  int *iwork_data;
  x_data = x->data;
  dim = 0;
  if (x->size[0] != 1) {
    dim = -1;
  }

  if (dim + 2 <= 1) {
    i = x->size[0];
  } else {
    i = 1;
  }

  vlen = i - 1;
  emxInit_real_T(&vwork, 1);
  i1 = vwork->size[0];
  vwork->size[0] = i;
  emxEnsureCapacity_real_T(vwork, i1);
  vwork_data = vwork->data;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= x->size[0];
  }

  emxInit_int32_T(&iidx);
  emxInit_int32_T(&iwork);
  emxInit_real_T(&xwork, 1);
  emxInit_int32_T(&b_iwork);
  emxInit_real_T(&b_xwork, 1);
  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }

    i = iidx->size[0];
    iidx->size[0] = vwork->size[0];
    emxEnsureCapacity_int32_T(iidx, i);
    iidx_data = iidx->data;
    dim = vwork->size[0];
    for (i = 0; i < dim; i++) {
      iidx_data[i] = 0;
    }

    if (vwork->size[0] != 0) {
      double x4[4];
      int idx4[4];
      int bLen2;
      int i2;
      int i3;
      int i4;
      int iidx_tmp;
      int n;
      int nNonNaN;
      n = vwork->size[0];
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      i = iwork->size[0];
      iwork->size[0] = vwork->size[0];
      emxEnsureCapacity_int32_T(iwork, i);
      iwork_data = iwork->data;
      dim = vwork->size[0];
      for (i = 0; i < dim; i++) {
        iwork_data[i] = 0;
      }

      i = xwork->size[0];
      xwork->size[0] = vwork->size[0];
      emxEnsureCapacity_real_T(xwork, i);
      xwork_data = xwork->data;
      dim = vwork->size[0];
      for (i = 0; i < dim; i++) {
        xwork_data[i] = 0.0;
      }

      bLen2 = 0;
      dim = 0;
      for (k = 0; k < n; k++) {
        if (rtIsNaN(vwork_data[k])) {
          iidx_tmp = (n - bLen2) - 1;
          iidx_data[iidx_tmp] = k + 1;
          xwork_data[iidx_tmp] = vwork_data[k];
          bLen2++;
        } else {
          dim++;
          idx4[dim - 1] = k + 1;
          x4[dim - 1] = vwork_data[k];
          if (dim == 4) {
            double d;
            double d1;
            signed char b_i1;
            signed char b_i2;
            signed char b_i3;
            signed char b_i4;
            dim = k - bLen2;
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
                b_i1 = (signed char)i1;
                b_i2 = (signed char)i2;
                b_i3 = (signed char)i3;
                b_i4 = (signed char)i4;
              } else if (d <= x4[i4 - 1]) {
                b_i1 = (signed char)i1;
                b_i2 = (signed char)i3;
                b_i3 = (signed char)i2;
                b_i4 = (signed char)i4;
              } else {
                b_i1 = (signed char)i1;
                b_i2 = (signed char)i3;
                b_i3 = (signed char)i4;
                b_i4 = (signed char)i2;
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[i2 - 1] <= d1) {
                  b_i1 = (signed char)i3;
                  b_i2 = (signed char)i1;
                  b_i3 = (signed char)i2;
                  b_i4 = (signed char)i4;
                } else {
                  b_i1 = (signed char)i3;
                  b_i2 = (signed char)i1;
                  b_i3 = (signed char)i4;
                  b_i4 = (signed char)i2;
                }
              } else {
                b_i1 = (signed char)i3;
                b_i2 = (signed char)i4;
                b_i3 = (signed char)i1;
                b_i4 = (signed char)i2;
              }
            }

            iidx_data[dim - 3] = idx4[b_i1 - 1];
            iidx_data[dim - 2] = idx4[b_i2 - 1];
            iidx_data[dim - 1] = idx4[b_i3 - 1];
            iidx_data[dim] = idx4[b_i4 - 1];
            vwork_data[dim - 3] = x4[b_i1 - 1];
            vwork_data[dim - 2] = x4[b_i2 - 1];
            vwork_data[dim - 1] = x4[b_i3 - 1];
            vwork_data[dim] = x4[b_i4 - 1];
            dim = 0;
          }
        }
      }

      i3 = vwork->size[0] - bLen2;
      if (dim > 0) {
        signed char perm[4];
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (dim == 1) {
          perm[0] = 1;
        } else if (dim == 2) {
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

        i = (unsigned char)dim;
        for (k = 0; k < i; k++) {
          iidx_tmp = perm[k] - 1;
          i1 = (i3 - dim) + k;
          iidx_data[i1] = idx4[iidx_tmp];
          vwork_data[i1] = x4[iidx_tmp];
        }
      }

      dim = bLen2 >> 1;
      for (k = 0; k < dim; k++) {
        i1 = i3 + k;
        i2 = iidx_data[i1];
        iidx_tmp = (n - k) - 1;
        iidx_data[i1] = iidx_data[iidx_tmp];
        iidx_data[iidx_tmp] = i2;
        vwork_data[i1] = xwork_data[iidx_tmp];
        vwork_data[iidx_tmp] = xwork_data[i1];
      }

      if ((bLen2 & 1) != 0) {
        i1 = i3 + dim;
        vwork_data[i1] = xwork_data[i1];
      }

      nNonNaN = vwork->size[0] - bLen2;
      i1 = 2;
      if (nNonNaN > 1) {
        if (vwork->size[0] >= 256) {
          int nBlocks;
          nBlocks = nNonNaN >> 8;
          if (nBlocks > 0) {
            for (b = 0; b < nBlocks; b++) {
              double c_xwork[256];
              int c_iwork[256];
              i4 = (b << 8) - 1;
              for (b_b = 0; b_b < 6; b_b++) {
                n = 1 << (b_b + 2);
                bLen2 = n << 1;
                i = 256 >> (b_b + 3);
                for (k = 0; k < i; k++) {
                  i2 = (i4 + k * bLen2) + 1;
                  for (i1 = 0; i1 < bLen2; i1++) {
                    dim = i2 + i1;
                    c_iwork[i1] = iidx_data[dim];
                    c_xwork[i1] = vwork_data[dim];
                  }

                  i3 = 0;
                  i1 = n;
                  dim = i2 - 1;
                  int exitg1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (c_xwork[i3] <= c_xwork[i1]) {
                      iidx_data[dim] = c_iwork[i3];
                      vwork_data[dim] = c_xwork[i3];
                      if (i3 + 1 < n) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx_data[dim] = c_iwork[i1];
                      vwork_data[dim] = c_xwork[i1];
                      if (i1 + 1 < bLen2) {
                        i1++;
                      } else {
                        dim -= i3;
                        for (i1 = i3 + 1; i1 <= n; i1++) {
                          iidx_tmp = dim + i1;
                          iidx_data[iidx_tmp] = c_iwork[i1 - 1];
                          vwork_data[iidx_tmp] = c_xwork[i1 - 1];
                        }

                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }

            i1 = nBlocks << 8;
            dim = nNonNaN - i1;
            if (dim > 0) {
              merge_block(iidx, vwork, i1, dim, 2, iwork, xwork);
              xwork_data = xwork->data;
              iwork_data = iwork->data;
            }

            i1 = 8;
          }
        }

        dim = iwork->size[0];
        i = b_iwork->size[0];
        b_iwork->size[0] = iwork->size[0];
        emxEnsureCapacity_int32_T(b_iwork, i);
        iidx_data = b_iwork->data;
        for (i = 0; i < dim; i++) {
          iidx_data[i] = iwork_data[i];
        }

        i = b_xwork->size[0];
        b_xwork->size[0] = xwork->size[0];
        emxEnsureCapacity_real_T(b_xwork, i);
        vwork_data = b_xwork->data;
        dim = xwork->size[0];
        for (i = 0; i < dim; i++) {
          vwork_data[i] = xwork_data[i];
        }

        merge_block(iidx, vwork, 0, nNonNaN, i1, b_iwork, b_xwork);
        vwork_data = vwork->data;
      }
    }

    for (k = 0; k <= vlen; k++) {
      x_data[j + k * vstride] = vwork_data[k];
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
    int b_iac;
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
    if (x_data[k] != 0) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/*
 * ,
 * ,
 *
 * Arguments    : TypeBasicsInfo *BasicsInfo
 *                const TypeChassisInfo *ChassisInfo
 *                const TypeLaneChangeInfo *LaneChangeInfo
 *                const TypeAvoMainRoVehInfo *AvoMainRoVehInfo
 *                const TypeAvoPedInfo *AvoPedInfo
 *                const TypeTrafficLightInfo *TrafficLightInfo
 *                const TypeAvoOncomingVehInfo *AvoOncomingVehInfo
 *                const TypeAvoFailVehInfo *AvoFailVehInfo
 *                const TypeTurnAroundInfo *TurnAroundInfo
 *                TypeStopSignInfo *StopSignInfo
 *                short LaneChangeActive
 *                short PedestrianActive
 *                short TrafficLightActive
 *                short VehicleCrossingActive
 *                short VehicleOncomingActive
 *                short TurnAroundActive
 *                short GlosaActive
 *                short PlannerLevel
 *                TypeGlobVars *GlobVars
 *                const TypeCalibrationVars *CalibrationVars
 *                const TypeParameters *Parameters
 *                struct0_T *Trajectory
 *                struct1_T *Decision
 *                struct2_T *Refline
 * Return Type  : void
 */
void UrbanPlanner(TypeBasicsInfo *BasicsInfo, const TypeChassisInfo *ChassisInfo,
                  const TypeLaneChangeInfo *LaneChangeInfo, const
                  TypeAvoMainRoVehInfo *AvoMainRoVehInfo, const TypeAvoPedInfo
                  *AvoPedInfo, const TypeTrafficLightInfo *TrafficLightInfo,
                  const TypeAvoOncomingVehInfo *AvoOncomingVehInfo, const
                  TypeAvoFailVehInfo *AvoFailVehInfo, const TypeTurnAroundInfo
                  *TurnAroundInfo, TypeStopSignInfo *StopSignInfo, short
                  LaneChangeActive, short PedestrianActive, short
                  TrafficLightActive, short VehicleCrossingActive, short
                  VehicleOncomingActive, short TurnAroundActive, short
                  GlosaActive, short PlannerLevel, TypeGlobVars *GlobVars, const
                  TypeCalibrationVars *CalibrationVars, const TypeParameters
                  *Parameters, struct0_T *Trajectory, struct1_T *Decision,
                  struct2_T *Refline)
{
  static const int iv[2] = { 1, 6 };

  static const int iv1[2] = { 1, 5 };

  double stopdistance_array[8];
  double b_AvoOncomingVehInfo[6];
  double b_s_veh1[6];
  double b_s_vehapostrophe[6];
  double s_veh1[6];
  double s_vehapostrophe[6];
  double FailLaneFrontDis[5];
  double AvoMainRoVehInfo_data[4];
  double b_a_soll_Fail[3];
  double b_a_soll_SpeedPlanAvoidPedestri[2];
  double BasicsInfo_tmp;
  double CurrentLaneFrontDis;
  double TargetLaneFrontDisAvoidVehicle;
  double TargetLaneFrontLenAvoidVehicle;
  double TargetLaneFrontVelAvoidVehicle;
  double a_soll;
  double a_soll_ACC;
  double a_soll_Fail;
  double a_soll_SpeedPlanAvoidPedestrian;
  double a_soll_TrafficLightActive;
  double a_soll_veh2goal;
  double c_a_soll_SpeedPlanAvoidOncoming;
  double d_veh2crossStopline;
  double d_veh2stopline_ped;
  double d_veh2trafficStopline;
  double d_veh2waitingArea;
  double pos_l_CurrentLane;
  double pos_s;
  double speed_tmp_tmp;
  double t_TargetLaneFront2int_idx_0;
  double t_TargetLaneFront2int_idx_1;
  double t_TargetLaneFront2int_idx_2;
  int AvoMainRoVehInfo_size[2];
  int b_tmp_size[2];
  int tmp_size[2];
  int b_wait;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int trueCount;
  short AEBActive;
  short BackupTargetLaneIndex;
  short CurrentLaneIndex;
  short DurationLaneChange_RePlan;
  short TargetGear;
  short TargetLaneIndex;
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
  boolean_T unnamed_idx_0;
  boolean_T unnamed_idx_1;
  boolean_T unnamed_idx_2;
  boolean_T unnamed_idx_3;

  /* 打印入参 */
  logInput(BasicsInfo->currentLaneFrontDis, BasicsInfo->currentLaneFrontVel,
           BasicsInfo->currentLaneFrontLen, BasicsInfo->pos_s, BasicsInfo->pos_l,
           BasicsInfo->pos_psi, BasicsInfo->pos_l_CurrentLane,
           BasicsInfo->currentLaneIndex, BasicsInfo->widthOfLanes,
           BasicsInfo->targetLaneIndex, BasicsInfo->v_max,
           BasicsInfo->goalLaneIndex, BasicsInfo->d_veh2goal,
           BasicsInfo->sampleTime, ChassisInfo->speed, ChassisInfo->currentGear,
           LaneChangeInfo, AvoMainRoVehInfo, AvoPedInfo, TrafficLightInfo,
           AvoOncomingVehInfo, AvoFailVehInfo, TurnAroundInfo, *StopSignInfo,
           LaneChangeActive, PedestrianActive, TrafficLightActive,
           VehicleCrossingActive, VehicleOncomingActive, TurnAroundActive,
           GlosaActive, PlannerLevel, GlobVars, CalibrationVars, *Parameters);

  /* , */
  /* 入参 */
  pos_s = BasicsInfo->pos_s;
  if ((BasicsInfo->d_veh2goal < 60.0) && (BasicsInfo->goalLaneIndex ==
       BasicsInfo->currentLaneIndex) && (intnnz(BasicsInfo->widthOfLanes) ==
       BasicsInfo->goalLaneIndex)) {
    pos_l_CurrentLane = BasicsInfo->pos_l_CurrentLane - ((0.5 *
      BasicsInfo->widthOfLanes[BasicsInfo->goalLaneIndex - 1] - 0.5 *
      Parameters->w_veh) - 0.2);
  } else {
    pos_l_CurrentLane = BasicsInfo->pos_l_CurrentLane;
  }

  if (BasicsInfo->d_veh2goal < 60.0) {
    trueCount = 0;
    for (i = 0; i < 6; i++) {
      if (BasicsInfo->widthOfLanes[i] > 0.0) {
        trueCount++;
      }
    }

    if (trueCount > BasicsInfo->goalLaneIndex) {
      TargetLaneIndex = BasicsInfo->goalLaneIndex;
    } else {
      TargetLaneIndex = (short)trueCount;
    }
  } else {
    TargetLaneIndex = BasicsInfo->targetLaneIndex;
  }

  CurrentLaneIndex = BasicsInfo->currentLaneIndex;

  /*  d_veh2int = BasicsInform.d_veh2int; */
  speed_tmp_tmp = ChassisInfo->speed;

  /*  故障车所在车道序号,数组大小5 */
  DurationLaneChange_RePlan =
    GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan;

  /*  TargetLaneBehindDisAvoidVehicle = AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle; */
  /*  TargetLaneBehindVelAvoidVehicle = AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle; */
  /*  TargetLaneFrontDisAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle; */
  /*  TargetLaneFrontVelAvoidVehicle = AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle; */
  /*  AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的前车数据放入数组，默认值为200 */
  /*  AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle“大小为4的数组”，AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle(i)对应AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle(i)，默认值为20 */
  minimum(AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle, &a_soll_veh2goal,
          &b_wait);
  TargetLaneFrontDisAvoidVehicle =
    AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[b_wait - 1];
  TargetLaneFrontVelAvoidVehicle =
    AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[b_wait - 1];
  TargetLaneFrontLenAvoidVehicle =
    AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[b_wait - 1];
  a_soll_TrafficLightActive = 2.0 * AvoMainRoVehInfo->d_veh2converge /
    (ChassisInfo->speed + 2.2204460492503131E-16);
  trueCount = 0;
  c_a_soll_SpeedPlanAvoidOncoming = (AvoMainRoVehInfo->d_veh2converge -
    AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[0]) /
    (AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[0] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_0 = c_a_soll_SpeedPlanAvoidOncoming;
  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[0] < 200.0);
  unnamed_idx_0 = unnamed_idx_3;
  b_unnamed_idx_3 = (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive);
  b_unnamed_idx_0 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount = 1;
  }

  c_a_soll_SpeedPlanAvoidOncoming = (AvoMainRoVehInfo->d_veh2converge -
    AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[1]) /
    (AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[1] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_1 = c_a_soll_SpeedPlanAvoidOncoming;
  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[1] < 200.0);
  unnamed_idx_1 = unnamed_idx_3;
  b_unnamed_idx_3 = (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive);
  b_unnamed_idx_1 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  c_a_soll_SpeedPlanAvoidOncoming = (AvoMainRoVehInfo->d_veh2converge -
    AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[2]) /
    (AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[2] +
     2.2204460492503131E-16);
  t_TargetLaneFront2int_idx_2 = c_a_soll_SpeedPlanAvoidOncoming;
  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[2] < 200.0);
  unnamed_idx_2 = unnamed_idx_3;
  b_unnamed_idx_3 = (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive);
  b_unnamed_idx_2 = b_unnamed_idx_3;
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  c_a_soll_SpeedPlanAvoidOncoming = (AvoMainRoVehInfo->d_veh2converge -
    AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[3]) /
    (AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[3] +
     2.2204460492503131E-16);
  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[3] < 200.0);
  b_unnamed_idx_3 = (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive);
  if (unnamed_idx_3 && b_unnamed_idx_3) {
    trueCount++;
  }

  i = 0;

  /* , */
  if (unnamed_idx_0 && b_unnamed_idx_0) {
    tmp_data[0] = 1;
    i = 1;
  }

  if (unnamed_idx_1 && b_unnamed_idx_1) {
    tmp_data[i] = 2;
    i++;
  }

  if (unnamed_idx_2 && b_unnamed_idx_2) {
    tmp_data[i] = 3;
    i++;
  }

  if (unnamed_idx_3 && b_unnamed_idx_3) {
    tmp_data[i] = 4;
  }

  /* , */
  if (trueCount != 0) {
    AvoMainRoVehInfo_size[0] = 1;
    AvoMainRoVehInfo_size[1] = trueCount;
    for (b_wait = 0; b_wait < trueCount; b_wait++) {
      AvoMainRoVehInfo_data[b_wait] =
        AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[tmp_data[b_wait] - 1];
    }

    b_minimum(AvoMainRoVehInfo_data, AvoMainRoVehInfo_size, &a_soll_veh2goal,
              &b_wait);
    TargetLaneFrontDisAvoidVehicle =
      AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[tmp_data[b_wait - 1] - 1];
    i = 0;
    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[0] < 200.0) &&
        (t_TargetLaneFront2int_idx_0 < a_soll_TrafficLightActive)) {
      b_tmp_data[0] = 1;
      i = 1;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[1] < 200.0) &&
        (t_TargetLaneFront2int_idx_1 < a_soll_TrafficLightActive)) {
      b_tmp_data[i] = 2;
      i++;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[2] < 200.0) &&
        (t_TargetLaneFront2int_idx_2 < a_soll_TrafficLightActive)) {
      b_tmp_data[i] = 3;
      i++;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[3] < 200.0) &&
        (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive)) {
      b_tmp_data[i] = 4;
    }

    TargetLaneFrontVelAvoidVehicle =
      AvoMainRoVehInfo->targetLaneFrontVelAvoidVehicle[b_tmp_data[b_wait - 1] -
      1];
    i = 0;
    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[0] < 200.0) &&
        (t_TargetLaneFront2int_idx_0 < a_soll_TrafficLightActive)) {
      c_tmp_data[0] = 1;
      i = 1;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[1] < 200.0) &&
        (t_TargetLaneFront2int_idx_1 < a_soll_TrafficLightActive)) {
      c_tmp_data[i] = 2;
      i++;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[2] < 200.0) &&
        (t_TargetLaneFront2int_idx_2 < a_soll_TrafficLightActive)) {
      c_tmp_data[i] = 3;
      i++;
    }

    if ((AvoMainRoVehInfo->targetLaneFrontDisAvoidVehicle[3] < 200.0) &&
        (c_a_soll_SpeedPlanAvoidOncoming < a_soll_TrafficLightActive)) {
      c_tmp_data[i] = 4;
    }

    TargetLaneFrontLenAvoidVehicle =
      AvoMainRoVehInfo->targetLaneFrontLenAvoidVehicle[c_tmp_data[b_wait - 1] -
      1];
  }

  /*  AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle为“大小为4的数组”，将各条laneCross上的后车数据放入数组，默认值为-200 */
  /*  AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle为“大小为4的数组”，AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle(i)对应AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle(i)，默认值为20 */
  t_TargetLaneFront2int_idx_0 = -200.0;
  t_TargetLaneFront2int_idx_1 = 20.0;
  t_TargetLaneFront2int_idx_2 = 5.0;
  trueCount = 0;
  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[0] > -200.0);
  unnamed_idx_0 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount = 1;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[1] > -200.0);
  unnamed_idx_1 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount++;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[2] > -200.0);
  unnamed_idx_2 = unnamed_idx_3;
  if (unnamed_idx_3) {
    trueCount++;
  }

  unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[3] > -200.0);
  if (unnamed_idx_3) {
    trueCount++;
  }

  tmp_size[0] = 1;
  tmp_size[1] = trueCount;
  i = 0;
  b_wait = 0;
  if (unnamed_idx_0) {
    d_tmp_data[0] = 1;
    i = 1;
  }

  b_unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[0] >
                     -200.0);
  unnamed_idx_0 = b_unnamed_idx_3;
  if (b_unnamed_idx_3) {
    b_wait = 1;
  }

  if (unnamed_idx_1) {
    d_tmp_data[i] = 2;
    i++;
  }

  b_unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[1] >
                     -200.0);
  unnamed_idx_1 = b_unnamed_idx_3;
  if (b_unnamed_idx_3) {
    b_wait++;
  }

  if (unnamed_idx_2) {
    d_tmp_data[i] = 3;
    i++;
  }

  b_unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[2] >
                     -200.0);
  unnamed_idx_2 = b_unnamed_idx_3;
  if (b_unnamed_idx_3) {
    b_wait++;
  }

  if (unnamed_idx_3) {
    d_tmp_data[i] = 4;
  }

  b_unnamed_idx_3 = (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[3] >
                     -200.0);
  if (b_unnamed_idx_3) {
    b_wait++;
  }

  b_tmp_size[0] = 1;
  b_tmp_size[1] = b_wait;
  i = 0;
  if (unnamed_idx_0) {
    e_tmp_data[0] = 1;
    i = 1;
  }

  if (unnamed_idx_1) {
    e_tmp_data[i] = 2;
    i++;
  }

  if (unnamed_idx_2) {
    e_tmp_data[i] = 3;
    i++;
  }

  if (b_unnamed_idx_3) {
    e_tmp_data[i] = 4;
  }

  if (trueCount != 0) {
    if (trueCount == b_wait) {
      AvoMainRoVehInfo_size[0] = 1;
      AvoMainRoVehInfo_size[1] = trueCount;
      for (b_wait = 0; b_wait < trueCount; b_wait++) {
        AvoMainRoVehInfo_data[b_wait] = (AvoMainRoVehInfo->d_veh2converge -
          AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[d_tmp_data[b_wait] -
          1]) / (AvoMainRoVehInfo->
                 targetLaneBehindVelAvoidVehicle[e_tmp_data[b_wait] - 1] +
                 2.2204460492503131E-16);
      }

      b_minimum(AvoMainRoVehInfo_data, AvoMainRoVehInfo_size, &a_soll_veh2goal,
                &b_wait);
    } else {
      binary_expand_op(AvoMainRoVehInfo, d_tmp_data, tmp_size, e_tmp_data,
                       b_tmp_size, &a_soll_veh2goal, &b_wait);
    }

    t_TargetLaneFront2int_idx_0 =
      AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[d_tmp_data[b_wait - 1] -
      1];
    t_TargetLaneFront2int_idx_1 =
      AvoMainRoVehInfo->targetLaneBehindVelAvoidVehicle[e_tmp_data[b_wait - 1] -
      1];
    i = 0;
    if (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[0] > -200.0) {
      f_tmp_data[0] = 1;
      i = 1;
    }

    if (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[1] > -200.0) {
      f_tmp_data[i] = 2;
      i++;
    }

    if (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[2] > -200.0) {
      f_tmp_data[i] = 3;
      i++;
    }

    if (AvoMainRoVehInfo->targetLaneBehindDisAvoidVehicle[3] > -200.0) {
      f_tmp_data[i] = 4;
    }

    t_TargetLaneFront2int_idx_2 =
      AvoMainRoVehInfo->targetLaneBehindLenAvoidVehicle[f_tmp_data[b_wait - 1] -
      1];
  }

  /*  time2nextSwitch = TrafficLightInfo.time2nextSwitch; */
  AEBActive = GlobVars->AEBDecision.AEBActive;

  /* --------------------------------------------------------- */
  TargetGear = ChassisInfo->currentGear;

  /* ------------------------------------------------------- */
  memset(&Trajectory->traj_s[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_l[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_psi[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_vs[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_vl[0], 0, 80U * sizeof(double));
  memset(&Trajectory->traj_omega[0], 0, 80U * sizeof(double));
  Trajectory->planning_states = 0;

  /* 0:Undefined 1:终点 2:AEB触发 */
  /* 掉头参考线 */
  Refline->NumRefLaneTurnAround = 0;
  memset(&Refline->SRefLaneTurnAround[0], 0, 100U * sizeof(double));
  memset(&Refline->LRefLaneTurnAround[0], 0, 100U * sizeof(double));
  Refline->TurnAroundReflineState = 0;
  if (TurnAroundActive == 1) {
    GlobVars->TrajPlanTurnAround.turnAroundActive = 1;
  }

  TurnAroundActive = GlobVars->TrajPlanTurnAround.turnAroundActive;

  /*  Environmental car */
  /*  CurrentLaneFrontDis = CurrentLaneFrontDis-CurrentLaneFrontLen; */
  /*  s_vehapostrophe=s_vehapostrophe+l_vehapostrophe; */
  /*  RightLaneFrontDis=RightLaneFrontDis-RightLaneFrontLen; */
  /*  LeftLaneFrontDis=LeftLaneFrontDis-LeftLaneFrontLen; */
  BasicsInfo_tmp = 0.5 * Parameters->l_veh;
  BasicsInfo->d_veh2goal -= BasicsInfo_tmp;

  /* 车中心转车头 */
  CurrentLaneFrontDis = BasicsInfo->currentLaneFrontDis - 0.5 *
    (BasicsInfo->currentLaneFrontLen + Parameters->l_veh);

  /* 车头到前车车尾距离 */
  /* 避让对向车 */
  /* 车中心距离转为车头距离 */
  for (b_wait = 0; b_wait < 6; b_wait++) {
    s_veh1[b_wait] = AvoOncomingVehInfo->s_veh[b_wait] - 0.5 *
      AvoOncomingVehInfo->l_veh[b_wait];
    s_vehapostrophe[b_wait] = AvoOncomingVehInfo->s_vehapostrophe[b_wait] + 0.5 *
      AvoOncomingVehInfo->l_vehapostrophe[b_wait];
  }

  /* 车中心距离转为车尾距离 */
  d_veh2waitingArea = AvoOncomingVehInfo->d_veh2waitingArea - BasicsInfo_tmp;

  /* 车中心距离转为车头距离 */
  /* 换道入参： */
  /* 车中心距离转为车头距离 */
  /* 车头到车尾距离 */
  /* 车头到车头距离 */
  /* 避让同向车入参：车中心距离转为车头距离 */
  d_veh2crossStopline = AvoMainRoVehInfo->d_veh2stopline - BasicsInfo_tmp;
  TargetLaneFrontDisAvoidVehicle += 0.5 * (TargetLaneFrontLenAvoidVehicle -
    Parameters->l_veh);

  /* 车头到车头距离 */
  t_TargetLaneFront2int_idx_0 += 0.5 * (Parameters->l_veh -
    t_TargetLaneFront2int_idx_2);

  /* 车头到车头距离 */
  /* 故障车位置： */
  for (b_wait = 0; b_wait < 5; b_wait++) {
    FailLaneFrontDis[b_wait] = AvoFailVehInfo->failLaneFrontDis[b_wait] + 0.5 *
      (AvoFailVehInfo->failLaneFrontLen[b_wait] - Parameters->l_veh);
  }

  /* 车中心距离转为车头与车头距离 */
  /* 停车让行停止线距离 */
  StopSignInfo->d_veh2stopline -= BasicsInfo_tmp;

  /* 车中心距离转为车头距离 */
  /* 信号灯通行 */
  d_veh2trafficStopline = TrafficLightInfo->d_veh2stopline - BasicsInfo_tmp;

  /* 车中心距离转为车头距离 */
  /* 行人 */
  /* 车中心距离转为车头距离 */
  /* 掉头 */
  /* 车中心距离转为车头距离 */
  /* 对向车中心坐标转车头坐标 */
  /* 同向车中心坐标转车头坐标 */
  /*  避让故障车功能（搜寻本车所在link前方故障车） */
  BackupTargetLaneIndex = -1;
  a_soll_Fail = 100.0;
  if (vectorAny(AvoFailVehInfo->lanesWithFail, iv)) {
    /* 避让故障车 */
    BackupTargetLaneIndex = LaneSelectionWithBlockedLanes
      (BasicsInfo->widthOfLanes, AvoFailVehInfo->lanesWithFail, &TargetLaneIndex,
       BasicsInfo->currentLaneIndex);
  }

  if (BasicsInfo->d_veh2goal < 60.0) {
    /* 靠边停车 */
    if (BasicsInfo->currentLaneIndex != TargetLaneIndex) {
      /* 未在目标车道较晚减速 */
      if (BasicsInfo->d_veh2goal <
          (CalibrationVars->TrajPlanLaneChange.v_max_int
           * CalibrationVars->TrajPlanLaneChange.v_max_int - BasicsInfo->v_max *
           BasicsInfo->v_max) / -3.0 +
          CalibrationVars->TrajPlanLaneChange.v_max_int
          * CalibrationVars->TrajPlanLaneChange.t_permit) {
        a_soll_veh2goal = ACC(8.3333333333333339, 0.0, BasicsInfo->d_veh2goal +
                              CalibrationVars->ACC.d_wait, ChassisInfo->speed,
                              1.0, CalibrationVars->ACC.a_max,
                              CalibrationVars->ACC.a_min,
                              CalibrationVars->ACC.d_wait2faultyCar,
                              CalibrationVars->ACC.tau_v_com,
                              CalibrationVars->ACC.tau_v,
                              CalibrationVars->ACC.tau_d,
                              CalibrationVars->ACC.tau_v_bre,
                              CalibrationVars->ACC.tau_v_emg,
                              CalibrationVars->ACC.tau_d_emg,
                              CalibrationVars->ACC.t_acc,
                              CalibrationVars->ACC.d_wait);
      } else {
        a_soll_veh2goal = 100.0;
      }
    } else {
      a_soll_veh2goal = ACC(BasicsInfo->v_max, 0.0, BasicsInfo->d_veh2goal +
                            CalibrationVars->ACC.d_wait, ChassisInfo->speed, 1.0,
                            CalibrationVars->ACC.a_max,
                            CalibrationVars->ACC.a_min,
                            CalibrationVars->ACC.d_wait2faultyCar,
                            CalibrationVars->ACC.tau_v_com,
                            CalibrationVars->ACC.tau_v,
                            CalibrationVars->ACC.tau_d,
                            CalibrationVars->ACC.tau_v_bre,
                            CalibrationVars->ACC.tau_v_emg,
                            CalibrationVars->ACC.tau_d_emg,
                            CalibrationVars->ACC.t_acc,
                            CalibrationVars->ACC.d_wait);
      if ((a_soll_veh2goal <= 0.0) && (ChassisInfo->speed < 0.2)) {
        Trajectory->planning_states = 1;
      }
    }
  } else {
    a_soll_veh2goal = 100.0;
  }

  a_soll_ACC = ACC(BasicsInfo->v_max, BasicsInfo->currentLaneFrontVel,
                   CurrentLaneFrontDis, ChassisInfo->speed, 0.0,
                   CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
                   CalibrationVars->ACC.d_wait2faultyCar,
                   CalibrationVars->ACC.tau_v_com, CalibrationVars->ACC.tau_v,
                   CalibrationVars->ACC.tau_d, CalibrationVars->ACC.tau_v_bre,
                   CalibrationVars->ACC.tau_v_emg,
                   CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
                   CalibrationVars->ACC.d_wait);
  if (vectorAny(AvoFailVehInfo->failLaneindex, iv1)) {
    /* 异常车避免碰撞 */
    for (i = 0; i < 5; i++) {
      short b_i;
      b_i = AvoFailVehInfo->failLaneindex[i];
      if ((b_i != 0) && (b_i == CurrentLaneIndex)) {
        a_soll_Fail = fmin(a_soll_Fail, ACC(BasicsInfo->v_max,
          AvoFailVehInfo->failLaneFrontVel[i], FailLaneFrontDis[i] -
          AvoFailVehInfo->failLaneFrontLen[i], speed_tmp_tmp, 0.0,
          CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
          CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
          CalibrationVars->ACC.tau_v, CalibrationVars->ACC.tau_d,
          CalibrationVars->ACC.tau_v_bre, CalibrationVars->ACC.tau_v_emg,
          CalibrationVars->ACC.tau_d_emg, CalibrationVars->ACC.t_acc,
          CalibrationVars->ACC.d_wait));
      }
    }
  }

  b_a_soll_Fail[0] = a_soll_Fail;
  b_a_soll_Fail[1] = a_soll_ACC;
  b_a_soll_Fail[2] = a_soll_veh2goal;
  a_soll = d_minimum(b_a_soll_Fail);
  if (PedestrianActive != 0) {
    double b_AvoPedInfo[40];
    for (b_wait = 0; b_wait < 40; b_wait++) {
      b_AvoPedInfo[b_wait] = AvoPedInfo->s_ped[b_wait] - BasicsInfo_tmp;
    }

    SpeedPlanAvoidPedestrian(BasicsInfo->pos_s, ChassisInfo->speed,
      AvoPedInfo->d_veh2cross - 0.5 * Parameters->l_veh, AvoPedInfo->w_cross,
      b_AvoPedInfo, AvoPedInfo->l_ped, AvoPedInfo->v_ped, AvoPedInfo->psi_ped,
      CurrentLaneFrontDis, BasicsInfo->currentLaneFrontVel, BasicsInfo->v_max,
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

  for (i1 = 0; i1 < 6; i1++) {
    b_s_veh1[i1] = s_veh1[i1];
  }

  for (i2 = 0; i2 < 6; i2++) {
    b_AvoOncomingVehInfo[i2] = AvoOncomingVehInfo->v_veh[i2];
  }

  for (i3 = 0; i3 < 6; i3++) {
    b_s_vehapostrophe[i3] = s_vehapostrophe[i3];
  }

  AEBDecision(&AEBActive, ChassisInfo->speed, d_veh2stopline_ped,
              d_veh2crossStopline, d_veh2waitingArea, b_s_veh1,
              b_AvoOncomingVehInfo, AvoOncomingVehInfo->d_veh2conflict,
              b_s_vehapostrophe, d_veh2trafficStopline,
              TrafficLightInfo->greenLight, CurrentLaneFrontDis,
              BasicsInfo->currentLaneFrontVel, BasicsInfo->currentLaneIndex,
              GlobVars, CalibrationVars, *Parameters);

  /* , */
  if (AEBActive != 0) {
    if (rtIsNaN(ChassisInfo->speed)) {
      a_soll_veh2goal = rtNaN;
    } else if (ChassisInfo->speed < 0.0) {
      a_soll_veh2goal = -1.0;
    } else {
      a_soll_veh2goal = (ChassisInfo->speed > 0.0);
    }

    b_a_soll_SpeedPlanAvoidPedestri[0] = -4.0 * a_soll_veh2goal;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
    Trajectory->planning_states = 2;
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 0) {
    /* 停车让行 */
    if ((StopSignInfo->d_veh2stopline < 60.0) && (StopSignInfo->d_veh2stopline >=
         1.0)) {
      GlobVars->SpeedPlanStopSign.wait_stopsign = 1;
    }
  } else if (((ChassisInfo->speed <= 0.05) && (StopSignInfo->d_veh2stopline <
               1.0)) || (StopSignInfo->d_veh2stopline < 0.0) ||
             (StopSignInfo->d_veh2stopline >= 200.0)) {
    GlobVars->SpeedPlanStopSign.wait_stopsign = 0;
  }

  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 1) {
    b_a_soll_SpeedPlanAvoidPedestri[0] = 0.0;
    b_a_soll_SpeedPlanAvoidPedestri[1] = StopSignInfo->d_veh2stopline +
      CalibrationVars->ACC.d_wait;
    a_soll_TrafficLightActive = maximum(b_a_soll_SpeedPlanAvoidPedestri);
    b_a_soll_SpeedPlanAvoidPedestri[0] = ACC(BasicsInfo->v_max, 0.0,
      a_soll_TrafficLightActive, ChassisInfo->speed, 0.0,
      CalibrationVars->ACC.a_max, CalibrationVars->ACC.a_min,
      CalibrationVars->ACC.d_wait2faultyCar, CalibrationVars->ACC.tau_v_com,
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
      d_veh2trafficStopline, CurrentLaneFrontDis,
      BasicsInfo->currentLaneFrontVel, TrafficLightInfo->greenLight,
      TrafficLightInfo->phase[0], BasicsInfo->v_max, GlobVars,
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
    t_TargetLaneFront2int_idx_2 = SpeedPlanAvoidVehicle(ChassisInfo->speed,
      AvoMainRoVehInfo->d_veh2converge - 0.5 * Parameters->l_veh,
      d_veh2crossStopline, CurrentLaneFrontDis + BasicsInfo->currentLaneFrontLen,
      BasicsInfo->currentLaneFrontVel, BasicsInfo->currentLaneFrontLen,
      TargetLaneFrontDisAvoidVehicle, TargetLaneFrontVelAvoidVehicle,
      TargetLaneFrontLenAvoidVehicle, t_TargetLaneFront2int_idx_0,
      t_TargetLaneFront2int_idx_1, t_TargetLaneFront2int_idx_2, GlobVars,
      CalibrationVars->SpeedPlanAvoidVehicle, &CalibrationVars->ACC,
      Parameters->l_veh);
    b_a_soll_SpeedPlanAvoidPedestri[0] = t_TargetLaneFront2int_idx_2;
    b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
    a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
  } else {
    t_TargetLaneFront2int_idx_2 = 100.0;
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
    c_a_soll_SpeedPlanAvoidOncoming = SpeedPlanAvoidOncomingVehicle
      (ChassisInfo->speed, d_veh2waitingArea, CurrentLaneFrontDis,
       BasicsInfo->currentLaneFrontVel, s_veh1, AvoOncomingVehInfo->v_veh,
       AvoOncomingVehInfo->d_veh2conflict, s_vehapostrophe, BasicsInfo->v_max,
       GlobVars, CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_max_com,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.a_min,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.v_max_int,
       CalibrationVars->SpeedPlanAvoidOncomingVehicle.d_safe,
       &CalibrationVars->ACC, Parameters->w_veh, Parameters->l_veh);

    /* , */
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

  a_soll_veh2goal = 0.0;
  if (LaneChangeActive != 0) {
    if (DurationLaneChange_RePlan == 0) {
      for (i4 = 0; i4 < 6; i4++) {
        s_veh1[i4] = BasicsInfo->widthOfLanes[i4];
      }

      TrajPlanLaneChange(CurrentLaneFrontDis, BasicsInfo->currentLaneFrontVel,
                         LaneChangeInfo->leftLaneBehindDis + 0.5 *
                         (Parameters->l_veh - LaneChangeInfo->leftLaneBehindLen),
                         LaneChangeInfo->leftLaneBehindVel,
                         LaneChangeInfo->leftLaneFrontDis - 0.5 *
                         (LaneChangeInfo->leftLaneFrontLen + Parameters->l_veh),
                         LaneChangeInfo->leftLaneFrontVel,
                         LaneChangeInfo->rightLaneBehindDis + 0.5 *
                         (Parameters->l_veh - LaneChangeInfo->rightLaneBehindLen),
                         LaneChangeInfo->rightLaneBehindVel,
                         LaneChangeInfo->rightLaneFrontDis - 0.5 *
                         (LaneChangeInfo->rightLaneFrontLen + Parameters->l_veh),
                         LaneChangeInfo->rightLaneFrontVel, ChassisInfo->speed,
                         BasicsInfo->pos_s, pos_l_CurrentLane, BasicsInfo->pos_l,
                         BasicsInfo->currentLaneIndex, TargetLaneIndex,
                         BasicsInfo->goalLaneIndex, BackupTargetLaneIndex,
                         LaneChangeInfo->d_veh2int - 0.5 * Parameters->l_veh,
                         BasicsInfo->d_veh2goal, s_veh1, BasicsInfo->v_max,
                         AvoFailVehInfo->lanesWithFail, AEBActive, GlobVars,
                         CalibrationVars, *Parameters, &a_soll_veh2goal,
                         Trajectory->traj_s, Trajectory->traj_l,
                         Trajectory->traj_psi, Trajectory->traj_vs,
                         Trajectory->traj_vl, Trajectory->traj_omega);

      /* , */
      b_a_soll_SpeedPlanAvoidPedestri[0] = a_soll_veh2goal;
      b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
      a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
    }
  } else {
    if (GlobVars->TrajPlanLaneChange.countLaneChange != 0) {
      GlobVars->TrajPlanLaneChange.countLaneChange = 0;
    }

    if (GlobVars->TrajPlanLaneChange.durationLaneChange != 0) {
      GlobVars->TrajPlanLaneChange.durationLaneChange = 0;
    }
  }

  /* 最近停车位置 */
  for (b_wait = 0; b_wait < 8; b_wait++) {
    stopdistance_array[b_wait] = 200.0;
  }

  if (GlobVars->SpeedPlanAvoidPedestrian.wait_ped == 1) {
    stopdistance_array[0] = d_veh2stopline_ped -
      CalibrationVars->SpeedPlanAvoidPedestrian.d_gap2ped;
  }

  if (GlobVars->SpeedPlanTrafficLight.wait_TrafficLight == 1) {
    stopdistance_array[3] = d_veh2trafficStopline -
      CalibrationVars->SpeedPlanTrafficLight.d_gap2stopline;
  }

  if (GlobVars->SpeedPlanAvoidVehicle.wait_AvoidVehicle == 1) {
    stopdistance_array[1] = d_veh2crossStopline;
  }

  if (GlobVars->SpeedPlanAvoidOncomingVehicle.wait_avoidOncomingVehicle == 1) {
    stopdistance_array[2] = d_veh2waitingArea;
  }

  /*  if TurnAroundActive==1&&GlobVars.TrajPlanTurnAround.wait_turnAround==1%激活第二帧有值 */
  /*      stopdistance_array(5)=GlobVars.TrajPlanTurnAround.PosCircle(1)-BasicsInfo.pos_s; */
  /*  end */
  if (GlobVars->SpeedPlanStopSign.wait_stopsign == 1) {
    stopdistance_array[5] = StopSignInfo->d_veh2stopline;
  }

  if (BasicsInfo->currentLaneFrontVel < 0.2) {
    b_wait = 0;
    for (i = 0; i < 6; i++) {
      if (CurrentLaneIndex == AvoFailVehInfo->lanesWithFail[i]) {
        b_wait = -1;
      }
    }

    if (b_wait == -1) {
      stopdistance_array[6] = CurrentLaneFrontDis - 17.0;
    } else {
      stopdistance_array[6] = CurrentLaneFrontDis - CalibrationVars->ACC.d_wait;
    }
  }

  stopdistance_array[7] = BasicsInfo->d_veh2goal;
  d_veh2waitingArea = g_minimum(stopdistance_array);

  /*  车偏离参考线轨迹规划（靠边停车的右偏轨迹规划，换道重归划） */
  if (((PlannerLevel == 1) && (GlobVars->TrajPlanLaneChange.durationLaneChange ==
        0) && (TurnAroundActive == 0) && (a_soll_veh2goal != 100.0) && ((fabs
         (BasicsInfo->pos_l - pos_l_CurrentLane) > 0.3) || (fabs
         (BasicsInfo->pos_psi - 90.0) > 10.0))) || (DurationLaneChange_RePlan !=
       0)) {
    /* , */
    TrajPlanLaneChange_RePlan(a_soll, ChassisInfo->speed, BasicsInfo->pos_s,
      BasicsInfo->pos_l, BasicsInfo->pos_psi, pos_l_CurrentLane,
      d_veh2waitingArea, BasicsInfo->sampleTime, a_soll_ACC,
      BasicsInfo->currentLaneFrontVel, GlobVars,
      CalibrationVars->TrajPlanLaneChange.a_lateral,
      CalibrationVars->TrajPlanLaneChange_RePlan.frontWheelAnglelLimit,
      CalibrationVars->UrbanPlanner.jerkLimit, Parameters->l_veh,
      Trajectory->traj_s, Trajectory->traj_l, Trajectory->traj_psi,
      Trajectory->traj_vs, Trajectory->traj_vl, Trajectory->traj_omega);

    /* , */
  }

  if ((TurnAroundActive != 0) &&
      (GlobVars->TrajPlanLaneChange.durationLaneChange == 0) &&
      (GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan == 0)) {
    double b_TurnAroundInfo[20];
    double c_TurnAroundInfo[10];
    for (b_wait = 0; b_wait < 20; b_wait++) {
      b_TurnAroundInfo[b_wait] = TurnAroundInfo->posSOppositeCar[b_wait] - 0.5 *
        TurnAroundInfo->lengthOppositeCar[b_wait];
    }

    for (b_wait = 0; b_wait < 10; b_wait++) {
      c_TurnAroundInfo[b_wait] = TurnAroundInfo->posSCodirectCar[b_wait] + 0.5 *
        TurnAroundInfo->lengthCodirectCar[b_wait];
    }

    TrajPlanTurnAround(CurrentLaneFrontDis, BasicsInfo->currentLaneFrontVel,
                       ChassisInfo->speed, pos_l_CurrentLane, BasicsInfo->pos_s,
                       BasicsInfo->pos_l, TurnAroundInfo->numOfLanesOpposite,
                       TurnAroundInfo->widthOfLanesOpposite,
                       TurnAroundInfo->widthOfGap, BasicsInfo->widthOfLanes,
                       TurnAroundInfo->s_turnaround_border - 0.5 *
                       Parameters->l_veh, TurnAroundInfo->indexOfLaneOppositeCar,
                       TurnAroundInfo->speedOppositeCar, b_TurnAroundInfo,
                       TurnAroundInfo->lengthOppositeCar,
                       TurnAroundInfo->indexOfLaneCodirectCar,
                       TurnAroundInfo->speedCodirectCar, c_TurnAroundInfo,
                       TurnAroundInfo->lengthCodirectCar,
                       BasicsInfo->currentLaneIndex, BasicsInfo->v_max, a_soll,
                       ChassisInfo->currentGear, &TurnAroundActive, &AEBActive,
                       d_veh2waitingArea, a_soll_ACC, BasicsInfo->sampleTime,
                       GlobVars, CalibrationVars, *Parameters,
                       &t_TargetLaneFront2int_idx_1,
                       &t_TargetLaneFront2int_idx_0, Refline, Trajectory->traj_s,
                       Trajectory->traj_l, Trajectory->traj_psi,
                       Trajectory->traj_vs, Trajectory->traj_vl,
                       Trajectory->traj_omega, &TargetGear);

    /* , */
    if (t_TargetLaneFront2int_idx_1 != 100.0) {
      b_a_soll_SpeedPlanAvoidPedestri[0] = t_TargetLaneFront2int_idx_1;
      b_a_soll_SpeedPlanAvoidPedestri[1] = a_soll;
      a_soll = c_minimum(b_a_soll_SpeedPlanAvoidPedestri);
    }
  } else {
    t_TargetLaneFront2int_idx_0 = 100.0;
    if (GlobVars->TrajPlanTurnAround.dec_trunAround != 0) {
      GlobVars->TrajPlanTurnAround.dec_trunAround = 0;
    }

    if (GlobVars->TrajPlanTurnAround.turnAroundState != 0) {
      GlobVars->TrajPlanTurnAround.turnAroundState = 0;
    }

    if (GlobVars->TrajPlanTurnAround.targetLaneIndexOpposite != 0) {
      GlobVars->TrajPlanTurnAround.targetLaneIndexOpposite = 0;
    }

    if (GlobVars->TrajPlanTurnAround.wait_turnAround != 0) {
      GlobVars->TrajPlanTurnAround.wait_turnAround = 0;
    }

    if (GlobVars->TrajPlanTurnAround.typeOfTurnAround != 0) {
      GlobVars->TrajPlanTurnAround.typeOfTurnAround = 0;
    }
  }

  /* 更新掉头参考线标志位 */
  if (PlannerLevel == 1) {
    Refline->TurnAroundReflineState = TurnAroundActive;
  } else if ((GlobVars->TrajPlanTurnAround.reflineSend != 0.0) &&
             (GlobVars->TrajPlanTurnAround.reflineLend != 0.0) &&
             (BasicsInfo->pos_s <= GlobVars->TrajPlanTurnAround.reflineSend -
              Parameters->l_veh) && (((GlobVars->TrajPlanTurnAround.reflineLend
                - Parameters->w_veh) - BasicsInfo->pos_l) *
              ((GlobVars->TrajPlanTurnAround.reflineLend + Parameters->w_veh) -
               BasicsInfo->pos_l) < 0.0)) {
    Refline->TurnAroundReflineState = 0;
    GlobVars->TrajPlanTurnAround.reflineSend = 0.0;
    GlobVars->TrajPlanTurnAround.reflineLend = 0.0;
  } else if (GlobVars->TrajPlanTurnAround.reflineSend != 0.0) {
    Refline->TurnAroundReflineState = 1;
  }

  /* Decider */
  if ((PlannerLevel == 2) || (PlannerLevel == 3)) {
    Decider(PlannerLevel, BasicsInfo->currentLaneFrontDis,
            BasicsInfo->currentLaneFrontVel, BasicsInfo->currentLaneFrontLen,
            BasicsInfo->pos_s, BasicsInfo->currentLaneIndex,
            BasicsInfo->widthOfLanes, BasicsInfo->v_max, BasicsInfo->d_veh2goal,
            BasicsInfo->sampleTime, ChassisInfo->speed, LaneChangeInfo,
            AvoMainRoVehInfo, AvoPedInfo, TrafficLightInfo, AvoOncomingVehInfo, *
            StopSignInfo, LaneChangeActive, PedestrianActive, TrafficLightActive,
            VehicleCrossingActive, VehicleOncomingActive, GlosaActive, AEBActive,
            TargetGear, a_soll_ACC, a_soll_SpeedPlanAvoidPedestrian,
            a_soll_TrafficLightActive, t_TargetLaneFront2int_idx_2,
            c_a_soll_SpeedPlanAvoidOncoming, t_TargetLaneFront2int_idx_0,
            a_soll_Fail, TargetLaneIndex, BackupTargetLaneIndex,
            d_veh2stopline_ped, GlobVars, CalibrationVars, *Parameters, Decision);
  } else {
    Decision->AEBactive = AEBActive;
    Decision->TargetGear = TargetGear;
    Decision->states = 0;
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
    Decision->TurnAroundState = 0;
  }

  /* -------------------------------------------------------------------------------------------------------------------- */
  /*  轨迹生成 */
  if ((GlobVars->TrajPlanLaneChange.durationLaneChange == 0) &&
      (GlobVars->TrajPlanTurnAround.turnAroundActive == 0) &&
      (GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan == 0) &&
      (a_soll_veh2goal != 100.0)) {
    /* , */
    b_wait = 0;

    /* 停车速度规划  停止线前停车或跟车停车场景且以最小减速度-4制动距离小于停车距离 */
    if (d_veh2waitingArea < 200.0 - BasicsInfo_tmp) {
      a_soll_TrafficLightActive = ChassisInfo->speed * ChassisInfo->speed;
      if (a_soll_TrafficLightActive / 8.0 <= d_veh2waitingArea) {
        a_soll_TrafficLightActive *= 0.44444444444444442;
        c_a_soll_SpeedPlanAvoidOncoming = -(a_soll_TrafficLightActive /
          (0.66666666666666663 * d_veh2waitingArea));
        if (((c_a_soll_SpeedPlanAvoidOncoming <= a_soll_ACC) ||
             (BasicsInfo->currentLaneFrontVel < 0.2)) && (AEBActive == 0)) {
          boolean_T guard1 = false;

          /*          已知初速度v_0、目标停车距离s、初始加速度a_0，确定停车轨迹（匀加加速度） */
          /*          1建立关于加加速度J和停车时间t的方程： */
          /*          s=v_0 t+1/2 a_0 t^2+1/6 Jt^3      */
          /*          0=v=v_0+a_0 t+1/2 Jt^2 */
          /*          2.当a_0为正，求解方程，因为t应为正数，所有只有一个解： */
          /*          t=(-2/3 v_0+√(4/9 〖v_0〗^2+2/3 a_0 s))/(1/3 a) */
          /*          J=(-2(v_0+a_0 t))/t^2 */
          /*          3.当a_0为负，求解方程： */
          /*          有解的条件如下，也就是目标停车距离不大于最大停车距离（匀加加速度行驶到v=0、a=0的时候的s）： */
          /*          s≤-(4/9 〖v_0〗^2)/(2/3 a_0 ) */
          /*          有解的情况下，存在一个（目标停车距离等于最大停车距离）或者两个解。两个解的情况下，较大的解为“先行驶经过目标停车位置，接着倒车到达目标停车位置”，不符合要求，取较小的解，如下： */
          /*          t=(-2/3 v_0+√(4/9 〖v_0〗^2+2/3 a_0 s))/(1/3 a) */
          /*          J=(-2(v_0+a_0 t))/t^2 */
          a_soll = fmax(a_soll, c_a_soll_SpeedPlanAvoidOncoming);
          JerkLimit(GlobVars->Decider.a_sollpre2traj, BasicsInfo->sampleTime,
                    &a_soll, CalibrationVars->UrbanPlanner.jerkLimit);
          guard1 = false;
          if (fabs(a_soll) <= 1.0E-5) {
            /* a_soll为0的情况 */
            if ((ChassisInfo->speed > 0.0) && (d_veh2waitingArea > 0.0)) {
              t_TargetLaneFront2int_idx_0 = 3.0 * d_veh2waitingArea / 2.0 /
                ChassisInfo->speed;
              a_soll_veh2goal = -8.0 * rt_powd_snf(ChassisInfo->speed, 3.0) /
                9.0 / (d_veh2waitingArea * d_veh2waitingArea);
              b_wait = 1;
              guard1 = true;
            }
          } else if (a_soll >= c_a_soll_SpeedPlanAvoidOncoming) {
            t_TargetLaneFront2int_idx_0 = (3.0 * sqrt(fmax(0.0,
              a_soll_TrafficLightActive + 0.66666666666666663 * a_soll *
              d_veh2waitingArea)) - 2.0 * ChassisInfo->speed) / (a_soll +
              2.2204460492503131E-16);
            a_soll_veh2goal = -2.0 * (ChassisInfo->speed + a_soll *
              t_TargetLaneFront2int_idx_0) / (t_TargetLaneFront2int_idx_0 *
              t_TargetLaneFront2int_idx_0);
            b_wait = 1;
            guard1 = true;
          }

          if (guard1) {
            for (i = 0; i < 80; i++) {
              t_TargetLaneFront2int_idx_1 = 0.05 * ((double)i + 1.0);
              if (t_TargetLaneFront2int_idx_1 <= t_TargetLaneFront2int_idx_0) {
                a_soll_TrafficLightActive = t_TargetLaneFront2int_idx_1 *
                  t_TargetLaneFront2int_idx_1;
                Trajectory->traj_vs[i] = (speed_tmp_tmp + a_soll *
                  t_TargetLaneFront2int_idx_1) + 0.5 * a_soll_veh2goal *
                  a_soll_TrafficLightActive;
                Trajectory->traj_s[i] = ((pos_s + speed_tmp_tmp *
                  t_TargetLaneFront2int_idx_1) + 0.5 * a_soll *
                  a_soll_TrafficLightActive) + 0.16666666666666666 *
                  a_soll_veh2goal * rt_powd_snf(t_TargetLaneFront2int_idx_1, 3.0);
              } else {
                Trajectory->traj_vs[i] = 0.0;
                Trajectory->traj_s[i] = pos_s + d_veh2waitingArea;
              }

              Trajectory->traj_vl[i] = 0.0;
              Trajectory->traj_omega[i] = 0.0;
              Trajectory->traj_l[i] = pos_l_CurrentLane;
              Trajectory->traj_psi[i] = 90.0;
            }
          }
        }
      }
    }

    if (b_wait == 0) {
      JerkLimit(GlobVars->Decider.a_sollpre2traj, BasicsInfo->sampleTime,
                &a_soll, CalibrationVars->UrbanPlanner.jerkLimit);
      if (a_soll < 0.0) {
        a_soll_veh2goal = 0.0;
      } else if (a_soll > 0.0) {
        a_soll_veh2goal = fmax(BasicsInfo->v_max, ChassisInfo->speed);
      } else {
        a_soll_veh2goal = ChassisInfo->speed;
      }

      if (fabs(a_soll) <= 0.001) {
        a_soll_veh2goal = ChassisInfo->speed;
        t_TargetLaneFront2int_idx_0 = 0.0;
      } else {
        t_TargetLaneFront2int_idx_0 = (a_soll_veh2goal - ChassisInfo->speed) /
          (a_soll + 2.2204460492503131E-16);
      }

      for (i = 0; i < 80; i++) {
        t_TargetLaneFront2int_idx_1 = 0.05 * ((double)i + 1.0);
        Trajectory->traj_vl[i] = 0.0;
        Trajectory->traj_omega[i] = 0.0;
        Trajectory->traj_l[i] = pos_l_CurrentLane;
        Trajectory->traj_psi[i] = 90.0;
        if (t_TargetLaneFront2int_idx_1 <= t_TargetLaneFront2int_idx_0) {
          Trajectory->traj_vs[i] = speed_tmp_tmp + a_soll *
            t_TargetLaneFront2int_idx_1;
          Trajectory->traj_s[i] = (pos_s + speed_tmp_tmp *
            t_TargetLaneFront2int_idx_1) + 0.5 * a_soll *
            (t_TargetLaneFront2int_idx_1 * t_TargetLaneFront2int_idx_1);
        } else {
          Trajectory->traj_vs[i] = a_soll_veh2goal;
          Trajectory->traj_s[i] = (pos_s + (a_soll_veh2goal * a_soll_veh2goal -
            speed_tmp_tmp * speed_tmp_tmp) / (2.0 * a_soll +
            2.2204460492503131E-16)) + (t_TargetLaneFront2int_idx_1 -
            t_TargetLaneFront2int_idx_0) * a_soll_veh2goal;
        }
      }
    }
  }

  /* -------------------------------------------------------------------------------------------------------------------- */
  /*  if a_soll~=100 */
  /*      traj_s=zeros([1 80]); */
  /*      traj_l=zeros([1 80]); */
  /*      traj_psi=zeros([1 80]); */
  /*      traj_vs=zeros([1 80]); */
  /*      traj_vl=zeros([1 80]); */
  /*      traj_omega=zeros([1 80]); */
  /*      for count_1=1:1:80 */
  /*          t_count_1=0.05*count_1; */
  /*          traj_vs(count_1)=max([0 speed+a_soll*t_count_1]); */
  /*          traj_vl(count_1)=0; */
  /*          traj_omega(count_1)=0; */
  /*          traj_l(count_1)=pos_l_CurrentLane; */
  /*          traj_psi(count_1)=90; */
  /*          if traj_vs(count_1)==0 */
  /*              traj_s(count_1)=pos_s+(0-speed.^2)/(2*a_soll+eps); */
  /*          else */
  /*              traj_s(count_1)=pos_s+(traj_vs(count_1)+speed)*t_count_1/2; */
  /*          end */
  /*      end */
  /*  end */
  GlobVars->AEBDecision.AEBActive = AEBActive;
  GlobVars->TrajPlanTurnAround.turnAroundActive = TurnAroundActive;

  /*  全局变量类型设置 */
  if ((GlobVars->TrajPlanLaneChange_RePlan.durationLaneChange_RePlan == 0) &&
      (TurnAroundActive == 0)) {
    GlobVars->Decider.a_sollpre2traj = a_soll;
  }

  /* 打印出参 */
  logOutput(Trajectory->traj_s, Trajectory->traj_l, Trajectory->traj_psi,
            Trajectory->traj_vs, Trajectory->traj_vl, Trajectory->traj_omega,
            Trajectory->planning_states, Decision, Refline, CalibrationVars);
  b_wait = CalibrationVars->UrbanPlanner.logTrigger[0];
  for (i = 0; i < 31; i++) {
    b_wait += CalibrationVars->UrbanPlanner.logTrigger[i + 1];
  }

  if (b_wait > 0) {
    printf("***----------------------------------------------- log end -------------------------------------------------------***\n");
    fflush(stdout);
  }
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
}

/*
 * File trailer for UrbanPlanner.c
 *
 * [EOF]
 */
