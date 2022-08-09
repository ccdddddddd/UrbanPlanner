
#include <iostream>
#include <algorithm>
#include<string.h>
#include<math.h>
#include <dlfcn.h>

typedef struct { // 定义车辆底盘信息结构体
  double speed; // 车速
  short CurrentGear; // 当前档位
} TypeChassisInfo;

typedef struct { // 定义换道子功能所需信息结构体
  double d_veh2int; // 本车车头距道路frenet坐标系结束位置的距离
  double LeftLaneBehindDis; // 左侧车道后车距离
  double LeftLaneBehindVel;  // 左侧车道后车速度
  double LeftLaneFrontDis; // 左侧车道前车距离
  double LeftLaneFrontVel; // 左侧车道前车速度
  double RightLaneBehindDis; // 右侧车道后车距离
  double RightLaneBehindVel; // 右侧车道后车速度
  double RightLaneFrontDis; // 右侧车道前车距离
  double RightLaneFrontVel; // 右侧车道前车速度
  double LeftLaneBehindLen;// 左侧车道后车车长
  double LeftLaneFrontLen;// 左侧车道前车车长
  double RightLaneBehindLen;// 右侧车道后车车长
  double RightLaneFrontLen;// 右侧车道前车车长
} TypeLaneChangeInfo;

typedef struct { //定义停车让行功能所需信息结构体
    double d_veh2stopline;
} TypeStopSignInfo;

typedef struct { // 定义避让同向车子功能所需信息结构体
    double TargetLaneBehindDisAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆与本车的距离（Frenet坐标系下）
    double TargetLaneBehindVelAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆的速度
    double TargetLaneFrontDisAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆与本车的距离（Frenet坐标系下）
    double TargetLaneFrontVelAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆的速度
    double TargetLaneBehindLenAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆的长度
    double TargetLaneFrontLenAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆的长度
    double d_veh2converge; //当前车道与同向车道汇聚时，本车与汇聚点的距离
    double d_veh2stopline; //当前车道与同向车道汇聚时，本车与停止线的距离
} TypeAvoMainRoVehInfo;

typedef struct { // 定义避让行人子功能所需信息结构体
    double d_veh2cross; //自车与人行横道距离（Frenet坐标系下）
    double w_cross; //人行横道宽度  
    double s_ped[40]; //行人在frenet坐标系下的s坐标       
    double v_ped[40]; //行人速度        
    double l_ped[40]; //行人在frenet坐标系下的l坐标        
    double psi_ped[40]; //行人在frenet坐标系下的航向角
} TypeAvoPedInfo;

typedef struct { // 定义信号灯通行决策子功能所需信息结构体
  double greenLight; //前方交通信号灯是否为绿灯（0为否，1为是）
  double time2nextSwitch; //前方交通信号当前灯色的剩余时间
  double d_veh2stopline; //本车与十字路口停止线的距离
} TypeTrafficLightInfo;

typedef struct { // 定义AEB子功能所需全局变量结构体
  short AEBActive;
} GlobAEBDecision;

typedef struct { // 定义避让行人子功能所需全局变量结构体
  short dec_ped;
  short wait_ped;
} GlobSpeedPlanAvoidPedestrian;

typedef struct { // 定义信号灯通行决策子功能所需全局变量结构体
  short dec_fol_TrafficLight;
  short dec_bre_TrafficLight;
  short wait_TrafficLight;
} GlobSpeedPlanTrafficLight;

typedef struct {// 定义避让同向车子功能所需全局变量结构体
  short dec_fol_AvoidVehicle;
  short dec_bre_AvoidVehicle;
  short wait_AvoidVehicle;
} GlobSpeedPlanAvoidVehicle;

typedef struct { // 定义避让对向车子功能所需全局变量结构体
  short dec_avoidOncomingVehicle;
  short wait_avoidOncomingVehicle;
} c_GlobSpeedPlanAvoidOncomingVeh;

typedef struct { // 定义决策子功能所需全局变量结构体
  short dec_start;
  short dir_start;
  short CountLaneChangeDecider;
  short CurrentTargetLaneIndexDecider;
  double a_soll_pre;
  short wait_pullover;
  double distBehindGoal;
  short dec_follow;
} GlobDecider;

typedef struct { //定义停车让行子功能所需全局变量结构体
  short wait_stopsign;
} GlobSpeedPlanStopSign;

typedef struct { // 定义掉头子功能所需标定量结构体
  double D_safe1;
  double D_safe2;
  double dec2line;
  double a_min;
  double a_max_com;
  double v_max_turnAround;
} CalibTrajPlanTurnAround;

typedef struct {// 定义避让行人子功能所需标定量结构体
  double a_max;
  double a_min;
  double v_max_int;
  double v_max_int_emg;
  double d_gap2ped;
} CalibSpeedPlanAvoidPedestrian;

typedef struct {// 定义信号灯通行决策子功能所需标定量结构体
  double a_min_com;
  double a_max;
  double a_min;
  double v_max_int;
  double t_acc;
} CalibSpeedPlanTrafficLight;

typedef struct {// 定义避让同向车子功能所需标定量结构体
  double a_min_com;
  double a_max;
  double a_min;
  double v_max;
  double t_re;
  double GapIndex;
} CalibSpeedPlanAvoidVehicle;

typedef struct {// 定义避让对向车子功能所需标定量结构体
  double a_max_com;
  double a_min;
  double v_max_int;
  double D_safe;
} c_CalibSpeedPlanAvoidOncomingVe;

typedef struct {// 定义换道子功能所需标定量结构体
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

typedef struct {// 定义中止换道子功能所需标定量结构体
  double t_re;
  double index_accel;
  double a_min_comfort;
  double a_min;
} CalibTrajPlanLaneChange_RePlan;

typedef struct {// 定义ACC子功能所需标定量结构体
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

typedef struct {// 定义ACCcust子功能所需标定量结构体
  double tau_v_com;
  double tau_v;
  double tau_d;
} CalibACCcust;

typedef struct {// 定义ACClowSpeed子功能所需标定量结构体
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

typedef struct { //定义决策子功能所需标定量结构体
  double a_bre;
  double a_bre_com;
  double idle_speed;
  double dist_wait2pilot;
  double dist_wait2veh;
} CalibDecider;

typedef struct {// 定义标定量结构体
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

typedef struct {// 定义车辆参数结构体
  double TurningRadius; // 转弯半径
  double w_veh; // 车辆宽度
  double l_veh; // 车辆长度
} TypeParameters;

typedef struct { // 定义基本信息结构体
    double CurrentLaneFrontDis; // 前车距离
    double CurrentLaneFrontVel; // 前车速度
    double CurrentLaneFrontLen; //前车长度
    double pos_s; //  车道frenet坐标系下s坐标
    double pos_l; //  车道frenet坐标系下l坐标
    double pos_l_CurrentLane; // 车道frenet坐标系下当前车道中心线l坐标
    short CurrentLaneIndex; // 当前车道序号
    double WidthOfLanes[6]; // 当前道路车道宽度序列
    short TargetLaneIndex; // 目标车道序号
    double v_max; // 当前车道限速
    short GoalLaneIndex; //终点停车车道序号
    double d_veh2goal;//与终点的距离
    double SampleTime;//采样时间
} TypeBasicsInfo;

typedef struct { // 定义避让对向车子功能所需信息结构体
  double d_veh2waitingArea; //本车与左转待转区停止线的距离
  double s_veh[6]; //对向车道上汇聚点之前最近车辆与汇聚点的距离
  double v_veh[6]; //对向车道上汇聚点之前最近车辆的速度
  double l_veh[6]; //对向车道上汇聚点之前最近车辆的长度
  double d_veh2conflict[6]; //本车与当前车道、对向车道汇聚点的距离
  double s_vehapostrophe[6]; //对向车道上汇聚点之后最近车辆与汇聚点的距离
  double l_vehapostrophe[6]; //对向车道上汇聚点之后最近车辆的长度
} TypeAvoOncomingVehInfo;

typedef struct { // 定义避让故障车子功能所需信息结构体
    short LanesWithFail[6]; //需避开故障车所在车道序号
    short FailLaneindex[5]; //异常车辆所在车道号
    double FailLaneFrontDis[5]; //异常车与本车距离（Frenet坐标系下）
    double FailLaneFrontVel[5]; //异常车速度
    double FailLaneFrontLen[5]; //异常车长度
} TypeAvoFailVehInfo;

typedef struct { // 定义掉头子功能所需信息结构体
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

typedef struct { // 定义掉头子功能所需全局变量结构体
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

typedef struct { // 定义掉头子功能所需全局变量结构体
  short CountLaneChange;
  short DurationLaneChange;
  double LaneChangePath[720];
  double t_lc_traj;
  short CurrentTargetLaneIndex;
} GlobTrajPlanLaneChange;

typedef struct { // 定义中止换道子功能所需全局变量结构体
  short DurationLaneChange_RePlan;
  double LaneChangePath_RePlan[720];
  double t_lc_RePlan;
} GlobTrajPlanLaneChange_RePlan;

typedef struct {// 定义全局变量结构体
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

typedef struct { // 定义决策量输出
  short AEBactive; //紧急制动指示，对应辅助决策车云协议里emgBrak
  short TargetGear; //挡位指令
  short LaneChange; //换道指示，对应辅助决策车云协议里laneChgExp
  short SlowDown; //减速指示，对应辅助决策车云协议里slowDown
  double TargetSpeed; //目标速度 对应辅助决策车云协议里targetSpd
  short Wait; //停止指示，对应辅助决策车云协议里stopTips
  double WaitDistance; //停车距离，对应辅助决策车云协议里stopDis
  short Start; //起步指示，对应辅助决策车云协议里startTips
  double a_soll; //目标加速度
  short PedestrianState; //行人决策状态
  short TrafficLightState; //信号灯通行决策状态
  short VehicleCrossingState;  //避让同向车决策状态
  short VehicleOncomingState; //避让对向车决策状态
  short StopSignState; //停车让行决策状态
  short FollowState; //跟车决策状态
  short PullOverState; //靠边停车决策状态
} TypeDecider;

typedef struct { // 定义轨迹输出
  double traj_s[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标）
  double traj_l[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下l坐标）
  double traj_psi[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下航向角）
  double traj_vs[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标变化率）
  double traj_vl[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标变化率）
  double traj_omega[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下航向角变化率）
} TypeTraj;

int main()
{
	// so输入、输出的声明

  TypeBasicsInfo BasicsInfo; // 基本信息
  TypeChassisInfo ChassisInfo; // 车辆底盘信息
  TypeLaneChangeInfo LaneChangeInfo; // 换道子功能所需信息
  TypeAvoMainRoVehInfo AvoMainRoVehInfo; // 避让同向车子功能所需信息
  TypeAvoPedInfo AvoPedInfo; // 避让行人子功能所需信息
  TypeTrafficLightInfo TrafficLightInfo; //  信号灯通行决策子功能所需信息
  TypeAvoOncomingVehInfo AvoOncomingVehInfo; // 避让对向车子功能所需信息
  TypeAvoFailVehInfo AvoFailVehInfo; // 避让故障车子功能所需信息
  TypeTurnAroundInfo TurnAroundInfo;  // 掉头子功能所需信息
  TypeStopSignInfo StopSignInfo;  // 停车让行子功能所需信息
  short LaneChangeActive; // 换道子功能激活标识
  short PedestrianActive; // 避让行人子功能激活标识
  short TrafficLightActive; // 信号灯通行决策子功能激活标识
  short VehicleCrossingActive; // 避让同向车子功能激活标识
  short VehicleOncomingActive; // 避让对向车子功能激活标识
  short TurnAroundActive; // 掉头子功能激活标识
  short PlannerLevel; // 规划等级标识
  TypeGlobVars GlobVars; // 全局变量
  TypeCalibrationVars CalibrationVars; // 标定量
  TypeParameters Parameters; // 车辆参数
  TypeTraj Trajectory;  // 轨迹输出
  TypeDecider Decider;  // 决策量输出


  // so所需输入的赋值
	
  // 全局变量的初始化
  memset(&GlobVars, 0, sizeof(GlobVars));
  GlobVars.TrajPlanLaneChange.t_lc_traj=2;
  GlobVars.Decider.a_soll_pre=100;

	// 默认值的赋值
  LaneChangeActive=1; //换道场景激活
  PedestrianActive=0; //避让行人场景激活
  TrafficLightActive=0; //信号灯通行场景激活
  VehicleCrossingActive=0; //避让同向车场景激活
  VehicleOncomingActive=0; //避让对向车场景激活
  TurnAroundActive=0; //掉头场景激活

  BasicsInfo.CurrentLaneFrontDis=200;
  BasicsInfo.CurrentLaneFrontVel=20;
  BasicsInfo.CurrentLaneFrontLen=5;
  BasicsInfo.pos_l_CurrentLane=0;
  BasicsInfo.SampleTime=0.2; 

  memset(&AvoFailVehInfo, 0, sizeof(AvoFailVehInfo));
  memset(AvoFailVehInfo.FailLaneFrontLen, 5, sizeof(AvoFailVehInfo.FailLaneFrontLen));

  LaneChangeInfo.d_veh2int = 200;
  LaneChangeInfo.LeftLaneBehindDis=-200;
  LaneChangeInfo.LeftLaneBehindVel=20;
  LaneChangeInfo.LeftLaneFrontDis=200;
  LaneChangeInfo.LeftLaneFrontVel=20;
  LaneChangeInfo.RightLaneBehindDis=-200;
  LaneChangeInfo.RightLaneBehindVel=20;
  LaneChangeInfo.RightLaneFrontDis=200;
  LaneChangeInfo.RightLaneFrontVel=20;
  LaneChangeInfo.LeftLaneBehindLen=5;
  LaneChangeInfo.LeftLaneFrontLen=5;
  LaneChangeInfo.RightLaneBehindLen=5;
  LaneChangeInfo.RightLaneFrontLen=5;

  memset(AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle,-200,sizeof(AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle));
  memset(AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle,20,sizeof(AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle));
  memset(AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle,5,sizeof(AvoMainRoVehInfo.TargetLaneBehindLenAvoidVehicle));
  memset(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle,200,sizeof(AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle));
  memset(AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle,20,sizeof(AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle));
  memset(AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle,5,sizeof(AvoMainRoVehInfo.TargetLaneFrontLenAvoidVehicle));
  AvoMainRoVehInfo.d_veh2converge=200;
  AvoMainRoVehInfo.d_veh2stopline=200;

  StopSignInfo.d_veh2stopline=200;
  
  memset(&AvoOncomingVehInfo, 0, sizeof(AvoOncomingVehInfo));
  memset(AvoOncomingVehInfo.s_veh, 200, sizeof(AvoOncomingVehInfo.s_veh));
  memset(AvoOncomingVehInfo.l_veh, 5, sizeof(AvoOncomingVehInfo.l_veh));
  memset(AvoOncomingVehInfo.s_vehapostrophe, -200, sizeof(AvoOncomingVehInfo.s_vehapostrophe));
  memset(AvoOncomingVehInfo.l_vehapostrophe, 5, sizeof(AvoOncomingVehInfo.l_vehapostrophe));
  AvoOncomingVehInfo.d_veh2waitingArea=200;
  
  memset(&AvoPedInfo, 0, sizeof(AvoPedInfo));
  AvoPedInfo.d_veh2cross=200;
  AvoPedInfo.w_cross=6.4;
  memset(AvoPedInfo.v_ped,-1,sizeof(AvoPedInfo.v_ped));

  TrafficLightInfo.greenLight=1;
  TrafficLightInfo.time2nextSwitch=100;
  TrafficLightInfo.d_veh2stopline=200;

  memset(&TurnAroundInfo, 0, sizeof(TurnAroundInfo));
  memset(TurnAroundInfo.PosSOppositeCar, -200, sizeof(TurnAroundInfo.PosSOppositeCar));
  memset(TurnAroundInfo.SpeedCodirectCar, -1, sizeof(TurnAroundInfo.SpeedCodirectCar));
  memset(TurnAroundInfo.PosSCodirectCar, -200, sizeof(TurnAroundInfo.PosSCodirectCar));

  Parameters.TurningRadius=5;
  Parameters.w_veh=1.8;
  Parameters.l_veh=5;

  CalibrationVars.TrajPlanTurnAround.D_safe1=0.5;
  CalibrationVars.TrajPlanTurnAround.D_safe2=2;
  CalibrationVars.TrajPlanTurnAround.dec2line=0.2;
  CalibrationVars.TrajPlanTurnAround.a_min=-2;
  CalibrationVars.TrajPlanTurnAround.a_max_com=1.5;
  CalibrationVars.TrajPlanTurnAround.v_max_turnAround=5;
  CalibrationVars.SpeedPlanAvoidPedestrian.a_max=2.5;
  CalibrationVars.SpeedPlanAvoidPedestrian.a_min=-2;
  CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int=30/3.6;
  CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg=20/3.6;
  CalibrationVars.SpeedPlanAvoidPedestrian.d_gap2ped=0.5;
  CalibrationVars.SpeedPlanTrafficLight.a_min_com=-1.5;
  CalibrationVars.SpeedPlanTrafficLight.a_max=2.5;
  CalibrationVars.SpeedPlanTrafficLight.a_min=-3;
  CalibrationVars.SpeedPlanTrafficLight.v_max_int=30/3.6;
  CalibrationVars.SpeedPlanTrafficLight.t_acc=1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_min_com=-1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_max=1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_min=-3;
  CalibrationVars.SpeedPlanAvoidVehicle.v_max=40/3.6;
  CalibrationVars.SpeedPlanAvoidVehicle.t_re=1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.GapIndex=2;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com=1.5;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min=-3;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int=30/3.6;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.D_safe=2;
  CalibrationVars.TrajPlanLaneChange.v_max_int=30/3.6;
  CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int=1;
  CalibrationVars.TrajPlanLaneChange.t_permit=3;
  CalibrationVars.TrajPlanLaneChange.t_re=0.5;
  CalibrationVars.TrajPlanLaneChange.index_accel=0.5;
  CalibrationVars.TrajPlanLaneChange.a_max_comfort=1;
  CalibrationVars.TrajPlanLaneChange.a_min=-3.5;
  CalibrationVars.TrajPlanLaneChange.a_max=2.5;
  CalibrationVars.TrajPlanLaneChange.a_min_comfort=-1;
  CalibrationVars.TrajPlanLaneChange_RePlan.t_re=0.5;
  CalibrationVars.TrajPlanLaneChange_RePlan.index_accel=0.5;
  CalibrationVars.TrajPlanLaneChange_RePlan.a_min_comfort=-1;
  CalibrationVars.TrajPlanLaneChange_RePlan.a_min=-3.5;
  CalibrationVars.ACC.a_max=2.5;
  CalibrationVars.ACC.a_min=-4;
  CalibrationVars.ACC.a_min_com=-1.5;
  CalibrationVars.ACC.tau_v_com=4;
  CalibrationVars.ACC.tau_v=2;
  CalibrationVars.ACC.tau_d=5;
  CalibrationVars.ACC.tau_v_bre=1;
  CalibrationVars.ACC.tau_v_emg=0.5;
  CalibrationVars.ACC.tau_d_emg=2;
  CalibrationVars.ACC.t_acc=2;
  CalibrationVars.ACC.d_wait=4;
  CalibrationVars.ACCcust.tau_v_com=4;
  CalibrationVars.ACCcust.tau_v=2;
  CalibrationVars.ACCcust.tau_d=5;
  CalibrationVars.ACClowSpeed.a_max=2.5;
  CalibrationVars.ACClowSpeed.a_min=-4;
  CalibrationVars.ACClowSpeed.a_min_com=-1.5;
  CalibrationVars.ACClowSpeed.tau_v_com=4;
  CalibrationVars.ACClowSpeed.tau_v=2;
  CalibrationVars.ACClowSpeed.tau_d=5;
  CalibrationVars.ACClowSpeed.tau_v_bre=1;
  CalibrationVars.ACClowSpeed.tau_v_emg=0.5;
  CalibrationVars.ACClowSpeed.tau_d_emg=2;
  CalibrationVars.ACClowSpeed.tau_d_lowspeed=5/2;
  CalibrationVars.ACClowSpeed.t_acc=2;
  CalibrationVars.ACClowSpeed.d_wait=4;
  CalibrationVars.Decider.a_bre=-3;
  CalibrationVars.Decider.a_bre_com=-1.5;
  CalibrationVars.Decider.idle_speed=7;
  CalibrationVars.Decider.dist_wait2pilot=10;
  CalibrationVars.Decider.dist_wait2veh=15;

	// 动态配置量的赋值（请对照ppt“城区规划器主流程设计集相关需求”的算法输入部分）
  PlannerLevel=1; //车端请求云端规划级别，对应辅助决策车云协议里planLevel

  BasicsInfo.pos_s=100;  //  车道frenet坐标系下s坐标
  BasicsInfo.pos_l=0.3; //  车道frenet坐标系下l坐标
  BasicsInfo.CurrentLaneIndex=2;  // 当前车道序号
  double WidthOfLanes[]={3.61,3.62,3.63,3.64,0,0};
  for (int i = 0;i < 6;i++) {
    BasicsInfo.WidthOfLanes[i]=WidthOfLanes[i];  // 当前道路车道宽度序列
  }
  BasicsInfo.TargetLaneIndex=1;  // 目标车道序号
  BasicsInfo.v_max=60/3.6;  // 当前车道限速
  BasicsInfo.GoalLaneIndex=1; // 终点靠边车道序号
  BasicsInfo.d_veh2goal=200; // 与终点距离
  
  LaneChangeInfo.d_veh2int=70;  // 本车车头距道路frenet坐标系结束位置的距离

  ChassisInfo.speed=10;  // 车速
  ChassisInfo.CurrentGear=4;  // 当前档位

  
  // so的链接

  void *handle = dlopen("./UrbanPlanner.so", RTLD_LAZY);
  if (!handle) {
      fprintf (stderr, "%s\n", dlerror());
      exit(1);
  }
  printf("lib loaded\n");

  void (*myFn)(TypeBasicsInfo*,TypeChassisInfo*,TypeLaneChangeInfo* ,TypeAvoMainRoVehInfo*, TypeAvoPedInfo* ,TypeTrafficLightInfo*, TypeAvoOncomingVehInfo* ,TypeAvoFailVehInfo*, 
  TypeTurnAroundInfo* ,TypeStopSignInfo* ,short, short,short, short ,short, short,short, TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDecider*) 
  = (void(*)(TypeBasicsInfo*,TypeChassisInfo*, TypeLaneChangeInfo* ,TypeAvoMainRoVehInfo*, TypeAvoPedInfo* ,TypeTrafficLightInfo*, TypeAvoOncomingVehInfo* ,TypeAvoFailVehInfo*, 
  TypeTurnAroundInfo* ,TypeStopSignInfo* ,short, short,short, short ,short, short ,short,TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDecider*) ) dlsym(handle, "UrbanPlanner");

  if (myFn == NULL){
      printf("err\n");
      return -1;
  }

  // so的调用

  myFn(&BasicsInfo, &ChassisInfo,&LaneChangeInfo, &AvoMainRoVehInfo, &AvoPedInfo, &TrafficLightInfo, &AvoOncomingVehInfo, &AvoFailVehInfo, &TurnAroundInfo, &StopSignInfo,  LaneChangeActive, 
                    PedestrianActive,  TrafficLightActive,  VehicleCrossingActive,  VehicleOncomingActive,  TurnAroundActive, PlannerLevel, &GlobVars, &CalibrationVars, &Parameters, &Trajectory, &Decider);


  // so调用结果的展示

	std::cout << "\n--------------------------------------------------output--------------------------------------------------" << "\n";
  for (int i = 0;i < 5;i++) {
    std::cout << "\n第"<<i+1<<"帧轨迹"<< "\n";
    std::cout << Trajectory.traj_s[i]<< "\t";
    std::cout << Trajectory.traj_l[i]<< "\t";
    std::cout << Trajectory.traj_psi[i]<< "\t"; // 坐标转换请参考http://wiki.tusvn.net/pages/worddav/preview.action?fileName=frenet%E5%9D%90%E6%A0%87%E7%B3%BB%E4%B8%8B%E8%88%AA%E5%90%91%E8%A7%92%E7%9A%84%E8%AE%A1%E7%AE%97.pptx&pageId=97626362
    std::cout << Trajectory.traj_vs[i]<< "\t"; // traj_vs[i]和traj_vl[i]的平方根作为给网联车下发的目标车速
    std::cout << Trajectory.traj_vl[i]<< "\t";
    std::cout << Trajectory.traj_omega[i]<< "\t";
  }
  int i=43;
  std::cout << "\n第"<<i+1<<"帧轨迹"<< "\n";
  std::cout << Trajectory.traj_s[i]<< "\t";
  std::cout << Trajectory.traj_l[i]<< "\t";
  std::cout << Trajectory.traj_psi[i]<< "\t";
  std::cout << Trajectory.traj_vs[i]<< "\t";
  std::cout << Trajectory.traj_vl[i]<< "\t";
  std::cout << Trajectory.traj_omega[i]<< "\t";
	std::cout << "\n\n全局变量DurationLaneChange"<< "\t";
	std::cout << GlobVars.TrajPlanLaneChange.DurationLaneChange << "\n\n";
  // 注意GlobVars是全局变量！

	/* 调用结果
	--------------------------------------------------output--------------------------------------------------

  第1帧轨迹

  100.50063631120632	
  4.1158896977296034E-4	
  89.86032012787759	
  10.025238872232103	
  0.02444031427315525	
  0.0	

  第2帧轨迹

  101.00249827333674	
  0.003190974449616558	
  89.46576945361488	
  10.048925205128588	
  0.09369971800280624	
  -7.891013485254064	

  第3帧轨迹

  101.5054897218298	
  0.01042784696268893	
  88.85234493561671	
  10.070259621413122	
  0.2017379018184248	
  -12.26849035996338	

  第4帧轨迹

  102.0094676807498	
  0.0239111986760705	
  88.0551652143479	
  10.088208428663421	
  0.34256340577798244	
  -15.943594425376375	

  第5帧轨迹

  102.51423607087919	
  0.045131345064449525	
  87.10854216991632	
  10.101713453490923	
  0.5102208199801571	
  -18.93246088863151	

  第44帧轨迹

  121.9999999993962	
  3.6149999999999998	
  89.99999999999994	
  10.0	
  9.313225746154784E-15	
  2.7935936699961417	

  全局变量DurationLaneChange

  2	

  --------------------------------------------------output--------------------------------------------------

	*/
	
    /*
  std::cout << "决策输出"<< "\n";
  std::cout << "AEBactive " << Decider.AEBactive<< "\n";
  std::cout << "TargetGear " << Decider.TargetGear<< "\n";
  std::cout << "LaneChange " << Decider.LaneChange<< "\n";
  std::cout << "SlowDown " << Decider.SlowDown<< "\n";
  std::cout << "TargetSpeed " << Decider.TargetSpeed<< "\n";
  std::cout << "Wait " << Decider.Wait<< "\n";
  std::cout << "WaitDistance " << Decider.WaitDistance<< "\n";
  std::cout << "a_soll " << Decider.a_soll<< "\n";
  std::cout << "PedestrianState " << Decider.PedestrianState<< "\n";
  std::cout << "TrafficLightState " << Decider.TrafficLightState<< "\n";
  std::cout << "VehicleCrossingState " << Decider.VehicleCrossingState<< "\n";
  std::cout << "VehicleOncomingState " << Decider.VehicleOncomingState<< "\n";
  std::cout << "StopSignState " << Decider.StopSignState<< "\n";
  std::cout << "FollowState " << Decider.FollowState<< "\n";
  std::cout << "PullOverState " << Decider.PullOverState<< "\n";
	std::cout << "\n--------------------------------------------------output--------------------------------------------------\n";
*/

}
