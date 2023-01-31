
#include <iostream>
#include <algorithm>
#include<string.h>
#include<math.h>
#include <dlfcn.h>

typedef struct { // 定义车辆底盘信息结构体
  double speed; // 车速
  short currentGear; // 当前档位
} TypeChassisInfo;

typedef struct { // 定义换道子功能所需信息结构体
  double d_veh2int; // 本车车头距道路frenet坐标系结束位置的距离
  double leftLaneBehindDis; // 左侧车道后车距离
  double leftLaneBehindVel;  // 左侧车道后车速度
  double leftLaneFrontDis; // 左侧车道前车距离
  double leftLaneFrontVel; // 左侧车道前车速度
  double rightLaneBehindDis; // 右侧车道后车距离
  double rightLaneBehindVel; // 右侧车道后车速度
  double rightLaneFrontDis; // 右侧车道前车距离
  double rightLaneFrontVel; // 右侧车道前车速度
  double leftLaneBehindLen;// 左侧车道后车车长
  double leftLaneFrontLen;// 左侧车道前车车长
  double rightLaneBehindLen;// 右侧车道后车车长
  double rightLaneFrontLen;// 右侧车道前车车长
} TypeLaneChangeInfo;

typedef struct { //定义停车让行功能所需信息结构体
    double d_veh2stopline;
} TypeStopSignInfo;

typedef struct { // 定义避让同向车子功能所需信息结构体
    double targetLaneBehindDisAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆与本车的距离（Frenet坐标系下）
    double targetLaneBehindVelAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆的速度
    double targetLaneFrontDisAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆与本车的距离（Frenet坐标系下）
    double targetLaneFrontVelAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆的速度
    double targetLaneBehindLenAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上后方最近车辆的长度
    double targetLaneFrontLenAvoidVehicle[4]; //当前车道与同向车道汇聚时，同向车道上前方最近车辆的长度
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
  double d_veh2stopline; //本车与十字路口停止线的距离
  double phase[10]; //前方交通信号当前相位[当前灯色剩余时间，下一灯色时间，下一灯色时间,···]
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
} GlobSpeedPlanAvoidOnComVeh;

typedef struct { // 定义决策子功能所需全局变量结构体
  short dec_start;
  short dir_start;
  short countLaneChangeDecider;
  short currentTargetLaneIndexDecider;
  double a_soll_pre;
  double a_sollpre2traj;
  short wait_pullover;
  double distBehindGoal;
  short dec_follow;
} GlobDecider;

typedef struct { //定义停车让行子功能所需全局变量结构体
  short wait_stopsign;
} GlobSpeedPlanStopSign;

typedef struct { // 定义掉头子功能所需标定量结构体
  double d_safe1;
  double d_safe2;
  double dec2line;
  double a_min;
  double a_max_com;
  double v_max_turnAround;
  double d_gap2stop;
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
  double d_gap2stopline;
} CalibSpeedPlanTrafficLight;

typedef struct {// 定义避让同向车子功能所需标定量结构体
  double a_min_com;
  double a_max;
  double a_min;
  double v_max;
  double t_re;
  double gapIndex;
} CalibSpeedPlanAvoidVehicle;

typedef struct {// 定义避让对向车子功能所需标定量结构体
  double a_max_com;
  double a_min;
  double v_max_int;
  double d_safe;
} CalibSpeedPlanAvoidOnComVeh;

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
  double a_lateral;
} CalibTrajPlanLaneChange;

typedef struct {// 定义轨迹重规划子功能所需标定量结构体
  double frontWheelAnglelLimit;
} CalibTrajPlanLaneChange_RePlan;

typedef struct {// 定义ACC子功能所需标定量结构体
  double a_max;
  double a_min;
  double d_wait2faultyCar;
  double tau_v_com;
  double tau_v;
  double tau_d;
  double tau_v_bre;
  double tau_v_emg;
  double tau_d_emg;
  double t_acc;
  double d_wait;
} CalibACC;

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
  double glosaAdp;
  double mrg;
  double desRate;
  double dIntxn;
  double dMin;
  double dec;
  double glosaAverageIndex;
  double d_veh2endpoint;
} CalibDecider;

typedef struct {// 定义标定量结构体
  CalibTrajPlanTurnAround TrajPlanTurnAround;
  CalibSpeedPlanAvoidPedestrian SpeedPlanAvoidPedestrian;
  CalibSpeedPlanTrafficLight SpeedPlanTrafficLight;
  CalibSpeedPlanAvoidVehicle SpeedPlanAvoidVehicle;
  CalibSpeedPlanAvoidOnComVeh SpeedPlanAvoidOncomingVehicle;
  CalibTrajPlanLaneChange TrajPlanLaneChange;
  CalibTrajPlanLaneChange_RePlan TrajPlanLaneChange_RePlan;
  CalibACC ACC;
  CalibACClowSpeed ACClowSpeed;
  CalibDecider Decider;
} TypeCalibrationVars;

typedef struct {// 定义车辆参数结构体
  double turningRadius; // 转弯半径
  double w_veh; // 车辆宽度
  double l_veh; // 车辆长度
} TypeParameters;

typedef struct { // 定义基本信息结构体
    double currentLaneFrontDis; // 前车距离
    double currentLaneFrontVel; // 前车速度
    double currentLaneFrontLen; //前车长度
    double pos_s; //  车道frenet坐标系下s坐标
    double pos_l; //  车道frenet坐标系下l坐标
    double pos_psi;//  车道frenet坐标系下航向角
    double pos_l_CurrentLane; // 车道frenet坐标系下当前车道中心线l坐标
    short currentLaneIndex; // 当前车道序号
    double widthOfLanes[6]; // 当前道路车道宽度序列
    short targetLaneIndex; // 目标车道序号
    double v_max; // 当前车道限速
    short goalLaneIndex; //终点停车车道序号
    double d_veh2goal;//与终点的距离
    double sampleTime;//采样时间
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
    short lanesWithFail[6]; //需避开故障车所在车道序号
    short failLaneindex[5]; //异常车辆所在车道号
    double failLaneFrontDis[5]; //异常车与本车距离（Frenet坐标系下）
    double failLaneFrontVel[5]; //异常车速度
    double failLaneFrontLen[5]; //异常车长度
} TypeAvoFailVehInfo;

typedef struct { // 定义掉头子功能所需信息结构体
  short numOfLanesOpposite;
  double widthOfLanesOpposite[6];
  double widthOfGap;
  double s_turnaround_border;
  short indexOfLaneOppositeCar[20];
  double speedOppositeCar[20];
  double posSOppositeCar[20];
  short indexOfLaneCodirectCar[10];
  double speedCodirectCar[10];
  double posSCodirectCar[10];
  double lengthOppositeCar[20];
  double lengthCodirectCar[10];
} TypeTurnAroundInfo;

typedef struct { // 定义掉头子功能所需全局变量结构体
  double posCircle[2];
  double posCircle2[2];
  double posCircle3[2];
  double pos_start[2];
  double pos_mid1[4];
  double pos_mid2[4];
  double pos_mid1_rear[4];
  double pos_mid2_rear[4];
  double pos_end[2];
  double laneCenterline[7];
  short dec_trunAround;
  short wait_turnAround;
  short typeOfTurnAround;
  short turnAroundState;
  short targetLaneIndexOpposite;
  short turnAroundActive;
  double reflineSend;
  double reflineLend;
} GlobTrajPlanTurnAround;

typedef struct { // 定义掉头子功能所需全局变量结构体
  short countLaneChange;
  short durationLaneChange;
  double laneChangePath[720];
  double t_lc_traj;
  short currentTargetLaneIndex;
} GlobTrajPlanLaneChange;

typedef struct { // 定义中止换道子功能所需全局变量结构体
  short durationLaneChange_RePlan;
  double para[6];
  double s_end;
  double l_end;
  double para1[5];
  double para2[16];
  double para3;
} GlobTrajPlanLaneChange_RePlan;

typedef struct {// 定义全局变量结构体
  GlobAEBDecision AEBDecision;
  GlobTrajPlanTurnAround TrajPlanTurnAround;
  GlobSpeedPlanAvoidPedestrian SpeedPlanAvoidPedestrian;
  GlobSpeedPlanTrafficLight SpeedPlanTrafficLight;
  GlobSpeedPlanAvoidVehicle SpeedPlanAvoidVehicle;
  GlobSpeedPlanAvoidOnComVeh SpeedPlanAvoidOncomingVehicle;
  GlobTrajPlanLaneChange TrajPlanLaneChange;
  GlobTrajPlanLaneChange_RePlan TrajPlanLaneChange_RePlan;
  GlobDecider Decider;
  GlobSpeedPlanStopSign SpeedPlanStopSign;
} TypeGlobVars;

typedef struct { // 定义决策量输出
  short AEBactive; //紧急制动指示，对应辅助决策车云协议里emgBrak
  short TargetGear; //挡位指令
  short states; // 规划状态的指示
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
  short TurnAroundState;//掉头决策状态
} TypeDecider;

typedef struct { // 定义轨迹输出
  double traj_s[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标）
  double traj_l[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下l坐标）
  double traj_psi[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下航向角）
  double traj_vs[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标变化率）
  double traj_vl[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下s坐标变化率）
  double traj_omega[80]; // 未来4秒内车辆的目标轨迹（车道frenet坐标系下航向角变化率）
  short planning_states; // 规划状态的指示
} TypeTraj;

typedef struct {//定义参考线输出
  short NumRefLaneTurnAround;
  double SRefLaneTurnAround[100];
  double LRefLaneTurnAround[100];
  short TurnAroundReflineState;
} TypeRefline;

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
  short GlosaActive;//GLOSA功能激活标识
  short PlannerLevel; // 规划等级标识
  TypeGlobVars GlobVars; // 全局变量
  TypeCalibrationVars CalibrationVars; // 标定量
  TypeParameters Parameters; // 车辆参数
  TypeTraj Trajectory;  // 轨迹输出
  TypeDecider Decider;  // 决策量输出
  TypeRefline Refline;  //掉头参考线输出


  // so所需输入的赋值
	
  // 全局变量的初始化
  memset(&GlobVars, 0, sizeof(GlobVars));
  GlobVars.TrajPlanLaneChange.t_lc_traj=2;
  GlobVars.Decider.a_soll_pre=100;
  GlobVars.Decider.a_sollpre2traj=100;

	// 默认值的赋值
  LaneChangeActive=1; //换道场景激活
  PedestrianActive=0; //避让行人场景激活
  TrafficLightActive=0; //信号灯通行场景激活
  VehicleCrossingActive=0; //避让同向车场景激活
  VehicleOncomingActive=0; //避让对向车场景激活
  TurnAroundActive=0; //掉头场景激活
  

  BasicsInfo.currentLaneFrontDis=200;
  BasicsInfo.currentLaneFrontVel=20;
  BasicsInfo.currentLaneFrontLen=5;
  BasicsInfo.pos_l_CurrentLane=0;
  BasicsInfo.sampleTime=0.2; 

  memset(&AvoFailVehInfo, 0, sizeof(AvoFailVehInfo));
  memset(AvoFailVehInfo.failLaneFrontLen, 5, sizeof(AvoFailVehInfo.failLaneFrontLen));

  LaneChangeInfo.d_veh2int = 200;
  LaneChangeInfo.leftLaneBehindDis=-200;
  LaneChangeInfo.leftLaneBehindVel=20;
  LaneChangeInfo.leftLaneFrontDis=200;
  LaneChangeInfo.leftLaneFrontVel=20;
  LaneChangeInfo.rightLaneBehindDis=-200;
  LaneChangeInfo.rightLaneBehindVel=20;
  LaneChangeInfo.rightLaneFrontDis=200;
  LaneChangeInfo.rightLaneFrontVel=20;
  LaneChangeInfo.leftLaneBehindLen=5;
  LaneChangeInfo.leftLaneFrontLen=5;
  LaneChangeInfo.rightLaneBehindLen=5;
  LaneChangeInfo.rightLaneFrontLen=5;

  memset(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle,-200,sizeof(AvoMainRoVehInfo.targetLaneBehindDisAvoidVehicle));
  memset(AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle,20,sizeof(AvoMainRoVehInfo.targetLaneBehindVelAvoidVehicle));
  memset(AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle,5,sizeof(AvoMainRoVehInfo.targetLaneBehindLenAvoidVehicle));
  memset(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle,200,sizeof(AvoMainRoVehInfo.targetLaneFrontDisAvoidVehicle));
  memset(AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle,20,sizeof(AvoMainRoVehInfo.targetLaneFrontVelAvoidVehicle));
  memset(AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle,5,sizeof(AvoMainRoVehInfo.targetLaneFrontLenAvoidVehicle));
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
  double phase[]={30,-3,-30,-3,30,0,0,0,0,0};
  for (int i = 0;i < 10;i++) {
    TrafficLightInfo.phase[i]=phase[i];  // 当前车道信号灯相位
  }
  TrafficLightInfo.d_veh2stopline=200;

  memset(&TurnAroundInfo, 0, sizeof(TurnAroundInfo));
  memset(TurnAroundInfo.posSOppositeCar, -200, sizeof(TurnAroundInfo.posSOppositeCar));
  memset(TurnAroundInfo.speedCodirectCar, -1, sizeof(TurnAroundInfo.speedCodirectCar));
  memset(TurnAroundInfo.posSCodirectCar, -200, sizeof(TurnAroundInfo.posSCodirectCar));

  Parameters.turningRadius=5;
  Parameters.w_veh=1.8;
  Parameters.l_veh=5;

  CalibrationVars.TrajPlanTurnAround.d_safe1=0.5;
  CalibrationVars.TrajPlanTurnAround.d_safe2=2;
  CalibrationVars.TrajPlanTurnAround.dec2line=0.2;
  CalibrationVars.TrajPlanTurnAround.a_min=-2;
  CalibrationVars.TrajPlanTurnAround.a_max_com=1.5;
  CalibrationVars.TrajPlanTurnAround.v_max_turnAround=5;
  CalibrationVars.TrajPlanTurnAround.d_gap2stop=0.1;
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
  CalibrationVars.SpeedPlanTrafficLight.d_gap2stopline=0.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_min_com=-1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_max=1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.a_min=-3;
  CalibrationVars.SpeedPlanAvoidVehicle.v_max=40/3.6;
  CalibrationVars.SpeedPlanAvoidVehicle.t_re=1.5;
  CalibrationVars.SpeedPlanAvoidVehicle.gapIndex=2;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_max_com=1.5;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.a_min=-3;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.v_max_int=30/3.6;
  CalibrationVars.SpeedPlanAvoidOncomingVehicle.d_safe=2;
  CalibrationVars.TrajPlanLaneChange.v_max_int=30/3.6;
  CalibrationVars.TrajPlanLaneChange.indexAfterLaneChangeDis2Int=1;
  CalibrationVars.TrajPlanLaneChange.t_permit=3;
  CalibrationVars.TrajPlanLaneChange.t_re=0.5;
  CalibrationVars.TrajPlanLaneChange.index_accel=0.5;
  CalibrationVars.TrajPlanLaneChange.a_max_comfort=1;
  CalibrationVars.TrajPlanLaneChange.a_min=-3.5;
  CalibrationVars.TrajPlanLaneChange.a_max=2.5;
  CalibrationVars.TrajPlanLaneChange.a_min_comfort=-1;
  CalibrationVars.TrajPlanLaneChange_RePlan.frontWheelAnglelLimit=15; 
  CalibrationVars.TrajPlanLaneChange.a_lateral=4; 
  CalibrationVars.ACC.a_max=2.5;
  CalibrationVars.ACC.a_min=-4;
  CalibrationVars.ACC.d_wait2faultyCar=13;
  CalibrationVars.ACC.tau_v_com=4;
  CalibrationVars.ACC.tau_v=2;
  CalibrationVars.ACC.tau_d=5;
  CalibrationVars.ACC.tau_v_bre=1;
  CalibrationVars.ACC.tau_v_emg=0.5;
  CalibrationVars.ACC.tau_d_emg=2;
  CalibrationVars.ACC.t_acc=2;
  CalibrationVars.ACC.d_wait=4;
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
  CalibrationVars.Decider.glosaAdp=1.5;
  CalibrationVars.Decider.mrg=4;
  CalibrationVars.Decider.desRate=0.75;
  CalibrationVars.Decider.dIntxn=10;
  CalibrationVars.Decider.dMin=2;
  CalibrationVars.Decider.dec=1;
  CalibrationVars.Decider.glosaAverageIndex=0.8;
  CalibrationVars.Decider.d_veh2endpoint = 0.2;

  // 动态配置量的赋值（请对照ppt“城区规划器主流程设计集相关需求”的算法输入部分）
  PlannerLevel=1; //车端请求云端规划级别，对应辅助决策车云协议里planLevel
  GlosaActive=0; //车端请求云端激活GLOSA标志位

  BasicsInfo.pos_s=100;  //  车道frenet坐标系下s坐标
  BasicsInfo.pos_l=0.3; //  车道frenet坐标系下l坐标
  BasicsInfo.pos_psi = 90.0; //  车道frenet坐标系下航向角
  BasicsInfo.currentLaneIndex=2;  // 当前车道序号
  double WidthOfLanes[]={3.61,3.62,3.63,3.64,0,0};
  for (int i = 0;i < 6;i++) {
    BasicsInfo.widthOfLanes[i]=WidthOfLanes[i];  // 当前道路车道宽度序列
  }
  BasicsInfo.targetLaneIndex=1;  // 目标车道序号
  BasicsInfo.v_max=60/3.6;  // 当前车道限速
  BasicsInfo.goalLaneIndex=1; // 终点靠边车道序号
  BasicsInfo.d_veh2goal=200; // 与终点距离
  
  LaneChangeInfo.d_veh2int=70;  // 本车车头距道路frenet坐标系结束位置的距离

  ChassisInfo.speed=10;  // 车速
  ChassisInfo.currentGear=4;  // 当前档位

  
  // so的链接

  void *handle = dlopen("./UrbanPlanner.so", RTLD_LAZY);
  if (!handle) {
      fprintf (stderr, "%s\n", dlerror());
      exit(1);
  }
  printf("lib loaded\n");

  void (*myFn)(TypeBasicsInfo*,TypeChassisInfo*,TypeLaneChangeInfo* ,TypeAvoMainRoVehInfo*, TypeAvoPedInfo* ,TypeTrafficLightInfo*, TypeAvoOncomingVehInfo* ,TypeAvoFailVehInfo*, 
  TypeTurnAroundInfo* ,TypeStopSignInfo* ,short, short,short, short ,short, short, short,short, TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDecider*,TypeRefline*) 
  = (void(*)(TypeBasicsInfo*,TypeChassisInfo*, TypeLaneChangeInfo* ,TypeAvoMainRoVehInfo*, TypeAvoPedInfo* ,TypeTrafficLightInfo*, TypeAvoOncomingVehInfo* ,TypeAvoFailVehInfo*, 
  TypeTurnAroundInfo* ,TypeStopSignInfo* ,short, short,short, short ,short, short, short ,short,TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDecider*,TypeRefline*) ) dlsym(handle, "UrbanPlanner");

  if (myFn == NULL){
      printf("err\n");
      return -1;
  }

  // so的调用

  myFn(&BasicsInfo, &ChassisInfo,&LaneChangeInfo, &AvoMainRoVehInfo, &AvoPedInfo, &TrafficLightInfo, &AvoOncomingVehInfo, &AvoFailVehInfo, &TurnAroundInfo, &StopSignInfo,  LaneChangeActive, 
                    PedestrianActive,  TrafficLightActive,  VehicleCrossingActive,  VehicleOncomingActive,  TurnAroundActive, GlosaActive, PlannerLevel, &GlobVars, &CalibrationVars, &Parameters, &Trajectory, &Decider, &Refline);


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
	  std::cout << Trajectory.planning_states << "\t";
  }
  int i=43;
  std::cout << "\n第"<<i+1<<"帧轨迹"<< "\n";
  std::cout << Trajectory.traj_s[i]<< "\t";
  std::cout << Trajectory.traj_l[i]<< "\t";
  std::cout << Trajectory.traj_psi[i]<< "\t";
  std::cout << Trajectory.traj_vs[i]<< "\t";
  std::cout << Trajectory.traj_vl[i]<< "\t";
  std::cout << Trajectory.traj_omega[i]<< "\t";
  std::cout << Trajectory.planning_states << "\t";
  std::cout << "\n\n全局变量DurationLaneChange"<< "\t";
  std::cout << GlobVars.TrajPlanLaneChange.durationLaneChange << "\n\n";
  // 注意GlobVars是全局变量！

	/* 调用结果
	--------------------------------------------------output--------------------------------------------------

	第1帧轨迹
	100.501	0.300377	89.872	10.0213	0.0223945	0	0	
	第2帧轨迹
	101.002	0.302923	89.5105	10.0413	0.0857943	-7.22991	0
	第3帧轨迹
	101.505	0.309547	88.9487	10.0592	0.184598	-11.2358	0
	第4帧轨迹
	102.008	0.321881	88.2188	10.0744	0.313288	-14.5972	0
	第5帧轨迹
	102.512	0.341283	87.3522	10.0858	0.466422	-17.3318	0
	第44帧轨迹
	122	3.615	90	10	0	2.56076		0

	全局变量DurationLaneChange	2

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
