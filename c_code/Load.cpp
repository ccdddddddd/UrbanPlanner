
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
} TypeLaneChangeInfo;

typedef struct { // 定义避让同向车子功能所需信息结构体
  double CurrentLaneFrontDisAvoidVehicle;
  double CurrentLaneFrontVelAvoidVehicle;
  double TargetLaneBehindDisAvoidVehicle;
  double TargetLaneBehindVelAvoidVehicle;
  double TargetLaneFrontDisAvoidVehicle;
  double TargetLaneFrontVelAvoidVehicle;
  double d_veh2converge;
  double d_veh2stopline;
} TypeAvoMainRoVehInfo;

typedef struct { // 定义避让行人子功能所需信息结构体
  double d_veh2cross;
  double w_cross;
  double s_ped;
  double v_ped;
} TypeAvoPedInfo;

typedef struct { // 定义信号灯通行决策子功能所需信息结构体
  double greenLight;
  double time2nextSwitch;
  double d_veh2stopline;
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
} CalibSpeedPlanAvoidPedestrian;

typedef struct {// 定义信号灯通行决策子功能所需标定量结构体
  double a_min_com;
  double a_max;
  double a_min;
  double v_max;
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
} CalibACClowSpeed;

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
} TypeCalibrationVars;

typedef struct {// 定义车辆参数结构体
  double TurningRadius; // 转弯半径
  double w_veh; // 车辆宽度
  double l_veh; // 车辆长度
} TypeParameters;

typedef struct { // 定义基本信息结构体
  double CurrentLaneFrontDis; // 前车距离
  double CurrentLaneFrontVel; // 前车速度
  double pos_s; //  车道frenet坐标系下s坐标
  double pos_l; //  车道frenet坐标系下l坐标
  double pos_l_CurrentLane; // 车道frenet坐标系下当前车道中心线l坐标
  short CurrentLaneIndex; // 当前车道序号
  double WidthOfLanes[6]; // 当前道路车道宽度序列
  short TargetLaneIndex; // 目标车道序号
  double v_max; // 当前车道限速
} TypeBasicsInfo;

typedef struct { // 定义避让对向车子功能所需信息结构体
  double d_veh2waitingArea;
  double s_veh[6];
  double v_veh[6];
  double d_veh2conflict[6];
  double s_veh1apostrophe[6];
} TypeAvoOncomingVehInfo;

typedef struct { // 定义避让故障车子功能所需信息结构体
  short LanesWithFail[6];
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
} TypeGlobVars;

typedef struct { // 定义决策量输出
  double Driverwait;
  short wait_avoidOncomingVehicle;
  short wait_ped;
  short wait_AvoidVehicle;
  short wait_TrafficLight;
  short wait_turnAround;
  double a_soll;
  short AEBActive;
  short TargetGear;
} TypeDesider;

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
  short LaneChangeActive; // 换道子功能激活标识
  short PedestrianActive; // 避让行人子功能激活标识
  short TrafficLightActive; // 信号灯通行决策子功能激活标识
  short VehicleCrossingActive; // 避让同向车子功能激活标识
  short VehicleOncomingActive; // 避让对向车子功能激活标识
  short TurnAroundActive; // 掉头子功能激活标识
  TypeGlobVars GlobVars; // 全局变量
  TypeCalibrationVars CalibrationVars; // 标定量
  TypeParameters Parameters; // 车辆参数
  TypeTraj Trajectory;  // 轨迹输出
  TypeDesider Desider;  // 决策量输出


  // so所需输入的赋值
	
  // 全局变量的初始化
  memset(&GlobVars, 0, sizeof(GlobVars));
  GlobVars.TrajPlanLaneChange.t_lc_traj=2;

	// 默认值的赋值
  LaneChangeActive=1;
  PedestrianActive=0;
  TrafficLightActive=0;
  VehicleCrossingActive=0;
  VehicleOncomingActive=0;
  TurnAroundActive=0;

  BasicsInfo.CurrentLaneFrontDis=200;
  BasicsInfo.CurrentLaneFrontVel=20;

  memset(AvoFailVehInfo.LanesWithFail, 0, sizeof(AvoFailVehInfo.LanesWithFail));

  LaneChangeInfo.d_veh2int = 200;
  LaneChangeInfo.LeftLaneBehindDis=-200;
  LaneChangeInfo.LeftLaneBehindVel=20;
  LaneChangeInfo.LeftLaneFrontDis=200;
  LaneChangeInfo.LeftLaneFrontVel=20;
  LaneChangeInfo.RightLaneBehindDis=-200;
  LaneChangeInfo.RightLaneBehindVel=20;
  LaneChangeInfo.RightLaneFrontDis=200;
  LaneChangeInfo.RightLaneFrontVel=20;

  AvoMainRoVehInfo.CurrentLaneFrontDisAvoidVehicle=200;
  AvoMainRoVehInfo.CurrentLaneFrontVelAvoidVehicle=20;
  AvoMainRoVehInfo.TargetLaneBehindDisAvoidVehicle=-200;
  AvoMainRoVehInfo.TargetLaneBehindVelAvoidVehicle=20;
  AvoMainRoVehInfo.TargetLaneFrontDisAvoidVehicle=200;
  AvoMainRoVehInfo.TargetLaneFrontVelAvoidVehicle=20;
  AvoMainRoVehInfo.d_veh2converge=200;
  AvoMainRoVehInfo.d_veh2stopline=200;

  memset(&AvoOncomingVehInfo, 0, sizeof(AvoOncomingVehInfo));
  memset(AvoOncomingVehInfo.s_veh, 200, sizeof(AvoOncomingVehInfo.s_veh));
  memset(AvoOncomingVehInfo.s_veh1apostrophe, -200, sizeof(AvoOncomingVehInfo.s_veh1apostrophe));
  AvoOncomingVehInfo.d_veh2waitingArea=200;

  AvoPedInfo.d_veh2cross=200;
  AvoPedInfo.w_cross=6.4;
  AvoPedInfo.s_ped=100;
  AvoPedInfo.v_ped=0;

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
  CalibrationVars.SpeedPlanAvoidPedestrian.a_min=-3;
  CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int=30/3.6;
  CalibrationVars.SpeedPlanAvoidPedestrian.v_max_int_emg=20/3.6;
  CalibrationVars.SpeedPlanTrafficLight.a_min_com=-1.5;
  CalibrationVars.SpeedPlanTrafficLight.a_max=2.5;
  CalibrationVars.SpeedPlanTrafficLight.a_min=-3;
  CalibrationVars.SpeedPlanTrafficLight.v_max=14;
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

	// 动态配置量的赋值（请对照ppt“城区规划器主流程设计集相关需求”的算法输入部分）
  BasicsInfo.pos_s=100;  //  车道frenet坐标系下s坐标
  BasicsInfo.pos_l=0.3; //  车道frenet坐标系下l坐标
  BasicsInfo.CurrentLaneIndex=2;  // 当前车道序号
  double WidthOfLanes[]={3.61,3.62,3.63,3.64,0,0};
  for (int i = 0;i < 6;i++) {
    BasicsInfo.WidthOfLanes[i]=WidthOfLanes[i];  // 当前道路车道宽度序列
  }
  BasicsInfo.TargetLaneIndex=1;  // 目标车道序号
  BasicsInfo.v_max=60/3.6;  // 当前车道限速
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
  TypeTurnAroundInfo* ,short, short,short, short ,short, short,TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDesider*) 
  = (void(*)(TypeBasicsInfo*,TypeChassisInfo*, TypeLaneChangeInfo* ,TypeAvoMainRoVehInfo*, TypeAvoPedInfo* ,TypeTrafficLightInfo*, TypeAvoOncomingVehInfo* ,TypeAvoFailVehInfo*, 
  TypeTurnAroundInfo* ,short, short,short, short ,short, short,TypeGlobVars*,TypeCalibrationVars*,TypeParameters*,TypeTraj*,TypeDesider*) ) dlsym(handle, "UrbanPlanner");

  if (myFn == NULL){
      printf("err\n");
      return -1;
  }


  // so的调用

  myFn(&BasicsInfo, &ChassisInfo,&LaneChangeInfo, &AvoMainRoVehInfo, &AvoPedInfo, &TrafficLightInfo, &AvoOncomingVehInfo, &AvoFailVehInfo, &TurnAroundInfo,  LaneChangeActive, 
                    PedestrianActive,  TrafficLightActive,  VehicleCrossingActive,  VehicleOncomingActive,  TurnAroundActive, &GlobVars, &CalibrationVars, &Parameters, &Trajectory, &Desider);


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
	std::cout << "\n\n全局变量DurationLaneChange"<< "\n";
	std::cout << GlobVars.TrajPlanLaneChange.DurationLaneChange << "\t";
	std::cout << "\n\n--------------------------------------------------output--------------------------------------------------\n";
  // 注意GlobVars是全局变量！

}
