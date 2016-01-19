
#ifndef _NJUST_MAP_APP_H_
#define _NJUST_MAP_APP_H_


#include<vector>
#include<string.h>
#include<string>
#include<algorithm>
#include<errno.h>
#include<cmath>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"


using namespace std;


#define _MAP_LOG 1
#if _MAP_LOG
	#define MAP_PRINT(fmt,v) printf(fmt,v);fflush(stdout)
#else
    #define MAP_PRINT(fmt,v) 
#endif

extern pthread_mutex_t     gMutex ;
extern pthread_cond_t      cond ;



//Map模块文件读取工具类
class MapFileStream{
private:
	char loadpath[20];			//数据文件目录
public:
	//构造函数 需要传入 数据文件的目录
	MapFileStream(const char* loadpath);

	//加载自建地图 路点信息
	void LoadMapNode(NJUST_MAP_BUILD_MAP &map);

	//加载自建地图中 任务路点(需要规划)
	void LoadMapTask(vector<MAP_TASK_NODE> &mapTaskNode);

	//加载比赛方提供的原始任务文件
	void LoadTask();

	//加载自建地图的邻接矩阵 注意，调用次方法前必须已经调用LoadMapNode 
	void LoadAdjMat(NJUST_MAP_BUILD_MAP &map);

	//加载指定路段gps序列  1+2.db 表示路口 1-2.db表示路段 isNode:true 节点 false 路段
	void ReadMapGPS(int a,int b,vector<MAP_DOUBLE_POINT> &GPSList,bool isnNode);



};

//Map模块通信工具类
class MapCommunion{
public:
	//初始化
	int Intialize();

	//在通信网络中 注册MAP模块
	int RegisterMap();

	//针对特定模块，注册回调函数
	int ReciveModuleMsg(const char * modulename,func_t func);

	//接收广播
	int ReciveBroadcastMsg(func_t func);
	
	//给PL发送规划路径
	int SendPL();
};

//Map 通用工具类
class MapTools{
	//初始化
public:
	//坐标转化

	//解码 编码数据包

	///数据类型转换
	///

	//文本存储类型转化到内存路口
	void static Node2ButtonNode(MAP_NODE &node,MAP_BUTTON_NOTE &buttonNode);

	//文本存储类型转化为算法道路
	void static Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine);

	//线程sleep
	void static ms_sleep( unsigned int msecs );

	//检测GPS值是否在合理区间内
	bool static CheckGPS(double lng,double lat,double lastlng,double lastlat);

	//纬度，经度(度），转化为大地坐标(cm)
	void static GPS2Earthy(double lat, double lng, int &earthx, int &earthy);

	//计算两对经纬度获取之间的直线距离(m)
	double static GetDistanceByGPS(double lng1,double lat1,double lng2,double lat2);

	//通过路口ID获得路口索引
	int static GetNodeIndexByID(const vector<MAP_BUTTON_NOTE> nodes,int id);

	//通过道路ID获得道路索引
	int static GetLineIndexByID(const vector<MAP_BUTTON_LINE> lines,int id);

	//转化成公共头文件道路结构
	void static StructTransformLine(MAP_BUTTON_LINE* line, NJUST_MAP_INFO_ROAD **road);

	//转化为公共头文件路口结构体
	void static StructTransformNote(MAP_BUTTON_NOTE* note, NJUST_MAP_INFO_NODE **node);

	//编码MAP数据包
	 int static NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[]);

	 //通过向量获取夹角
	 double static GetRotateAngle(double x1, double y1, double x2, double y2);//以度为单位

};

// Map模块主类（业务和算法）
class MapApp{
private:
	MapFileStream* mapFile;          //调用文件类
	bool exitFlag;                   //MAP模块结束标志
	bool checkExpOut;                //检查是否异常退出
	NJUST_MAP_BUILD_MAP map;		 //自建地图所有信息
	vector<MAP_TASK_NODE> mapTaskNode;		 //任务文件中给出的路径，转化成自建地图中的【路口编号序列】
	NJUST_PLAN_PATH planPath;				 //规划的路径
	vector<MAP_DOUBLE_POINT> GPSList;//当前道路或者路口的GPS序列
	//int curIndex;					//当前路口索引
	int frame_pl;					//发给PL的帧数
	

public :
	static NJUST_MAP_GPS_INFO GPSInfo;		 //MC发送过来的GPS信息

public:  
	//构造函数
	MapApp(const char* loadpath);
	~MapApp();

	//运行MAP模块 规划数据来自指定目录下
	void Run();

	//初始化参数 
	void Intialize(const char* loadpath);

	//响应PL
	int static PLCallBack(void* pl_to_map, size_t size, void* args);

	//响应MC
	int static MCCallBack(void* mc_to_map, size_t size, void* args);

	//响应MO
	int static MOCallBack(void* mo_to_map, size_t size, void* args);

	//规划路径(选取自建地图中的节点或道路)比如1,4 需要规划处1,1,2,2,3,3,4 
	void PathPlaning(int s,int e);

	//车启动时 定位到任务文件中的路口处start 以及下一个路口end
	void StartPlan(int& srart);

	//用Dijkstra找出自建地图中s路口到e路口的最短路径
	void Dijkstra(int s,int e,vector<int>&);

	//在车跑动的时候 确定车在自建地图中的位置
	int Location(double lng,double lat);

	//判断经纬度是否跟某条直线平行
	bool isInLine(double lng,double lat,int lineID);

	//在GPS序列中定位 当前位置的索引
	int LocationGPS(double lng,double lat);

	//发送信息到PL 该道路ID 下一个道路节点索引
	void Send2PL_Road(double lng,double lat,int curID,int nextIndex);

	//发送信息到PL 该路口ID 上一个路口索引 下一个路口索引
	void Send2PL_Node(double lng,double lat,int curID,int lastIndex,int nextIndex);

	//计算路口的转弯方向
	void GetDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex);

	//释放持有内存
	void Release(); 
};

#endif



///已知漏洞
///起始点如果靠近终点，导致车启动时定位到终点，在第一次进行pathplaning会出现越界错误