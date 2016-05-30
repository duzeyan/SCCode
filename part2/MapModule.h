
#ifndef _NJUST_MAP_APP_H_
#define _NJUST_MAP_APP_H_


#include<vector>
#include<string.h>
#include<string>
#include<algorithm>
#include<errno.h>
#include<cmath>
#include<limits>
#include<stdio.h>

#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include"MapCommunion.h"
#include"MapFileStream.h"
#include"MapTools.h"

using namespace std;




extern pthread_mutex_t     gMutex ;
extern pthread_cond_t      cond ;







// Map模块主类（业务和算法）
class MapApp{
private:
	MapFileStream* _mapFile;          //调用文件类
	bool _exitFlag;                   //MAP模块结束标志
	bool _checkExpOut;                //检查是否异常退出
	NJUST_MAP_BUILD_MAP _map;		 //自建地图所有信息
	vector<MAP_TASK_NODE> _mapTaskNode;		 //任务文件中给出的路径，转化成自建地图中的【路口编号序列】
	NJUST_PLAN_PATH _planPath;				 //规划的路径 编号
	vector<MAP_DOUBLE_POINT> _GPSList;//当前道路或者路口的GPS序列
	int _frame_pl;					//发给PL的帧数
	FILE *pRecord;					//状态文件的文件指针
	

public :
	static NJUST_MAP_GPS_INFO GPSInfo;		 //MC发送过来的GPS信息
	static MAP_PACKAGE mappackage;			//计算丢包率

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

	//给MO发送数据 数据 数据长度
	void Send2Mo(char buff[],int n);

	//计算路口的转弯方向
	void GetDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex);

	//检查location是否合理 错误输出到日志
	bool CheckLoaction(int ID);

	//填写状态文件
	void RecordWrite(int curID,int lastID);

	//释放持有内存
	void Release(); 
};

#endif



///已知漏洞
///起始点如果靠近终点，导致车启动时定位到终点，在第一次进行pathplaning会出现越界错误