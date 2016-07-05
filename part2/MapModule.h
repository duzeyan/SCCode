
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
#include"MAP_BASIC_data.h"
#include"MapCommunion.h"
#include"MapFileStream.h"
#include"MapTools.h"

using namespace std;

///已知BUG
///1.起始点如果靠近终点，导致车启动时定位到终点，在第一次进行pathplaning会出现越界错误
///2.未完成断电重启功能

//线程锁,同步读存数据
extern pthread_mutex_t     gMutex ;
extern pthread_cond_t      cond ;

// Map模块主类（业务和算法）
class MapApp{
private:
	MapFileStream* _mapFile;			    //文件类
	bool _exitFlag;						   //MAP模块结束标志
	bool _checkExpOut;					   //检查是否异常退出
	NJUST_MAP_BUILD_MAP _map;			   //自建地图信息
	vector<MAP_TASK_NODE> _mapTaskNode;	   //任务文件中给出的路径，转化成自建地图中的【路口编号序列】
	NJUST_PLAN_PATH _planPath;			   //规划的路径 编号
	vector<MAP_DOUBLE_POINT> _GPSList;     //当前道路或者路口的GPS序列 单位是分
	int _frameNum;					       //MAP自统计帧数(pl,mo接受）
	FILE *_pRecord;					       //文件指针(状态文件的)
	

public :
	static NJUST_MAP_GPS_INFO s_GPSInfo;		 //MC发送过来的GPS信息
	static MAP_PACKAGE s_mapPackage;			//打给MO状态包 主要是丢包率

public:  
	
	//************************************
	// 描述：构造函数
	// 返回值: 无
	// 参数:   const char * loadpath 地图信息目录
	//************************************
	MapApp();
	~MapApp();

	
	//************************************
	// 描述：运行MAP模块 规划数据来自指定目录下
	// 返回值: void
	//************************************
	void run();

	//************************************
	// 描述：模拟运行MAP模块,生成结果存在特定文件中
	// 返回值: void
	//************************************
	void simulate();

	//************************************
	// 描述：将结果存在特定文件中
	// 返回值: void
	//************************************
	void simOutResult(const vector<MAP_DOUBLE_POINT> &list);

	//************************************
	// 描述：模拟运行MAP模块,生成结果存在特定文件中
	// 返回值: void
	//************************************
	void simMakeGPS(double lng,double lat,vector<MAP_DOUBLE_POINT> &GPSList);
	
	//************************************
	// 描述：初始化参数 
	// 返回值: void
	// 参数:   const char * loadpath  地图信息目录
	//************************************
	void intialize(const char* loadpath);

	
	//************************************
	// 描述：监听PL的回调函数，主要接受PL命令
	// 返回值: int
	// 参数:   void * pl_to_map 数据包
	// 参数:   size_t size      大小
	// 参数:   void * args      额外参数
	//************************************
	int static PLCallBack(void* pl_to_map, size_t size, void* args);

	//************************************
	// 描述：监听MC的回调函数，主要存储MC发来的GPS信息
	// 返回值: int
	// 参数:   void * pl_to_map 数据包
	// 参数:   size_t size      大小
	// 参数:   void * args      额外参数
	//************************************
	int static MCCallBack(void* mc_to_map, size_t size, void* args);

	//************************************
	// 描述：监听MO的回调函数，主要接受响应MO的控制信息
	// 返回值: int
	// 参数:   void * pl_to_map 数据包
	// 参数:   size_t size      大小
	// 参数:   void * args      额外参数
	//************************************
	int static MOCallBack(void* mo_to_map, size_t size, void* args);

	
	//************************************
	// 描述：规划路径(选取自建地图中的节点)比如编号为1,4的节点 
	//		 则规划结构如:[1],1,[2],2,[3],3,[4] 
	//       以上[]中为路口编号,剩下的为道路编号
	//       结果存在_planPath中
	// 返回值: void
	// 参数:   int s 出发处节点的编号
	// 参数:   int e 终点处节点编号
	//************************************
	void pathPlaning(int s,int e);

	
	//************************************
	// 描述：  车启动时,定位到任务文件中的路口处
	// 返回值: void
	// 参数:   int & srart [out]返回定位结果
	//************************************
	void startPlan(int& srart);

	
	//************************************
	// 描述：  用Dijkstra算法找出自建地图中s路口到e路口的最短路径
	// 返回值: void
	// 参数:   int s 开始路口的编号
	// 参数:   int e 终点路口的编号
	// 参数:   vector<int> & [out]规划出的节点序列,内容为编号
	//************************************
	void dijkstra(int s,int e,vector<int>&);

	
	//************************************
	// 描述： 在车运行时,通过当前经纬度计算出车在自建地图中的位置
	// 返回值: int  ID   定位到的ID值 优先返回节点ID,其次为道路ID
	// 参数:   double lng 经度，单位度
	// 参数:   double lat 维度，单位度
	//************************************
	int location(double lng,double lat);

	
	//************************************
	// 描述： 判断经纬度是否存在某条道路主方向(南北或东西)中
	// 返回值: bool   true表示符合
	// 参数:   double lng 经度,单位度
	// 参数:   double lat 维度,单位度
	// 参数:   int lineID 道路ID
	//************************************
	bool isInLine(double lng,double lat,int lineID);

	//
	//************************************
	// 描述：根据当前GPS值,返回GPS序列中的索引
	// 返回值: int index   _GPSList GPS序列索引
	// 参数:   double lng 经度,单位度
	// 参数:   double lat 维度,单位度
	//************************************
	int locationGPS(double lng,double lat);

	
	//************************************
	// 描述：  广播道路信息
	// 返回值: void 
	// 参数:   double lng 经度,单位度
	// 参数:   double lat 维度,单位度
	// 参数:   int curID  当前道路ID 道路x
	// 参数:   int nextIndex 道路x终点的路口索引
	//************************************
	void sendRoad(double lng,double lat,int curID,int nextIndex);

	
	//************************************
	// 描述：  广播路口信息   
	// 返回值: void
	// 参数:   double lng 经度,单位度
	// 参数:   double lat 维度,单位度
	// 参数:   int curID   该路口ID
	// 参数:   int lastIndex  上一个路口索引
	// 参数:   int nextIndex  下一个路口索引
	//************************************
	void sendNode(double lng,double lat,int curID,int lastIndex,int nextIndex);

	
	//************************************
	// 描述：  给MO发送状态数据
	// 返回值: void
	// 参数:   char buff[] 打包后的数据
	// 参数:   int n       数据长度
	//************************************
	void send2Mo(char buff[],int n);

	
	//************************************
	// 描述： 计算路口的转弯方向
	// 返回值: void
	// 参数:   NJUST_MAP_INFO_NODE & node
	// 参数:   int curIndex  当前节点ID
	// 参数:   int lastIndex 最近节点ID
	// 参数:   int nextIndex 下一个要路过节点ID
	//************************************
	void getDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex);

	
	//************************************
	// 描述：  检查location是否合理,错误则输出到日志
	// 返回值: bool		是否合理
	// 参数:   int ID   location计算出的ID
	//************************************
	bool checkLoaction(int ID);

	
	//************************************
	// 描述：  写当前状态
	// 返回值: void
	// 参数:   int curID   当前ID
	// 参数:   int lastID  刚经过路口的ID
	//************************************
	void recordWrite(int curID,int lastID);

	//************************************
	// 描述：  读上次意外退出的状态
	// 返回值: void
	// 参数:   int & curID 读取存储的当前ID
	// 参数:   int & lastID 读取存储的前ID
	//************************************
	void recordRead(int &curID,int &lastID);
	
	//************************************
	// 描述：  释放持有内存
	// 返回值: void
	//************************************
	void release(); 
};

#endif



