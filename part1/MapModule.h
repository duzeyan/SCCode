#include<vector>
#include<string>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include<algorithm>

using namespace std;




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
	void LoadMapTask(vector<int> &mapTaskNode);

	//加载比赛方提供的原始任务文件
	void LoadTask();

	//加载自建地图的邻接矩阵 
	void LoadAdjMat(NJUST_MAP_BUILD_MAP &map);

	//加载特定路口GPS序列
	void ReadMapNode(const char* filename);

	//加载特定道路GPS序列
	void ReadMapLine(const char* filename);
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
	//文本存储类型转化到内存路口
	void static Node2ButtonNode(MAP_NODE &node,MAP_BUTTON_NOTE &buttonNode);

	//文本存储类型转化为算法道路
	void static Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine);
};



// Map模块主类（业务和算法）
class MapApp{
private:
	MapFileStream* mapFile;          //调用文件类
	bool exitFlag;                   //MAP模块结束标志
	bool checkExpOut;                //检查是否异常退出
	NJUST_MAP_BUILD_MAP map;		 //自建地图所有信息
	vector<int> mapTaskNode;		 //任务文件中给出的路径，转化成自建地图中的【路口编号序列】
	NJUST_MAP_GPS_INFO GPSInfo;		 //MC发送过来的GPS信息
	NJUST_PLAN_PATH planPath;				 //规划的路径


public:  
	//构造函数
	MapApp(const char* loadpath);
	~MapApp();

	//运行MAP模块 规划数据来自指定目录下
	void Run(const char* loadpath);

	//初始化参数 
	void Intialize();

	//响应PL
	int PLCallBack(void* pl_to_map, size_t size, void* args);

	//响应MC
	int MCCallBack(void* mc_to_map, size_t size, void* args);

	//响应MO
	int MOCallBack(void* mo_to_map, size_t size, void* args);

	//规划路径(选取自建地图中的节点或道路)
	void PathPlaning();

	//车启动时 定位到任务文件中的路口处start 以及下一个路口end
	void StartPlan(int& srart,int& end);

	//释放持有内存
	void Release(); 
};