#ifndef _NJUST_MAP_FILE_H_
#define _NJUST_MAP_FILE_H_


#include<string.h>
#include<algorithm>
#include"MAP__BASIC_data.h"
#include "NJUST_ALV_BYD.h"

///
///Map模块文件读取工具类
///
class MapFileStream{
private:
	char loadpath[20];			//数据文件目录 不同场地放在不同目录下
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

#endif