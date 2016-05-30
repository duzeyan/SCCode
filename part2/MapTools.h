#ifndef _NJUST_MAP_TOOLS_H_
#define _NJUST_MAP_TOOLS_H_

#include<cmath>
#include<errno.h>
#include<string.h>
#include"MAP__BASIC_data.h"
#include "NJUST_ALV_BYD.h"




//Map 通用工具类
class MapTools{
public:
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
	int static GetNodeIndexByID(const vector<MAP_BUTTON_NOTE> &nodes,int id);

	//根据距离且在给出GPS序列点上的障碍物对象指针，没有符合条件返回NULL
	//当前经纬度(单位度),设定距离(m) 
	static void GetObsByDistance(double lng,double lat,
								double distanceM,const vector<NJUST_MAP_OBSTACLE> &obs
								,vector<NJUST_MAP_OBSTACLE> &outObs);

	//通过路口ID获取路口属性
	MAP_BUTTON_NOTE static GetNodeByID(const vector<MAP_BUTTON_NOTE> &nodes,int id);

	//通过道路ID获得道路索引
	int static GetLineIndexByID(const vector<MAP_BUTTON_LINE> &lines,int id);

	//通过道路ID获得道路属性
	MAP_BUTTON_LINE static GetLineByID(const vector<MAP_BUTTON_LINE> &lines,int id);


	//转化成公共头文件道路结构
	void static StructTransformLine(MAP_BUTTON_LINE* line, NJUST_MAP_INFO_ROAD **road);

	//转化为公共头文件路口结构体
	void static StructTransformNote(MAP_BUTTON_NOTE* note, NJUST_MAP_INFO_NODE **node);

	//编码MAP数据包
	 int static NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[]);

	 //通过向量获取夹角
	 double static GetRotateAngle(double x1, double y1, double x2, double y2);//以度为单位

	 //ID转化为编号
	 int static ID2Code(int id);


	 //编号转化为IDtype: 1,节点node 2.道路road 3.。。
	 int static Code2ID(int code,int type);
};

#endif