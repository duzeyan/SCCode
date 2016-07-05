﻿#include<vector>
#include "NJUST_ALV_BYD.h"
#include"MAP_BASIC_data.h"
#include"MapModule.h"
#include <time.h>
#include <stdarg.h>
#include <unistd.h>
#include <fstream>

///
/// MapApp
///

#define RECORD_FILE_NAME "record.db"

//静态变量 定义
NJUST_MAP_GPS_INFO MapApp::s_GPSInfo;
MAP_PACKAGE MapApp::s_mapPackage;
pthread_mutex_t     gMutex= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t      cond =PTHREAD_COND_INITIALIZER;

FILE* gDEBUG_OUT=NULL;
FILE* gLOG_OUT=NULL;
void initDeBug(){
	char logName[1024] = {0};  

	time_t now;  
	time(&now);  
	struct tm *local;  
	local = localtime(&now);  
	sprintf(logName,"DEBUGMAP-%04d-%02d-%02d_%02d:%02d:%02d", local->tm_year+1900, local->tm_mon,  
		local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);  
	gDEBUG_OUT = fopen(logName,"a+");  
	printf("初始化日志");
	return ;  
}

void initLog(){
	char logName[1024] = {0};  
	time_t now;  
	time(&now);  
	struct tm *local;  
	local = localtime(&now);  
	sprintf(logName,"LOGMAP-%04d-%02d-%02d_%02d:%02d:%02d", local->tm_year+1900, local->tm_mon,  
		local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec);  
	gLOG_OUT = fopen(logName,"a+");  
	return ;  
}

#define START_DIS_THRESHOLD_M 30  //启动点距路口距离(单位M)

///
/// 编号 索引 ID的定义
/// 节点的ID为：10000~19999
/// 道路的ID为：20000~
/// 索引只在邻接矩阵处出现，且只针对节点而言
/// 编号，在作图软件中，路口道路已1开头 如道路1 道路2 节点1 
/// 另外GPS是按照编号方式进行命名的
/// 编号转ID为 编号-1+10000|20000


MapApp::MapApp(){
	//初始化
	//this->Intialize(loadpath); 
}

MapApp::~MapApp(){
	release();
}


void MapApp::run(){
	double curLng,curLat,lastLng,lastLat;

	//启动时定位
	int SIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	int lastNode=-1;						 //上一次经过的节点编号，用来判断当前方向

	//Step 1 -----------判断是否异常退出--------------
	if(_checkExpOut){ //异常退出
		recordRead(curID,lastID); //读取记录文件
	}else{           //正常启动
		startPlan(SIndex);
		if(SIndex==-1){
			MAP_PRINT("附近没有路口,无法启动 %d\n",SIndex);
			return;
		}
		pathPlaning(_mapTaskNode[SIndex].noderesult,_mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath

		//启动时的初始化
		lastID=MapTools::Code2ID(_planPath.planPathQueue[0],1);
		curID=lastID;
		//启动的特殊处理 认为启动点是路段 节点1-》节点2之间的读点
		_mapFile->ReadMapGPS(_planPath.planPathQueue[0],_planPath.planPathQueue[2],_GPSList,false);
		recordWrite(curID,lastID);//记录状态
	}

	
	//Step 2 -----------进入运行状态--------------
	while(1){
		//Step 2.1 -----------获取最新的GPS--------------
		pthread_mutex_lock(&gMutex);
		curLng=MapApp::s_GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::s_GPSInfo.curLatitude;   //最新维度 (度)
		lastLng=MapApp::s_GPSInfo.lastLongtitude;
		lastLat=MapApp::s_GPSInfo.lastlatitudel;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		
		MapTools::ms_sleep(20);		//控制接受频率 收发一次

		//Step 2.2 -----------丢弃不合理GPS数据--------------
		if (MapTools::CheckGPS(curLng,curLat,lastLng,lastLat))
		{			
			if((curID=location(curLng,curLat))==-1){
				MAP_PRINT("没有找到合适直线 ID为%d\n",curID);
				continue;
			}
			if(curID!=lastID){              //进入新的路口或道路 此时记录
				if(!checkLoaction(curID)){
					curID=lastID;goto CONTINUE;
				}
				DEBUG_PRINT("输出状态文件:","\n");
				recordWrite(curID,lastID); //记录最新状态

				_planPath.cur++;
				MAP_PRINT("=========================:%s\n","");
				if(_planPath.cur%2==0){
					MAP_PRINT("进入新路口:[路口%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}
				else{
					MAP_PRINT("进入新道路:[道路%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}

				size_t tcur=_planPath.cur;
				if(tcur<_planPath.planPathQueue.size()-1){//最近一次规划的路线没走完，读新路段文件
					MAP_PRINT("%s没有读完最近一次规划的节点\n","");
					bool isNode=(tcur%2==0);  //偶数是路口
					//读取指定段序列
					_mapFile->ReadMapGPS(_planPath.planPathQueue[tcur-1],
					_planPath.planPathQueue[tcur+1],_GPSList,
					isNode);
					//更新上一个路口编号
					if(isNode)
						lastNode=_planPath.planPathQueue[tcur-2];
					else
						lastNode=_planPath.planPathQueue[tcur-1];
					lastID=curID; //更新 lastID
				}else{								   //走到最近一次规划的最后一个节点
					MAP_PRINT("%s新的规划路线\n","");
					int lastRoad=_planPath.planPathQueue[_planPath.cur-1]; //记录最近一次规划走过的最后一段路，来构建路口过渡
					lastNode=_planPath.planPathQueue[_planPath.cur-2]; 
					SIndex++;
					if(SIndex+1==_mapTaskNode.size()){
						MAP_PRINT("到达最后的路口!%s","\n");
						remove(RECORD_FILE_NAME);  //删除记录文件
						break;
					}
					pathPlaning(_mapTaskNode[SIndex].noderesult,_mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath
					//读取过渡 路口信息
					_mapFile->ReadMapGPS(lastRoad,
					_planPath.planPathQueue[1],_GPSList,
					true);
					lastID=curID;
				}
			}//if find new ID
			CONTINUE:
			_frameNum++;
			
			//Step 3 -----------广播道路或路口信息--------------
			if(_planPath.cur%2==0){  //路口
				int lastIndex=MapTools::GetNodeIndexByID(_map.mapNode,lastNode-1+START_NODE_ID);
				int nextIndex=MapTools::GetNodeIndexByID(_map.mapNode,_planPath.planPathQueue[_planPath.cur+2]-1+START_NODE_ID);
				sendNode(curLng,curLat,curID,lastIndex,nextIndex);
			}else{                  //道路
				int theNestIndex=MapTools::GetNodeIndexByID(_map.mapNode,_planPath.planPathQueue[_planPath.cur+1]-1+START_NODE_ID);
				sendRoad(curLng,curLat,curID,theNestIndex);
			}
		}//checkGPS
	}//while
}//fun

void MapApp::simulate(){
	double curLng,curLat,lastLng,lastLat;    //GPS数据
	int SIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	int lastNode=-1;						 //上一次经过的节点编号，用来判断当前方向
	vector<MAP_DOUBLE_POINT> pathList;        //模拟运行中的输出结果
	vector<MAP_DOUBLE_POINT> wholeList;      //保存全局结果
	
	///Step 1 -----------启动初始化--------------
	SIndex=0;
	pathPlaning(_mapTaskNode[SIndex].noderesult,_mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath

	//启动时的初始化
	lastID=MapTools::Code2ID(_planPath.planPathQueue[0],1);
	curID=lastID;
	//启动的特殊处理 认为启动点是路段 节点1-》节点2之间的读点
	_mapFile->ReadMapGPS(_planPath.planPathQueue[0],_planPath.planPathQueue[2],_GPSList,false);
	lastLng=0.0;
	lastLat=0.0;
	curLng=_GPSList[0].x/60;
	curLat=_GPSList[0].y/60;
	//Step 2 -----------进入运行状态--------------
	while(1){
			if((curID=location(curLng,curLat))==-1){
				MAP_PRINT("没有找到合适直线 ID为%d\n",curID);
				continue;
			}
			if(curID!=lastID){              //进入新的路口或道路 此时记录
				if(!checkLoaction(curID)){
					curID=lastID;goto CONTINUE;
				}
				_planPath.cur++;
				MAP_PRINT("=========================:%s\n","");
				if(_planPath.cur%2==0){
					MAP_PRINT("进入新路口:[路口%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}
				else{
					MAP_PRINT("进入新道路:[道路%d]",MapTools::ID2Code(curID));
					MAP_PRINT("(%d)\n",curID);
				}

				size_t tcur=_planPath.cur;
				if(tcur<_planPath.planPathQueue.size()-1){//最近一次规划的路线没走完，读新路段文件
					MAP_PRINT("%s没有读完最近一次规划的节点\n","");
					bool isNode=(tcur%2==0);  //偶数是路口
					//读取指定段序列
					_mapFile->ReadMapGPS(_planPath.planPathQueue[tcur-1],
						_planPath.planPathQueue[tcur+1],_GPSList,
						isNode);
					//记录上一个路口编号
					if(isNode)
						lastNode=_planPath.planPathQueue[tcur-2];
					else
						lastNode=_planPath.planPathQueue[tcur-1];
					lastID=curID; //更新 lastID
				}else{								   //走到最近一次规划的最后一个节点
					MAP_PRINT("%s新的规划路线\n","");
					int lastRoad=_planPath.planPathQueue[_planPath.cur-1]; //记录最近一次规划走过的最后一段路，来构建路口过渡
					lastNode=_planPath.planPathQueue[_planPath.cur-2]; 
					SIndex++;
					if(SIndex+1==_mapTaskNode.size()){
						MAP_PRINT("到达最后的路口!%s","\n");
						break;
					}
					pathPlaning(_mapTaskNode[SIndex].noderesult,_mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath
					//读取过渡 路口信息
					_mapFile->ReadMapGPS(lastRoad,
						_planPath.planPathQueue[1],_GPSList,
						true);
					lastID=curID;
				}
			}//if find new ID
			CONTINUE:

			//Step 3 -----------定位记录GPS--------------
			simMakeGPS(curLng,curLat,pathList);
		    for(int i=0;i<pathList.size();i++){
				wholeList.push_back(pathList[i]);
			}
			DEBUG_PRINT("输出+%s","\n");
			//更新GPS
			lastLng=curLng;
			lastLat=curLat; 
			curLng=pathList.back().x/60; //经度维度
			curLat=pathList.back().y/60;
			DEBUG_PRINT("更新GPS lng:%lf \n",curLng);
			DEBUG_PRINT("更新GPS lat:%lf ",curLat);
	}//while
	simOutResult(wholeList);//输出结果
}

void MapApp::simOutResult(const vector<MAP_DOUBLE_POINT> &list){
	char *logName="simResult";  
	int i;
	FILE *pFile;

	pFile = fopen(logName,"w+"); 
	for(i=0;i<list.size();i++){
		fprintf(pFile,"%lf %lf\n",list[i].x,list[i].y);
	}
	fclose(pFile);
	return ;  
}

void MapApp::simMakeGPS(double lng,double lat,vector<MAP_DOUBLE_POINT> &GPSList){
	size_t indexGPS=locationGPS(lng,lat);
	GPSList.clear();
	//最多读20个GPS点
	int len=0;
	MAP_DOUBLE_POINT point;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			point.x = _GPSList[i].x;
			point.y =_GPSList[i].y;
			GPSList.push_back(point);
		}
	}
}

void MapApp::intialize(const char* loadpath){
	//Step 0 -----------声明--------------
	MapCommunion communion;

	//Step 1 -----------初始化通信模块--------------
	communion.RegisterMap();//注册本模块
	communion.ReciveModuleMsg("MO",MOCallBack);	//响应MO命令
	communion.ReciveBroadcastMsg(MCCallBack);//对MC发出的数据，进行记录

	//Step 1 -----------初始化日志系统--------------
	initDeBug();
	initLog();

	//Step 2 -----------从文件中读取地图数据--------------
	try{
		_mapFile=new MapFileStream(loadpath);
	}catch(const bad_alloc& e){
		MAP_PRINT("%s 实例化文件流对象失败\n","MapApp::Inialize");
		exit(-1);
	}
	_mapFile->LoadMapNode(_map);//加载自建地图:路口和道路信息
	_mapFile->LoadAdjMat(_map);//加载地图中的邻接表
	_mapFile->LoadMapTask(_mapTaskNode); //加载需要进一步规划的路径信息



	//Step 3 -----------接收的GPS信息初始化--------------
	MapApp::s_GPSInfo.curLongtitude=INITL_GPS_VALUE;
	_frameNum=0;//帧号初始化
	memset(&s_mapPackage,0,sizeof(MAP_PACKAGE));
	
	//Step 4 -----------初始化记录文件--------------
	if((access(RECORD_FILE_NAME,F_OK))==0){
		_checkExpOut=true;
	}else{
		_checkExpOut=false;
	}
}


void MapApp::startPlan(int& start){
	double curLng,curLat;
	double dis=0;//启动点附近距离 单位m

	//Step 1 -----------等待接收到有效数据--------------
	while(1){
		pthread_mutex_lock(&gMutex);
		curLng=MapApp::s_GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::s_GPSInfo.curLatitude;   //最新维度 (度)
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		if(curLng==INITL_GPS_VALUE)
			MapTools::ms_sleep(100);
		else
			break;
		MAP_PRINT("%s挂起100ms 等待MC发送坐标\n","");

	}

	//Step 2 -----------获取最近的节点--------------
	DEBUG_PRINT("当前URLNG %lfm \n",curLng);
	DEBUG_PRINT("当前URLAT %lfm \n",curLat);
	for(unsigned int i=0;i<_mapTaskNode.size();i++){
		dis=MapTools::GetDistanceByGPS(curLng,curLat,_mapTaskNode[i].longtitude,_mapTaskNode[i].latitude);
		if(dis<START_DIS_THRESHOLD_M){
			MAP_PRINT("离最近节点相差%lfm \n",dis);
			start=i;
			break;
		}
	}
	DEBUG_PRINT("失败搜索最近节点相差%lfm \n",dis);
	return;
}

int MapApp::location(double lng,double lat){
	//Step 1 -----------寻找离该GPS最近的路口--------------
	double min=5000;//5km
	double dis=0;
	int minNodeIndex=-1; //最近节点索引
	for(size_t i=0;i<_map.mapNode.size();i++){
		dis=MapTools::GetDistanceByGPS(lng,lat,_map.mapNode[i].gpsx,_map.mapNode[i].gpsy);
		/*MAP_PRINT("curLng%lf \n",lng);
		MAP_PRINT("curLat%lf \n",lat);
		MAP_PRINT("gpsx %lf\n",_map.mapNode[i].gpsx);
		MAP_PRINT("gpsy %lf\n",_map.mapNode[i].gpsy);*/
		if(dis<min){
			min=dis;
			minNodeIndex=i;
		}
	}
	//DEBUG_PRINT("min dis:%lf \n",dis);

	//Step 2 -----------确定当前位置是否在路口上--------------
	if(min<30){				// 在30m范围之内
		return _map.mapNode[minNodeIndex].idself;
	}
	
	//Step 3 -----------确定最近路口邻接的路口是否有符合条件的,路口优先--------------
	for(int i=0;i<_map.mapNode[minNodeIndex].neigh;i++){
		int ID=_map.mapNode[minNodeIndex].NeighNoteID[i];
		int index=MapTools::GetNodeIndexByID(_map.mapNode,ID);
		dis=MapTools::GetDistanceByGPS(lng,lat,_map.mapNode[index].gpsx,_map.mapNode[index].gpsy);
		if(dis<30){
			return ID;
		}
	}

	//Step 4 -----------返回最近路口邻接的道路和该位置最近的道路--------------
	double minE=numeric_limits<double>::max();
	int LineID=-1;

	//Step 5 -----------找距离最近的道路--------------
	for(int i=0;i<_map.mapNode[minNodeIndex].neigh;i++){
		int ID=_map.mapNode[minNodeIndex].NeighLineID[i];  //先按照经纬度舍弃直线
		int index=MapTools::GetLineIndexByID(_map.mapLine,ID);
		if(isInLine(lng,lat,index)){   //在符合条件的道路中 寻找最近道路
			double b=_map.mapLine[index].b;
			double c=_map.mapLine[index].c;
			double k=_map.mapLine[index].k;
			int ex,ey;
			double dex,dey;
			MapTools::GPS2Earthy(lat,lng,ex,ey);
			dex=ex-_map.adjustPoint.x;dey=ey-_map.adjustPoint.y; //防止溢出和统一坐标系	
			double temp = abs((k*dex + b*dey + c)) / sqrt(b*b + k*k);
			if(temp<minE){
				minE=temp;
				LineID=ID;
			}
		}
	}
	return LineID;
}

int MapApp::locationGPS(double lng,double lat){
	double min=100000; //10W km
	int index=-1;
	for(size_t i=0;i<_GPSList.size();i++){
		double dis=MapTools::GetDistanceByGPS(lng,lat,_GPSList[i].x/60,_GPSList[i].y/60);
		if(dis<min){
			min=dis;
			index=i;
		}
	}
	return index;
}


bool MapApp::isInLine(double lng,double lat,int index){
	//Step 1 -------------获取前后两个路口的ID------------
	int IDs=_map.mapLine[index].idstart;			//起点终点节点 ID
	int IDe=_map.mapLine[index].idend;
	
	//Step 2 ------------ID换算成路口数组索引-------------
	int indexS=MapTools::GetNodeIndexByID(_map.mapNode,IDs); //起点终点 index
	int indexE=MapTools::GetNodeIndexByID(_map.mapNode,IDe);

	//Step 3 -----------获取经纬度--------------
	double lngS=_map.mapNode[indexS].gpsx;
	double latS=_map.mapNode[indexS].gpsy;
	double lngE=_map.mapNode[indexE].gpsx;
	double latE=_map.mapNode[indexE].gpsy;

	//Step 4 -----------把s到e调整为经纬度递增--------------
	if(lngS>lngE)
		swap<double>(lngS,lngE);
	if(latS>latE)
		swap<double>(latS,latE);
	
	//Step 5 -----------判断是否在主要经纬度方向内--------------
	if((lngE-lngS)>(latE-latS)){ //道路主要走向为 经度方向 东西
		if(lng>lngS&&lng<lngE)
			return true;
	}else{						//道路主要走向为 维度方向 南北
		if(lat>latS&&lat<latE)
			return true;
	}
	return false;
}


void MapApp::pathPlaning(int s,int e){
	//Step 0 -----------声明--------------
	vector<int> pathNode;  //节点序列
	vector<int> pathWhole; //最终结果
	pathNode.reserve(10);
	int tempID,tempID2;   //临时道路ID
	size_t i;
	MAP_BUTTON_NOTE	theNode1,theNode2; //临时存储节点
	tempID2=tempID=0;

	//Step 1 -----------根据最短路径算法得到节点序列--------------
	dijkstra(s,e,pathNode);
	
	//Step 2 -----------在道路序列中插入对于的道路编号--------------
	pathWhole.reserve(pathNode.size()*2);
	for(i=0;i<pathNode.size()-1;i++){
		pathWhole.push_back(pathNode[i]); //当前节点编号
		theNode1=MapTools::GetNodeByID(_map.mapNode
									,MapTools::Code2ID(pathNode[i],1));
		for(int j=0;j<theNode1.neigh;j++){ //遍历该节点相邻路口之间的道路ID
			tempID=theNode1.NeighLineID[j];
			//找到公共边
			theNode2=MapTools::GetNodeByID(_map.mapNode
							,MapTools::Code2ID(pathNode[i+1],1));
			for(int k=0;k<theNode2.neigh;k++){ //下一个节点
				tempID2=theNode2.NeighLineID[k];
				if(tempID==tempID2){         //找到公共边ID
					break;
				}
			}
			if(tempID==tempID2){
					break;
			}
		}
		
		pathWhole.push_back(MapTools::ID2Code(tempID)); //添加查询到的道路编号
	}
	pathWhole.push_back(pathNode.back());  //添加最后一个节点

	//Step 3 -----------结果赋给指定变量--------------
	_planPath.planPathQueue=pathWhole;
	_planPath.cur=0;

	//打印
	for(size_t i=0;i<_planPath.planPathQueue.size();i++){
		if(i%2==0){
			MAP_PRINT("[%d] ",_planPath.planPathQueue[i]);
		}else{
			MAP_PRINT("%d ",_planPath.planPathQueue[i]);
		}
	}
	MAP_PRINT("%s","\n");
}


void MapApp::dijkstra(int s,int e,vector<int> &pathV){	
	//Step 0 -----------把节点编号转化成adj矩阵中的索引--------------
	int sID=MapTools::Code2ID(s,1);
	int eID=MapTools::Code2ID(e,1);
	int sIndex=MapTools::GetNodeIndexByID(_map.mapNode,sID);
	int eIndex=MapTools::GetNodeIndexByID(_map.mapNode,eID);

	int tIndex=eIndex;

	//Step 1 -----------初始化算法参数--------------
	int nodeCount=_map.mapNode.size();
	vector<int> pi(nodeCount,-1);      //最短路径 前驱节点
	vector<int> S(nodeCount,0);        //集合S S[i]=1表示i节点在S中
	vector<int> d(nodeCount,INF);//原点到某一点 的最短代价
	S[sIndex]=1;
	d[sIndex]=0;

	//Step 2 -----------初始化距离数组d 和前缀数组pi --------------
	for(int i=0;i<nodeCount;i++){
		if(_map.adjMat[sIndex+i*nodeCount]<INF){
			d[i]=_map.adjMat[sIndex+i*nodeCount];
			pi[i]=sIndex;
		}
	}
 	d[sIndex]=0;

	//Step 3 -----------迭代求解最短路径--------------
	for(int i=1;i<nodeCount;i++){ //迭代 n-1 次
		int min=INF;
		int newNode=-1;     //本轮迭代新加入的节点
		for(int j=0;j<nodeCount;j++){ 
			if(S[j]!=1){       //在S集合中添加新节点				 
				if(d[j]<min){
					min=d[j];
					newNode=j;
				}
			}//end S[j]!=1
		}//end for
		//用新加入节点更新 U中的d值和pi
		if(newNode==-1) //找不到最近点了 退出迭代
			break;
		S[newNode]=1;
		for(int j=0;j<nodeCount;j++){ 
			if(S[j]!=1){  //U
				if(_map.adjMat[newNode+j*nodeCount]+d[newNode]<d[j]){ //松弛变量:更新d[j]和pi[j]
					pi[j]=newNode;
					d[j]=_map.adjMat[newNode+j*nodeCount]+d[newNode];
				}

			}
		}
	}

	//Step 4 -----------利用最短路径算法结果,获取路径编号序列--------------
	pathV.push_back(e); //加入尾编号
	int code; //要转化编号
	while(pi[tIndex]!=sIndex){
		tIndex=pi[tIndex];				//索引(0...)换回编号(1...)
		int code=MapTools::ID2Code(_map.mapNode[tIndex].idself);
		pathV.push_back(code); 
	}
	pathV.push_back(s);  //等价s
	reverse(pathV.begin(),pathV.end()); //反转回正序
}


int  MapApp::MCCallBack(void* mc_to_map, size_t size, void* args){
	NJUST_MC_STATE_INFO  *pState; //当不是状态数据时,值为NULL
	NJUST_MC_NAV_INFO    *pNav; //当不是导航信息时,值为NULL
	NJUST_MC_DRIVE_INFO  *pDrive; //当不是执行数据时,值为NULL

	//Step 1 -----------调用MC解码函数--------------
	NJUST_MC_Decode_IP_Data(mc_to_map,  //IP数据,目标模块从网络收到的数据
		size,     //pIPData数据的字节个数
		&pState, //当不是状态数据时,值为NULL
		&pNav, //当不是导航信息时,值为NULL
		&pDrive  //当不是执行数据时,值为NULL
		);

	//Step 2 -----------对导航信息进行解析--------------
	if (pNav)//收到导航信息
	{
		pthread_mutex_lock(&gMutex);
		//更新curGPS
		if(MapApp::s_GPSInfo.curLongtitude!=INITL_GPS_VALUE){  //不是第一次取值
			MapApp::s_GPSInfo.lastLongtitude=MapApp::s_GPSInfo.curLongtitude;
			MapApp::s_GPSInfo.lastlatitudel=MapApp::s_GPSInfo.curLatitude;
			MapApp::s_mapPackage.endID=pNav->navID;
		}else                                               //第一次取值
		{
			MapApp::s_GPSInfo.lastLongtitude=pNav->Longitude_degree;
			MapApp::s_GPSInfo.lastlatitudel=pNav->Latitude_degree;
			MapApp::s_mapPackage.startID=pNav->navID;
			MapApp::s_mapPackage.endID=pNav->navID;

		}
		MapApp::s_GPSInfo.curLongtitude = pNav->Longitude_degree;
		MapApp::s_GPSInfo.curLatitude = pNav->Latitude_degree;
		//计算丢包率相关
		MapApp::s_mapPackage.count++;
		MapApp::s_mapPackage.startTime=NJUST_IP_get_time();
		printf("debug: %lf  ",pNav->Longitude_degree);
		printf("debug:%lf  \n",pNav->Latitude_degree);
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		
		
	}
	return 0;
}


int MapApp::MOCallBack(void* mo_to_map, size_t size, void* args){
		NJUST_FROM_MO_COMMAND *pCmd = NULL;
		NJUST_FROM_MO_CFG *pCfg=NULL;
		NJUST_MO_Decode_IP_Data_CMD(mo_to_map, size, &pCmd,&pCfg);
		if(pCmd!=NULL){
			if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_COMPUTER_RESTART)
			{
				NJUST_IP_req_pc_reboot();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_COMPUTER_SHUTDOWN)
			{
				NJUST_IP_req_pc_poweroff();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RESTART)
			{
				NJUST_IP_req_mod_reboot();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_DEBUG)
			{
				//SetDebug();
			}
			else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RELEASE)
			{
				//SetRelease();
			}
		}
		if(pCfg!=NULL){
			for(int i=0;i<pCfg->nCFG;i++){
				DEBUG_PRINT("cfg:%d ",pCfg->pCFG[i].cfg);
				DEBUG_PRINT("valuse:%d \n",pCfg->pCFG[i].value);
			}
		}
		return 1;
}


void MapApp::sendRoad(double lng,double lat,int curID,int nextIndex){
	NJUST_MAP_INFO_ROAD   road;
	NJUST_MAP_INFO_ROAD  *proad = &road;
	char buff[1024];
	//int index=curID-START_LINE_ID;
	int index=MapTools::GetLineIndexByID(_map.mapLine,curID);
	MapTools::StructTransformLine(&_map.mapLine[index],&proad);
	MAP_BUTTON_NOTE nextNode=_map.mapNode[nextIndex];
	double dis=	road.distToNextNodeM =MapTools::GetDistanceByGPS(lng,lat,
		nextNode.gpsx,
		nextNode.gpsy);//转化为度

	//填写前后节点的经纬度
	int lastNodeID; //道路上一个节点的ID
	if(_map.mapNode[nextIndex].idself==_map.mapLine[index].idstart){
		lastNodeID=_map.mapLine[index].idend;
	}else{
		lastNodeID=_map.mapLine[index].idstart;
	}

	MAP_BUTTON_NOTE lastNode=MapTools::GetNodeByID(_map.mapNode,lastNodeID);
	road.lastNodeGps.latitude=lastNode.gpsx;
	road.lastNodeGps.longtitude=lastNode.gpsy;
	road.nextNodeGps.latitude=nextNode.gpsx;
	road.nextNodeGps.longtitude=nextNode.gpsy;

	
	size_t indexGPS=locationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			road.nextGPSPointQueue[i-indexGPS].longtitude = _GPSList[i].x;
			road.nextGPSPointQueue[i-indexGPS].latitude =_GPSList[i].y;
			len++;
		}
	}
	road.GPSPointQueuelength = len;
	road.distToNextNodeM = dis;
	road.synTime = NJUST_IP_get_time();
	road.frameID = _frameNum;
	
	//添加障碍物
	vector<NJUST_MAP_OBSTACLE> outObs; //在特定范围内的障碍物
	MapTools::GetObsByDistance(lng,lat,
									NJUST_MAP_DETECT_OBSTACLE_RADIUS_M,
								_map.mapObs,
								outObs);
	road.obstacleQueueLength=outObs.size();
	for(size_t i=0;i<outObs.size();i++){
		road.mapObstacleQueue[i]=outObs[i];
		//MAP_PRINT("发送障碍物%d\n",i);
	}

	MapTools::NJUST_MAP_Encode_IP_Data(&road, 0, buff);
	//NJUST_IP_get_timeStr(road.synTime, Timeget1);
	
	if(_frameNum%10==0){
		if (len>3)
			NJUST_IP_udp_send_to("", buff, 1024);
			MapTools::ms_sleep(10);
			send2Mo(buff,1024);
	}
	
}


void MapApp::sendNode(double lng,double lat,int curID,int lastIndex,int nextIndex){
	NJUST_MAP_INFO_NODE  node;
	NJUST_MAP_INFO_NODE  *pnode = &node;
	char buff[1024];
	int index=MapTools::GetNodeIndexByID(_map.mapNode,curID);
	//获取节点属性
	MAP_BUTTON_NOTE theNode=MapTools::GetNodeByID(_map.mapNode,curID);
	MapTools::StructTransformNote(&theNode, &pnode);

	if(lastIndex==-1){ //没有上一个节点,也就是开始处
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
	}else{                     //正常判断方向
		getDirection(node,index,lastIndex,nextIndex);
	}

	size_t indexGPS=locationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<_GPSList.size()){
			node.nextGPSPointQueue[i-indexGPS].longtitude = _GPSList[i].x;
			node.nextGPSPointQueue[i-indexGPS].latitude =_GPSList[i].y;
			len++;
		}
	}
	node.GPSPointQueuelength = len;
	node.synTime = NJUST_IP_get_time();
	node.frameID = _frameNum;

	//添加障碍物
	vector<NJUST_MAP_OBSTACLE> outObs; //在特定范围内的障碍物
	MapTools::GetObsByDistance(lng,lat,
									NJUST_MAP_DETECT_OBSTACLE_RADIUS_M,
								_map.mapObs,
								outObs);
	node.obstacleQueueLength=outObs.size();
	for(size_t i=0;i<outObs.size();i++){
		node.mapObstacleQueue[i]=outObs[i];
		//MAP_PRINT("发送障碍物%d\n",i);
	}

	//发送PL,MO
	MapTools::NJUST_MAP_Encode_IP_Data(&node, 1, buff);
	//NJUST_IP_get_timeStr(node.synTime, Timeget1);
	//
	
	if(_frameNum%10==0)
	{	
		//MAP_PRINT("长度为%d\n",len);
		if (len>3){
			int r=NJUST_IP_udp_send_to("", buff, 1024);//广播
			//MAP_PRINT("广播返回值为:%d\n",r);
		}
		MapTools::ms_sleep(10);
		
		send2Mo(buff,1024);
	}
}


void MapApp::getDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex)
{
	//报出方向
	MAP_TURN m_turn;

	//STEP2 ...算出该线的方向向量int a,b   (a,b)(经度，纬度)
	double x1=_map.mapNode[curIndex].gpsx-_map.mapNode[lastIndex].gpsx;
	double y1=_map.mapNode[curIndex].gpsy-_map.mapNode[lastIndex].gpsy;
	//STEP4 ...算出该线的方向向量int c,d   (c,d)
	double x2=_map.mapNode[nextIndex].gpsx-_map.mapNode[lastIndex].gpsx;
	double y2=_map.mapNode[nextIndex].gpsy-_map.mapNode[lastIndex].gpsy;

	//STEP5 ...求出从第一个方向向量到第二个方向向量逆时针旋转的夹角
	double degree = MapTools::GetRotateAngle(x1 / 100, y1 / 100, x2 / 100, y2 / 100);
	//MAP_PRINT("degree:%lf\n",degree);
	if ((degree < 45 && degree>0) || (degree >= 315 && degree <= 360))//直行
	{
		m_turn.turn = 0;
		m_turn.turndegree = 0;
	}
	if (degree >= 45 && degree < 135)//左拐
	{
		m_turn.turn = 1;
		m_turn.turndegree = degree;
	}
	if (degree >= 135 && degree < 180)//左Uturn
	{
		m_turn.turn = 3;
		m_turn.turndegree = 180;
	}
	if (degree >= 180 && degree < 225)//右Uturn
	{
		m_turn.turn = 4;
		m_turn.turndegree = 180;
	}
	if (degree >= 225 && degree < 315)//右拐
	{
		m_turn.turn = 2;
		m_turn.turndegree = 360 - degree;//右转的角度为锐角
	}

	switch (m_turn.turn)
	{
	case 0://直行
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_STRAIGHTLINE;
		break;
	case 1://左转
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNLEFT;
		break;
	case 2://右转
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNRIGHT;
		break;
	case 3://左Uturn
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNAROUND;
		break;
	case 4://右Uturn
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_TURNAROUND;
		break;
	case 5:
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
		break;
	}
}


void MapApp::send2Mo(char buff[],int n){
		NJUST_TO_MO_WORKSTAT moState;
		NJUST_IP_TIME time2;
		time2 = NJUST_IP_get_time();
		moState.moduleID = 2;
		moState.myselfTimeOutMS = 2000;
		moState.stat = NJUST_MO_WORK_STAT_VAILD;
		moState.PELR = (MapApp::s_mapPackage.endID - MapApp::s_mapPackage.startID + 1 - MapApp::s_mapPackage.count) * 1000 /
						(MapApp::s_mapPackage.endID - MapApp::s_mapPackage.startID + 1);
		moState.timeConsumingMS = (int)NJUST_IP_get_time_GAP_ms(MapApp::s_mapPackage.startTime, time2);
		moState.errCode = NJUST_MO_ERRCODE_NOERR;
		sprintf(moState.pErrMsg, "%s", "/0");
		int nByte = 0;
		void *pStat = NULL;
		pStat = NJUST_MO_Encode_STA_IP_Data(&moState, &nByte);

		int r=NJUST_IP_udp_send_to("MO", pStat, nByte);
}

void MapApp::recordWrite(int curID,int lastID){
	int i,code;
	MAP_DOUBLE_POINT  point;
	_pRecord=fopen(RECORD_FILE_NAME,"wb");
	
	fwrite(&curID,sizeof(int),1,_pRecord);    //curID
	fwrite(&lastID,sizeof(int),1,_pRecord);   //lastID

	//规划路线
	int len=_planPath.planPathQueue.size();
	fwrite(&len,sizeof(int),1,_pRecord); 
	fwrite(&_planPath.cur,sizeof(int),1,_pRecord); 
	for(i=0;i<len;i++){
		code=_planPath.planPathQueue[i];
		//DEBUG_PRINT("code %d\n",code);
		fwrite(&code,sizeof(int),1,_pRecord);
	}

	//GPS点
	len=_GPSList.size();
	fwrite(&len,sizeof(int),1,_pRecord); 
	for(i=0;i<len;i++){
		point = _GPSList[i];
		fwrite(&point,sizeof(MAP_DOUBLE_POINT),1,_pRecord);
	}
	fclose(_pRecord);

	

}

void MapApp::recordRead(int &curID,int &lastID){
	if((access(RECORD_FILE_NAME,F_OK))!=0){
		MAP_PRINT("没有找到记录文件!%s","\n");
		return;
	}

	int len,code; 
	MAP_DOUBLE_POINT point;

	_pRecord=fopen(RECORD_FILE_NAME,"rb");

	fread(&curID,sizeof(int),1,_pRecord);    //curID
	fread(&lastID,sizeof(int),1,_pRecord);   //lastID

	//规划路线
	fread(&len,sizeof(int),1,_pRecord); 
	fread(&_planPath.cur,sizeof(int),1,_pRecord); 
	for(int i=0;i<len;i++){
		fread(&code,sizeof(int),1,_pRecord);
		_planPath.planPathQueue.push_back(code);
	}

	//GPS点
	fread(&len,sizeof(int),1,_pRecord); 
	for(int i=0;i<len;i++){
		fread(&point,sizeof(MAP_DOUBLE_POINT),1,_pRecord);
		_GPSList.push_back(point);
	}
	fclose(_pRecord);
}

bool MapApp::checkLoaction(int curID){
	if((_planPath.cur+1)%2==1) {     //应该发现新道路
		if(curID-START_LINE_ID+1!=_planPath.planPathQueue[_planPath.cur+1]){ //新节点无效
			MAP_PRINT("定位 ID为%d\n",curID);
			MAP_PRINT("期待 ID为%d\n",_planPath.planPathQueue[_planPath.cur+1]+START_LINE_ID-1);
			return false;
		}
	}else{
	   if(curID-START_NODE_ID+1!=_planPath.planPathQueue[_planPath.cur+1]){ //新节点无效
		   MAP_PRINT("定位 ID为%d\n",curID);
		   MAP_PRINT("期待 ID为%d\n",_planPath.planPathQueue[_planPath.cur+1]+START_NODE_ID-1);
		   return false;
	  }
    }
	return true;
}

void MapApp::release(){
	if(_mapFile!=NULL){
		delete _mapFile;
	}
	vector<MAP_TASK_NODE> tTasks;
	tTasks.swap(_mapTaskNode);
	
	vector<MAP_DOUBLE_POINT> tGPSList;
	tGPSList.swap(_GPSList);
}








