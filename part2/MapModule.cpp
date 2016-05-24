#include<vector>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include"MapModule.h"



///
/// MapApp
///
//静态变量 定义
NJUST_MAP_GPS_INFO MapApp::GPSInfo;
MAP_PACKAGE MapApp::mappackage;
pthread_mutex_t     gMutex= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t      cond =PTHREAD_COND_INITIALIZER;

MapApp::MapApp(const char* loadpath){
	//初始化
	this->Intialize(loadpath);
	
	
}

MapApp::~MapApp(){
	Release();
}


void MapApp::Run(){
	double curLng,curLat,lastLng,lastLat;

	//启动时定位
	int SIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	int lastNode=-1;						 //上一次经过的节点，用来判断当前方向

	StartPlan(SIndex);
	if(SIndex==-1){
		MAP_PRINT("附近没有路口,无法启动 %d\n",SIndex);
		return;
	}
	PathPlaning(mapTaskNode[SIndex].noderesult,mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath

	//启动时的初始化
	lastID=planPath.planPathQueue[0]-1+START_NODE_ID;
	curID=lastID;
	//启动的特殊处理 认为启动点是路段 节点0-》节点1之间的读点
	mapFile->ReadMapGPS(planPath.planPathQueue[0],planPath.planPathQueue[2],GPSList,false);

	while(1){
		pthread_mutex_lock(&gMutex);
		curLng=MapApp::GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::GPSInfo.curLatitude;   //最新维度 (度)
		lastLng=MapApp::GPSInfo.lastLongtitude;
		lastLat=MapApp::GPSInfo.lastlatitudel;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		
		MapTools::ms_sleep(20);		//20毫秒 收发一次
		if (MapTools::CheckGPS(curLng,curLat,lastLng,lastLat))
		{			
			if((curID=Location(curLng,curLat))==-1){
				MAP_PRINT("没有找到合适直线 ID为%d\n",curID);
				continue;
			}
			if(curID!=lastID){              //进入新的路口或道路
				if(!CheckLoaction(curID)){
					curID=lastID;goto CONTINUE;
				}

			    MAP_PRINT("=========================:%s\n","");
				MAP_PRINT("进入新路口或者道路 new ID:%d\n",curID);
				planPath.cur++;
				size_t tcur=planPath.cur;
				if(tcur<planPath.planPathQueue.size()-1){//最近一次规划的路线没走完，读新路段文件
					MAP_PRINT("%s没有读完最近一次规划的节点\n","");
					bool isNode=(tcur%2==0);  //偶数是路口
					//读取指定段序列
					mapFile->ReadMapGPS(planPath.planPathQueue[tcur-1],
					planPath.planPathQueue[tcur+1],GPSList,
					isNode);
					//更新上一个路口索引
					if(isNode)
						lastNode=planPath.planPathQueue[tcur-2];
					else
						lastNode=planPath.planPathQueue[tcur-1];
					lastID=curID; //更新 lastID
				}else{								   //走到最近一次规划的最后一个节点
					MAP_PRINT("%s新的规划路线\n","");
					int lastRoad=planPath.planPathQueue[planPath.cur-1]; //记录最近一次规划走过的最后一段路，来构建路口过渡
					lastNode=planPath.planPathQueue[planPath.cur-2]; 
					SIndex++;
					PathPlaning(mapTaskNode[SIndex].noderesult,mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath
					//读取过渡 路口信息
					mapFile->ReadMapGPS(lastRoad,
					planPath.planPathQueue[1],GPSList,
					true);
					lastID=curID;
				}
			}//if new ID
			CONTINUE:
			frame_pl++;
			if(planPath.cur%2==0){  //路口
				Send2PL_Node(curLng,curLat,curID,lastNode-1,planPath.planPathQueue[planPath.cur+2]-1);
			}else{
				Send2PL_Road(curLng,curLat,curID,planPath.planPathQueue[planPath.cur+1]-1);
			}
			//record记录当前状态

		}//checkGPS
	}//while
}//func./MAP

void MapApp::Intialize(const char* loadpath){
	//构建数据存储
	mapFile=new MapFileStream(loadpath);

	mapFile->LoadMapNode(map);			//加载自建地图 路口和道路信息
	mapFile->LoadAdjMat(map);			//加载自建地图 邻接表
	mapFile->LoadMapTask(mapTaskNode);  //加载需要进一步规划的路径信息


	//构建通信
	MapCommunion communion;
	communion.RegisterMap();							//注册本模块
	communion.ReciveModuleMsg("MO",MOCallBack);	//响应MO命令
	//communion.ReciveModuleMsg("PL",this->PLCallBack);	//响应PL命令
	communion.ReciveBroadcastMsg(MCCallBack);     //对MC发出的数据，进行记录
	MAP_PRINT("%s 注册MC响应\n","MapApp::Inialize");


	//接收的GPS 信息初始化
	MapApp::GPSInfo.curLongtitude=INITL_GPS_VALUE;
	frame_pl=0;
	mappackage.count=0;
	mappackage.startID=0;
	mappackage.endID=0;
	
	//记录保存
	if(fopen("record.db","rb")==NULL)
		checkExpOut=false;
	else   //启动时存在记录文件 说明上次程序退出是异常退出
		checkExpOut=true;
	this->pRecord=fopen("record.db","wb");
	if(this->pRecord==NULL){
		MAP_PRINT("创建日志文件失败%s","\n");
	}

}

//车启动时 定位到任务文件中的路口处start 以及下一个路口end
void MapApp::StartPlan(int& start){
	double curLng,curLat;
	double dis=0;
	double distanceThreshold=500;
	//如果还没接收到数据则进程挂起100ms 直到接收到有效数据
	while(1){
		pthread_mutex_lock(&gMutex);
		curLng=MapApp::GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::GPSInfo.curLatitude;   //最新维度 (度)
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		if(curLng==INITL_GPS_VALUE)
			MapTools::ms_sleep(100);
		else
			break;
		MAP_PRINT("%s挂起100ms 等待MC发送坐标\n","");

	}

	for(unsigned int i=0;i<mapTaskNode.size();i++){
		dis=MapTools::GetDistanceByGPS(curLng,curLat,mapTaskNode[i].longtitude,mapTaskNode[i].latitude);
		if(dis<distanceThreshold){
			MAP_PRINT("里最近节点相差%lfm \n",dis);
			start=i;
			break;
		}
	}
}

//在车跑动的时候 确定车在自建地图中的位置
int MapApp::Location(double lng,double lat){
	// step1  寻找该点在自建地图中最近的路口
	double min=5000;//5km
	double dis=0;
	int minNodeIndex=-1; //最近节点索引
	for(size_t i=0;i<map.mapNode.size();i++){
		dis=MapTools::GetDistanceByGPS(lng,lat,map.mapNode[i].gpsx/60,map.mapNode[i].gpsy/60);
		if(dis<min){
			min=dis;
			minNodeIndex=i;
		}
	}

	//step2 确定当前路径是否在找的路口上 
	if(min<30){ // 在30m范围之内
		return map.mapNode[minNodeIndex].idself;
		
	}

	//step3 确定最近路口邻接的路口是否有符合条件的 路口优先
	for(int i=0;i<map.mapNode[minNodeIndex].neigh;i++){
		int ID=map.mapNode[minNodeIndex].NeighNoteID[i];
		int index=MapTools::GetNodeIndexByID(map.mapNode,ID);
		dis=MapTools::GetDistanceByGPS(lng,lat,map.mapNode[index].gpsx/60,map.mapNode[index].gpsy/60);
		if(dis<30){
			return ID;
		}
	}

	//step4 返回最近路口邻接的道路和该位置最近的道路
	double minE=numeric_limits<double>::max();
	int LineID=-1;


	//找距离最近的道路
	for(int i=0;i<map.mapNode[minNodeIndex].neigh;i++){
		//先按照经纬度舍弃直线
		int ID=map.mapNode[minNodeIndex].NeighLineID[i];
		int index=MapTools::GetLineIndexByID(map.mapLine,ID);


		if(isInLine(lng,lat,index)){   //在符合条件的道路中 寻找最近道路
			double b=map.mapLine[index].b;
			double c=map.mapLine[index].c;
			double k=map.mapLine[index].k;
			int ex,ey;
			double dex,dey;
			MapTools::GPS2Earthy(lat,lng,ex,ey);
			dex=ex-map.adjustPoint.x;dey=ey-map.adjustPoint.y; //防止溢出和统一坐标系	
			double temp = abs((k*dex + b*dey + c)) / sqrt(b*b + k*k);
			if(temp<minE){
				minE=temp;
				LineID=ID;
			}
		}
	}
	return LineID;

}

//在GPS序列中定位 当前位置的索引
int MapApp::LocationGPS(double lng,double lat){
	double min=100000;
	int index=-1;
	for(size_t i=0;i<GPSList.size();i++){
		double dis=MapTools::GetDistanceByGPS(lng,lat,GPSList[i].x/60,GPSList[i].y/60);
		if(dis<min){
			min=dis;
			index=i;
		}
	}
	return index;
}

//判断经纬度是否跟某条直线平行
bool MapApp::isInLine(double lng,double lat,int index){
	//MAP_PRINT("index:%d ",index);  MAP_PRINT("%s ","\n");
	int IDs=map.mapLine[index].idstart;			//起点终点节点 ID
	int IDe=map.mapLine[index].idend;
	//MAP_PRINT("%d ",IDs); MAP_PRINT("%d ",IDe); MAP_PRINT("%s ","\n");
	int indexS=MapTools::GetNodeIndexByID(map.mapNode,IDs); //起点终点 index
	int indexE=MapTools::GetNodeIndexByID(map.mapNode,IDe);

	//换算成度
	double lngS=map.mapNode[indexS].gpsx/60;
	double latS=map.mapNode[indexS].gpsy/60;
	double lngE=map.mapNode[indexE].gpsx/60;
	double latE=map.mapNode[indexE].gpsy/60;
	//s 到 e经纬度递增
	if(lngS>lngE)
		swap<double>(lngS,lngE);
	if(latS>latE)
		swap<double>(latS,latE);
	
	if((lngE-lngS)>(latE-latS)){ //道路主要走向为 经度方向 东西
		if(lng>lngS&&lng<lngE)
			return true;
	}else{						//道路主要走向为 维度方向 南北
		if(lat>latS&&lat<latE)
			return true;
	}
	return false;
}

//规划路径(选取自建地图中的节点或道路)比如1,4 需要规划处1,1,2,2,3,3,4 
void MapApp::PathPlaning(int s,int e){
	vector<int> pathNode;  //节点顺序
	vector<int> pathWhole; //最终结果

	pathNode.reserve(10);

	//获取最短路径
	Dijkstra(s,e,pathNode);
	MAP_PRINT("%s进入最短路径算法\n","");
	for(size_t i=0;i<pathNode.size();i++){
		MAP_PRINT("%d ",pathNode[i]);
	}
	MAP_PRINT("%s","\n");
	pathWhole.reserve(pathNode.size()*2);

	//节点路径 中插入道路序号
	for(size_t i=0;i<pathNode.size()-1;i++){
		pathWhole.push_back(pathNode[i]); //当前节点
		int tempID=0;
		int tempID2=0;
		for(int j=0;j<map.mapNode[pathNode[i]-1].neigh;j++){ //找到相邻路口之间的道路ID
			tempID=map.mapNode[pathNode[i]-1].NeighLineID[j];
			for(int k=0;k<map.mapNode[pathNode[i+1]-1].neigh;k++){ //下一个节点
				tempID2=map.mapNode[pathNode[i+1]-1].NeighLineID[k];
				if(tempID==tempID2){
					break;
				}
			}
			if(tempID==tempID2){
					break;
			}
		}
		pathWhole.push_back(tempID-START_LINE_ID+1); //当前节点编号
	}
	pathWhole.push_back(pathNode[pathNode.size()-1]);

	//给全局规划
	planPath.planPathQueue=pathWhole;
	planPath.cur=0;

	for(size_t i=0;i<this->planPath.planPathQueue.size();i++){
		MAP_PRINT("%d ",planPath.planPathQueue[i]);
	}
	MAP_PRINT("%s","\n");
}

//用Dijkstra找出自建地图中s路口到e路口的最短路径
void MapApp::Dijkstra(int s,int e,vector<int> &pathV){
	int sIndex=s-1;
	int eIndex=e-1;
	int tIndex=eIndex;
	//用迪克斯特拉(Dijkstra)求最短路径
	int nodeCount=map.mapNode.size();
	vector<int> pi(nodeCount,-1);      //最短路径 前驱节点
	vector<int> S(nodeCount,0);        //集合S S[i]=1表示i节点在S中
	vector<int> d(nodeCount,1000000000);//原点到某一点 的最短代价
	
	S[sIndex]=1;
	d[sIndex]=0;

	//初始化最短代价 和路径
	for(int i=0;i<nodeCount;i++){
		if(map.adjMat[sIndex+i*nodeCount]<1000000000){
			d[i]=map.adjMat[sIndex+i*nodeCount];
			pi[i]=sIndex;
		}
	}
 	d[sIndex]=0;

	for(int i=1;i<nodeCount;i++){ //迭代 n-1 次
		int min=1000000000;
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
		S[newNode]=1;
		for(int j=0;j<nodeCount;j++){ 
			if(S[j]!=1){  //U
				if(map.adjMat[newNode+j*nodeCount]+d[newNode]<d[j]){ //松弛 更新d[j]和pi[j]
					pi[j]=newNode;
					d[j]=map.adjMat[newNode+j*nodeCount]+d[newNode];
				}

			}
		}
	}

	pathV.push_back(tIndex+1);
	while(pi[tIndex]!=sIndex){
		tIndex=pi[tIndex];
		pathV.push_back(tIndex+1); //索引(0...)换回编号(1...)
	}
	pathV.push_back(sIndex+1);
	reverse(pathV.begin(),pathV.end()); //反转回正序
}

//响应MC
int  MapApp::MCCallBack(void* mc_to_map, size_t size, void* args){
	NJUST_MC_STATE_INFO  *pState; //当不是状态数据时,值为NULL
	NJUST_MC_NAV_INFO    *pNav; //当不是导航信息时,值为NULL
	NJUST_MC_DRIVE_INFO  *pDrive; //当不是执行数据时,值为NULL

	//第一步，调用MC解码函数
	NJUST_MC_Decode_IP_Data(mc_to_map,  //IP数据,目标模块从网络收到的数据
		size,     //pIPData数据的字节个数
		&pState, //当不是状态数据时,值为NULL
		&pNav, //当不是导航信息时,值为NULL
		&pDrive  //当不是执行数据时,值为NULL
		);

	if (pNav)//收到导航信息
	{
		pthread_mutex_lock(&gMutex);
		//更新curGPS
		if(MapApp::GPSInfo.curLongtitude!=INITL_GPS_VALUE){  //不是第一次取值
			MapApp::GPSInfo.lastLongtitude=MapApp::GPSInfo.curLongtitude;
			MapApp::GPSInfo.lastlatitudel=MapApp::GPSInfo.curLatitude;
			MapApp::mappackage.endID=pNav->navID;
		}else                                               //第一次取值
		{
			MapApp::GPSInfo.lastLongtitude=pNav->Longitude_degree;
			MapApp::GPSInfo.lastlatitudel=pNav->Latitude_degree;
			MapApp::mappackage.startID=pNav->navID;
			MapApp::mappackage.endID=pNav->navID;

		}
		MapApp::GPSInfo.curLongtitude = pNav->Longitude_degree;
		MapApp::GPSInfo.curLatitude = pNav->Latitude_degree;
		MapApp::mappackage.count++;
		MapApp::mappackage.startTime=NJUST_IP_get_time();
		//MAP_PRINT("lng:%lf  ",pNav->Longitude_degree);
		//MAP_PRINT("lat:%lf  \n",pNav->Latitude_degree);
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		
		
	}
	return 0;
}

//响应MO
int MapApp::MOCallBack(void* mo_to_map, size_t size, void* args){
	NJUST_FROM_MO_COMMAND *pCmd = NULL;
		NJUST_MO_Decode_IP_Data_CMD(mo_to_map, size, &pCmd);
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
		return 1;
}

//发送信息到PL
void MapApp::Send2PL_Road(double lng,double lat,int curID,int nextIndex){
	NJUST_MAP_INFO_ROAD   road;
	NJUST_MAP_INFO_ROAD  *proad = &road;
	char buff[1024];
	int index=curID-START_LINE_ID;
	MapTools::StructTransformLine(&map.mapLine[index],&proad);
	double dis=	road.distToNextNodeM =MapTools::GetDistanceByGPS(lng,lat,
		map.mapNode[nextIndex].gpsx/60,
		map.mapNode[nextIndex].gpsy/60);

	size_t indexGPS=LocationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<GPSList.size()){
			road.nextGPSPointQueue[i-indexGPS].longtitude = GPSList[i].x;
			road.nextGPSPointQueue[i-indexGPS].latitude =GPSList[i].y;
			len++;
		}
	}
	road.GPSPointQueuelength = len;
	road.distToNextNodeM = dis;
	road.synTime = NJUST_IP_get_time();
	road.FrameID = frame_pl;
	MapTools::NJUST_MAP_Encode_IP_Data(&road, 0, buff);
	//NJUST_IP_get_timeStr(road.synTime, Timeget1);
	if (len>3)
	  NJUST_IP_tcp_send_to("PL", buff, 1024);
	if(frame_pl%10==0)
		Send2Mo(buff,1024);
	
}

//发送信息到PL
void MapApp::Send2PL_Node(double lng,double lat,int curID,int lastIndex,int nextIndex){
	NJUST_MAP_INFO_NODE  node;
	NJUST_MAP_INFO_NODE  *pnode = &node;
	char buff[1024];
	int index=curID-START_NODE_ID;
	MapTools::StructTransformNote(&map.mapNode[index], &pnode);

	if(lastIndex==-1){ //没有上一个节点,也就是开始处
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
	}else{                     //正常判断方向
		GetDirection(node,index,lastIndex,nextIndex);
	}

	size_t indexGPS=LocationGPS(lng,lat);
	//最多读20个GPS点
	int len=0;
	for(size_t i=indexGPS;i<indexGPS+20;i++){
		if(i<GPSList.size()){
			node.nextGPSPointQueue[i-indexGPS].longtitude = GPSList[i].x;
			node.nextGPSPointQueue[i-indexGPS].latitude =GPSList[i].y;
			len++;
		}
	}
	node.GPSPointQueuelength = len;
	node.synTime = NJUST_IP_get_time();
	node.FrameID = frame_pl;
	//发送PL,MO
	MapTools::NJUST_MAP_Encode_IP_Data(&node, 1, buff);
	//NJUST_IP_get_timeStr(node.synTime, Timeget1);
	if (len>3)
		 NJUST_IP_tcp_send_to("PL", buff, 1024);
	if(frame_pl%10==0)
		Send2Mo(buff,1024);
}

//获取方向信息
void MapApp::GetDirection(NJUST_MAP_INFO_NODE &node,int curIndex,int lastIndex,int nextIndex)
{
	//报出方向
	MAP_TURN m_turn;

	//STEP2 ...算出该线的方向向量int a,b   (a,b)(经度，纬度)
	double x1=map.mapNode[curIndex].gpsx-map.mapNode[lastIndex].gpsx;
	double y1=map.mapNode[curIndex].gpsy-map.mapNode[lastIndex].gpsy;
	//STEP4 ...算出该线的方向向量int c,d   (c,d)
	double x2=map.mapNode[nextIndex].gpsx-map.mapNode[lastIndex].gpsx;
	double y2=map.mapNode[nextIndex].gpsy-map.mapNode[lastIndex].gpsy;

	//STEP5 ...求出从第一个方向向量到第二个方向向量逆时针旋转的夹角
	double degree = MapTools::GetRotateAngle(x1 / 100, y1 / 100, x2 / 100, y2 / 100);
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

//给MO发送数据 数据 数据长度
void MapApp::Send2Mo(char buff[],int n){
		NJUST_TO_MO_WORKSTAT moState;
		NJUST_IP_TIME time2;
		time2 = NJUST_IP_get_time();
		moState.moduleID = 2;
		moState.myselfTimeOutMS = 2000;
		moState.stat = NJUST_MO_WORK_STAT_VAILD;
		moState.PELR = (MapApp::mappackage.endID - MapApp::mappackage.startID + 1 - MapApp::mappackage.count) * 1000 /
						(MapApp::mappackage.endID - MapApp::mappackage.startID + 1);
		moState.timeConsumingMS = (int)NJUST_IP_get_time_GAP_ms(MapApp::mappackage.startTime, time2);
		moState.errCode = NJUST_MO_ERRCODE_NOERR;
		sprintf(moState.pErrMsg, "%s", "/0");
		int nByte = 0;
		void *pStat = NULL;
		pStat = NJUST_MO_Encode_STA_IP_Data(&moState, &nByte);
		 NJUST_IP_udp_send_to("MO", pStat, nByte);
		 MapTools::ms_sleep(10);    //10ms
		NJUST_IP_udp_send_to("MO", buff, n);
}


//填写状态文件
void MapApp::RecordWrite(int curID,int lastID){
	fwrite(&curID,sizeof(int),1,pRecord);    //curID
	fwrite(&lastID,sizeof(int),1,pRecord);   //lastID

	//规划路线
	int len=planPath.planPathQueue.size();
	fwrite(&len,sizeof(int),1,pRecord); 
	/*for(int i=0;i<len;i++){
		int temp=planPath.planPathQueue[i];
		fwrite(&temp,sizeof(int),1,pRecord); 
	}*/
	fwrite(&planPath.planPathQueue[0],sizeof(int),len,pRecord);

	//GPS点
	len=GPSList.size();
	fwrite(&len,sizeof(int),1,pRecord); 
	fwrite(&GPSList[0],sizeof(MAP_DOUBLE_POINT),len,pRecord);
}



//检查location是否合理 错误输出到日志
bool MapApp::CheckLoaction(int curID){
	if((planPath.cur+1)%2==1) {     //应该发现新道路
		if(curID-START_LINE_ID+1!=planPath.planPathQueue[planPath.cur+1]){ //新节点无效
			MAP_PRINT("定位 ID为%d\n",curID);
			MAP_PRINT("期待 ID为%d\n",planPath.planPathQueue[planPath.cur+1]+START_LINE_ID-1);
			return false;
		}
	}else{
	   if(curID-START_NODE_ID+1!=planPath.planPathQueue[planPath.cur+1]){ //新节点无效
		   MAP_PRINT("定位 ID为%d\n",curID);
		   MAP_PRINT("期待 ID为%d\n",planPath.planPathQueue[planPath.cur+1]+START_NODE_ID-1);
		   return false;
	  }
    }
	return true;
}

//释放大块内存
void MapApp::Release(){
	if(mapFile!=NULL){
		delete mapFile;
	}
	mapTaskNode.clear();  //元素被清空 内存未释放
	GPSList.clear();      //
}








