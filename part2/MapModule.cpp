#include<vector>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include"MapModule.h"



///
/// MapApp
///
//静态变量 定义
NJUST_MAP_GPS_INFO MapApp::GPSInfo;

pthread_mutex_t     gMutex= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t      cond =PTHREAD_COND_INITIALIZER;

MapApp::MapApp(const char* loadpath){
	//初始化
	this->Intialize(loadpath);
	
	
}

MapApp::~MapApp(){
	if(mapFile!=NULL){
		delete mapFile;
	}
}


void MapApp::Run(){
	MAP_PRINT("%s进入Run函数\n","");
	double curLng,curLat,lastLng,lastLat;

	//启动时定位
	int SIndex=-1;                           //在任务节点中的位置索引
	int lastID;								 //最近离开道路或路口的 ID
	int curID;								 //当前所在的道路或节点 ID
	int lastNode=-1;						 //上一次经过的节点，用来当前判断方向
	StartPlan(SIndex);
	if(SIndex==-1){
		MAP_PRINT("附近没有路口,无法启动 %d\n",SIndex);
		return;
	}
	PathPlaning(mapTaskNode[SIndex].noderesult,mapTaskNode[SIndex+1].noderesult);//规划路径，结果存在planPath

	//启动时的初始化
	lastID=planPath.planPathQueue[0]-1+START_NODE_ID;
	curID=lastID;
	//启动的特殊处理 认为启动点是路段
	mapFile->ReadMapGPS(planPath.planPathQueue[0],planPath.planPathQueue[2],GPSList,false);

	while(1){
		pthread_mutex_lock(&gMutex);
		curLng=MapApp::GPSInfo.curLongtitude; //最新经度 (度)
		curLat=MapApp::GPSInfo.curLatitude;   //最新维度 (度)
		lastLng=MapApp::GPSInfo.lastLongtitude;
		lastLat=MapApp::GPSInfo.lastlatitudel;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
		
		if (MapTools::CheckGPS(curLng,curLat,lastLng,lastLat))
		{
			MapTools::ms_sleep(100);		//100毫秒 收发一次
			if((curID=Location(curLng,curLat))==-1){
				MAP_PRINT("没有找到合适直线 ID为%d\n",curID);
				continue;
			}

			if(curID!=lastID){              //进入新的路口或道路
				if((planPath.cur+1)%2==1) {     //应该发现新道路
					if(curID-START_LINE_ID+1!=planPath.planPathQueue[planPath.cur+1]){ //新节点无效
						MAP_PRINT("定位 ID为%d\n",curID);
						MAP_PRINT("期待 ID为%d\n",planPath.planPathQueue[planPath.cur+1]+START_LINE_ID-1);
						continue;
					}
				}else{
					if(curID-START_NODE_ID+1!=planPath.planPathQueue[planPath.cur+1]){ //新节点无效
						MAP_PRINT("定位 ID为%d\n",curID);
						MAP_PRINT("期待 ID为%d\n",planPath.planPathQueue[planPath.cur+1]+START_NODE_ID-1);
						continue;
					}
				}
			
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
			frame_pl++;
			if(planPath.cur%2==0){  //路口
				Send2PL_Node(curLng,curLat,curID,lastNode-1,planPath.planPathQueue[planPath.cur+2]-1);
			}else{
				Send2PL_Road(curLng,curLat,curID,planPath.planPathQueue[planPath.cur+1]-1);
			}
		}//checkGPS
	}//while
}//func




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
		MAP_PRINT("%s挂起\n","");

	}

	for(unsigned int i=0;i<mapTaskNode.size();i++){
		dis=MapTools::GetDistanceByGPS(curLng,curLat,mapTaskNode[i].longtitude,mapTaskNode[i].latitude);
		if(dis<distanceThreshold){
			MAP_PRINT("相差%lfm \n",dis);
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
	double minE=1000000000;
	int LineID=-1;
	//MAP_PRINT("minIndex %d",minNodeIndex);
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
			MapTools::GPS2Earthy(lng,lat,ex,ey);
			dex=ex-map.adjustPoint.x;dey=ey-map.adjustPoint.y; //防止溢出和统一坐标系		

			double temp = abs((k*dex + b*dey + c)) / sqrt(b*b + k*k);
			temp/=100000;
			//MAP_PRINT("temp:%lf",temp);MAP_PRINT("\n","");
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
	//MAP_PRINT("%d ",indexS); MAP_PRINT("%d ",indexE); MAP_PRINT("%s ","\n");
	//MAP_PRINT("%lf ",map.mapNode[indexS].gpsx); MAP_PRINT("%lf ",map.mapNode[indexS].gpsy); MAP_PRINT("%s ","\n");
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
		/*MAP_PRINT("%lf ",lngS); MAP_PRINT("%lf ",lngE); MAP_PRINT("%s ","\n");
		MAP_PRINT("%lf ",latS); MAP_PRINT("%lf ",latE);  MAP_PRINT("%s ","\n");
		MAP_PRINT("%lf ",lng); MAP_PRINT("%lf ",lat);  MAP_PRINT("%s ","\n");
		MAP_PRINT("%d ",indexS); MAP_PRINT("%d ",indexE); MAP_PRINT("%s ","\n");
		MAP_PRINT("%s ","========================================\n");*/
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
		}else                                               //第一次取值
		{
			MapApp::GPSInfo.lastLongtitude=pNav->Longitude_degree;
			MapApp::GPSInfo.lastlatitudel=pNav->Latitude_degree;
		}
		MapApp::GPSInfo.curLongtitude = pNav->Longitude_degree;
		MapApp::GPSInfo.curLatitude = pNav->Latitude_degree;
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
	
}

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



///
/// MapFileStream
///

//构造函数 需要传入 数据文件的目录
MapFileStream::MapFileStream(const char* loadpath){
	strcpy(this->loadpath,loadpath);
}

//加载自建地图 路点信息
void MapFileStream::LoadMapNode(NJUST_MAP_BUILD_MAP &map){
	MAP_BUILD_FILE_HEAD      mapHead;
	vector<MAP_NODE>::iterator itNode;
	vector<MAP_ROAD>::iterator itRoad;

	char filename[]="board.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	map.mapNode.reserve(100);
	map.mapLine.reserve(100);
	
	//中间变量
	MAP_NODE tNode;										  
	MAP_ROAD tRoad;
	MAP_BUTTON_NOTE tButtonNode;
	MAP_BUTTON_LINE tButtonLine;
	
	FILE *pFile = fopen(path, "rb");
	if(pFile==NULL){
		perror("perror");
		return ;
	}

	fread(&mapHead,sizeof(MAP_BUILD_FILE_HEAD),1,pFile);  //读取文件头 包含了节点和道路的个数
	map.adjustPoint.x=mapHead.m_adjustx;
	map.adjustPoint.y=mapHead.m_adjusty;


	for(int i=0;i<mapHead.notecounter;i++){              //读道路信息 路口
		fread(&tNode, sizeof(MAP_NODE), 1, pFile);
		MapTools::Node2ButtonNode(tNode,tButtonNode);
		map.mapNode.push_back(tButtonNode);
	}

	for(int i=0;i<mapHead.linecounter;i++){              //读道路信息  道路
		fread(&tRoad, sizeof(MAP_ROAD), 1, pFile);
		MapTools::Line2ButtonLine(tRoad,tButtonLine);
		map.mapLine.push_back(tButtonLine);
	}

	fclose(pFile);
	MAP_PRINT("%s加载路点信息\n","");
}

//加载自建地图中 任务路点(需要规划)
void MapFileStream::LoadMapTask(vector<MAP_TASK_NODE> &mapTaskNode){
	mapTaskNode.reserve(100);

	char filename[]="InitialNodeQueue.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		perror("perror");
		return ;
	}

	MAP_TASK_NODE buff[100];
	fseek(pf, 0L, SEEK_END);
	int len = ftell(pf) / sizeof(ROADNODE);
	fseek(pf, 0L, SEEK_SET);
	fread(buff, sizeof(ROADNODE), len, pf);

    for(int i=0;i<len;i++)
    {
		mapTaskNode.push_back(buff[i]);
    }
    
    fclose(pf);
	MAP_PRINT("%s待规划路径：","");
	for(unsigned int i=0;i<mapTaskNode.size();i++){
		MAP_PRINT("%d ",mapTaskNode[i].noderesult);
	}
	MAP_PRINT("%s","\n");
}

//加载自建地图的邻接矩阵 注意，调用次方法前必须已经调用LoadMapNode
void MapFileStream::LoadAdjMat(NJUST_MAP_BUILD_MAP &map){
	int noteCount=map.mapNode.size();
	map.adjMat.reserve(noteCount*noteCount);
	int i;
	int buff=0;

	//打开文件
    char filename[]="adjust.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		perror("perror");
		return ;
	}
	
	//读取矩阵
	for (i = 0; i<noteCount*noteCount; i++)
	{
		fread(&buff, sizeof(int), 1, pf);
		map.adjMat.push_back(buff);
	}
	fclose(pf);
	MAP_PRINT("%s加载邻接矩阵信息\n","");
}

//加载指定路段gps序列
void MapFileStream::ReadMapGPS(int a,int b,vector<MAP_DOUBLE_POINT> &GPSList,bool isNode){
	GPSList.clear();
	char cj=isNode?'+':'-';
	char path[50];
	char filename[20];
	bool isOrder=true;		//是否按照文件名顺序读取
	int GPSnum=0;
	MAP_DOUBLE_POINT tPoint;
	GPSList.reserve(2048);

	
	sprintf(filename,"%d%c%d.db",a,cj,b); //1-2.db or 1+2.db
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录
	

	FILE *pf = fopen(path ,"rb");
	

	//正序和逆序都尝试一遍
	if (pf == NULL)
	{
		isOrder=false;
		memset(path,0,50);
		sprintf(filename,"%d%c%d.db",b,cj,a);
		strcpy(path,loadpath);
		strcat(path,filename);  //拼接完整目录

		pf = fopen(path ,"rb");
	}
	if(pf==NULL){
		perror("perror");
		return ;
	}
	

	//读取GPS序列
	fseek(pf, 0L, SEEK_END);
	GPSnum = ftell(pf) / sizeof(MAP_DOUBLE_POINT);
	fseek(pf, 0L, SEEK_SET);
	for(int i=0;i<GPSnum;i++){
		fread(&tPoint, sizeof(MAP_DOUBLE_POINT), 1, pf);
		GPSList.push_back(tPoint);
	}
	//逆序处理
	if(!isOrder){
		reverse(GPSList.begin(),GPSList.end());
	}

	fclose(pf);

	MAP_PRINT("加载 %s文件\n",filename);
}





///
/// MAP通信模块
///

//注册MAP
int MapCommunion::RegisterMap(){
	//判断模块名是否存在
	if (NJUST_IP_moduleName_exist("MAP"))
	{
		//若不存在，则创建
		if (NJUST_IP_set_moduleName("MAP", 0))
		{
			return 0;
		}
	}
	return 1;
}

//针对特定模块，注册回调函数
int MapCommunion::ReciveModuleMsg(const char * modulename,func_t func){
	if (-1 == NJUST_IP_set_tcp_callBack(modulename, func, NULL))
	{
		return 0;
	}
	return 1;
}

//接收广播
int MapCommunion::ReciveBroadcastMsg(func_t func){
	NJUST_IP_set_broadcast_callBack(func, NULL);
	return 1;
}





////
/// MAP模块工具类
///
//节点转化
void  MapTools::Node2ButtonNode(MAP_NODE &node,MAP_BUTTON_NOTE &buttonNode){
		buttonNode.idself = node.idself;
		buttonNode.neigh = node.neigh;
		int j;
		for (j = 0; j<buttonNode.neigh; j++)
		{
			buttonNode.NeighNoteID[j] = node.NeighNoteID[j];
			buttonNode.NeighLineID[j] = node.NeighLineID[j];
		}
		buttonNode.HLD = node.HLD;
		buttonNode.HLDkind = node.HLDkind;
		buttonNode.lukou = node.lukou;
		buttonNode.zebra = node.zebra;
		buttonNode.gpsx = node.gpsx;
		buttonNode.gpsy = node.gpsy;
		buttonNode.earthx = node.earthx;
		buttonNode.earthy = node.earthy;
}

//道路转化
void  MapTools::Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine){
		buttonLine.idself = line.idself;
		buttonLine.idstart = line.idstart;
		buttonLine.idend = line.idend; //ID
		buttonLine.k = line.k;           //直线参数
		buttonLine.b = line.b;
		buttonLine.c = line.c;
		buttonLine.roadkind = line.roadkind; 
		buttonLine.wedth = line.wedth;
		buttonLine.length = line.length;
		buttonLine.maluyazi = line.maluyazi;
		buttonLine.hyazi = line.hyazi;
		buttonLine.hulan = line.hulan;
		buttonLine.hhulan = line.hhulan;
		buttonLine.xingdaoxiannum = line.xingdaoxiannum;
		buttonLine.leftxingdaoxian = line.leftxingdaoxian;
		buttonLine.middlexingdaoxian = line.middlexingdaoxian;
		buttonLine.rightxingdaoxian = line.rightxingdaoxian;
		buttonLine.chedaonum = line.chedaonum;
		buttonLine.leftdaolubianjie = line.leftdaolubianjie;
		buttonLine.rightdaolubianjie = line.rightdaolubianjie;
		buttonLine.idealspeed = line.idealspeed;
}

void MapTools::ms_sleep( unsigned int msecs ){
	int rc;
	struct timeval tv;

	tv.tv_sec = msecs / 1000;
	tv.tv_usec = ( msecs % 1000 ) * 1000;
	do
	{
		rc = select( 0, NULL, NULL, NULL, &tv );
	}
	while ( rc == -1 && errno == EINTR );
}

//检查GPS是否合理
bool MapTools::CheckGPS(double lng,double lat,double lastlng,double lastlat){
	if (lat>3 && lat<53&& lng>73 && lng<130)
		if(abs(lastlat-lat)<0.0001&&abs(lastlng-lng)<0.0001) //10m
			return true;
	return false;
}

void MapTools::GPS2Earthy(double x, double y, int &earthx, int &earthy)//纬度，经度，单位是度,单位是厘米
{
	int n, L0;
	double X, N54, W54, t, m, a54, e54, e_54;
	double iptr;
	double t_2 = 0, t_4 = 0, yita_2 = 0, yita_4 = 0;
	double lp = 0, lp_2 = 0;
	double SinL, CosL, CosL_2, SinL_2;
	double SinG, CosG;
	double daa, df, db2p, dl2p;
    //double	dahm;
	double deltabo, deltalo;
	double w84, n84, m84, a84, e842, f84, f54, dx, dy, dz;
	double lati, logi;
	double pi = 3.1415926535;
	lati = x;
	logi = y;
	lati = lati*pi / 180;
	logi = logi*pi / 180;
	SinL = sin(lati);
	CosL = cos(lati);
	SinG = sin(logi);
	CosG = cos(logi);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a84 = 6378137.0;
	e842 = 0.00669437999014132;
	f84 = 1.0 / 298.257223563;
	a54 = 6378245.0;
	f54 = 1.0 / 298.3;
	dx = -16.0;
	dy = 147.0;
	dz = 77.0;
	w84 = sqrt(1 - e842*SinL_2);
	n84 = a84 / w84;
	m84 = a84*(1 - e842) / (w84*w84*w84);
	daa = a54 - a84;
	df = f54 - f84;
	db2p = (-dx*SinL*CosG - dy*SinL*SinG + dz*CosL + (a84*df + f84*daa)*sin(2 * lati)) / (m84*sin(1 / 3600.0*pi / 180));
	dl2p = (-dx*SinG + dy*CosG) / (n84*CosL*sin(1 / 3600.0*pi / 180));
	//dahm = dx*CosL*CosG + dy*CosL*SinG + dz*SinL + (a84*df + f84*daa)*SinL_2 - daa;
	deltabo = (db2p / 3600.0)*pi / 180.0;
	deltalo = (dl2p / 3600.0)*pi / 180.0;
	logi = logi + deltalo;
	lati = lati + deltabo;
	SinL = sin(lati);
	CosL = cos(lati);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a54 = 6378245.0;
	e54 = 0.0066934274898192;
	W54 = sqrt(1.0 - e54*SinL_2);
	N54 = a54 / W54;
	e_54 = 0.0067385254147;
	logi = logi * 180 / pi;
	modf(logi / 6.0, &iptr);
	n = (int)iptr + 1;
	L0 = n * 6 - 3;
	lp = (logi - L0)*pi / 180;
	lp_2 = lp*lp;
	m = CosL_2*lp_2;
	yita_2 = e_54*CosL_2;
	yita_4 = yita_2 * yita_2;
	t = tan(lati);
	t_2 = t*t;
	t_4 = t_2*t_2;
	double gps_xx, gps_yy;
	X = 111134.8611*lati * 180 / pi
		- SinL*CosL*(32005.7799 + 133.9238*SinL_2 + 0.6973*SinL_2*SinL_2 + 0.0039*SinL_2*SinL_2*SinL_2);
	gps_yy = X + N54*t*m*(0.5 + 1.0 / 24.0*(5.0 - t_2 + 9.0*yita_2 + 4.0*yita_4)*m
		+ 1.0 / 720.0*(61.0 - 58.0*t_2 + t_4)*m*m);
	gps_xx = N54*CosL*lp*(1.0 + 1.0 / 6.0*(1 - t_2 + yita_2)*m
		+ 1.0 / 120.0*(5.0 - 18.0*t_2 + t_4 + 14.0*yita_2 - 58.0*yita_2*t_2)*m*m);
	gps_xx = gps_xx + 1000000 * n + 500000;
	gps_xx *= 100;
	gps_yy *= 100;
	earthx = (int)gps_xx;
	earthy = (int)gps_yy;
	return;
}

//计算两对经纬度获取之间的直线距离(m)
double MapTools::GetDistanceByGPS(double lng1,double lat1,double lng2,double lat2){
	int earthxcur, earthycur;
	int earthxtemp, earthytemp;
	MapTools::GPS2Earthy(lat1 , lng1 , earthxcur, earthycur);
	MapTools::GPS2Earthy(lat2, lng2, earthxtemp, earthytemp);
	double x = abs(earthxcur - earthxtemp) / 100;
	double y = abs(earthycur - earthytemp) / 100;
	double distance = sqrt(x*x + y*y);
	return distance;
}

//通过路口ID获得路口索引
int MapTools::GetNodeIndexByID(const vector<MAP_BUTTON_NOTE> nodes,int id){
	for(size_t i=0;i<nodes.size();i++){
		if(nodes[i].idself==id)
			return i;
	}
	return -1;
}

//通过道路ID获得道路索引
int  MapTools::GetLineIndexByID(const vector<MAP_BUTTON_LINE> lines,int id){
	for(size_t i=0;i<lines.size();i++){
		if(lines[i].idself==id)
			return i;
	}
	return -1;
}


void MapTools::StructTransformLine(MAP_BUTTON_LINE* line, NJUST_MAP_INFO_ROAD **road){
	(*road)->roadnum = line->idself - START_LINE_ID + 1;
	switch (line->roadkind)
	{
	case 0:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_NONE;
		break;
	case 1:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_ASPHALT;
		break;
	case 2:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_CEMENT;
		break;
	case 3:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_DIRT;
		break;
	case 4:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_COBBLED;
		break;
	case 5:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_HIGHWAY;
		break;
	case 6:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_BRIDGE;
		break;
	case 7:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_TUNNEL;
		break;
	case 8:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_CULVERT;
		break;
	}
	switch (line->leftxingdaoxian)
	{
	case 0:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;

	}
	switch (line->middlexingdaoxian)
	{
	case 0:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;

	}
	switch (line->rightxingdaoxian)
	{
	case 0:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;
	}
	switch (line->leftdaolubianjie)
	{
	case 0:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_NONE;
		break;
	case 1:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONVEXBOUNDARY;
		break;
	case 2:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONCAVEBOUNDARY;
		break;
	case 3:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_DANGEROUSBOUNDARY;
		break;
	}

	switch (line->rightdaolubianjie)
	{
	case 0:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_NONE;
		break;
	case 1:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONVEXBOUNDARY;
		break;
	case 2:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONCAVEBOUNDARY;
		break;
	case 3:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_DANGEROUSBOUNDARY;
		break;
	}
	(*road)->nLaneNum = line->chedaonum;
	(*road)->roadWidth_cm = line->wedth;
	(*road)->curbWidth_cm = line->hyazi;
	(*road)->idealSpeed_kmh = line->idealspeed;
	(*road)->nSize = sizeof(NJUST_MAP_INFO_ROAD);
	(*road)->checksum = 0;
}

int MapTools::NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[])
{
		char *pdata = (char *)pUnknow;
		switch (date)
		{
		case 0://road信息
		{
				   globle[0] = '0';
				   memcpy(&globle[1], pdata, sizeof(NJUST_MAP_INFO_ROAD));
				   break;
		}
		case 1://node信息
		{
				   globle[0] = '1';
				   memcpy(&globle[1], pdata, sizeof(NJUST_MAP_INFO_NODE));
				   break;
		}
		case 2://方向导引
		{
				   globle[0] = '2';
				   memcpy(&globle[1], pdata, sizeof(NJUST_MAP_INFO_DIRECTION));
				   break;
		}
		}
		return 0;
}

void MapTools::StructTransformNote(MAP_BUTTON_NOTE* note, NJUST_MAP_INFO_NODE **node)
{
	(*node)->nodenum = note->idself - START_NODE_ID + 1;
	(*node)->x_cm = note->earthx;
	(*node)->y_cm = note->earthy;
	switch (note->lukou)
	{
	case 0:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_NONE;
		break;
	case 1:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_CROSSROADS;
		break;
	case 2:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_TNODE;
		break;
	case 3:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_YNODE;
		break;
	case 4:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_RIGHTANGLENODE;
		break;
	case 5:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_STRAIGHTLINENODE;
		break;
	}
	switch (note->HLD)
	{
	case 0:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_NONE;
		break;
	case 1:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_RIGHTFRONT;
		break;
	case 2:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_LEFTRONT;
		break;
	case 3:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_FRONT;
		break;
	}
	switch (note->HLDkind)
	{
	case 0:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_NONE;
		break;
	case 1:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHTRIGHT;
		break;
	case 2:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHT;
		break;
	case 3:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_RIGHTSTRAIGHT;
		break;
	case 4:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTRIGHT;
		break;
	case 5:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_STRAIGHT;
		break;
	}

	(*node)->zebraCrossing = note->zebra;
	(*node)->nSize = sizeof(NJUST_MAP_INFO_NODE);
	(*node)->checksum = 0;
}

double  MapTools::GetRotateAngle(double x1, double y1, double x2, double y2)//以度为单位
{
		const double epsilon = 1.0e-6;
		const double nyPI = acos(-1.0);
		double dist, dot, degree, angle;
		// normalize
		dist = sqrt(x1 * x1 + y1 * y1);
		x1 /= dist;
		y1 /= dist;
		dist = sqrt(x2 * x2 + y2 * y2);
		x2 /= dist;
		y2 /= dist;
		// dot product
		dot = x1 * x2 + y1 * y2;
		if (fabs(dot - 1.0) <= epsilon)
			angle = 0.0;
		else if (fabs(dot + 1.0) <= epsilon)
			angle = nyPI;
		else {
			double cross;
			angle = acos(dot);	
			cross = x1 * y2 - x2 * y1;
			if (cross < 0) {
				angle = 2 * nyPI - angle;
			}
		}
		degree = angle *  180.0 / nyPI;
		return degree;
}