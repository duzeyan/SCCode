#include"MapTools.h"
#include"MapFileStream.h"

extern FILE* gDEBUG_OUT;

///
/// MapFileStream
///

#define MAP_FILEPATH_MAX 255 //文件路径长度上限

//构造函数 需要传入 数据文件的目录
MapFileStream::MapFileStream(const char* loadpath){
	strcpy(this->loadpath,loadpath);
}

//加载自建地图 路点信息
void MapFileStream::LoadMapNode(NJUST_MAP_BUILD_MAP &map){
	MAP_BUILD_FILE_HEAD      mapHead;
	vector<MAP_NODE>::iterator itNode;
	vector<MAP_ROAD>::iterator itRoad;
	//中间变量
	MAP_NODE tNode;										  
	MAP_ROAD tRoad;
	MAP_BUTTON_NOTE tButtonNode;
	MAP_BUTTON_LINE tButtonLine;
	NJUST_MAP_OBSTACLE obs;

	//Step 1 -----------拼接文件路径--------------
	char filename[]="board.db";
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	//Step 2 -----------打开文件--------------
	FILE *pFile = fopen(path, "rb");
	if(pFile==NULL){
		MAP_PRINT("打开地图结构文件失败\n","");
		return ;
	}

	//Step 3 -----------读取文件头--------------
	fread(&mapHead,sizeof(MAP_BUILD_FILE_HEAD),1,pFile);  //读取文件头 包含了节点和道路的个数
	map.adjustPoint.x=mapHead.m_adjustx;
	map.adjustPoint.y=mapHead.m_adjusty;
	MAP_PRINT("X:%lf\n",map.adjustPoint.x);
	MAP_PRINT("Y:%lf\n",map.adjustPoint.y);

	//Step 4 -----------读取节点序列--------------
	for(int i=0;i<mapHead.notecounter;i++){              //读道路信息 路口
		fread(&tNode, sizeof(MAP_NODE), 1, pFile);
		MapTools::Node2ButtonNode(tNode,tButtonNode);
		map.mapNode.push_back(tButtonNode);
	}

	//Step 5 -----------读取道路序列--------------
	for(int i=0;i<mapHead.linecounter;i++){              //读道路信息  道路
		fread(&tRoad, sizeof(MAP_ROAD), 1, pFile);
		MapTools::Line2ButtonLine(tRoad,tButtonLine);
		map.mapLine.push_back(tButtonLine);
	}

	//Step 5 -----------读取障碍物--------------
	for(int i=0;i<mapHead.obstaclecounter;i++){              //读道路信息  道路
		fread(&obs, sizeof(NJUST_MAP_OBSTACLE), 1, pFile);
		map.mapObs.push_back(obs);
	}

	fclose(pFile);
	MAP_PRINT("%s加载路点信息\n","");
}

//加载地图中的任务路点(需要规划)
void MapFileStream::LoadMapTask(vector<MAP_TASK_NODE> &mapTaskNode){
	//Step 1 -----------拼接文件路径--------------
	char filename[]="InitialNodeQueue.db";
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	//Step 2 ----------打开文件--------------
	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		MAP_PRINT("打开地图任务路点失败\n","");
		return ;
	}

	//Step 3 -----------读取文件内容--------------
	MAP_TASK_NODE buff[100];
	fseek(pf, 0L, SEEK_END);
	int len = ftell(pf) / sizeof(ROADNODE);
	fseek(pf, 0L, SEEK_SET);
    for(int i=0;i<len;i++)
	{
		fread(&(buff[i]), sizeof(ROADNODE), 1, pf);
		mapTaskNode.push_back(buff[i]);
    }
    
    fclose(pf);

	//MAP_PRINT("%s待规划路径：","");
	//FILE *flog=fopen("logInitNode" ,"w");
	//for(unsigned int i=0;i<mapTaskNode.size();i++){
	//	fprintf(pf,"%d ",mapTaskNode[i].num);
	//	fprintf(pf,"%lf ",mapTaskNode[i].longtitude);
	//	fprintf(pf,"%lf ",mapTaskNode[i].latitude);
	//	fprintf(pf,"%lf ",mapTaskNode[i].noderesult);
	//	fprintf(pf,"%lf ",mapTaskNode[i].shuxing1);
	//	fprintf(pf,"%s","\n");
	//}
	//fclose(flog);
	//MAP_PRINT("%s","\n");
}

//加载自建地图的邻接矩阵 注意，调用次方法前必须已经调用LoadMapNode
void MapFileStream::LoadAdjMat(NJUST_MAP_BUILD_MAP &map){
	int noteCount=map.mapNode.size();
	map.adjMat.reserve(noteCount*noteCount);
	int i;
	int buff=0;

	//打开文件
    char filename[]="adjust.db";
	char path[MAP_FILEPATH_MAX];
	strcpy(path,loadpath);
	strcat(path,filename);  //拼接完整目录

	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		MAP_PRINT("加载邻接矩阵文件失败\n","");
		return ;
	}
	
	//读取矩阵
	//读块内存的方法
	/*int tAdj[noteCount*noteCount];
	fread(tAdj, sizeof(int), noteCount*noteCount, pf);
	for (i = 0; i<noteCount*noteCount; i++)
	{		
		map.adjMat.push_back(tAdj[i]);
	}*/

	//按字节的方法
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
	if(pf==NULL){
		MAP_PRINT("加载GPS序列文件失败\n","");
		return ;
	}

	//正序和逆序都尝试一遍
	//if (pf == NULL)
	//{
	//	isOrder=false;
	//	memset(path,0,50);
	//	sprintf(filename,"%d%c%d.db",b,cj,a);
	//	strcpy(path,loadpath);
	//	strcat(path,filename);  //拼接完整目录

	//	pf = fopen(path ,"rb");
	//}
	//if(pf==NULL){
	//	perror("perror");
	//	return ;
	//}
	//

	//读取GPS序列
	fseek(pf, 0L, SEEK_END);
	GPSnum = ftell(pf) / sizeof(MAP_DOUBLE_POINT);
	fseek(pf, 0L, SEEK_SET);
	for(int i=0;i<GPSnum;i++){
		fread(&tPoint, sizeof(MAP_DOUBLE_POINT), 1, pf);
		GPSList.push_back(tPoint);
	}
	//逆序处理
	/*if(!isOrder){
		reverse(GPSList.begin(),GPSList.end());
	}*/

	fclose(pf);

	MAP_PRINT("加载 %s文件\n",filename);
}


