#include<vector>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include"MapModule.h"

///
/// MapApp
///
MapApp::MapApp(const char* loadpath){
	//参数设置默认值
	this->Intialize();

	//构建
	mapFile=new MapFileStream(loadpath);
	
}

MapApp::~MapApp(){
	if(mapFile!=NULL){
		delete mapFile;
	}
}

void MapApp::Intialize(){
	mapFile=NULL;
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

	fread(&mapHead,sizeof(MAP_BUILD_FILE_HEAD),1,pFile);  //读取文件头 包含了节点和道路的个数

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
}


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