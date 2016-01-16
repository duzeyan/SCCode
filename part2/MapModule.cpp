#include<vector>
#include "NJUST_ALV_BYD.h"
#include"MAP__BASIC_data.h"
#include"MapModule.h"

///
/// MapApp
///
MapApp::MapApp(const char* loadpath){
	//��������Ĭ��ֵ
	this->Intialize();

	//����
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

//���캯�� ��Ҫ���� �����ļ���Ŀ¼
MapFileStream::MapFileStream(const char* loadpath){
	strcpy(this->loadpath,loadpath);
}

//�����Խ���ͼ ·����Ϣ
void MapFileStream::LoadMapNode(NJUST_MAP_BUILD_MAP &map){
	MAP_BUILD_FILE_HEAD      mapHead;
	vector<MAP_NODE>::iterator itNode;
	vector<MAP_ROAD>::iterator itRoad;

	char filename[]="board.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //ƴ������Ŀ¼

	map.mapNode.reserve(100);
	map.mapLine.reserve(100);
	
	//�м����
	MAP_NODE tNode;										  
	MAP_ROAD tRoad;
	MAP_BUTTON_NOTE tButtonNode;
	MAP_BUTTON_LINE tButtonLine;
	
	FILE *pFile = fopen(path, "rb");
	if(pFile==NULL){
		perror("perror");
		return ;
	}

	fread(&mapHead,sizeof(MAP_BUILD_FILE_HEAD),1,pFile);  //��ȡ�ļ�ͷ �����˽ڵ�͵�·�ĸ���

	for(int i=0;i<mapHead.notecounter;i++){              //����·��Ϣ ·��
		fread(&tNode, sizeof(MAP_NODE), 1, pFile);
		MapTools::Node2ButtonNode(tNode,tButtonNode);
		map.mapNode.push_back(tButtonNode);
	}

	for(int i=0;i<mapHead.linecounter;i++){              //����·��Ϣ  ��·
		fread(&tRoad, sizeof(MAP_ROAD), 1, pFile);
		MapTools::Line2ButtonLine(tRoad,tButtonLine);
		map.mapLine.push_back(tButtonLine);
	}

	fclose(pFile);
}

//�����Խ���ͼ�� ����·��(��Ҫ�滮)
void MapFileStream::LoadMapTask(vector<MAP_TASK_NODE> &mapTaskNode){
	mapTaskNode.reserve(100);

	char filename[]="InitialNodeQueue.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //ƴ������Ŀ¼

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
}

//�����Խ���ͼ���ڽӾ��� ע�⣬���ôη���ǰ�����Ѿ�����LoadMapNode
void MapFileStream::LoadAdjMat(NJUST_MAP_BUILD_MAP &map){
	int noteCount=map.mapNode.size();
	map.adjMat.reserve(noteCount*noteCount);
	int i;
	int buff=0;

	//���ļ�
    char filename[]="adjust.db";
	char path[50];
	strcpy(path,loadpath);
	strcat(path,filename);  //ƴ������Ŀ¼

	FILE *pf = fopen(path ,"rb");
	if(pf==NULL){
		perror("perror");
		return ;
	}
	
	//��ȡ����
	for (i = 0; i<noteCount*noteCount; i++)
	{
		fread(&buff, sizeof(int), 1, pf);
		map.adjMat.push_back(buff);
	}
	fclose(pf);
}


//����ָ��·��gps����
void MapFileStream::ReadMapGPS(int a,int b,vector<MAP_DOUBLE_POINT> &GPSList,bool isNode){
	GPSList.clear();
	char cj=isNode?'+':'-';
	char path[50];
	char filename[20];
	bool isOrder=true;		//�Ƿ����ļ���˳���ȡ
	int GPSnum=0;
	MAP_DOUBLE_POINT tPoint;
	GPSList.reserve(2048);

	
	sprintf(filename,"%d%c%d.db",a,cj,b); //1-2.db or 1+2.db
	strcpy(path,loadpath);
	strcat(path,filename);  //ƴ������Ŀ¼
	

	FILE *pf = fopen(path ,"rb");
	

	//��������򶼳���һ��
	if (pf == NULL)
	{
		isOrder=false;
		memset(path,0,50);
		sprintf(filename,"%d%c%d.db",b,cj,a);
		strcpy(path,loadpath);
		strcat(path,filename);  //ƴ������Ŀ¼

		pf = fopen(path ,"rb");
	}
	if(pf==NULL){
		perror("perror");
		return ;
	}
	

	//��ȡGPS����
	fseek(pf, 0L, SEEK_END);
	GPSnum = ftell(pf) / sizeof(MAP_DOUBLE_POINT);
	fseek(pf, 0L, SEEK_SET);
	for(int i=0;i<GPSnum;i++){
		fread(&tPoint, sizeof(MAP_DOUBLE_POINT), 1, pf);
		GPSList.push_back(tPoint);
	}
	//������
	if(!isOrder){
		reverse(GPSList.begin(),GPSList.end());
	}

	fclose(pf);

	MAP_PRINT("Read:%s\n",path);

}










////
/// MAPģ�鹤����
///
//�ڵ�ת��
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

//��·ת��
void  MapTools::Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine){
		buttonLine.idself = line.idself;
		buttonLine.idstart = line.idstart;
		buttonLine.idend = line.idend; //ID
		buttonLine.k = line.k;           //ֱ�߲���
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