#include"MapModule.h"
#include<iostream>
#include <time.h>
#include <stdarg.h>
using namespace std;


/////////////////////////////////测试函数/////////////////////////////////////////
//测试 MapFileStream的LoadMapNode
void testLoadMapNode(string s){
	NJUST_MAP_BUILD_MAP buildMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapNode(buildMap);
	
	vector<MAP_BUTTON_LINE>::iterator it=buildMap.mapLine.begin();
	for(;it!=buildMap.mapLine.end();it++){
		printf("%d\n",(*it).idself);
		//MAP_PRINT("%lf\n",(*it).b);
	}

	for(unsigned int i=0;i<buildMap.mapNode.size();i++){
		printf("NodeID:%d\n",buildMap.mapNode[i].idself);
		printf("gpsx:%lf\n",buildMap.mapNode[i].gpsx);
		printf("gpsy:%lf\n",buildMap.mapNode[i].gpsy);
		//for(unsigned int j=0;j<buildMap.mapNode[i].neigh;j++){
			//MAP_PRINT("%d\n",buildMap.mapNode[i].NeighLineID[j]);
		//}
	}

	//for(unsigned int i=0;i<buildMap.mapObs.size();i++){
		//MAP_PRINT("障碍物中心:%lf\n",buildMap.mapObs[i].ObstacleCenterGPS.longtitude);
	//}
}

//测试 MapFileStream的LoadMapTask
void testLoadMapTask(string s){
	vector<MAP_TASK_NODE> taskMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapTask(taskMap);
	vector<MAP_TASK_NODE>::iterator it=taskMap.begin();
	for(;it!=taskMap.end();it++){
		printf("%d:\n",(*it).noderesult);
		printf("%lf\n",(*it).longtitude);
		printf("%lf\n",(*it).latitude);
	}
	printf("结构体长度 %d",sizeof(MAP_TASK_NODE));
}

//邻接矩阵
void testReadAdjst(string s){
	NJUST_MAP_BUILD_MAP buildMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapNode(buildMap);
	mapFile->LoadAdjMat(buildMap);
	vector<int>::iterator it=buildMap.adjMat.begin();
	int count=buildMap.mapNode.size();
	for(int i=0;i<count;i++){
		for(int j=0;j < count ;j++)
			MAP_PRINT("%10d ",*(it+i*count+j));
		MAP_PRINT("%s\n","");
	}
}

//测试 apFileStream的读取序列点
void testReadMapNode(string s){
	vector<MAP_DOUBLE_POINT> GPSlist;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->ReadMapGPS(1,2,GPSlist,true);

	int len=GPSlist.size();
	printf("count:%d\n",len);
	printf("fiset one: %lf  %lf\n",GPSlist[0].x,GPSlist[0].y);
	printf("fiset one: %lf  %lf\n",GPSlist[len-1].x,GPSlist[len-1].y);

	mapFile->ReadMapGPS(1,2,GPSlist,false);

	len=GPSlist.size();
	printf("count:%d\n",len);
	printf("fiset one: %lf  %lf\n",GPSlist[0].x,GPSlist[0].y);
	printf("fiset one: %lf  %lf\n",GPSlist[len-1].x,GPSlist[len-1].y);
}

//测试 读取记录文件
void testReadRecord(const char * filename){
	int len,code; 
	int curID,lastID;
	MAP_DOUBLE_POINT point;
	NJUST_PLAN_PATH _planPath;
	vector<MAP_DOUBLE_POINT> _GPSList;

	FILE *pFile=fopen(filename,"rb");

	fread(&curID,sizeof(int),1,pFile);    //curID
	fread(&lastID,sizeof(int),1,pFile);   //lastID

	//规划路线
	fread(&len,sizeof(int),1,pFile); 
	fread(&_planPath.cur,sizeof(int),1,pFile); 
	for(int i=0;i<len;i++){
		fread(&code,sizeof(int),1,pFile);
		_planPath.planPathQueue.push_back(code);
	}

	//GPS点
	fread(&len,sizeof(int),1,pFile);
	printf("len：%d\n",len);
	for(int i=0;i<len;i++){
		fread(&point,sizeof(MAP_DOUBLE_POINT),1,pFile);
		_GPSList.push_back(point);
	}
	fclose(pFile);

	printf("curID：%d\n",curID);
	printf("lastID：%d\n",lastID);
	printf("_planPath.cur：%d\n",_planPath.cur);
	for(int i=0;i<_planPath.planPathQueue.size();i++){
		printf("%d:%d\n",i+1,_planPath.planPathQueue[i]);
	}
	for(int i=0;i<_GPSList.size();i++){
		printf("%d:%lf\n",i+1,_GPSList[i].y);
	}

}
///////////////////////////////////初始化DEBUG输出///////////////////////////////////////



int main(int argc,char *argv[]){
	//Step 1 -----------检验输入参数--------------
	string s;
	if(argc>=2){
		s=argv[1];
	}else{
		s="njustmap/";
	}

	//Step 2 -----------运行--------------
	MapApp mapapp;
	mapapp.intialize(s.c_str());
	mapapp.run();
	//mapapp.simulate();

	//Step 3 -----------关闭DEBUG--------------
	fclose(gDEBUG_OUT);
	fclose(gLOG_OUT);

	//测试
	//testLoadMapNode(s.c_str());
	//testReadAdjst(s.c_str());
	//testLoadMapTask(s.c_str());
	//testReadRecord("record.db");
	return 0;
}
