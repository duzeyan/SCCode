#include"MapModule.h"
#include<iostream>
using namespace std;

//测试 MapFileStream的LoadMapNode
void testLoadMaoNode(string s){
	NJUST_MAP_BUILD_MAP buildMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapNode(buildMap);
	cout<<buildMap.mapLine.size()<<endl;
	cout<<buildMap.mapNode.size()<<endl;
	vector<MAP_BUTTON_LINE>::iterator it=buildMap.mapLine.begin();
	for(;it!=buildMap.mapLine.end();it++){
		printf("%d\n",(*it).idself);
	}
}

//测试 MapFileStream的LoadMapTask
void testLoadMapTask(string s){
	vector<MAP_TASK_NODE> taskMap;
	MapFileStream *mapFile=new MapFileStream(s.c_str());
	mapFile->LoadMapTask(taskMap);
	vector<MAP_TASK_NODE>::iterator it=taskMap.begin();
	for(;it!=taskMap.end();it++){
		printf("%d\n",(*it).noderesult);
	}
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
			printf("%10d ",*(it+i*count+j));
		printf("\n");
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


int main(int argc,char *argv[]){
	string s;
	if(argc>=2){
		s=argv[1];
	}else{
		s="";
	}
	MapApp mapapp(s.c_str());
	mapapp.Run();

	return 0;
}
