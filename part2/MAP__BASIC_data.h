#ifndef _MAPBASICDATA_H_
#define _MAPBASICDATA_H_
//----------------------------------------------------------------------------
#include<vector>
#include "NJUST_ALV_BYD.h"

using std::vector;
//缓存最近GPS个数
#define CACHE_GPS_LEN     5
#define INITL_GPS_VALUE   -1


//节点ID开始值
#define START_NODE_ID   10000
//道路DI开始值
#define START_LINE_ID   20000

#define _MAP_LOG 1
#if _MAP_LOG
	#define MAP_PRINT(fmt,v) printf(fmt,v);fflush(stdout)
#else
    #define MAP_PRINT(fmt,v) 
#endif


typedef struct turn
{
	int turn;//0直行，1左拐，2右拐，3左Uturn，4右Uturn,5未知
	double turndegree;
}MAP_TURN;


//采集地图信息时 路口结构
typedef struct MAP_BUTTON_NOTE{
	//节点对应与实际路口的信息

	int     neigh;//分支数
	int     HLD;//红绿灯位置
	int     HLDkind;//红绿灯类型
	int     lukou;//路口
	int     zebra;//斑马线
	double  gpsx;//gpsx对应于longtitude
	double  gpsy;//gpsy对应于latitude
	int     earthx;
	int     earthy;//大地坐标

	//节点在屏幕上的位置信息
	 //创建时可知，创建时被赋值

	int    idself;
	 //画线时可知，画线时被赋值
	int     NeighNoteID[4];
	int     NeighLineID[4];


}MAP_BUTTON_NOTE;
//----------------------------------------------------------------------------

//采集地图信息时 道路结构
typedef struct  MAP_BUTTON_LINE{
		//在屏幕上信息
        //创建时即可知，创建时被赋值

	int       idself;
    //画线时可知，画线时被赋值
	int       idstart;
	int       idend;
	//直线斜率方程,以画线区域为坐标系
	//kx+by+c=0
	double        k;
	double        b;
	double        c;

     //实际道路信息   
	int     roadkind;//道路种类
	float   wedth;//道路宽度,单位厘米
	float   length;//道路长度

	int     maluyazi;//马路牙子
	float   hyazi;//马路牙子高度，单位厘米
	int     hulan;//护栏
	float   hhulan;//护栏高度					
    int     xingdaoxiannum; //行道线数目
    int     leftxingdaoxian;//左行道线
	int     middlexingdaoxian;//中间行道线
	int     rightxingdaoxian;//右行道线

	int     chedaonum;//车道数目
    int     leftdaolubianjie;//左道路边界
	int     rightdaolubianjie;//右道路边界
	int     idealspeed;   //建议速度


}MAP_BUTTON_LINE;

//----------------------------------------------------------------------------
//数据类型为double 的点对
typedef struct MAP_DOUBLE_POINT{
	double x;
	double y;
}MAP_DOUBLE_POINT;


//---------------------------------------------------接受MC的gps信息结构体
typedef struct  MAP_MC_GPS{
	int FrameID;
    double longtitude_degree;
    double latitude_degree;

} MAP_MC_GPS;
//----------------------------------------------------------------------------



//线程同步结构体
//typedef struct CondForThread
//{
//    pthread_cond_t cond ;
//    pthread_mutex_t mutex ;
//} MAP_THTEAD_MUTEX;

//自建地图文件头
typedef struct data
{
	int     m_adjustx;
	int     m_adjusty;
	int     notecounter;
	int     linecounter;
}MAP_BUILD_FILE_HEAD;


//计算丢包率
typedef struct MAP_PACKAGE{
	int startID; //第一个接收的帧ID
	int endID;	 //最后一个接收的帧ID
	int count;   //统计接收了多少帧
	NJUST_IP_TIME startTime;//本帧开始接收时间
}MAP_PACKAGE;
typedef struct node
{
	int     idself;
    int     neigh;//分支数
	int     NeighNoteID[4]; //默认一个路口最多跟四个道路相连  跟本路口连接的路口ID
	int     NeighLineID[4]; //跟本路口连接的道路ID  上线为neigh
	int     HLD;//红绿灯位置
	int     HLDkind;//红绿灯类型
	int     lukou;//路口
	int     zebra;//斑马线
	double  gpsx;//gpsx对应于longtitude
	double  gpsy;//gpsy对应于latitude
	int     earthx;
	int     earthy;
	
}MAP_NODE;
	

typedef struct line
{
	int       idself;
	int       idstart;
	int       idend;
	//kx+by+c=0
	double        k;
	double        b;
	double        c; 
	int     roadkind;//道路种类
	float   wedth;//道路宽度,单位厘米
	float   length;//道路长度
	int     maluyazi;//马路牙子
	float   hyazi;//马路牙子高度，单位厘米
	int     hulan;//护栏
	float   hhulan;//护栏高度					
    int     xingdaoxiannum; //行道线数目
    int     leftxingdaoxian;//左行道线
	int     middlexingdaoxian;//中间行道线
	int     rightxingdaoxian;//右行道线
	int     chedaonum;//车道数目
    int     leftdaolubianjie;//左道路边界
	int     rightdaolubianjie;//右道路边界
	int     idealspeed;   //建议速度
}MAP_ROAD;

typedef struct record
{
	int nodenum;
	int cur;
	int next;
	int a;
	int b;
	int flag;
	int ludian;
}MAP_RECORD_NODE;


typedef struct roadfile
{
	int     num;
	double  longtitude;//度为单位
	double  latitude;
	double  hight;
	int     shuxing1;
	int     shuxing2;
}RoadFileNode;


enum PROPRITY   //路点属性
{
	START = 0x00, //起点
	STRAIGHT,     //直行
	LEFT,         //左拐
	RIGHT,        //右拐
	Uturn,        //Uturn
	TRAFIC_SIGN,   //交通标志
	SPECIAL_TASK,  //特殊任务
	END           //结尾
};

//在任务文件中给出的节点
typedef struct ROADNODE
{
    int       num;
	double    longtitude;
	double    latitude;               //以度为单位
	int       noderesult;            //在路网中则输出定位到的点,不在路网中则输出-1
	int       shuxing1;
	int       shuxing2;
	int       duiyingludianbianhao;  
}MAP_TASK_NODE;



//存储当前规划的路径信息
typedef struct NJUST_PLAN_PATH{
	vector<int> planPathQueue;			 //最新规划的路径 ， 节点编号，道路编号，节点编号，。。。，节点编号
	int cur;						 //当前时刻 车辆到达的位置(道路or路口)
}NJUST_PLAN_PATH;

//自建地图信息
typedef struct NJUST_MAP_BUILD_MAP{
	MAP_DOUBLE_POINT adjustPoint;		//转化大地坐标的原点,防止运算溢出 
	vector<MAP_BUTTON_NOTE>  mapNode;   //自建地图中的所有路口
	vector<MAP_BUTTON_LINE>  mapLine;   //自建地图中的所有道路
	vector<int> adjMat;			     //地图中领结矩阵 矩阵转一维向量存取
}NJUST_MAP_BUILD_MAP;

//GPS信息
typedef struct NJUST_MAP_GPS_INFO{
	double curLongtitude;			 //最新的经度 单位度
	double curLatitude;				 //最新的维度

	double lastLongtitude;			 //GPS点的平局值 经度
	double lastlatitudel;			 //GPS点的平局值 维度
	//double cacheLng[CACHE_GPS_LEN];  //缓存GPS点 经度
	//double cacheLat[CACHE_GPS_LEN];  //缓存GPS点 维度
	//int curOld;                      //缓存游标 指向最旧数据
}NJUST_MAP_GPS_INFO;
//----------------------------------------------------------------------------
#endif