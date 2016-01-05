#ifndef _DATASTRUCT_H_
#define _DATASTRUCT_H_
//----------------------------------------------------------------------------
typedef struct
{
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


}CButtonNote;
//----------------------------------------------------------------------------
typedef struct  
{
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


}CButtonLine;
//----------------------------------------------------------------------------
typedef struct 
{
	double x;
	double y;
}CPoint;


//---------------------------------------------------接受MC的gps信息结构体
typedef struct gpsinfor
{
	int FrameID;
    double longtitude_degree;
    double latitude_degree;

} GPSInfor;
//----------------------------------------------------------------------------
//GPS点序列文件存放的点的结构体
typedef struct 
{
	double gpsx;
	double gpsy;
}GpsPoint;

//线程同步结构体
typedef struct
{
    pthread_cond_t cond ;
    pthread_mutex_t mutex ;
} CondForThread;


typedef struct data
{
	int     m_adjustx;
	int     m_adjusty;
	int     notecounter;
	int     linecounter;
}DATA;

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
	
}NODE;
	

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
}ROAD;

typedef struct record
{
	int nodenum;
	int cur;
	int next;
	int a;
	int b;
	int flag;
	int ludian;
}RecordNode;


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


typedef struct road_node
{
    int       num;
	double    longtitude;
	double    latitude;               //以度为单位
	int       noderesult;            //在路网中则输出定位到的点,不在路网中则输出-1
	int       shuxing1;
	int       shuxing2;
	int       duiyingludianbianhao;  
}ROADNODE;




typedef struct turn
{
	int turn;//0直行，1左拐，2右拐，3左Uturn，4右Uturn,5未知
	double turndegree;
}Turn;
//----------------------------------------------------------------------------
#endif