
#ifndef _MAIN_H_
#define _MAIN_H_
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>  
#include <stdlib.h>
#include <time.h>
#include <memory.h>
#include <math.h>
#include <unistd.h>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <termios.h>
#include <pthread.h>
#include "NJUST_ALV_BYD.h"
#include "Parameter.h"
#include "DataStruct.h"

#define min(a,b) (a>b?b:a)

    void	    SysrtemInitial();      //初始化参数 
	void		ReadInitialInfor();    //读取路径信息 from board
	void	    ReadInitialNodeQueue();//读取初始化道路信息？
	void	    ReadNodeFileRestore(char * file);
	int         GetNearPassNode();
    void        OnClose();
	double      TranDegreeToMinute(double* a);
	void        StructTransformNote(CButtonNote* note,NJUST_MAP_INFO_NODE **node);
	void        StructTransformLine(CButtonLine* line,NJUST_MAP_INFO_ROAD **road);
	void        ProcessingAndSend(double wholelongtitude,double wholelatitude);
    void        blh2xy(double x,double y,int &earthx,int &earthy);
	int         FindgpsCur(double m_gpsxcur,double m_gpsycur);
	double      TranDegreeToMinute(double *a);
	int         RoadNodeLocation(double longtitude, double latitude);//以分为单位，返回值从0开始
	void        PointToPointDistance(CPoint point1,CPoint point2,double &distance);
	double      Distance(double longtitude1,double latitude1,double longtitude2,double latitude2);//分，分，分，分
    double      DistanceToNode(double longtitude1, double latitude1, int pInitialNodeQueue);//分，分，度，度
	double      DistanceToOri(double longtitude1, double latitude1, int pludianori);//分，分，度，度
    int         locationGPS(double longtitude,double latitude);
	double      NextNodeDistance(CButtonLine*  pLine,double longtitude,double latitude);
	void        Send(double longtitude,double latitude,int curlocation,int igpscur);
	int         Location(double longtitude,double latitude);
    void        GPSToOffset(double longtitude,double latitude,CPoint &point);
    int         FindAjustNoteOffset(CPoint point);
    void        PathPlaning(int startPoint, int endPoint, int escapePoint[], int escapePointNum,
				            int passPoint[], int passPointNum, int pathPlanningQueue[], int &Queuelength);
    void        dijstra(int startPoint, int endPoint, int escapes[], int pathPlanning[], int& planninglength);
	void        ReadLinjie();
	void        WriteLinjie();
	void        ReadRecord(int &node, int &cur, int &next, int &a, int & b, int &flag, int &ludian);
	void        Record(int node,int cur,int next,int a,int b, int flag, int ludian);
	void        RecordLujing(int lu[]);
	void        ReadLineGPS(int a,int b);//1开始，1开始
	void        ReadNodeGPS(int a,int b);//1开始，1开始
	void        mitoa(int i,char* a);
	int         GetAnotherNode(int line,int onenode);
	int         GetAnotherLine(int node,int line1,int line2);
    void        GetTurnChange(int m_lastline, int m_lastnode,int m_nextline,Turn & m_turn);//第一个参数是当前点，第二个参数是方向结构体
	double      getRotateAngle(double x1, double y1, double x2, double y2);//以度为单位
	void        ReadNodeFileRestore();
	int         RoadNodeLocation(double longtitude, double latitude);//以分为单位，返回值从0开始
    void        Modify_1_2(char * file);
    void        JudgeDirection(int ori_or_ininode,int numth, NJUST_MAP_PROPRITY & proprity);
	void        ReadGPSSpecial(char * file);
    int         MCCallBack( void* mc_to_map, size_t size, void* args);
    int		    MOCallBack( void* mo_to_map, size_t size, void* args);
    int         PLCallBack( void* pl_to_map, size_t size, void* args);
    void        *Processing(void *ptr);
    int         NJUST_MAP_Encode_IP_Data(const void* pUnknow,int date,char globle[]);
    bool        IsDebug();
    void        SetDebug();
    void        SetRelease();
	void        CreatDebug();
	void        GetFirstFiveGps(double wholelongtitude,double wholelatitude);
	void        GetMeanGps(double wholelongtitude,double wholelatitude,double &longtitude, double &latitude);
	void        ChongqiChengxuBegin();
	void        ChongqiChengxuHalf();
	void        Send_TO_PL_MO_line(double longtitude, double latitude, int curlocation, int igpscur);
	void        Send_TO_PL_MO_Node(double longtitude,double  latitude, int curlocation,int  igpscur);
	void        GetDirection(NJUST_MAP_INFO_NODE &node);

//初始化用到的全集变量-------------------------------------------------------------------------
    CButtonNote*       pButtonNote[NOTESIZE];
	CButtonLine*       pButtonLine[LINESIZE];
	int                notecounter;
	int                linecounter;
    double             m_adjustx;
	double             m_adjusty;
//算法部分用到的全局变量-------------------------------------------
    int                m_flaglocation;            ///第一次定位找最近节点标志
	int                m_ncur;                       //上一次定位的节点
	int                m_nnext;
	int                m_locationresultlineornote;//上一次定位的节点（<nodecounter,）或者直线(>nodecounter)
	int                m_counterflag;             //首先存储30个GPS点求平均值标志
	int                m_counter;                 //0~29
	int                m_counterzero;             //30个GPS点中0的个数
	double             templongtitude[SIZESHUZU]; //存放30个GPS点
	double             templatitude[SIZESHUZU];	  //同上
	double             m_longtitudemeannew;       //三十个GPS均值
	double			   m_latitudemeannew;         //同上
	GpsPoint           m_gpsArry[2048];  //每条路径的2048*200（每隔2米才一个点）=4096m=4km
	int                m_arryNum;   //保存的GPS点序列个数
	int                m_gpscur;    //当前定位到的GPS点序列中的位置0~m_arryNum
	int                m_firstflag; //标记初始时搜索当时所在的GPS序列位置
	int                m_lastnode;//标记上一个节点,堆每一个经过的路口都记录，从1开始
	int                m_lastline;//标记上一个经过的道路，这个道路是m_lastnode前的那个道路,而不是lujing[plujing]前的，从1开始
	//**********************************************定位用变量
	int                adjust[MAX_NODE_NUM][MAX_NODE_NUM];
    ROADNODE           InitialNodeQueue[MAX_NODE_NUM];//路点文件中给出的路口数组
	int                InitialNodeQueueNum;//路点文件中给出的路口的个数
	int                pInitialNodeQueue; //下一个要通过的路口
	//处理路点所用到的变量
	RoadFileNode       m_roadnode[300];//存放最初的未加工的路点文件
	int                ludiannum;//最初的路点的个数    
	int                pludianori;//指示下一个要经过的原始路点
	int                pludianorilast;//指示上一个要经过的原始路点
	int                begain;
	int                lujing[MAX_NODE_NUM]; //规划处的路径的数组,1开始
	int                lujingnum;  //路径的总个数
	int                plujing;   //下一个要到的节点
	int                PassNodeQueue[MAX_NODE_NUM];
	int                PassNodeQueueNum;
	int                EscapePointQueue[MAX_NODE_NUM];
	int                pEscapePointQueue;
	int                EscapePointQueueNum;
	int                chongguihuaflag;//初始化微1，PL发过来的重规划标志
	int                planningstate;
    int                ReadLineGPSflag;
    int                ReadNodeGPSflag;
	int                lineflag;
	int                special;
	int                chongqiinitial;
	//记录状态用
	int                nodeth;//上一次经历过的ROADNODE结构体中InitialNodeQueue[MAX_NODE_NUM]中的点的序号;0开始
	int                m_cur;//上一次经过的节点
	int                m_next;//下一个要经过的节点
	int                start;//最近一次加载的路径的起始
	int                end;//最近一次家在路径的终点
	int                flag;//上述加载的路径是道路还是路口过度序列，取值为0或者1，0表示记录的序列为道路的序列，即两端为节点
	int                ludianorith;//上一次经历过的原始路点的序号从0开始
//main函数所到的全局变量---------------------------------------------------
   NJUST_IP_TIME       Iptime;
   char                Timeget1[24];
   char                Timeget2[24];
   int                 frame;
   int                 cmdid;
   NJUST_IP_TIME       iptimemc1;
   NJUST_IP_TIME       iptimelocal;
   int                 m_order;        //正向搜寻还是逆向搜寻
   double              longtitude_degree;  //从MC获取的GPS的信息
   double              latitude_degree;
   char                buff[BUFFSIZE];  //存放发送结构体的数组
   int                 mocounter;//显控发送频率控制
   pthread_mutex_t     gMutex = PTHREAD_MUTEX_INITIALIZER;
   pthread_cond_t      cond = PTHREAD_COND_INITIALIZER;
   //计算丢包率
   int                 startNavID ;
   int                 endNavID ;
   int                 countNavID;
   bool                isFirst;
   bool                gIsDebug;
#endif
