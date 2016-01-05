#include "main.h"
using namespace std;
ofstream         oflocal;
int main()
{
	//处理线程id
	pthread_t processing;//****************************************************************************第一步，注册本模块
	//判断模块名是否存在
	if (NJUST_IP_moduleName_exist("MAP"))
	{
		//若不存在，则创建
		if (NJUST_IP_set_moduleName("MAP", 0))
		{
			return 0;
		}
	}
	
	CreatDebug();
	
	SysrtemInitial();
	ReadInitialInfor();              //读board.db
	ReadInitialNodeQueue();          //读InitialNodeQueue.db  定位原始路点文件中，路口(起点，终点)属性点 
                                      //在路网(board.db)中的定位结果
	ReadNodeFileRestore();           //读task.txt路点文件
	ReadLinjie();                    //读adjust.db
	
	
	if (-1 == NJUST_IP_set_tcp_callBack("MO", MOCallBack, NULL))
	{
		return 0;
	}
	//****************************************************************************第四步，开启PL回调函数
	if (-1 == NJUST_IP_set_tcp_callBack("PL", PLCallBack, NULL))
	{
		return 0;
	}
	////****************************************************************************第六步，广播接收来自MC的数据
	NJUST_IP_set_broadcast_callBack(MCCallBack, NULL);
	//创建数据处理（定位）线程
	if (pthread_create(&processing, NULL, Processing, NULL))
	{
		return 0;
	}
	//阻塞主线程,等待处理线程结束(外部指令退出)
	pthread_join(processing, NULL);
	OnClose();
	return 0;
}

int MCCallBack(void* mc_to_map, size_t size, void* args)//mc_to_map是包的指针，size是包的大小，args是附加参数
{

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
		if (isFirst)
		{
			startNavID = pNav->navID;
			endNavID = pNav->navID;
			isFirst = false;
		}
		else
		{
			endNavID = pNav->navID;
		}
		countNavID++;
		iptimemc1 = NJUST_IP_get_time();
		char  TIME2[24];
		NJUST_IP_get_timeStr(iptimemc1,TIME2);
				                             //   if (IsDebug())          oflocal<<"RECV MC iptimemc="<<TIME2<<endl;
				                              //  if (IsDebug())          oflocal<<"MC fRAMEid="<<pNav->navID<<endl;
		pthread_mutex_lock(&gMutex);
		longtitude_degree = pNav->Longitude_degree;
		latitude_degree = pNav->Latitude_degree;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
	}
	return 0;
}

double TranDegreeToMinute(double *a)
{
	double b=(*a)*60;
   return b;
}

void *Processing(void *ptr)
{
	double wholelongtitude;
	double wholelatitude;
	while (1)
	{
		memset(buff, 0, BUFFSIZE);
		pthread_mutex_lock(&gMutex);
		pthread_cond_wait(&cond, &gMutex);
		wholelongtitude = TranDegreeToMinute(&longtitude_degree);
		wholelatitude = TranDegreeToMinute(&latitude_degree);
		pthread_mutex_unlock(&gMutex);
		//处理数据
		if (
			wholelatitude>1560 && wholelatitude<2220
			&& wholelongtitude>6300 && wholelongtitude<7380
			)//设置经纬度范围
		{
			ProcessingAndSend(wholelongtitude, wholelatitude);
		}
	}
	return 0;
}

//接收来自MO的命令
int MOCallBack(void* mo_to_pl, size_t size, void* args)
{
	if (IsDebug())
	{
		NJUST_FROM_MO_COMMAND *pCmd = NULL;
		NJUST_MO_Decode_IP_Data_CMD(mo_to_pl, size, &pCmd);
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
			SetDebug();
		}
		else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RELEASE)
		{
			SetRelease();
		}
	}
	else
	{
		NJUST_FROM_MO_COMMAND *pCmd = NULL;
		NJUST_MO_Decode_IP_Data_CMD(mo_to_pl, size, &pCmd);
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
			SetDebug();
		}
		else if (pCmd->cmd == NJUST_MO_COMMAND_TYPE_MODULE_RELEASE)
		{
			SetRelease();
		}
	}
	return 0;
}

bool IsDebug()

{
	bool b;
	pthread_mutex_lock(&gMutex);
	b = gIsDebug;
	pthread_mutex_unlock(&gMutex);
	return b;
}

void SetDebug()

{
	pthread_mutex_lock(&gMutex);
	gIsDebug = true;
	pthread_mutex_unlock(&gMutex);
}


void SetRelease()

{
	pthread_mutex_lock(&gMutex);
	gIsDebug = false;
	pthread_mutex_unlock(&gMutex);
}

void  SysrtemInitial()
{
	m_flaglocation = 1;
	m_counterflag = 1;
	m_counter = 0;
	m_counterzero = 0;
	m_firstflag = 1;
	int i, j;
	for (i = 0; i<SIZESHUZU; i++)
	{
		templongtitude[i] = 0;
		templatitude[i] = 0;
	}

	for (i = 0; i<NOTESIZE; i++)
	{
		pButtonNote[i] = NULL;
	}
	for (i = 0; i<LINESIZE; i++)
	{
		pButtonLine[i] = NULL;
	}
	for (i = 0; i<MAX_NODE_NUM; i++)
	{
		for (j = 0; j<MAX_NODE_NUM; j++)
		{
			adjust[i][j] = INF;
		}
	}
	for (i = 0; i<MAX_NODE_NUM; i++)
	{
		PassNodeQueue[i] = 0;
	}
	for (i = 0; i<MAX_NODE_NUM; i++)
	{
		EscapePointQueue[i] = 0;
	}
	for (i = 0; i<MAX_NODE_NUM; i++)
	{
		lujing[i] = 0;
	}
	planningstate = 0;
	chongqiinitial = 1;
	ReadLineGPSflag = 0;
	ReadNodeGPSflag = 0;
	pInitialNodeQueue = 0;
	pludianori = 0;
	plujing = 0;
	pEscapePointQueue = 0;
	EscapePointQueueNum = 0;
	begain = 1;
	special = 1;
    frame = 0;
    cmdid = 0;
    m_order = 1;        //正向搜寻还是逆向搜寻	
    mocounter = 0;//显控发送频率控制
	startNavID = 0;
    endNavID = 1;
    countNavID = 0;
    isFirst = true;
	gIsDebug=true;
}
void ReadInitialInfor()
{
	                                                   if (IsDebug())         oflocal<<"Into readinitialinfoe!"<<endl;
	DATA      m_data1;
	NODE      m_node1[100];
	ROAD      m_road1[100];
	FILE *pfile = fopen("board.db", "rb");
	fread(&m_data1, sizeof(DATA), 1, pfile); //读总结构
	fread(m_node1, sizeof(NODE), m_data1.notecounter, pfile); //先读节点个数
	fread(m_road1, sizeof(ROAD), m_data1.linecounter, pfile); //读线路个数
	fclose(pfile);
	m_adjustx = m_data1.m_adjustx;
	m_adjusty = m_data1.m_adjusty;
	notecounter = m_data1.notecounter;
	linecounter = m_data1.linecounter;
	//读入节点的信息
	int i;
	for (i = 0; i<notecounter; i++)
	{
		pButtonNote[i] = new CButtonNote;
		pButtonNote[i]->idself = m_node1[i].idself;
		                                                                    if (IsDebug())        oflocal<<pButtonNote[i]->idself<<endl;
		pButtonNote[i]->neigh = m_node1[i].neigh;
		int j;
		for (j = 0; j<pButtonNote[i]->neigh; j++)
		{
			pButtonNote[i]->NeighNoteID[j] = m_node1[i].NeighNoteID[j];
			pButtonNote[i]->NeighLineID[j] = m_node1[i].NeighLineID[j];
		}
		pButtonNote[i]->HLD = m_node1[i].HLD;
		pButtonNote[i]->HLDkind = m_node1[i].HLDkind;
		pButtonNote[i]->lukou = m_node1[i].lukou;
		pButtonNote[i]->zebra = m_node1[i].zebra;
		pButtonNote[i]->gpsx = m_node1[i].gpsx;
		pButtonNote[i]->gpsy = m_node1[i].gpsy;
		pButtonNote[i]->earthx = m_node1[i].earthx;
		pButtonNote[i]->earthy = m_node1[i].earthy;
	}
	//读入道路的信息
	for (i = 0; i<linecounter; i++)
	{
																		 
		pButtonLine[i] = new CButtonLine;
		pButtonLine[i]->idself = m_road1[i].idself;
		                                                              if (IsDebug())       oflocal<<pButtonLine[i]->idself<<endl;
		pButtonLine[i]->idstart = m_road1[i].idstart;
		pButtonLine[i]->idend = m_road1[i].idend; //ID
		pButtonLine[i]->k = m_road1[i].k;           //直线参数
		pButtonLine[i]->b = m_road1[i].b;
		pButtonLine[i]->c = m_road1[i].c;
		pButtonLine[i]->roadkind = m_road1[i].roadkind; 
		pButtonLine[i]->wedth = m_road1[i].wedth;
		pButtonLine[i]->length = m_road1[i].length;
		pButtonLine[i]->maluyazi = m_road1[i].maluyazi;
		pButtonLine[i]->hyazi = m_road1[i].hyazi;
		pButtonLine[i]->hulan = m_road1[i].hulan;
		pButtonLine[i]->hhulan = m_road1[i].hhulan;
		pButtonLine[i]->xingdaoxiannum = m_road1[i].xingdaoxiannum;
		pButtonLine[i]->leftxingdaoxian = m_road1[i].leftxingdaoxian;
		pButtonLine[i]->middlexingdaoxian = m_road1[i].middlexingdaoxian;
		pButtonLine[i]->rightxingdaoxian = m_road1[i].rightxingdaoxian;
		pButtonLine[i]->chedaonum = m_road1[i].chedaonum;
		pButtonLine[i]->leftdaolubianjie = m_road1[i].leftdaolubianjie;
		pButtonLine[i]->rightdaolubianjie = m_road1[i].rightdaolubianjie;
		pButtonLine[i]->idealspeed = m_road1[i].idealspeed;
	}
}


void ReadInitialNodeQueue()
{
                if(IsDebug())        oflocal<<"Into ReadInitialNodeQueue!"<<endl;
	FILE *pf = fopen("InitialNodeQueue.db", "rb");
	fseek(pf, 0L, SEEK_END);
	InitialNodeQueueNum = ftell(pf) / sizeof(ROADNODE);
                if(IsDebug())        oflocal<<"InitialnodeQueueNum="<<InitialNodeQueueNum<<endl;
	fseek(pf, 0L, SEEK_SET);
	fread(InitialNodeQueue, sizeof(ROADNODE), InitialNodeQueueNum, pf);
    int i;
    for(i=0;i<InitialNodeQueueNum;i++)
    {
        if(IsDebug())          oflocal<<InitialNodeQueue[i].noderesult<<"   ";
    }
    if(IsDebug())         oflocal<<endl;
    fclose(pf);
}


void ReadNodeFileRestore()//程序启动和重启时读入路点文件，路点文件的经多的点在flagluidianori中标识，都放在了Record函数中
//flagludianori指示下一个要经过的路点
{
	vector<string>strnode;
	ifstream f_task;
	string line;
	f_task.open("task.txt");
	if (f_task.is_open())
	{
		//step1--------一次读入路点文件，存到string的vector容器中
		while (getline(f_task, line))
		{
			strnode.push_back(line);
		}
		f_task.close();
		//step2--------读取每一行，将每一行中的单词依次读入结构体RoadFileNode数组m_roadnode中
		vector<string>::iterator iter = strnode.begin();
		string temp;
		int n = 0;
		while (iter != strnode.end())
		{
			temp = *iter;
			istringstream stream(temp);
            //数据格式： 经度 维度 高 属性1 属性2
			stream >> m_roadnode[n].num >> m_roadnode[n].longtitude
				>> m_roadnode[n].latitude >> m_roadnode[n].hight
				>> m_roadnode[n].shuxing1 >> m_roadnode[n].shuxing2;//读取的经纬度与是以度为单位！
			iter++;
			n++;
		}
		ludiannum = n;
	}
}

int   FindgpsCur(double m_gpsxcur, double m_gpsycur)
{
	int i, x, y, a;
	double distance;
	int earthxcur1, earthycur1;
	blh2xy(m_gpsycur / 60, m_gpsxcur / 60, earthxcur1, earthycur1);
	int earthxtemp, earthytemp;
	blh2xy(m_gpsArry[0].gpsy / 60, m_gpsArry[0].gpsx / 60, earthxtemp, earthytemp);
	x = (earthxcur1 - earthxtemp) / 100;
	y = (earthycur1 - earthytemp) / 100;
	double min = x*x + y*y;
	a = 0;
	for (i = 0; i<m_arryNum; i++)
	{
		blh2xy(m_gpsArry[i].gpsy / 60, m_gpsArry[i].gpsx / 60, earthxtemp, earthytemp);
		x = (earthxcur1 - earthxtemp) / 100;
		y = (earthycur1 - earthytemp) / 100;
		distance = x*x + y*y;
		if ((min>distance) && (distance>0))//保护，防止对坏数据处理
		{
			min = distance;
			a = i;
		}
	}
	return a;
}

int locationGPS(double longtitude, double latitude)
{
	m_gpscur = FindgpsCur(longtitude, latitude);
	if (m_order == 1)//正序
	{
		if (m_gpscur >= 0 && m_gpscur<m_arryNum - STEP)//判断数组边界
			return (m_gpscur + STEP);
		else
			return (m_arryNum - 1);
	}
	if (m_order == 0)//逆序
	{
		if (m_gpscur >= STEP&&m_gpscur<m_arryNum)
			return (m_gpscur - STEP);
		else
			return 0;
	}
	return -1;
}

double NextNodeDistance(CButtonLine*  pline, double longtitude, double latitude)//返回值为米
{
	int m, n;
	int earthxtemp, earthytemp;
	blh2xy(latitude / 60, longtitude / 60, earthxtemp, earthytemp);
	m = pline->idstart - IDC_NODE;
	n = pline->idend - IDC_NODE;
	if ((m_lastnode - 1) == m)
	{
		int x = pButtonNote[n]->earthx;
		int y = pButtonNote[n]->earthy;
		int a = abs(earthxtemp - x) / 100;
		int b = abs(earthytemp - y) / 100;
		double c = sqrt(a*a + b*b);
		return c;
	}
	if ((m_lastnode - 1) == n)
	{
		int x = pButtonNote[m]->earthx;
		int y = pButtonNote[m]->earthy;
		int a = abs(earthxtemp - x) / 100;
		int b = abs(earthytemp - y) / 100;
		double c = sqrt(a*a + b*b);
		return c;
	}
	return -1;
}

void OnClose()
{
	// TODO: Add your message handler code here and/or call default
	for (int i = 0; i<notecounter; i++)
	{
		delete pButtonNote[i];
		pButtonNote[i] = NULL;
	}
	for (int j = 0; j<linecounter; j++)
	{
		delete pButtonLine[j];
		pButtonLine[j] = NULL;
	}
}

void StructTransformNote(CButtonNote* note, NJUST_MAP_INFO_NODE **node)
{
	(*node)->nodenum = note->idself - IDC_NODE + 1;
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

void StructTransformLine(CButtonLine* line, NJUST_MAP_INFO_ROAD **road)
{
	(*road)->roadnum = line->idself - IDC_LINE + 1;
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

void blh2xy(double x, double y, int &earthx, int &earthy)//纬度，经度，单位是度,单位是厘米
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

//判断是否在起点处启动map 且返回路径长度
int GetNearPassNode()
{
	int length;
	FILE *pf = fopen("AlreadyPassNodeQueue.db", "rb");
	if (pf == NULL)//不存在，返回0，表示初始在起点附近启动
	{
		return 0;
	}
	else//存在，表示在中途重启MAP
	{
		fseek(pf, 0L, SEEK_END);
		length = ftell(pf) / sizeof(RecordNode);
		fseek(pf, 0L, SEEK_SET);
		fclose(pf);
		return length;
	}
}

void ReadGPSSpecial(char * file)
{

	FILE *pf = fopen(file, "rb");
	fseek(pf, 0L, SEEK_END);
	m_arryNum = ftell(pf) / sizeof(GpsPoint);
	fseek(pf, 0L, SEEK_SET);
	fread(m_gpsArry, sizeof(GpsPoint), m_arryNum, pf);
	fclose(pf);
}

void ReadNodeGPS(int a, int b)//1开始，1开始
{
	//防止出现段错误，但是平时不加，来检查完整性
	if (a == b)//说明是遇到1-1-2-2-3-2-2-3-4的情况，定位到节点3的情况
	{
		ReadLineGPS(m_ncur + 1, m_nnext + 1);
		return;
	}
	char chara[10];
	char charb[10];
	mitoa(a, chara);
	mitoa(b, charb);
	std::string str1 = "+";
	str1.insert(0, chara);
	str1.insert(str1.size(), charb);
	str1.insert(str1.size(), ".db");
	char *p1 = (char *)str1.data();
	FILE *pf = fopen(p1, "rb");
															   if(IsDebug())          oflocal<<p1<<endl;
													// if(IsDebug())	     oflocal<<"into ReadNodeGPS"<<endl;
	if (pf == NULL)
	{
		std::string str2 = "+";
		str2.insert(0, charb);
		str2.insert(str2.size(), chara);
		str2.insert(str2.size(), ".db");
		char *p2 = (char *)str2.data();
		pf = fopen(p2, "rb");
		if (pf == NULL)
			return;
		m_order = 0;
	}
	else
	{
		m_order = 1;
	}
	fseek(pf, 0L, SEEK_END);
	m_arryNum = ftell(pf) / sizeof(GpsPoint);
	fseek(pf, 0L, SEEK_SET);
	fread(m_gpsArry, sizeof(GpsPoint), m_arryNum, pf);
	                                             if(IsDebug())       oflocal<<"read NodeGPS sucessful!"<<"    m_arrynum=="<<m_arryNum<<endl;
	fclose(pf);
}

//读取一段路上的GPS点
void ReadLineGPS(int a, int b)//1开始，1开始 
{
	if (a == b)
		return;

	char chara[10];
	char charb[10];
	mitoa(a, chara);
	mitoa(b, charb);
	std::string str1 = "-";
	str1.insert(0, chara);
	str1.insert(str1.size(), charb);
	str1.insert(str1.size(), ".db");
	char *p1 = (char *)str1.data();
													 if(IsDebug())        oflocal<<p1<<endl;
                                                    //  if(IsDebug())        oflocal<<"into ReadLineGPS"<<endl;
	FILE *pf = fopen(p1, "rb");
	if (pf == NULL)
	{
		std::string str2 = "-";
		str2.insert(0, charb);
		str2.insert(str2.size(), chara);
		str2.insert(str2.size(), ".db");
		char *p2 = (char *)str2.data();
		pf = fopen(p2, "rb");
		//下面的代码是防止加载了没有的序列后程序出现段错误，但平时不加	   
		if (pf == NULL)
			return;
		m_order = 0;
	}
	else
	{
		m_order = 1;
	}
	fseek(pf, 0L, SEEK_END);
	m_arryNum = ftell(pf) / sizeof(GpsPoint);
	fseek(pf, 0L, SEEK_SET);
	fread(m_gpsArry, sizeof(GpsPoint), m_arryNum, pf);
													    if(IsDebug())        oflocal<<"read LineGPS sucessful!"<<"    m_arrynum=="<<m_arryNum<<endl;
	fclose(pf);
}

void Record(int nodeth, int cur, int next, int a, int b, int flag, int ludian)
//0开始，表示刚经过的initialnodequeue中的点的序号，从0开始
//0开始，表示上一个或者刚要离开的节点
//0开始，表示下一个节点
//1开始，表示加载路段的起点
//1开始，表示加载路段的终点
//取值为0或者1，0表示记录的序列为道路的序列，即两端为节点
//1表示记录的序列为节点序列，即两端为道路。
//0开始，上一次经历过的原始路点的序号
{
	RecordNode  R_N;
	R_N.nodenum = nodeth;
	R_N.cur = cur;
	R_N.next = next;
	R_N.a = a;
	R_N.b = b;
	R_N.flag = flag;
	R_N.ludian = ludian;
	FILE *pf = fopen("AlreadyPassNodeQueue.db", "wb");
	fwrite(&R_N, sizeof(RecordNode), 1, pf);
	fclose(pf);

}

void ReadRecord(int &nodeth, int &cur, int &next, int &a, int & b, int &flag, int &ludian)
{
	RecordNode  R_N;
	FILE *pf = fopen("AlreadyPassNodeQueue.db", "rb");
	fread(&R_N, sizeof(RecordNode), 1, pf);
	nodeth = R_N.nodenum;
	cur = R_N.cur;
	next = R_N.next;
	a = R_N.a;
	b = R_N.b;
	flag = R_N.flag;
	ludian = R_N.ludian;
	fclose(pf);
}

void  ReadLinjie()
{

	FILE *pf = fopen("adjust.db", "rb");
	int i, j;
	for (i = 0; i<notecounter; i++)
	{
		for (j = 0; j<notecounter; j++)
		{
			fread(&adjust[i][j], sizeof(int), 1, pf);
		}
	}
	fclose(pf);
}

void WriteLinjie()
{
	FILE *pf = fopen("adjust.db", "wb");
	int i, j;
	for (i = 0; i<notecounter; i++)
	for (j = 0; j<notecounter; j++)
	{
		fwrite(&adjust[i][j], sizeof(int), 1, pf);
	}
	fclose(pf);
}

void   dijstra(int startPoint, int endPoint, int escapes[], int pathPlanning[], int& planninglength)
{
	int dis[MAX_NODE_NUM] = { 0 };
	int lastPoint[MAX_NODE_NUM] = { 0 };
	int vis[MAX_NODE_NUM] = { 0 };
	int i;
	int j;
	for (i = 0; i < notecounter; i++)
	{
		dis[i] = INF, lastPoint[i] = -1;
	}

	dis[endPoint] = 0;

	for (i = 0; i < notecounter; i++)
	{
		int k = -1, tmp = INF;
		for (j = 0; j < notecounter; j++)
		{
			if (escapes[j] == 0 && vis[j] == 0 && tmp > dis[j])
			{
				tmp = dis[k = j];
			}
		}
		if (k == -1 || k == startPoint)
		{
			break;
		}
		vis[k] = 1;
		for (j = 0; j < notecounter; j++)
		{
			if (vis[j] == 0 && escapes[j] == 0)
			{
				int cur = dis[k] + adjust[k][j];
				if (cur < dis[j])
				{
					dis[j] = cur;
					lastPoint[j] = k;
				}
			}
		}
	}
	while (lastPoint[startPoint] != endPoint)
	{
		pathPlanning[planninglength++] = lastPoint[startPoint] + 1;
		startPoint = lastPoint[startPoint];
	}
}

/*****************************************************************************************
* 函数功能描述：
* 求从startPoint到endPoint的满足一下条件的最短路径：
* 1、要求从startPoint出发，到达endPoint；
* 2、不能经过escapePoint[]数组中的点，该数组存放的不能经过的点的数目为escapePointNum；
* 3、必须经过passPoint[]数组中存放的点,该数组存放的要经过的点的数目为passPointNum；
* 4、对于passPoint[]数组中存放的点要依次顺序经过，不可跳跃；
* 5、把规划好的路径节点填入pathPlanningQueue[]数组中（包括起点和终点）；
* 6、并把规划好的路径的结点的个数（包括起点和终点）赋值给Queuelength；
****************************************************************************************/
void   PathPlaning(int startPoint, int endPoint, int escapePoint[], int escapePointNum, int passPoint[], int passPointNum,
	int pathPlanningQueue[], int &Queuelength)
{
	int pathPlanning[30];
	int planninglength;
	int escapes[MAX_NODE_NUM] = { 0 };
	int i, j;
	for (i = 0; i < escapePointNum; i++)
	{
		escapes[escapePoint[i] - 1] = 1;
	}
	for (i = 0; i < passPointNum; i++)
	{
		escapes[passPoint[i] - 1] = 1;
	}
	escapes[endPoint - 1] = 1;
	pathPlanning[(planninglength = 0)++] = startPoint;
	int lastPoint = startPoint;
	for (i = 0; i < passPointNum; i++)
	{
		escapes[passPoint[i] - 1] = 0;
		dijstra(lastPoint - 1, passPoint[i] - 1, escapes, pathPlanning, planninglength);
		pathPlanning[planninglength++] = lastPoint = passPoint[i];
	}
	escapes[endPoint - 1] = 0;
	dijstra(lastPoint - 1, endPoint - 1, escapes, pathPlanning, planninglength);
	pathPlanning[planninglength++] = endPoint;
											  for(int m=0;m<planninglength;m++)
											 {
									          	if(IsDebug())			 oflocal<<pathPlanning[m]<<"* ";
											 }
											 oflocal<<endl;
	pathPlanningQueue[(Queuelength = 0)++] = pathPlanning[0];
	for (i = 1; i<planninglength; i++)
	{
		for (j = 0; j<linecounter; j++)
		{
			if (
				(
				(pButtonLine[j]->idstart - IDC_NODE + 1) == pathPlanning[i - 1] &&
				(pButtonLine[j]->idend - IDC_NODE + 1) == pathPlanning[i]
				) ||
				(
				(pButtonLine[j]->idstart - IDC_NODE + 1) == pathPlanning[i] &&
				(pButtonLine[j]->idend - IDC_NODE + 1) == pathPlanning[i - 1]
				)
				)
			{
				pathPlanningQueue[Queuelength++] = pButtonLine[j]->idself - IDC_LINE + 1;
				break;
			}
		}
		pathPlanningQueue[Queuelength++] = pathPlanning[i];
											  for(int n=0;n<Queuelength;n++)
											  {
										        if(IsDebug())		  oflocal<<pathPlanningQueue[n]<<"+";
											  }
										        if(IsDebug())	      oflocal<<endl;
	}
}

void mitoa(int i, char* a)
{
		int power, j;
		j = i;
		for (power = 1; j >= 10; j /= 10)
			power *= 10;
		for (; power>0; power /= 10)
		{
			*a++ = '0' + i / power;
			i %= power;
		}
		*a = '\0';
}

int GetAnotherNode(int line, int onenode)
		//line从1开始，表示一条道路，onenode为该道路其中一个端的节点号，从1开始
{
		int a = pButtonLine[line - 1]->idstart - IDC_NODE + 1;//起点节点编号
		int b = pButtonLine[line - 1]->idend - IDC_NODE + 1;//终点节点编号
		if (onenode == a)
		{
			return b;
		}
		if (onenode == b)
		{
			return a;
		}
		return -1;
}

int PLCallBack(void* pl_to_map, size_t size, void* args)
{
		NJUST_PL_TO_MAP * plcome = (NJUST_PL_TO_MAP *)pl_to_map;
		NJUST_IP_TIME time;
		NJUST_PL_MAP_COMMAND_TYPE  command;
		time = plcome->synTime;
		command = plcome->cmd;
		if (command == NJUST_PL_MAP_COMMAND_TYPE_REPLAN_ASK)
		{
			chongguihuaflag = 1;//初始化微0，PL发过来的重规划标志
			planningstate = 1;
		}
		return 0;
}

int GetAnotherLine(int node, int line1, int line2)
{
		int a = pButtonNote[node - 1]->neigh;
		int i;
		for (i = 0; i<a; i++)
		{
			int c;
			c = pButtonNote[node - 1]->NeighLineID[i] - IDC_LINE + 1;
			if (c == line1 || c == line2)
			{
				continue;
			}
			else
			{
				return c;
			}

		}
		return -1;
}

void ProcessingAndSend(double wholelongtitude, double wholelatitude)
{
		if (m_counterflag == 1)
		{
			GetFirstFiveGps(wholelongtitude,wholelatitude);
		}
		else
		{
            //过滤掉 大的偏差值
			if (
				(
				abs(wholelongtitude - templongtitude[m_counter - 1])>1
				) ||
				(
				abs(wholelatitude - templatitude[m_counter - 1])>1
				)
				)
			{
				return;
			}
			GetMeanGps(wholelongtitude,wholelatitude,m_longtitudemeannew, m_latitudemeannew);
                                                      //      int lon=m_longtitudemeannew*10000;
													//		int lai=m_latitudemeannew*10000;
												//	 if(IsDebug())		oflocal<<"m_longtitudemeannew=="<<lon<<endl;
												//	 if(IsDebug())		oflocal<<"m_latitudemeannew  =="<<lai<<endl;
												//	 int i;
												//	 if(IsDebug())		oflocal<<"所有的值："<<endl;
													//		for(i=0;i<SIZESHUZU;i++)
													//		{
													//			lon=templongtitude[i]*10000;
													//			lai=templatitude[i]*10000;
												//	 if(IsDebug())			oflocal<<lon<<"   "<<lai<<endl;
													//		}
			if (chongqiinitial == 1)//在程序启动时执行，且只执行一次，
			{
				//读alreadypassnode文件，判断是在 起点启动程序 还是 中途重启了程序
				int length = GetNearPassNode();
				if (length == 0)//说明是起点附近处启动程序
                    ChongqiChengxuBegin();
				else//说明是在道路行驶过程途中重启MAP模块,此时不要考虑该条道路能不能走，只需要恢复到原来的状态即可。
					//因为也有可能是道路能走，但由于其他原因误掉电，若回复到原来的状态不能走，则会重新调用规划，
				ChongqiChengxuHalf();
				chongqiinitial = 0;
			}
			//正常通行情况下，不重启，不重定位，但要record*****************************************************************
			if (planningstate == 0)
			{
				int result = RoadNodeLocation(m_longtitudemeannew, m_latitudemeannew);//用的原始点的GPS，THRESHOLD=3400	         
				if ((result + 1) != m_lastnode)
					//离开了上个节点，要么在路网的线上，要么在路网的新的节点上
				{
					double distancetonode =
							Distance(m_longtitudemeannew, m_latitudemeannew, pButtonNote[lujing[plujing] - 1]->gpsx, pButtonNote[lujing[plujing] - 1]->gpsy);
					if ((distancetonode<THREODNODE) && (distancetonode>0))//在路网的新的节点上，THREODNODE=4500
					{
						if (plujing == lujingnum - 1)//说明在路点文件中的路口处，此时说明该段路走完，需要重新规划下一段路
						{				
   							m_lastnode = lujing[plujing];
						    m_lastline = lujing[plujing - 1];					
							pInitialNodeQueue = min(pInitialNodeQueue + 1, InitialNodeQueueNum - 1);//更新下一个路网中给出的路口		
							//以该点为起点，下一个网路中的路口点为终点，重新规划新的路径
							//但是要先判断一下下一个点在不在路网中
							PassNodeQueueNum = 0;
							PathPlaning(m_lastnode, InitialNodeQueue[pInitialNodeQueue].noderesult,
										EscapePointQueue, EscapePointQueueNum,
										PassNodeQueue, PassNodeQueueNum,
										lujing, lujingnum);//路径中包含了线的编号
							plujing = 0;
								//一下代码为在特殊路段加载特殊序列与通用序列								
							ReadNodeGPS(m_lastline, lujing[plujing + 1]);
                            //m_lastnode 实际编号从1开始 刚刚经过的节点 
							m_ncur = m_lastnode - 1;
							m_nnext = lujing[plujing + 2] - 1;	
							nodeth = pInitialNodeQueue - 1;
							m_cur = m_ncur;
							m_next = m_nnext;
							start = m_lastline;
							end = lujing[plujing + 1];
							flag = 1;
							ludianorith = pludianori - 1;
							if (m_lastline != lujing[1])//这个判断是为了试用于1-1-2-2-3-2-2-3-4的情况出现！此时并不记录3这个节点，并且上面加载node路径时，其实加载的是line2路径
							{
								Record(nodeth, m_cur, m_next, start, end, flag, ludianorith);//节点序列	
								plujing = min(plujing + 2, lujingnum - 1);
								ReadLineGPSflag = 1;
							}
						}
						else//说明在路点文件中给的相邻两个路口之间的规划路径中，但此时仍然在路网上。
								//还在上一次规划的路径中，未走完
						{
								//加载一下判断的原因是：当遇到转弯或者连续的弯道时，为了构建地图的方便，会吧两个点采集的很近，
								//以至于两个点之间的过度不需要经过道路，
								//这样加上下面的判断会在这种情况出现的情况下也能记录新的节点的信息m_lastnode是记录上一个经过的节点
							m_lastnode = lujing[plujing];
							m_lastline = lujing[plujing - 1];
							ReadNodeGPS(m_lastline, lujing[plujing + 1]);
                            
							m_ncur = m_lastnode - 1;
							m_nnext = lujing[plujing + 2] - 1;
							nodeth = pInitialNodeQueue - 1;
							m_cur = m_ncur;
							m_next = m_nnext;
							start = m_lastline;
							end = lujing[plujing + 1];
							flag = 1;
							ludianorith = pludianori - 1;
                            
							Record(nodeth, m_cur, m_next, start, end, flag, ludianorith);//节点序列							
							plujing = min(plujing + 2, lujingnum - 1);//指向下一个节点						
							ReadLineGPSflag = 1;
						}
					}
					else//在路网中，但是不在节点上，那就是在线上
					{
						if ((result + 1))//若RoadNodeLocation定位不为-1，说明在节点上，但是Distance大于阈值，仍然定位在线上，则会进入此情况，
								//若此时仍然发点，则会在send中的location函数中定诶到点，但状态没有更新，从而发送上一个节点的方向信息，所以此时返回，不做定位
								//等待Distance小于阈值，从而更改状态，发送正确的方向信息！
						{
							special = 0;//在send函数中的标注位，使得在这种情况下若定位到点仍然发送点的信息，
						}
						else
						{
							if (ReadLineGPSflag == 1)//说明可以加载
							{
								int curlocation = Location(m_longtitudemeannew, m_latitudemeannew);
								if (curlocation >= notecounter)//
								{
									int linenum = curlocation - notecounter + 1;//linenum表示第几条线，从1开始
									if (linenum == lujing[plujing - 1])//这个判断是防止进入节点后由于GPS漂移又进入道路误加载！,上面注释两种方法均可。
									{							
									    ReadLineGPS(m_ncur + 1, m_nnext + 1);	
										ReadLineGPSflag = 0;
										ReadNodeGPSflag = 1;
									}
								}
							}
						}
					}
				}
				int igpscur = locationGPS(m_longtitudemeannew, m_latitudemeannew);
				int curlocation = Location(m_longtitudemeannew, m_latitudemeannew);//此时定位要么在节点，要么在道路，location已经被调用
				Send(m_longtitudemeannew, m_latitudemeannew, curlocation, igpscur);		
			}//正常情况下
			m_counter++;
			m_counterzero = 0;
		}	//else 15个数组之外
}//processingAndSend


int NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[])
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

double getRotateAngle(double x1, double y1, double x2, double y2)//以度为单位
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


void Send(double longtitude, double latitude, int curlocation, int igpscur)
{
		frame++;
		if (curlocation >= notecounter)
		{
			//定位到线
			Send_TO_PL_MO_line(longtitude,latitude,curlocation,igpscur);
		}
		else
		{
			Send_TO_PL_MO_Node(longtitude, latitude, curlocation, igpscur);
		}
}


double    Distance(double longtitude1, double latitude1, double longtitude2, double latitude2)//返回值为米
{
	int earthxcur, earthycur;
	int earthxtemp, earthytemp;
	blh2xy(latitude1 / 60, longtitude1 / 60, earthxcur, earthycur);
	blh2xy(latitude2 / 60, longtitude2 / 60, earthxtemp, earthytemp);
	double x = abs(earthxcur - earthxtemp) / 100;
	double y = abs(earthycur - earthytemp) / 100;
	double distance = sqrt(x*x + y*y);
	return distance;
}

//GPS经纬度转化成大地坐标的相对坐标，以校准点为原点,单位是cm
void  GPSToOffset(double longtitude, double latitude, CPoint &point)
{
	int earthx1, earthy1;
	blh2xy(latitude / 60, longtitude / 60, earthx1, earthy1);
	point.x = (earthx1 - m_adjustx);
	point.y = (earthy1 - m_adjusty);
}
void PointToPointDistance(CPoint point1,CPoint point2,double &distance)//参数单位为  cm，返回值为  m
{
	double x=(point1.x-point2.x)/100;
	double y=(point1.y-point2.y)/100;
	distance=sqrt(x*x+y*y);
}

int FindAjustNoteOffset(CPoint point) //参数为 cm
{

	int i;
	int n=-1;
	CPoint point2;
	double distance, min;
	point2.x = pButtonNote[0]->earthx - m_adjustx;
	point2.y = pButtonNote[0]->earthy - m_adjusty;
	PointToPointDistance(point, point2, distance);
	min = distance;
	for (i = 0; i<notecounter; i++)
	{
		point2.x = pButtonNote[i]->earthx - m_adjustx;
		point2.y = pButtonNote[i]->earthy - m_adjusty;
		PointToPointDistance(point, point2, distance);
		if ((distance <= min) && (distance>0))
		{
			min = distance;
			n = i;
		}
	}
	return n;
}

int Location(double longtitude, double latitude)
{
	CPoint point;
	CPoint point2;
	GPSToOffset(longtitude, latitude, point);
	int i;
	//先确定该店距离哪个节点最近，比较距离的平方
	if (m_flaglocation == 1)
	{
		m_ncur = FindAjustNoteOffset(point);
		m_flaglocation = 0;
	}
	//以上程序只执行一次，找到距离该点最近的节点,且一定能找到，
	point2.x = pButtonNote[m_ncur]->earthx - m_adjustx;
	point2.y = pButtonNote[m_ncur]->earthy - m_adjusty;
	double xx = abs(point.x - point2.x) / 100;
	double yy = abs(point.y - point2.y) / 100;
	if (sqrt(xx*xx + yy*yy)<(THRESHOLD + 4))
	{
		m_locationresultlineornote = m_ncur;
		return m_locationresultlineornote;
	}
		else
		{	//第二步，确定点在不在上面if中得到的n=i的那个节点相邻的节点中

			for (i = 0; i<pButtonNote[m_ncur]->neigh; i++)
			{
				int id = pButtonNote[m_ncur]->NeighNoteID[i];
				point2.x = pButtonNote[id - IDC_NODE]->earthx - m_adjustx;
				point2.y = pButtonNote[id - IDC_NODE]->earthy - m_adjusty;
				double x1 = abs(point.x - point2.x) / 100;
				double y1 = abs(point.y - point2.y) / 100;
				if (sqrt(x1*x1 + y1*y1)<(THRESHOLD + 4))//在圆内
				{
					m_ncur = id - IDC_NODE;
					m_locationresultlineornote = m_ncur;
					return m_locationresultlineornote;
				}
			}
			//第三步，若没找到，找他在哪条直线上
			double xbegain = pButtonNote[m_ncur]->earthx - m_adjustx;
			double ybegain = pButtonNote[m_ncur]->earthy - m_adjusty;
			int min = 1000000000;
			int idlinetemp = pButtonNote[m_ncur]->NeighLineID[0] - IDC_LINE;
			CPoint pointcur;
			GPSToOffset(longtitude, latitude, pointcur);
			for (i = 0; i<pButtonNote[m_ncur]->neigh; i++)
			{
				int idnote = pButtonNote[m_ncur]->NeighNoteID[i];
				double xend = pButtonNote[idnote - IDC_NODE]->earthx - m_adjustx;
				double yend = pButtonNote[idnote - IDC_NODE]->earthy - m_adjusty;
				if
				(
				  (
					(
					  (pointcur.x>xbegain&&pointcur.x<xend) ||
					  (pointcur.x>xend&&pointcur.x<xbegain)
					) &&
					(
					  abs(xbegain - xend)>abs(ybegain - yend)
					)
				  ) ||
				  (
					(
					  (pointcur.y>ybegain&&pointcur.y<yend) ||
					  (pointcur.y>yend&&pointcur.y<ybegain)
					) &&
					(
					  abs(ybegain - yend)>abs(xbegain - xend)
					)
				  )
				)
				{
					int idline = pButtonNote[m_ncur]->NeighLineID[i];
					double k = pButtonLine[idline - IDC_LINE]->k;
					double b = pButtonLine[idline - IDC_LINE]->b;
					double c = pButtonLine[idline - IDC_LINE]->c;
					double temp = abs((k*point.x + b*point.y + c)) / sqrt(b*b + k*k);
					if ((int)temp<min)
					{
						min = (int)temp;
						idlinetemp = idline - IDC_LINE;
					}
				}
			}
			m_locationresultlineornote = idlinetemp + notecounter;
			return m_locationresultlineornote;
		}
	}

double  DistanceToNode(double longtitude1, double latitude1, int pInitialNodeQueue)//分，分，度，度
{
	int earthxcur, earthycur;
	int earthxtemp, earthytemp;
	blh2xy(latitude1 / 60, longtitude1 / 60, earthxcur, earthycur);
	blh2xy(InitialNodeQueue[pInitialNodeQueue].latitude, InitialNodeQueue[pInitialNodeQueue].longtitude, earthxtemp, earthytemp);
	double x = abs(earthxcur - earthxtemp) / 100;
	double y = abs(earthycur - earthytemp) / 100;
	double distance = sqrt(x*x + y*y);
	return distance;
}


double  DistanceToOri(double longtitude1, double latitude1, int pludianori)
{
	int earthxcur, earthycur;
	int earthxtemp, earthytemp;
	blh2xy(latitude1 / 60, longtitude1 / 60, earthxcur, earthycur);
	blh2xy(m_roadnode[pludianori].latitude, m_roadnode[pludianori].longtitude, earthxtemp, earthytemp);
	double x = abs(earthxcur - earthxtemp) / 100;
	double y = abs(earthycur - earthytemp) / 100;
	double distance = sqrt(x*x + y*y);
	return distance;
}

int RoadNodeLocation(double longtitude, double latitude)//以分为单位，返回值从0开始
{
	CPoint point;
	CPoint point2;
	GPSToOffset(longtitude, latitude, point);
	//先确定该点距离哪个节点最近，比较距离的平方
	int t = FindAjustNoteOffset(point);
	point2.x = pButtonNote[t]->earthx - m_adjustx;
	point2.y = pButtonNote[t]->earthy - m_adjusty;

	double xx = abs(point.x - point2.x) / 100;
	double yy = abs(point.y - point2.y) / 100;
	if (sqrt(xx*xx + yy*yy)<THRESHOLD)
	{
		m_locationresultlineornote = t;
		return m_locationresultlineornote;
	}
	else
	{
		return -1;//返回-1说明并不在任何节点附近
	}	
}

void GetTurnChange(int m_lastline, int m_lastnode, int m_nextline, Turn & m_turn)//第一个参数是当前点，第二个参数是方向结构体
{
	int node2 = GetAnotherNode(m_lastline, m_lastnode);
	int node3 = GetAnotherNode(m_nextline, m_lastnode);
	if (node2 == node3)
	{
		m_turn.turn = 0;//未
		m_turn.turndegree = 0;
		return;
	}
	//STEP2 ...算出该线的方向向量int a,b   (a,b)(经度，纬度)
	double x1 = pButtonNote[m_lastnode - 1]->gpsx - pButtonNote[node2 - 1]->gpsx;
	double y1 = pButtonNote[m_lastnode - 1]->gpsy - pButtonNote[node2 - 1]->gpsy;
	//STEP4 ...算出该线的方向向量int c,d   (c,d)
	double x2 = pButtonNote[node3 - 1]->gpsx - pButtonNote[m_lastnode - 1]->gpsx;
	double y2 = pButtonNote[node3 - 1]->gpsy - pButtonNote[m_lastnode - 1]->gpsy;
	//STEP5 ...求出从第一个方向向量到第二个方向向量逆时针旋转的夹角
	double degree = getRotateAngle(x1 / 100, y1 / 100, x2 / 100, y2 / 100);
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
	return;
}


void  CreatDebug()
{
	std::string str1 = "wlx--oflocal--";
	NJUST_IP_TIME   syntime;
	char timeget[20];
	syntime = NJUST_IP_get_time();
	NJUST_IP_get_timeStr(syntime, timeget);
	str1.insert(str1.size(), timeget);
	str1.insert(str1.size(), ".txt");
	char *p1 = (char *)str1.data();
	oflocal.open(p1);
}

void GetFirstFiveGps(double wholelongtitude,double wholelatitude)
{
	if (m_counter<SIZESHUZU - 1)
	{
		templongtitude[m_counter] = wholelongtitude;
		templatitude[m_counter] = wholelatitude;
		m_counter++;
		return;
	}
	else if (m_counter == SIZESHUZU - 1)
	{
		templongtitude[m_counter] = wholelongtitude;
		templatitude[m_counter] = wholelatitude;
		m_counter++;
		m_counterflag = 0;
		return;
	}
}

void GetMeanGps(double wholelongtitude,double wholelatitude,double &longtitude, double &latitude)
{
	m_counter %= SIZESHUZU;
	templongtitude[m_counter] = wholelongtitude;
	templatitude[m_counter] = wholelatitude;
	int i;
	for (i = 0; i<SIZESHUZU; i++)
	{
		if (templongtitude[i] != 0) //可能会有0值？
		{
			m_counterzero++;
		}
	}
	double tempalllatitude = 0;
	double tempalllongtitude = 0;
	for (i = 0; i<SIZESHUZU; i++)
	{
		tempalllatitude += templatitude[i];
		tempalllongtitude += templongtitude[i];
	}
	longtitude = (tempalllongtitude) / m_counterzero;
	latitude = (tempalllatitude) / m_counterzero;
}

void FindStartEnd(int &sstart, int &eend)
{
	int find=-1;
	double min = 1000000000;
	for (int i = 0; i<InitialNodeQueueNum; i++)
	{
		double dis = DistanceToNode(m_longtitudemeannew, m_latitudemeannew, i);
		//从initialnodequeue中查找！！
		if ((dis<min) && (dis>0))
		{
			min = dis;
			find = i;
		}
	}
	if (InitialNodeQueue[find].noderesult == 1) //在起点附近
	{
		sstart = 1;
		eend = 2;
		pInitialNodeQueue = find;
	}
	else
	{
		int curlocation;
		curlocation = Location(m_longtitudemeannew, m_latitudemeannew);//此时定位要么在节点，要么在道路，
		if (curlocation<notecounter)//定位到点
		{
											if(IsDebug())          oflocal<<"local  in  NODE!!!"<<endl;
										    if(IsDebug())          oflocal<<"find="<<find<<endl;
			sstart = InitialNodeQueue[find].noderesult;
			eend = InitialNodeQueue[find + 1].noderesult;
											 if(IsDebug())          oflocal<<"sstrat="<<sstart<<"   "<<"eend="<<eend<<endl;
			pInitialNodeQueue = find + 1;
		}
		else//定位到线
		{
			                                if(IsDebug())          oflocal<<"local  in  line!!!"<<endl;
			int templine = curlocation - notecounter + 1;
			int anothernode = GetAnotherNode(templine, InitialNodeQueue[find].noderesult);
			sstart = InitialNodeQueue[find].noderesult;
			eend = anothernode;
			for (int i = 0; i<InitialNodeQueueNum; i++)
			{
				//注意当sstart与eend其中有一个不是initialnodequeue中的点时，下面的判断 会出错！！！会出现错误！！！！！！
				//所以要在一个道路两端都在路点的线开始
				if ((InitialNodeQueue[i].noderesult == sstart) && (InitialNodeQueue[i + 1].noderesult == eend))
				{
					pInitialNodeQueue = i + 1;
					break;
				}
			}
		}
	}
}

void  ChongqiChengxuBegin()
{
	//下面这段代码是为了从道路的任何位置导引时都能开始
	//首先找到最近的路点中给出的路口
												if(IsDebug())          oflocal<<                  "******************起点附近处启动程序*************************************"<<endl;
	int sstart, eend;
	FindStartEnd(sstart, eend);
											  if(IsDebug())    oflocal<<"起点  终点==="<<sstart<<"   "<<eend <<endl;
	//规划路径
	PassNodeQueueNum = 0;
	PathPlaning(sstart, eend,
		EscapePointQueue, EscapePointQueueNum,
		PassNodeQueue, PassNodeQueueNum,
		lujing, lujingnum);//路径中包含了线的编号
		
												 if(IsDebug())       oflocal<<"after pathplanning!"<<endl;
				                                      if(IsDebug())       oflocal<<"lujingnum="<<lujingnum<<endl;
													  int i;
			                                            	for(i=0;i<lujingnum;i++)
			                                                {
				                                       if(IsDebug())    	oflocal<<lujing[i]<<"  ";
				                                            }
				                                       if(IsDebug())      oflocal<<endl;
	plujing = min(plujing + 2, lujingnum - 1);	//指向下一个要通过的规划路径数组lujing[]中的节点    
	//默认初始位置定位到节点，要求采集地图时在起始位置要采集到
	m_ncur = lujing[0] - 1;
	m_nnext = lujing[plujing] - 1;
	m_lastnode = m_ncur + 1;
	m_lastline = lujing[plujing - 1];
	ReadLineGPS(m_lastnode, lujing[plujing]);
	nodeth = pInitialNodeQueue - 1;
	m_cur = m_ncur;
	m_next = m_nnext;
	start = m_ncur + 1;
	end = m_nnext + 1;
	flag = 0;
	ludianorith = 0;
	Record(nodeth, m_cur, m_next, start, end, flag, ludianorith);//直线序列	                      
	int curlocation;
	curlocation = Location(m_longtitudemeannew, m_latitudemeannew);//此时定位要么在节点，要么在道路，
	if (curlocation<notecounter)//定位到点
	{
		ReadLineGPSflag = 1;
		ReadNodeGPSflag = 0;//节点和道路处加载的是不同的道路
	}
}

void ChongqiChengxuHalf()
{
															 	if(IsDebug())        oflocal<<"在中途重启了模块！****************************************************************"<<endl;
	//读文件,nodeth上一次经历过的ROADNODE结构体中InitialNodeQueue[MAX_NODE_NUM]中的点的序号;
	//m_cur为规划路径中刚离开或者正要离开的节点，m_next为规划路径中下一个要到达的节点，		//start为上一次加载路口过度路径的起点，b为上一次加载路口过度路径的终点，flag为表示加载的路径为道路路径（0）还是节点的过渡路径（1），
	//flag现在没用，因为加载的为节点的过渡路径，道路不加载。始终为1.
	//上一个经过的原始路点的编号
	ReadRecord(nodeth, m_cur, m_next, start, end, flag, ludianorith);
																    if (IsDebug())       oflocal << "ReadRecord(" << InitialNodeQueue[nodeth].noderesult<< ",当前点或上一点" << m_cur + 1 << ",下一点" << m_next + 1 << ",LINE" << start << ",LINE" << end << "," << flag << "上一个经过的原始路点的序号" << ludianorith+1<<")" << endl;
	pInitialNodeQueue = min(nodeth + 1, InitialNodeQueueNum - 1);
																	   if(IsDebug())         oflocal<<"下一个要经过的节点"<<InitialNodeQueue[pInitialNodeQueue].noderesult<<endl;
	//重新规划
	PassNodeQueueNum = 0;
	PathPlaning(m_cur + 1, InitialNodeQueue[pInitialNodeQueue].noderesult,
		EscapePointQueue, EscapePointQueueNum,
		PassNodeQueue, PassNodeQueueNum,
		lujing, lujingnum);//路径中包含了线的编号
																	    if(IsDebug())       oflocal<<"after pathplanning!"<<endl;
				                                      if(IsDebug())       oflocal<<"lujingnum="<<lujingnum<<endl;
													  int i;
			                                            	for(i=0;i<lujingnum;i++)
			                                                {
				                                       if(IsDebug())    	oflocal<<lujing[i]<<"  ";
				                                            }
				                                       if(IsDebug())      oflocal<<endl;
	plujing = 0;
	plujing = min(plujing + 2, lujingnum - 1);
	m_lastnode = m_cur + 1;
	m_lastline = start;
	//重启默认是在路网中，否则不知道该怎么办
	int curlocation;
	curlocation = Location(m_longtitudemeannew, m_latitudemeannew);//此时定位要么在节点，要么在道路，
	if (curlocation<notecounter)//定位到点
	{
		ReadNodeGPS(m_lastline, lujing[plujing - 1]);
		ReadLineGPSflag = 1;
	}
	else//定位到线
	{
		ReadLineGPS(m_lastnode, lujing[plujing]);
		ReadLineGPSflag = 0;
	}
	m_ncur = m_lastnode - 1;
	m_nnext = lujing[plujing] - 1;
}

void Send_TO_PL_MO_line(double longtitude, double latitude, int curlocation, int igpscur)
{
	int lenline=0;
	int i;
	int temp = curlocation - notecounter;
	NJUST_MAP_INFO_ROAD   road;
	NJUST_MAP_INFO_ROAD  *proad = &road;
	StructTransformLine(pButtonLine[temp], &proad);
	int dis = NextNodeDistance(pButtonLine[temp], longtitude, latitude);
	//第一个赋值*************************************************************************
	if (m_order == 1)//正序
	{
		lenline = (20>(m_arryNum - igpscur) ? (m_arryNum - igpscur) : 20);
		memset(road.nextGPSPointQueue, 0, 20 * sizeof(GpsPoint));
		for (i = 0; i<lenline; i++)
		{
			road.nextGPSPointQueue[i].longtitude = m_gpsArry[igpscur + i].gpsx;
			road.nextGPSPointQueue[i].latitude = m_gpsArry[igpscur + i].gpsy;
		}
	}
	if (m_order == 0)//逆序
	{
		lenline = (20>(igpscur) ? (igpscur) : 20);
		memset(road.nextGPSPointQueue, 0, 20 * sizeof(GpsPoint));
		for (i = 0; i<lenline; i++)
		{
			road.nextGPSPointQueue[i].longtitude = m_gpsArry[igpscur - i].gpsx;
			road.nextGPSPointQueue[i].latitude = m_gpsArry[igpscur - i].gpsy;
		}
	}
	road.GPSPointQueuelength = lenline;
	road.distToNextNodeM = dis;
	road.synTime = NJUST_IP_get_time();
	road.FrameID = frame;
	NJUST_MAP_Encode_IP_Data(&road, 0, buff);
	NJUST_IP_get_timeStr(road.synTime, Timeget1);
	if (lenline>3)
	 NJUST_IP_tcp_send_to("PL", buff, 1024);
	if (mocounter % 6 == 0)
	{
		NJUST_TO_MO_WORKSTAT moState;
		NJUST_IP_TIME time2;
		time2 = NJUST_IP_get_time();
		moState.moduleID = 2;
		moState.myselfTimeOutMS = 2000;
		moState.stat = NJUST_MO_WORK_STAT_VAILD;
		moState.PELR = (endNavID - startNavID + 1 - countNavID) * 1000 / (endNavID - startNavID + 1);
		moState.timeConsumingMS = (int)NJUST_IP_get_time_GAP_ms(iptimemc1, time2);
		moState.errCode = NJUST_MO_ERRCODE_NOERR;
		sprintf(moState.pErrMsg, "%s", "/0");
		int nByte = 0;
		void *pStat = NULL;
		pStat = NJUST_MO_Encode_STA_IP_Data(&moState, &nByte);
		NJUST_IP_get_timeStr(time2, Timeget2);
	   NJUST_IP_udp_send_to("MO", pStat, nByte);
		usleep(1000 * 10);//10ms
		NJUST_IP_udp_send_to("MO", buff, 1024);
	}
	mocounter++;
}

void Send_TO_PL_MO_Node(double longtitude, double latitude, int curlocation, int igpscur)
{
	//定位到点
	int lennode=0;
	int i;
	NJUST_MAP_INFO_NODE  node;
	NJUST_MAP_INFO_NODE  *pnode = &node;
	StructTransformNote(pButtonNote[curlocation], &pnode);
	if (special == 0)
		node.nodepassType = NJUST_MAP_NODE_PASS_TYPE_NONE;
	else
	{
		GetDirection(node);
	}
	special = 1;
	//第一个赋值*************************************************************************
	if (m_order == 1)//正序
	{
		lennode = (20>(m_arryNum - igpscur) ? (m_arryNum - igpscur) : 20);
		memset(node.nextGPSPointQueue, 0, 20 * sizeof(GpsPoint));
		for (i = 0; i<lennode; i++)
		{
			node.nextGPSPointQueue[i].longtitude = m_gpsArry[igpscur + i].gpsx;
			node.nextGPSPointQueue[i].latitude = m_gpsArry[igpscur + i].gpsy;
		}
	}
	if (m_order == 0)//逆序
	{
		lennode = (20>(igpscur) ? (igpscur) : 20);
		memset(node.nextGPSPointQueue, 0, 20 * sizeof(GpsPoint));
		for (i = 0; i<lennode; i++)
		{
			node.nextGPSPointQueue[i].longtitude = m_gpsArry[igpscur - i].gpsx;
			node.nextGPSPointQueue[i].latitude = m_gpsArry[igpscur - i].gpsy;
		}
	}
	node.GPSPointQueuelength = lennode;
	node.synTime = NJUST_IP_get_time();
	node.FrameID = frame;
	//发送PL,MO
	NJUST_MAP_Encode_IP_Data(&node, 1, buff);
	NJUST_IP_get_timeStr(node.synTime, Timeget1);
	if (lennode>3)
	 NJUST_IP_tcp_send_to("PL", buff, 1024);
	if (mocounter % 10 == 0)
	{
		NJUST_TO_MO_WORKSTAT moState;
		NJUST_IP_TIME time2;
		time2 = NJUST_IP_get_time();
		moState.moduleID = 2;
		moState.myselfTimeOutMS = 2000;
		moState.stat = NJUST_MO_WORK_STAT_VAILD;
		moState.PELR = (endNavID - startNavID + 1 - countNavID) * 1000 / (endNavID - startNavID + 1);
		moState.timeConsumingMS = (int)NJUST_IP_get_time_GAP_ms(iptimemc1, time2);
		moState.errCode = NJUST_MO_ERRCODE_NOERR;
		sprintf(moState.pErrMsg, "%s", "/0");
		int nByte = 0;
		void *pStat = NULL;
		pStat = NJUST_MO_Encode_STA_IP_Data(&moState, &nByte);
		NJUST_IP_get_timeStr(time2, Timeget2);
		 NJUST_IP_udp_send_to("MO", pStat, nByte);
		usleep(1000 * 10);//10ms
		NJUST_IP_udp_send_to("MO", buff, 1024);
	}
	mocounter++;
}

void GetDirection(NJUST_MAP_INFO_NODE &node)
{
	//报出方向
	Turn m_turn;
	GetTurnChange(m_lastline, m_lastnode, lujing[plujing - 1], m_turn);
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