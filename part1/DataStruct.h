#ifndef _DATASTRUCT_H_
#define _DATASTRUCT_H_
//----------------------------------------------------------------------------
typedef struct
{
	//�ڵ��Ӧ��ʵ��·�ڵ���Ϣ

	int     neigh;//��֧��
	int     HLD;//���̵�λ��
	int     HLDkind;//���̵�����
	int     lukou;//·��
	int     zebra;//������
	double  gpsx;//gpsx��Ӧ��longtitude
	double  gpsy;//gpsy��Ӧ��latitude
	int     earthx;
	int     earthy;//�������

	//�ڵ�����Ļ�ϵ�λ����Ϣ
	 //����ʱ��֪������ʱ����ֵ

	int    idself;
	 //����ʱ��֪������ʱ����ֵ
	int     NeighNoteID[4];
	int     NeighLineID[4];


}CButtonNote;
//----------------------------------------------------------------------------
typedef struct  
{
		//����Ļ����Ϣ
        //����ʱ����֪������ʱ����ֵ

	int       idself;
    //����ʱ��֪������ʱ����ֵ
	int       idstart;
	int       idend;
	//ֱ��б�ʷ���,�Ի�������Ϊ����ϵ
	//kx+by+c=0
	double        k;
	double        b;
	double        c;

     //ʵ�ʵ�·��Ϣ   
	int     roadkind;//��·����
	float   wedth;//��·���,��λ����
	float   length;//��·����

	int     maluyazi;//��·����
	float   hyazi;//��·���Ӹ߶ȣ���λ����
	int     hulan;//����
	float   hhulan;//�����߶�					
    int     xingdaoxiannum; //�е�����Ŀ
    int     leftxingdaoxian;//���е���
	int     middlexingdaoxian;//�м��е���
	int     rightxingdaoxian;//���е���

	int     chedaonum;//������Ŀ
    int     leftdaolubianjie;//���·�߽�
	int     rightdaolubianjie;//�ҵ�·�߽�
	int     idealspeed;   //�����ٶ�


}CButtonLine;
//----------------------------------------------------------------------------
typedef struct 
{
	double x;
	double y;
}CPoint;


//---------------------------------------------------����MC��gps��Ϣ�ṹ��
typedef struct gpsinfor
{
	int FrameID;
    double longtitude_degree;
    double latitude_degree;

} GPSInfor;
//----------------------------------------------------------------------------
//GPS�������ļ���ŵĵ�Ľṹ��
typedef struct 
{
	double gpsx;
	double gpsy;
}GpsPoint;

//�߳�ͬ���ṹ��
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
    int     neigh;//��֧��
	int     NeighNoteID[4]; //Ĭ��һ��·�������ĸ���·����  ����·�����ӵ�·��ID
	int     NeighLineID[4]; //����·�����ӵĵ�·ID  ����Ϊneigh
	int     HLD;//���̵�λ��
	int     HLDkind;//���̵�����
	int     lukou;//·��
	int     zebra;//������
	double  gpsx;//gpsx��Ӧ��longtitude
	double  gpsy;//gpsy��Ӧ��latitude
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
	int     roadkind;//��·����
	float   wedth;//��·���,��λ����
	float   length;//��·����
	int     maluyazi;//��·����
	float   hyazi;//��·���Ӹ߶ȣ���λ����
	int     hulan;//����
	float   hhulan;//�����߶�					
    int     xingdaoxiannum; //�е�����Ŀ
    int     leftxingdaoxian;//���е���
	int     middlexingdaoxian;//�м��е���
	int     rightxingdaoxian;//���е���
	int     chedaonum;//������Ŀ
    int     leftdaolubianjie;//���·�߽�
	int     rightdaolubianjie;//�ҵ�·�߽�
	int     idealspeed;   //�����ٶ�
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
	double  longtitude;//��Ϊ��λ
	double  latitude;
	double  hight;
	int     shuxing1;
	int     shuxing2;
}RoadFileNode;


enum PROPRITY   //·������
{
	START = 0x00, //���
	STRAIGHT,     //ֱ��
	LEFT,         //���
	RIGHT,        //�ҹ�
	Uturn,        //Uturn
	TRAFIC_SIGN,   //��ͨ��־
	SPECIAL_TASK,  //��������
	END           //��β
};


typedef struct road_node
{
    int       num;
	double    longtitude;
	double    latitude;               //�Զ�Ϊ��λ
	int       noderesult;            //��·�����������λ���ĵ�,����·���������-1
	int       shuxing1;
	int       shuxing2;
	int       duiyingludianbianhao;  
}ROADNODE;




typedef struct turn
{
	int turn;//0ֱ�У�1��գ�2�ҹգ�3��Uturn��4��Uturn,5δ֪
	double turndegree;
}Turn;
//----------------------------------------------------------------------------
#endif