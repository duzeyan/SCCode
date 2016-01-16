#include "NJUST_MAP_proc.h"
#include "stdlib.h"
#include "memory.h"

////////////////////////////////////////////////////////////////////////////////////////////

//�ڴ�
static NJUST_MAP_INFO_ROAD       gNJUST_Map_Road;
static NJUST_MAP_INFO_NODE       gNJUST_Map_Node;
static NJUST_MAP_INFO_DIRECTION  gNJUST_Map_Direction;

////////////////////////////////////////////////////////////////////////////////////////////
int  NJUST_MAP_Decode_IP_Data( const void* pIPData, const int nBytes,
							 NJUST_MAP_INFO_ROAD  **pRoad, //�����ǵ�·����ʱ,ֵΪNULL
							 NJUST_MAP_INFO_NODE  **pNode,//������·����Ϣʱ,ֵΪNULL
							 NJUST_MAP_INFO_DIRECTION      **pProprity//�����Ƿ�����ʱ��ֵΪNULL
							 )
{
	char date;
	char *pdata=(char *)pIPData;
    memcpy(&date,pdata,1);


	switch (date)
	{
	case '0'://road��Ϣ
		{
			memcpy(&gNJUST_Map_Road,&pdata[1],sizeof(NJUST_MAP_INFO_ROAD));
			*pRoad=&gNJUST_Map_Road;
			*pNode=NULL;
			*pProprity=NULL;
			break;
		}
	case '1'://node��Ϣ
		{
			memcpy(&gNJUST_Map_Node,&pdata[1],sizeof(NJUST_MAP_INFO_NODE));
			*pNode = &gNJUST_Map_Node;
			*pRoad=NULL;
			*pProprity=NULL;
			break;
		}
	case '2'://������
		{
			memcpy(&gNJUST_Map_Direction,&pdata[1],sizeof(NJUST_MAP_INFO_DIRECTION));
			*pProprity=&gNJUST_Map_Direction;
			*pNode = NULL;
			*pRoad=NULL;
			break;
		}
	}

	return 0;

}

int NJUST_MAP_Read_offline_data( const char* pFileName, //����ȡ�������ļ���
                                 NJUST_MAP_INFO_ROAD  **pRoad, //����������·����ʱ,ֵΪNULL
			                     NJUST_MAP_INFO_NODE  **pNode, //��������·����Ϣʱ,ֵΪNULL
								 NJUST_MAP_INFO_DIRECTION      **pProprity//�����Ƿ�����ʱ��ֵΪNULL
				               )
{
  return 0;

}

int NJUST_MAP_Write_offline_data( const void* pIPData,  //���������IP���ݽṹ��ͬ�����ݿ�
					              const int   nBytes,   //pIPData���ݵ��ֽڸ���
							      const char* pFileName //�洢���ݵ��ļ���
			                     )
{

   return 0;
}
