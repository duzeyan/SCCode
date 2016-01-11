#include "NJUST_MAP_proc.h"
#include "stdlib.h"
#include "memory.h"

////////////////////////////////////////////////////////////////////////////////////////////

//内存
static NJUST_MAP_INFO_ROAD       gNJUST_Map_Road;
static NJUST_MAP_INFO_NODE       gNJUST_Map_Node;
static NJUST_MAP_INFO_DIRECTION  gNJUST_Map_Direction;

////////////////////////////////////////////////////////////////////////////////////////////
int  NJUST_MAP_Decode_IP_Data( const void* pIPData, const int nBytes,
							 NJUST_MAP_INFO_ROAD  **pRoad, //当不是道路数据时,值为NULL
							 NJUST_MAP_INFO_NODE  **pNode,//当不是路口信息时,值为NULL
							 NJUST_MAP_INFO_DIRECTION      **pProprity//当不是方向导引时。值为NULL
							 )
{
	char date;
	char *pdata=(char *)pIPData;
    memcpy(&date,pdata,1);


	switch (date)
	{
	case '0'://road信息
		{
			memcpy(&gNJUST_Map_Road,&pdata[1],sizeof(NJUST_MAP_INFO_ROAD));
			*pRoad=&gNJUST_Map_Road;
			*pNode=NULL;
			*pProprity=NULL;
			break;
		}
	case '1'://node信息
		{
			memcpy(&gNJUST_Map_Node,&pdata[1],sizeof(NJUST_MAP_INFO_NODE));
			*pNode = &gNJUST_Map_Node;
			*pRoad=NULL;
			*pProprity=NULL;
			break;
		}
	case '2'://方向导引
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

int NJUST_MAP_Read_offline_data( const char* pFileName, //待读取的数据文件名
                                 NJUST_MAP_INFO_ROAD  **pRoad, //当不包含道路数据时,值为NULL
			                     NJUST_MAP_INFO_NODE  **pNode, //当不包含路口信息时,值为NULL
								 NJUST_MAP_INFO_DIRECTION      **pProprity//当不是方向导引时。值为NULL
				               )
{
  return 0;

}

int NJUST_MAP_Write_offline_data( const void* pIPData,  //待保存的与IP数据结构相同的数据块
					              const int   nBytes,   //pIPData数据的字节个数
							      const char* pFileName //存储数据的文件名
			                     )
{

   return 0;
}
