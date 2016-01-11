#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<time.h>
#include"NJUST_IP_comm.h"
#include"NJUST_MAP_data.h"
#include"NJUST_MAP_proc.h"


#define random(x) (rand()%x)
#define MODLE "PL"

#define DATA_SIZE 1024*2


int count_map=0;
void ms_sleep( unsigned int msecs )
{
	int rc;
	struct timeval tv;

	tv.tv_sec = msecs / 1000;
	tv.tv_usec = ( msecs % 1000 ) * 1000;
	do
	{
		rc = select( 0, NULL, NULL, NULL, &tv );
	}
	while ( rc == -1 && errno == EINTR );
}

int func(void* buff,size_t mysize,void *argv){
//	 printf("===== PL has receive udp data  mysize=%d  count=%d\n",(int) mysize ,count_udp++);
//	 fflush(stdout);
	NJUST_MAP_INFO_ROAD *pRoad;
	NJUST_MAP_INFO_NODE *pNode;
 	NJUST_MAP_INFO_DIRECTION *pDirection;
	

	NJUST_MAP_Decode_IP_Data(buff,mysize,&pRoad,&pNode,&pDirection);
	count_map++;
	double lo=0,la=0;
	if(count_map%5==0){
		if(pNode){
			if(pNode->GPSPointQueuelength>0){
						printf("%d\n",pNode->GPSPointQueuelength); fflush(stdout);
				for(int i=0;i<pNode->GPSPointQueuelength;i++){
					lo=pNode->nextGPSPointQueue[i].longtitude;
					la=pNode->nextGPSPointQueue[i].latitude;
					lo/=60;
					la/=60;
						printf("%.8lf %.8lf\n",lo,la); fflush(stdout);
				}
			}
		}
		
	if(pRoad){
			if(pRoad->GPSPointQueuelength>0){
						printf("%d\n",pRoad->GPSPointQueuelength); fflush(stdout);
				for(int i=0;i<pRoad->GPSPointQueuelength;i++){
					lo=pRoad->nextGPSPointQueue[i].longtitude;
					la=pRoad->nextGPSPointQueue[i].latitude;
					lo/=60;
					la/=60;
						printf("%.8lf %.8lf\n",lo,la); fflush(stdout);
				}//for
			}//if 0
		}//if NULL

	}//if %5
	return 1;
}
int main()
{
	NJUST_IP_set_moduleName( MODLE, 0);
	srand((int)time(0));
	

//	NJUST_IP_set_broadcast_callBack(func_udp, NULL);
	NJUST_IP_set_tcp_callBack("MAP",func,NULL);

	while(1)
	{
		ms_sleep(200);
	}
}
