#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<time.h>
#include"NJUST_IP_comm.h"
#include<fstream>
#include<iostream>
#include<vector>

#include"NJUST_MC_data.h"

using namespace std;
#define MAX_NUM 20000

#define MODLE "MC"


//NJUST_MC_NAV_INFO tempNAV;
ifstream myif;
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

//读取真实MC发送的GPS数据
void readGPS(){
	
       

        myif.open("GPSReal.txt",ios::in);
        if(!myif.is_open()){
			printf("打开文件失败\n");
		fflush(stdout);
	}



	int a=0,b=0;   //读取整数分
    double a1,b1;//转化为浮点数°
	int count=0;
	//NJUST_MC_NAV_INFO* tempNAV=new NJUST_MC_NAV_INFO();


	NJUST_MC_NAV_INFO Nav;
	Nav.Head=0x0824;

	int sendtime=0;
			//printf("开始读取文件经纬度\n"); fflush(stdout);
	while(myif>>a&&myif>>b){
	
		count++;
        a1=double(a)/60/10000;		
        b1=double(b)/60/10000;
        Nav.Longitude_degree=a1;
        Nav.Latitude_degree=b1;
		Nav.nSize=sizeof(NJUST_MC_NAV_INFO);
        char* driveinfo=(char *)&Nav;
		char Sendforothers[4096];
		unsigned char sum;
		int i;
		for(i=0,sum=0;i<sizeof(NJUST_MC_NAV_INFO)-1;i++){
			Sendforothers[i]=driveinfo[i];
			sum+=Sendforothers[i];
		}
		Sendforothers[i]=sum;
        NJUST_IP_udp_send_to("",Sendforothers,sizeof(NJUST_MC_NAV_INFO));//广播
		//int ssize=0;
		sendtime++;
		ms_sleep(50);
			printf("%lf %lf \n",Nav.Longitude_degree,Nav.Latitude_degree); fflush(stdout);
             
	}
	myif.close();
}
int main()
{
	NJUST_IP_set_moduleName( MODLE, 0);
	srand((int)time(0));

	readGPS();


    return 0;
}
