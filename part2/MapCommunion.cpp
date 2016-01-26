#include"MapCommunion.h"


///
/// MAP通信模块
///

//注册MAP
int MapCommunion::RegisterMap(){
	//判断模块名是否存在
	if (NJUST_IP_moduleName_exist("MAP"))
	{
		//若不存在，则创建
		if (NJUST_IP_set_moduleName("MAP", 0))
		{
			return 0;
		}
	}
	return 1;
}

//针对特定模块，注册回调函数
int MapCommunion::ReciveModuleMsg(const char * modulename,func_t func){
	if (-1 == NJUST_IP_set_tcp_callBack(modulename, func, NULL))
	{
		return 0;
	}
	return 1;
}

//接收广播
int MapCommunion::ReciveBroadcastMsg(func_t func){
	NJUST_IP_set_broadcast_callBack(func, NULL);
	return 1;
}

