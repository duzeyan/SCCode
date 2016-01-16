////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, 南京理工大学计算机科学与工程学院, 智能科学与技术系
//  FileName:  NJUST_MO_data.h
//  Author: 张重阳
//  Date:   2015.6.6
//  Description: 显控模块和其他模块间的命令和状态
//  Modification: 
//          2015.7.2, 任明武
//  Declare:
//          NJUST_FROM_MO_COMMAND
//          NJUST_TO_MO_WORKSTAT
//
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_MO_DATA_H_
#define _NJUST_MO_DATA_H_

#include "NJUST_Global_Def.h"
#include "NJUST_IP_comm.h" 

//系统定义
#define  NJUST_MO_MAX_COMMAND_PARA_LEN          32     //最大命令参数长度
#define  NJUST_MO_WORK_STAT_ERRMSG_MAXLEN       32     //工作状态错误消息的最大长度
#define  NJUST_MO_DEBUGMSG_MAXLEN              128     //调试信息的最大长度

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  命令: 显控发给各模块
//
////////////////////////////////////////////////////////////////////////////////////////////////
enum NJUST_MO_COMMAND_TYPE
{
	NJUST_MO_COMMAND_TYPE_NO = 0x00,                  //无命令
	NJUST_MO_COMMAND_TYPE_COMPUTER_RESTART,           //计算机重新启动
	NJUST_MO_COMMAND_TYPE_COMPUTER_SHUTDOWN,          //计算机关机
	NJUST_MO_COMMAND_TYPE_MODULE_RESTART,             //模块重新启动
	NJUST_MO_COMMAND_TYPE_MODULE_DEBUG,               //模块切换到调试模式
	NJUST_MO_COMMAND_TYPE_MODULE_RELEASE,             //模块切换到运行模式
    NJUST_MO_COMMAND_TYPE_TOTAL_NUM	                 //最多命令种类数
};

enum NJUST_MO_MODULE_ID
{

};

typedef struct tagNJUST_FROM_MO_COMMAND
{   
	int frameID; //命令编号(从0开始)
	NJUST_IP_TIME synTime;  //系统时间
    int moduleID;   //模块ID
    NJUST_MO_COMMAND_TYPE cmd; //命令
    char pPara[NJUST_MO_MAX_COMMAND_PARA_LEN+1]; //命令参数
	int nPara; //有效参数的字节数
	int nSize; //该结构体的大小 
}NJUST_FROM_MO_COMMAND;

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  模块工作状态: 各模块发给显控
//
////////////////////////////////////////////////////////////////////////////////////////////////
enum NJUST_MO_MODULE_STATUS
{ 
    NJUST_MO_WORK_STAT_INVAILD = 0x00, //模块无结果输出
    NJUST_MO_WORK_STAT_VAILD,          //模块输出有效
    NJUST_MO_WORK_STAT_TOTAL_NUM	  //最多状态种类数
};
enum NJUST_MO_MODULE_ERRCODE
{ 
    NJUST_MO_ERRCODE_NOERR = 0x00,   //无错误
    NJUST_MO_ERRCODE_ERROR,          //错误
    NJUST_MO_ERRCODE_TOTAL_NUM	    //最多错误码种类数
};

//各模块给显控的工作状态数据
typedef struct tagNJUST_TO_MO_WORKSTAT
{
	NJUST_IP_TIME synTime;  //系统时间
    int moduleID;   //模块ID
	int myselfTimeOutMS; //模块的超时,单位:ms;若总控在该时间内没有收到状态数据,则认为本模块不在线
    NJUST_MO_MODULE_STATUS stat;    // 工作状态
	int PELR; //按千分比表示的丢包率
	int timeConsumingMS; //本帧处理所用的时间,单位ms
	NJUST_MO_MODULE_ERRCODE errCode;     //错误码
    char pErrMsg[NJUST_MO_WORK_STAT_ERRMSG_MAXLEN+1];   //错误消息
}NJUST_TO_MO_WORKSTAT;

#endif

