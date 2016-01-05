///Parameter头文件
#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#define IDC_NODE   10000
#define IDC_LINE   20000

#define MAX_NODE_NUM  200
#define INF  1000000000

#define THRESHOLD  35  //路口节点的范围，单位是  m
#define THREODNODE 35  //判断GPS点距离路点的远近的阈值

#define SIZESHUZU  5

#define NOTESIZE   300  
#define LINESIZE   300
#define BUFFSIZE   1024
      
//阈值方案一，可用------------------------------------------------------------
#define STEP  1                            //下一个GPS点的步长
#define DISTANCETHREOD1 600                 //当前点是否在当前GPS序列点的阈值范围
#define DISTANCETHREOD2 30                  //开始时从GPS点序列中搜索匹配点的阈值范围
//#define DISTANCETHREOD  500//可用           //采集GSS序列时的范围，这里用不到

/*
//阈值方案二------------------------------------------------------------
#define STEP2  2
#define DISTANCETHREOD1 400
#define DISTANCETHREOD2 17
#define DISTANCETHREOD  200

//阈值方案三------------------------------------------------------------
#define STEP2  3
#define DISTANCETHREOD1 500
#define DISTANCETHREOD2 17
#define DISTANCETHREOD  200
*/


#endif