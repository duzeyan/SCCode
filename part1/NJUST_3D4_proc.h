﻿////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, 南京理工大学计算机科学与工程学院, 智能科学与技术系
//  FileName:  NJUST_3D4_proc.h
//  Author: 蔡云飞
//  Date:   2015.6.27
//  Description: 3D模块提供给目标模块的解码函数
//
//           int NJUST_3D4_Decode_IP_Data( const void* pIPData, const int nBytes,
//                                        NJUST_3D4_OBS_TO_FU            **pOBS,       //当不是障碍数据时,值为NULL
//							              NJUST_3D4_ROAD_BOUNDARYTO_FU   **pBoundary,  //当不是边界信息时,值为NULL
//							              NJUST_3D4_ROAD_SLOPETO_FU      **pSlope,     //当不是坡度数据时,值为NULL
//                                        NJUST_3D4_GRID_TO_FU           **pGrid       //当不是网格数据时,值为NULL
//						                );
//           int NJUST_3D4_Read_offline_data( const char* pFileName, //待读取的数据文件名
//                                           NJUST_3D4_OBS_TO_FU            *pOBS,       //当不是障碍数据时,值为NULL
//							                 NJUST_3D4_ROAD_BOUNDARY_TO_FU  *pBoundary,  //当不是边界信息时,值为NULL
//							                 NJUST_3D4_ROAD_SLOPE_TO_FU     *pSlope,     //当不是坡度数据时,值为NULL
//                                           NJUST_3D4_GRID_TO_FU           **pGrid      //当不是网格数据时,值为NULL
//							               );
//           int NJUST_3D4_Write_offline_data( const void* pIPData,  //待保存的与IP数据结构相同的数据块
//							                  const int   nBytes,   //pIPData数据的字节个数							   
//							  	              const char* pFileName //存储数据的文件名
// 						                    );
//
// 
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_3D4_PROC_H_
#define _NJUST_3D4_PROC_H_

#include "NJUST_3D4_data.h" 


#ifdef __cplusplus
extern "C" {
#endif


struct  NJUST_3D4_ROAD_BOUNDARY_GLOBAL
{
	int debug_info;
	NJUST_3D4_ROAD_BOUNDARY   boundary;
};
struct NJUST_3D4_OBS_GLOBAL
{
	int debug_info;
	NJUST_3D4_OBS   obs;
};
struct NJUST_3D4_SLOP_GLOBAL
{
	int debug_info;
	NJUST_3D4_ROAD_SLOPE   slop;
};
	
struct NJUST_3D4_DataInfo_LIDAR4
{
	int debug_info;
	NJUST_3D4_OBS  obs;
	NJUST_3D4_ROAD_BOUNDARY   boundary;
};
////////////////////////////////////////////////////////////////////////////////////////////////
/*
//---------------------------------------压缩编码，自己使用-----------------------------//
 @brief 压缩编码函数，自己使用
 * @param p3DRes [in] ,需要压缩的结构体数据指针
 * @param p3DBuf [out] ,压缩之后的的char型数组
 * @return {int}, 成功返回0, 否则-1
//--------------------------------------------------------------------------------------//
*/
int  NJUST_3D4_Encode_Compress(NJUST_3D4_DataInfo_LIDAR4 *p3DRes,unsigned char *p3DBuf);
////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////
//
//  提供给目标模块的解码函数
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 提供给目标模块的解码函数
 * @param pIPData {const void*} [in] IP数据,目标模块从网络收到的数据
 * @param nBytes {const int} [in] pIPData数据的字节个数
 * @param pOBS {NJUST_3D4_OBS *} [out] 指向障碍数据; 不是障碍数据时,值为空指针NULL
 * @param pBoundary {NJUST_3D4_ROAD_BOUNDARY *} [out] 指向边界信息; 不是边界信息时,值为空指针NULL
 * @param pSlope {NJUST_3D4_ROAD_SLOPE *} [out] 指向坡度数据; 不是坡度数据时,值为空指针NULL
 * @return {int}, 成功返回0, 否则-1
 */
int NJUST_3D4_Decode_IP_Data( const void* pIPData, const int nBytes,
                              NJUST_3D4_OBS_TO_FU            **pOBS,       //当不是障碍数据时,值为NULL
							  NJUST_3D4_ROAD_BOUNDARY_TO_FU  **pBoundary,  //当不是边界信息时,值为NULL
							  NJUST_3D4_ROAD_SLOPE_TO_FU     **pSlope      //当不是坡度数据时,值为NULL
   						    );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  读取离线数据文件
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 读取离线数据文件
 * @param pFileName {const char*} [in] 待读取的数据文件名
 * @param pOBS {NJUST_3D4_OBS *} [out] 指向障碍数据; 不是障碍数据时,值为空指针NULL
 * @param pBoundary {NJUST_3D4_ROAD_BOUNDARY *} [out] 指向边界信息; 不是边界信息时,值为空指针NULL
 * @param pSlope {NJUST_3D4_ROAD_SLOPE *} [out] 指向坡度数据; 不是坡度数据时,值为空指针NULL
 * @return {int}, 成功返回0, 否则-1
 */
int NJUST_3D4_Read_offline_data( const char* pFileName, //待读取的数据文件名
                                NJUST_3D4_OBS_TO_FU            *pOBS,       //当不是障碍数据时,值为NULL
							    NJUST_3D4_ROAD_BOUNDARY_TO_FU  *pBoundary,  //当不是边界信息时,值为NULL
							    NJUST_3D4_ROAD_SLOPE_TO_FU     *pSlope     //当不是坡度数据时,值为NULL 
							    );
		
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  写数据文件
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 将数据写成文件
 * @param pIPData {const void*} [in] 待保存的与IP数据结构相同的数据块
 * @param nBytes {const int} [in] pIPData数据的字节个数
 * @param pFileName {const char*} [in] 存储数据的文件名
 * @return {int}, 成功返回0, 否则-1
 */
int NJUST_3D4_Write_offline_data( const void* pIPData,  //待保存的与IP数据结构相同的数据块
							     const int   nBytes,   //pIPData数据的字节个数							   
							  	 const char* pFileName //存储数据的文件名
 						       );

#ifdef __cplusplus
}
#endif

#endif
