/******************************************************************************
*    Copyright (C), 2015 by HappyTown                                         *
*    FileName:    gpsParse.h
*    Author:      WangYing	                                             *
*    Description:                                                             *
*    History:                                                                 *
*      <author>          <time>          <version>          <description>     *
*        Xzj        2016-07-26 15:26      V1.0.0                build         *
*                                                                             *
******************************************************************************/
#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
typedef struct __DATE_TIME_INFO__ {  
	int year;			///< 2016年显示为16
	int month;   
	int day;  
	int hour;  
	int minute;  
	int second;  
} stDateTime, *pstDateTime;

typedef struct __GPS_INFO__ {  
	stDateTime D;		///< 时间  
	char status;		///< 接收状态(A=有效定位，V=无效定位)
	double latitude;	///< 纬度(ddmm.mmmm(度分)格式)
	double longitude;	///< 经度(dddmm.mmmm(度分)格式)
	char NS;			///< 南北极(N(北半球)或S(南半球))
	char EW;			///< 东西(E(东经)或W(西经))
	double speed;		///< 速度(速率值是海里/时，单位是节，要把它转换成千米/时，根据：1海里=1.85公里，把得到的速率乘以1.85)
	double high;		///< 高度(-9999.9到9999.9米)
	int satelliteNum;	///< 使用的卫星个数(从00到12)
} stGpsInfo, *pstGpsInfo;

typedef struct __SATELLITE_INFO__ {  
	int total;			///< 可视卫星总数
	int PRN;			///< 卫星编号
	int elevation;		///< 仰角(00～90度)
	int azimuth;		///< 方位角(000～359度)
	int SNR;			///< 信噪比(00～99dB)
} stSatelliteInfo, *pstSatelliteInfo;

typedef void (*pfnOnGpsInfo)(const pstGpsInfo pInfo);
typedef void (*pfnOnSatelliteInfo)(const pstSatelliteInfo pInfo);

typedef struct __GPS_CALLBACK_INFO__ {  
	pfnOnGpsInfo onGpsInfo;
	pfnOnSatelliteInfo onSatelliteInfo;
} stGpsCallbackInfo, *pstGpsCallbackInfo;

void init(const char* pSerialName, pstGpsCallbackInfo pfnCallback);
void uninit();
void start();

#endif //__GPS_PARSE_H__
