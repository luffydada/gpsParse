/******************************************************************************
*    Copyright (C), 2015 by HappyTown                                         *
*    FileName:    gpsParse.cpp
*    Author:      WangYing	                                             *
*    Description:                                                             *
*    History:                                                                 *
*      <author>          <time>          <version>          <description>     *
*        Xzj        2016-07-26 15:27      V1.0.0                build         *
*                                                                             *
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>

#include "gpsParse.h"
#define MAX_LENGTH_BUFFER (4*1024)
static const char *GPRMC_HEAD = "$GPRMC";///< 最小定位信息
static const char *GNRMC_HEAD = "$GNRMC";///< 最小定位信息
static const char *BDRMC_HEAD = "$BDRMC";///< 最小定位信息
static const char *GPGGA_HEAD = "$GPGGA";///< GPS定位数据
static const char *GNGGA_HEAD = "$GNGGA";///< GPS定位数据
static const char *BDGGA_HEAD = "$BDGGA";///< GPS定位数据
static const char *GPVTG_HEAD = "$GPVTG";///< 地面速度信息
static const char *GPGSV_HEAD = "$GPGSV";///< 可视卫星状态
static const char *GNGSV_HEAD = "$GNGSV";///< 可视卫星状态
static const char *BDGSV_HEAD = "$BDGSV";///< 可视卫星状态
static const char *GPGSA_HEAD = "$GPGSA";///< 当前卫星信息 
static const char *GNGSA_HEAD = "$GNGSA";///< 当前卫星信息 
static const char *BDGSA_HEAD = "$BDGSA";///< 当前卫星信息 
static const char *GPGLL_HEAD = "$GPGLL";///< 地理定位信息
static const char *GNGLL_HEAD = "$GNGLL";///< 地理定位信息
static const char *BDGLL_HEAD = "$BDGLL";///< 地理定位信息
static const char FLAG = ',';
static const char FLAG2 = '*';
#define DEBUG 0

// 串口设备信息结构
typedef struct tty_info_t
{
    int fd;// 串口设备ID
    pthread_mutex_t mt;// 线程同步互斥对象
    char name[24];// 串口设备名称，例："/dev/ttyS0"
    struct termios ntm;// 新的串口设备选项
    struct termios otm;// 旧的串口设备选项
} TTY_INFO, *pTTY_INFO;

int identifyPacket(const char* pcDataBuffer, int nLen);
int parsePacket(char* pcDataBuffer, int nLen);
char* getGpsData(char* pIn, char* pOut);
char* getGpsDataEx(char* pIn, char* pOut);

char* getRmcUtcTime(char* pIn, int *pHour, int *pMinute, int *pSecond);
char* getRmcValid(char* pIn, char *pValue);
char* getRmcLatitudeValue(char* pIn, double *pValue);
char* getRmcLatitudeNS(char* pIn, char *pValue);
char* getRmcLongitudeValue(char* pIn, double *pValue);
char* getRmcLongitudeEW(char* pIn, char *pValue);
char* getRmcSpeed(char* pIn, double *pValue);
char* getRmcGeographyCourse(char* pIn, double *pValue);
char* getRmcUtcDate(char* pIn, int *pYear, int *pMonth, int *pDay);
char* getRmcMagneticDeclination(char* pIn, double *pValue);
char* getRmcMagneticDeclinationEW(char* pIn, char *pValue);
char* getRmcModeEx(char* pIn, char *pValue);

char* getGgaUtcTime(char* pIn, int *pHour, int *pMinute, int *pSecond);
char* getGgaLatitudeValue(char* pIn, double *pValue);
char* getGgaLatitudeNS(char* pIn, char *pValue);
char* getGgaLongitudeValue(char* pIn, double *pValue);
char* getGgaLongitudeEW(char* pIn, char *pValue);
char* getGgaQuality(char* pIn, int *pValue);
char* getGgaSatelliteNum(char* pIn, int *pValue);
char* getGgaHorizonPrecision(char* pIn, double *pValue);
char* getGgaSeaLevel(char* pIn, double *pValue);
char* getGgaGeoidLevel(char* pIn, double *pValue);
char* getGgaDifferenceGps(char* pIn, int *pValue);
char* getGgaDifferenceReferenceStationLabelEx(char* pIn, int *pValue);

char* getGsvPacketTotal(char* pIn, int *pValue);
char* getGsvPacketIdx(char* pIn, int *pValue);
char* getGsvSatelliteTotal(char* pIn, int *pValue);
char* getGsvPRN(char* pIn, int *pValue);
char* getGsvElevation(char* pIn, int *pValue);
char* getGsvAzimuth(char* pIn, int *pValue);
char* getGsvSNR(char* pIn, int *pValue);
char* getGsvSNREx(char* pIn, int *pValue);

char* getGsaMode(char* pIn, char *pValue);
char* getGsaLocateType(char* pIn, int *pValue);
char* getGsaPRN(char* pIn, int *pValue);
char* getGsaPDOP(char* pIn, double *pValue);
char* getGsaHDOP(char* pIn, double *pValue);
char* getGsaVDOPEx(char* pIn, double *pValue);

// 串口操作函数
TTY_INFO *readyTTY(const char *pSerialName);
int setTTYSpeed(TTY_INFO *ptty, int speed);
int setTTYParity(TTY_INFO *ptty,int databits,int parity,int stopbits);
int cleanTTY(TTY_INFO *ptty);

pstGpsCallbackInfo g_pfnCallback = NULL;
pTTY_INFO g_pInfo = NULL;
char* g_pcDataBuffer = NULL;
int g_nDataLen = 0;
pstGpsInfo g_pGpsInfo = NULL;
pstSatelliteInfo g_pSatelliteInfo = NULL;

void* readDataThread(void * pData)
{
#if DEBUG 
	printf("readDataThread,id:%d\n", g_pInfo->fd);
#endif
	if ( g_pInfo ) {
		struct pollfd pfds;
		pfds.fd = g_pInfo->fd;
		pfds.events = POLLIN;
		pfds.revents = 0;	

		int rc = 0;
		char rbuf[260] = {0};
		int poll_ret = 0;
		do {
			poll_ret = poll(&pfds, 1, -1);
//			printf("poll ret %d\n", poll_ret);
			if (pfds.revents & POLLIN) {
				unsigned sec = 0;
				while ((rc = read(pfds.fd, rbuf, sizeof(rbuf))) > 0) {
#if DEBUG 
					printf("read %d bytes,rbuf:%s\n", rc, rbuf);
#endif
					if(rc + g_nDataLen > MAX_LENGTH_BUFFER || g_nDataLen < 0) {
						g_nDataLen = 0;
						memset(g_pcDataBuffer, 0, MAX_LENGTH_BUFFER);
					}
					memcpy(g_pcDataBuffer+g_nDataLen, rbuf, rc);
					g_nDataLen += rc;
					int nPacketLen = identifyPacket(g_pcDataBuffer, g_nDataLen);
//					printf("identifyPacket,nPacketLen:%d\n", nPacketLen);
					if(nPacketLen > 0) {
						g_nDataLen -= nPacketLen;
						if(g_nDataLen > 0) {
							memcpy(g_pcDataBuffer, g_pcDataBuffer + nPacketLen, g_nDataLen);
						}
					}
					memset(rbuf, 0, 260);
				} while (rc > 0);

				if (rc < 0) {
					if (errno == EAGAIN) {
//						printf("no more data\n");
					} else {
						printf("read failed, errno %d\n", errno);
						poll_ret = -1;
					}
				}
			}
		} while (poll_ret > 0);
	}
	return (void *)0;
}

void init(const char* pSerialName, pstGpsCallbackInfo pfnCallback)
{
	printf("init,pSerialName:%s\n", pSerialName);
	g_pfnCallback = pfnCallback;
	if ( !g_pcDataBuffer ) {
		g_pcDataBuffer = new char[MAX_LENGTH_BUFFER];
		memset(g_pcDataBuffer, 0, MAX_LENGTH_BUFFER);
		g_nDataLen = 0;
	}
	if ( !g_pGpsInfo ) {
		g_pGpsInfo = new stGpsInfo();
		memset(g_pGpsInfo, 0, sizeof(stGpsInfo));
	}
	if ( !g_pSatelliteInfo ) {
		g_pSatelliteInfo = new stSatelliteInfo();
		memset(g_pSatelliteInfo, 0, sizeof(stSatelliteInfo));
	} 
	if ( !g_pInfo ) {
		g_pInfo = readyTTY(pSerialName);
		setTTYSpeed(g_pInfo,9600);
		setTTYParity(g_pInfo, 8, 'N', 1);
	}
}

void uninit()
{
	g_pfnCallback = NULL;
	if ( g_pcDataBuffer ) {
		delete[] g_pcDataBuffer;
		g_pcDataBuffer = NULL;
	}
	if ( g_pGpsInfo ) {
		delete g_pGpsInfo;
		g_pGpsInfo = NULL;
	}
	if ( g_pSatelliteInfo ) {
		delete g_pSatelliteInfo;
		g_pSatelliteInfo = NULL;
	} 
	cleanTTY(g_pInfo);
}

void start()
{
#if DEBUG 
	printf("start\n");
#endif
	pthread_t pid;
	pthread_create(&pid, NULL, readDataThread, NULL);
}

int identifyPacket(const char* pcDataBuffer, int nLen)
{
#if DEBUG 
//	printf("identifyPacket,nLen:%d,data:%s\n", nLen, pcDataBuffer);
#endif

	int nCount = 0;
	bool isStart = false;
	int pos = 0;
	for(int i=0; i<nLen - 1; ++i)
	{
		if('$' == *(pcDataBuffer+i) ) {
			isStart = true;
			pos = i;
		} else if ( '\n' == *(pcDataBuffer + i) && isStart) {
			isStart = false;
			char buffer[256] = {0};
			memcpy(buffer, pcDataBuffer + pos, i - pos);
			parsePacket(buffer, i - pos);
			nCount = i;
		}
	}
	return nCount;
}

int parsePacket(char* pcDataBuffer, int nLen)
{
#if DEBUG 
	printf("parsePacket,nLen:%d,data:%s\n", nLen, pcDataBuffer);
#endif
	bool isDiffer = false;
	if ( !strncasecmp(pcDataBuffer, GPRMC_HEAD, strlen(GPRMC_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GNRMC_HEAD, strlen(GNRMC_HEAD)) ||\
			!strncasecmp(pcDataBuffer, BDRMC_HEAD, strlen(BDRMC_HEAD))) {
#if DEBUG 
		printf("get rmc data");
#endif
		char *pPos = strchr(pcDataBuffer, FLAG);
		if ( !pPos ) { printf("\n"); return nLen; }

		int hour = 0, minute = 0, second = 0;
		pPos = getRmcUtcTime(pPos, &hour, &minute, &second);
		if ( hour != g_pGpsInfo->D.hour ) {
			g_pGpsInfo->D.hour = hour;
			isDiffer = true;
		}
		if ( minute != g_pGpsInfo->D.minute ) {
			g_pGpsInfo->D.minute = minute;
			isDiffer = true;
		}
		if ( second != g_pGpsInfo->D.second ) {
			g_pGpsInfo->D.second = second;
			isDiffer = true;
		}
		
		char valid = 0;
		pPos = getRmcValid(pPos, &valid);
		if ( valid != g_pGpsInfo->status ) {
			g_pGpsInfo->status = valid;
			isDiffer = true;
		}

		double latitude = 0.0;
		pPos = getRmcLatitudeValue(pPos, &latitude);
		if ( latitude != g_pGpsInfo->latitude ) {
			g_pGpsInfo->latitude = latitude;
			isDiffer = true;
		}

		char NS = 0;
		pPos = getRmcLatitudeNS(pPos, &NS);
		if ( NS != g_pGpsInfo->NS ) {
			g_pGpsInfo->NS = NS;
			isDiffer = true;
		}

		double longitude = 0.0;
		pPos = getRmcLongitudeValue(pPos, &longitude);
		if ( longitude != g_pGpsInfo->longitude ) {
			g_pGpsInfo->longitude = longitude;
			isDiffer = true;
		}

		char EW = 0;
		pPos = getRmcLongitudeEW(pPos, &EW);
		if ( EW != g_pGpsInfo->EW ) {
			g_pGpsInfo->EW = EW;
			isDiffer = true;
		}

		double speed = 0.0;
		pPos = getRmcSpeed(pPos, &speed);
		if ( speed != g_pGpsInfo->speed ) {
			g_pGpsInfo->speed = speed;
			isDiffer = true;
		}

		double course = 0.0;
		pPos = getRmcGeographyCourse(pPos, &course);

		int year = 0, month = 0, day = 0;
		pPos = getRmcUtcDate(pPos, &year, &month, &day);
		if ( day != g_pGpsInfo->D.day ) {
			g_pGpsInfo->D.day = day;
			isDiffer = true;
		}
		if ( month != g_pGpsInfo->D.month ) {
			g_pGpsInfo->D.month = month;
			isDiffer = true;
		}

		if ( year != g_pGpsInfo->D.year ) {
			g_pGpsInfo->D.year = year;
			isDiffer = true;
		}

		double magneticDeclination = 0.0;
		pPos = getRmcMagneticDeclination(pPos, &magneticDeclination );

		char magneticDeclinationEW = 0;
		pPos = getRmcMagneticDeclinationEW(pPos, &magneticDeclinationEW);

		char mode = 0;
		pPos = getRmcModeEx(pPos, &mode);
#if DEBUG 
		printf("\n");
#endif
		if ( g_pfnCallback && isDiffer ) {
			g_pfnCallback->onGpsInfo(g_pGpsInfo);
		}
	} else if ( !strncasecmp(pcDataBuffer, GPGGA_HEAD, strlen(GPGGA_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GPGGA_HEAD, strlen(GPGGA_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GPGGA_HEAD, strlen(GPGGA_HEAD))) {
#if DEBUG 
		printf("get gga data");
#endif
		char *pPos = strchr(pcDataBuffer, FLAG);
		if ( !pPos ) { printf("\n"); return nLen; }

		int hour = 0, minute = 0, second = 0;
		pPos = getGgaUtcTime(pPos, &hour, &minute, &second);

		double latitude = 0.0;
		pPos = getGgaLatitudeValue(pPos, &latitude);

		char NS = 0;
		pPos = getGgaLatitudeNS(pPos, &NS);

		double longitude = 0.0;
		pPos = getRmcLongitudeValue(pPos, &longitude);

		char EW = 0;
		pPos = getGgaLongitudeEW(pPos, &EW);

		int quality = 0;
		pPos = getGgaQuality(pPos, &quality);

		int satelliteNum = 0;
		pPos = getGgaSatelliteNum(pPos, &satelliteNum);
		if ( satelliteNum != g_pGpsInfo->satelliteNum ) {
			g_pGpsInfo->satelliteNum = satelliteNum;
			isDiffer = true;
		}

		double horizonPrecision = 0.0;
		pPos = getGgaHorizonPrecision(pPos, &horizonPrecision);

		double seaLevel = 0.0;
		pPos = getGgaSeaLevel(pPos, &seaLevel);

		if ( pPos && pPos + 1 ) {
			pPos = strchr(pPos + 1, FLAG);
		}
		
		double geoidLevel = 0.0;
		pPos = getGgaGeoidLevel(pPos, &geoidLevel);
		if ( geoidLevel != g_pGpsInfo->high ) {
			g_pGpsInfo->high = geoidLevel;
			isDiffer = true;
		}

		if ( pPos && pPos + 1 ) {
			pPos = strchr(pPos + 1, FLAG);
		}

		int differenceGps = 0;
		pPos = getGgaDifferenceGps(pPos, &differenceGps);

		int differenceReferenceStationLabel = 0;
		pPos = getGgaDifferenceReferenceStationLabelEx(pPos, &differenceReferenceStationLabel);

#if DEBUG 
		printf("\n");
#endif
		if ( g_pfnCallback && isDiffer ) {
			g_pfnCallback->onGpsInfo(g_pGpsInfo);
		}
	} else if ( !strncasecmp(pcDataBuffer, GPVTG_HEAD, strlen(GPVTG_HEAD)) ) {
#if DEBUG 
		printf("get vtg data\n");
#endif
		
// $GPVTG,<1>,T,<2>,M,<3>,N,<4>,K,<5>*hh
// 　　<1> 以正北为参考基准的地面航向(000~359度，前面的0也将被传输)
// 　　<2> 以磁北为参考基准的地面航向(000~359度，前面的0也将被传输)
// 　　<3> 地面速率(000.0~999.9节，前面的0也将被传输)
// 　　<4> 地面速率(0000.0~1851.8公里/小时，前面的0也将被传输)
// 　　<5> 模式指示(仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效   
	} else if ( !strncasecmp(pcDataBuffer, GPGSV_HEAD, strlen(GPGSV_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GNGSV_HEAD, strlen(GNGSV_HEAD)) ||\
			!strncasecmp(pcDataBuffer, BDGSV_HEAD, strlen(BDGSV_HEAD))) {
		bool isDiffer = false;
#if DEBUG 
		printf("get gsv data");
#endif
		char *pPos = strchr(pcDataBuffer, FLAG);
		if ( !pPos ) { printf("\n"); return nLen; }

		int packetTotal = 0;
		pPos = getGsvPacketTotal(pPos, &packetTotal);

		int idx = 0;
		pPos = getGsvPacketIdx(pPos, &idx);

		int total = 0;
		pPos = getGsvSatelliteTotal(pPos, &total);
		if ( total != g_pSatelliteInfo->total ) {
			g_pSatelliteInfo->total = total;
			isDiffer = true;
		}

		int PRN = 0;
		pPos = getGsvPRN(pPos, &PRN);
		if ( PRN != g_pSatelliteInfo->PRN ) {
			g_pSatelliteInfo->PRN = PRN;
			isDiffer = true;
		}

		int elevation = 0;
		pPos = getGsvElevation(pPos, &elevation);
		if ( elevation != g_pSatelliteInfo->elevation ) {
			g_pSatelliteInfo->elevation = elevation;
			isDiffer = true;
		}

		int azimuth = 0;
		pPos = getGsvAzimuth(pPos, &azimuth);
		if ( azimuth != g_pSatelliteInfo->azimuth ) {
			g_pSatelliteInfo->azimuth = azimuth;
			isDiffer = true;
		}

		bool isHaveLast = true;
		int SNR = 0;
		char *pSNR = getGsvSNR(pPos, &SNR);
		if ( !pSNR ) {
			pSNR = getGsvSNREx(pPos, &SNR);
			if ( !pSNR ) {
				return nLen;
			}
			isHaveLast = false;
		}
		pPos = pSNR;
		if ( SNR != g_pSatelliteInfo->SNR ) {
			g_pSatelliteInfo->SNR = SNR;
			isDiffer = true;
		}

		if ( isDiffer && g_pfnCallback ) {
			g_pfnCallback->onSatelliteInfo(g_pSatelliteInfo);
		}

		if ( isHaveLast ) {
			g_pSatelliteInfo->PRN = 0;
			g_pSatelliteInfo->SNR = 0;
			g_pSatelliteInfo->azimuth = 0;
			g_pSatelliteInfo->elevation = 0;
			int PRN = 0;
			pPos = getGsvPRN(pPos, &PRN);
			if ( PRN != g_pSatelliteInfo->PRN ) {
				g_pSatelliteInfo->PRN = PRN;
				isDiffer = true;
			}

			int elevation = 0;
			pPos = getGsvElevation(pPos, &elevation);
			if ( elevation != g_pSatelliteInfo->elevation ) {
				g_pSatelliteInfo->elevation = elevation;
				isDiffer = true;
			}

			int azimuth = 0;
			pPos = getGsvAzimuth(pPos, &azimuth);
			if ( azimuth != g_pSatelliteInfo->azimuth ) {
				g_pSatelliteInfo->azimuth = azimuth;
				isDiffer = true;
			}

			bool isHaveLast = true;
			int SNR = 0;
			char *pSNR = getGsvSNR(pPos, &SNR);
			if ( !pSNR ) {
				pSNR = getGsvSNREx(pPos, &SNR);
				if ( !pSNR ) {
					return nLen;
				}
				isHaveLast = false;
			}
			pPos = pSNR;
			if ( SNR != g_pSatelliteInfo->SNR ) {
				g_pSatelliteInfo->SNR = SNR;
				isDiffer = true;
			}
			if ( isDiffer && g_pfnCallback ) {
				g_pfnCallback->onSatelliteInfo(g_pSatelliteInfo);
			}

			if ( isHaveLast ) {
				g_pSatelliteInfo->PRN = 0;
				g_pSatelliteInfo->SNR = 0;
				g_pSatelliteInfo->azimuth = 0;
				g_pSatelliteInfo->elevation = 0;
				int PRN = 0;
				pPos = getGsvPRN(pPos, &PRN);
				if ( PRN != g_pSatelliteInfo->PRN ) {
					g_pSatelliteInfo->PRN = PRN;
					isDiffer = true;
				}

				int elevation = 0;
				pPos = getGsvElevation(pPos, &elevation);
				if ( elevation != g_pSatelliteInfo->elevation ) {
					g_pSatelliteInfo->elevation = elevation;
					isDiffer = true;
				}

				int azimuth = 0;
				pPos = getGsvAzimuth(pPos, &azimuth);
				if ( azimuth != g_pSatelliteInfo->azimuth ) {
					g_pSatelliteInfo->azimuth = azimuth;
					isDiffer = true;
				}

				bool isHaveLast = true;
				int SNR = 0;
				char *pSNR = getGsvSNR(pPos, &SNR);
				if ( !pSNR ) {
					pSNR = getGsvSNREx(pPos, &SNR);
					if ( !pSNR ) {
						return nLen;
					}
					isHaveLast = false;
				}
				pPos = pSNR;
				if ( SNR != g_pSatelliteInfo->SNR ) {
					g_pSatelliteInfo->SNR = SNR;
					isDiffer = true;
				}
				if ( isDiffer && g_pfnCallback ) {
					g_pfnCallback->onSatelliteInfo(g_pSatelliteInfo);
				}

				if ( isHaveLast ) {
					g_pSatelliteInfo->PRN = 0;
					g_pSatelliteInfo->SNR = 0;
					g_pSatelliteInfo->azimuth = 0;
					g_pSatelliteInfo->elevation = 0;
					int PRN = 0;
					pPos = getGsvPRN(pPos, &PRN);
					if ( PRN != g_pSatelliteInfo->PRN ) {
						g_pSatelliteInfo->PRN = PRN;
						isDiffer = true;
					}

					int elevation = 0;
					pPos = getGsvElevation(pPos, &elevation);
					if ( elevation != g_pSatelliteInfo->elevation ) {
						g_pSatelliteInfo->elevation = elevation;
						isDiffer = true;
					}

					int azimuth = 0;
					pPos = getGsvAzimuth(pPos, &azimuth);
					if ( azimuth != g_pSatelliteInfo->azimuth ) {
						g_pSatelliteInfo->azimuth = azimuth;
						isDiffer = true;
					}

					int SNR = 0;
					pPos = getGsvSNREx(pPos, &SNR);
					if ( SNR != g_pSatelliteInfo->SNR ) {
						g_pSatelliteInfo->SNR = SNR;
						isDiffer = true;
					}
					if ( isDiffer && g_pfnCallback ) {
						g_pfnCallback->onSatelliteInfo(g_pSatelliteInfo);
					}
				}
			}
		}
#if DEBUG 
		printf("\n");
#endif
	} else if ( !strncasecmp(pcDataBuffer, GPGSA_HEAD, strlen(GPGSA_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GNGSA_HEAD, strlen(GNGSA_HEAD)) ||\
			!strncasecmp(pcDataBuffer, BDGSA_HEAD, strlen(BDGSA_HEAD))) {
#if DEBUG 
		printf("get gsa data");
#endif
		char *pPos = strchr(pcDataBuffer, FLAG);
		if ( !pPos ) { printf("\n"); return nLen; }

		char mode = 0;
		pPos = getGsaMode(pPos, &mode);

		int locateType = 0;
		pPos = getGsaLocateType(pPos, &locateType);

		int PRN1 = 0, PRN2 = 0, PRN3 = 0, PRN4 = 0, PRN5 = 0, PRN6 = 0, PRN7 = 0, PRN8 = 0, PRN9 = 0, PRN10 = 0, PRN11 = 0, PRN12 = 0;
		pPos = getGsaPRN(pPos, &PRN1);
		pPos = getGsaPRN(pPos, &PRN2);
		pPos = getGsaPRN(pPos, &PRN3);
		pPos = getGsaPRN(pPos, &PRN4);
		pPos = getGsaPRN(pPos, &PRN5);
		pPos = getGsaPRN(pPos, &PRN6);
		pPos = getGsaPRN(pPos, &PRN7);
		pPos = getGsaPRN(pPos, &PRN8);
		pPos = getGsaPRN(pPos, &PRN9);
		pPos = getGsaPRN(pPos, &PRN10);
		pPos = getGsaPRN(pPos, &PRN11);
		pPos = getGsaPRN(pPos, &PRN12);

		double PDOP = 0;
		pPos = getGsaPDOP(pPos, &PDOP);

		double HDOP = 0;
		pPos = getGsaHDOP(pPos, &HDOP);

		double VDOP = 0;
		pPos = getGsaVDOPEx(pPos, &VDOP);
#if DEBUG 
		printf("\n");
#endif
	} else if ( !strncasecmp(pcDataBuffer, GPGLL_HEAD, strlen(GPGLL_HEAD)) ||\
			!strncasecmp(pcDataBuffer, GNGLL_HEAD, strlen(GNGLL_HEAD)) ||\
			!strncasecmp(pcDataBuffer, BDGLL_HEAD, strlen(BDGLL_HEAD))) {
#if DEBUG 
		printf("get gsa data\n");
#endif
	}
	return nLen;
}

char* getGpsData(char* pIn, char* pOut)
{
	char *pRet = NULL;
	if ( pIn && pIn + 1 && pOut ) {
		pIn += 1;
		pRet = strchr(pIn, FLAG);
		if ( pRet ) {
			int len = strlen(pIn) - strlen(pRet);
			if ( len > 0 ) {
				strncpy(pOut, pIn, len);
				pOut[len] = '\0';
			}
		}
	}
	return pRet;
}

char* getGpsDataEx(char* pIn, char* pOut)
{
	char *pRet = NULL;
	if ( pIn && pIn + 1 && pOut ) {
		pIn += 1;
		pRet = strchr(pIn, FLAG2);
		if ( pRet ) {
			int len = strlen(pIn) - strlen(pRet);
			if ( len > 0 ) {
				strncpy(pOut, pIn, len);
				pOut[len] = '\0';
			}
		}
	}
	return pRet;
}

char* getGpsCharData(char* pIn, char *pValue)
{
	char validStatus[32] = {0};
	char *pRet = getGpsData(pIn, validStatus);
	if ( pRet ) {
		*pValue = validStatus[0];
	}
	return pRet;
}

char* getGpsCharDataEx(char* pIn, char *pValue)
{
	char validStatus[32] = {0};
	char *pRet = getGpsDataEx(pIn, validStatus);
	if ( pRet ) {
		*pValue = validStatus[0];
	}
	return pRet;
}

char* getGpsIntData(char* pIn, int *pValue)
{
	char latitudeValue[32] = {0};
	char *pRet = getGpsData(pIn, latitudeValue);
	if ( pRet ) {
		*pValue = atoi(latitudeValue);
	}
	return pRet;
}

char* getGpsIntDataEx(char* pIn, int *pValue)
{
	char latitudeValue[32] = {0};
	char *pRet = getGpsDataEx(pIn, latitudeValue);
	if ( pRet ) {
		*pValue = atoi(latitudeValue);
	}
	return pRet;
}

char* getGpsDoubleData(char* pIn, double *pValue)
{
	char latitudeValue[32] = {0};
	char *pRet = getGpsData(pIn, latitudeValue);
	if ( pRet ) {
		*pValue = atof(latitudeValue);
	}
	return pRet;
}

char* getGpsDoubleDataEx(char* pIn, double *pValue)
{
	char latitudeValue[32] = {0};
	char *pRet = getGpsDataEx(pIn, latitudeValue);
	if ( pRet ) {
		*pValue = atof(latitudeValue);
	}
	return pRet;
}

char* getRmcUtcTime(char* pIn, int *pHour, int *pMinute, int *pSecond) 
{
	char utcTime[32] = {0};
	char *pRet = getGpsData(pIn, utcTime);
	if ( pRet ) {
		char hour[3] = {0};
		strncpy(hour, utcTime, 2);
		hour[2] = '\0';
		*pHour = atoi(hour);

		char min[3] = {0};
		strncpy(min, &utcTime[2], 2);
		min[2] = '\0';
		*pMinute = atoi(min);

		char sec[3] = {0};
		strncpy(sec, &utcTime[4], 2);
		sec[2] = '\0';
		*pSecond = atoi(sec);
	}
	return pRet;
}

char* getRmcValid(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getRmcLatitudeValue(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getRmcLatitudeNS(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getRmcLongitudeValue(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getRmcLongitudeEW(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getRmcSpeed(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getRmcGeographyCourse(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getRmcUtcDate(char* pIn, int *pYear, int *pMonth, int *pDay) 
{
	char date[32] = {0};
	char *pRet = getGpsData(pIn, date);
	if ( pRet ) {
		char day[3] = {0};
		strncpy(day, date, 2);
		day[2] = '\0';
		*pDay = atoi(day);

		char month[3] = {0};
		strncpy(month, &date[2], 2);
		month[2] = '\0';
		*pMonth = atoi(month);

		char year[3] = {0};
		strncpy(year, &date[4], 2);
		year[2] = '\0';
		*pYear = atoi(year);
	}
	return pRet;
}

char* getRmcMagneticDeclination(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getRmcMagneticDeclinationEW(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getRmcModeEx(char* pIn, char *pValue)
{
	return getGpsCharDataEx(pIn, pValue);
}

char* getGgaUtcTime(char* pIn, int *pHour, int *pMinute, int *pSecond)
{
	char utcTime[32] = {0};
	char *pRet = getGpsData(pIn, utcTime);
	if ( pRet ) {
		char hour[3] = {0};
		strncpy(hour, utcTime, 2);
		hour[2] = '\0';
		*pHour = atoi(hour);

		char min[3] = {0};
		strncpy(min, &utcTime[2], 2);
		min[2] = '\0';
		*pMinute = atoi(min);

		char sec[3] = {0};
		strncpy(sec, &utcTime[4], 2);
		sec[2] = '\0';
		*pSecond = atoi(sec);
	}
	return pRet;
}

char* getGgaLatitudeValue(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGgaLatitudeNS(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getGgaLongitudeValue(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGgaLongitudeEW(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getGgaQuality(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGgaSatelliteNum(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
} 

char* getGgaHorizonPrecision(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGgaSeaLevel(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGgaGeoidLevel(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGgaDifferenceGps(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
} 

char* getGgaDifferenceReferenceStationLabelEx(char* pIn, int *pValue)
{
	return getGpsIntDataEx(pIn, pValue);
} 

char* getGsvPacketTotal(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
} 

char* getGsvPacketIdx(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
} 

char* getGsvSatelliteTotal(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsvPRN(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsvElevation(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsvAzimuth(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsvSNR(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsvSNREx(char* pIn, int *pValue)
{
	return getGpsIntDataEx(pIn, pValue);
}

char* getGsaMode(char* pIn, char *pValue)
{
	return getGpsCharData(pIn, pValue);
}

char* getGsaLocateType(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsaPRN(char* pIn, int *pValue)
{
	return getGpsIntData(pIn, pValue);
}

char* getGsaPDOP(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGsaHDOP(char* pIn, double *pValue)
{
	return getGpsDoubleData(pIn, pValue);
}

char* getGsaVDOPEx(char* pIn, double *pValue)
{
	return getGpsDoubleDataEx(pIn, pValue);
}
////////////////////////////////////////////////////////////////
// 初始化串口设备并进行原有设置的保存
TTY_INFO *readyTTY(const char *pSerialName)
{
	if ( !pSerialName ) {
		return NULL;
	}
	TTY_INFO *ptty;
	ptty = (TTY_INFO *)malloc(sizeof(TTY_INFO));
	if(ptty == NULL)
		return NULL;
	memset(ptty,0,sizeof(TTY_INFO));
	pthread_mutex_init(&ptty->mt,NULL);
	strcpy(ptty->name, pSerialName);
	//
	// 打开并且设置串口
	ptty->fd = open(ptty->name, O_RDWR | O_NOCTTY |O_NDELAY);
	if (ptty->fd <0)
	{
		free(ptty);
		return NULL;
	}
	//
	// 取得并且保存原来的设置
	tcgetattr(ptty->fd,&ptty->otm);
	return ptty;
}
// 清理串口设备资源
int cleanTTY(TTY_INFO *ptty)
{
	//
	// 关闭打开的串口设备
	if(ptty->fd>0) {
		tcsetattr(ptty->fd,TCSANOW,&ptty->otm);
		close(ptty->fd);
		ptty->fd = -1;
		free(ptty);
		ptty = NULL;
	}
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
// 设置串口通信速率
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针
// speed 参数类型(int),用来设置串口的波特率
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值
///////////////////////////////////////////////////////////////////////////////
int setTTYSpeed(TTY_INFO *ptty, int speed)
{
	//
	// 进行新的串口设置,数据位为8位
	bzero(&ptty->ntm, sizeof(ptty->ntm));
	tcgetattr(ptty->fd,&ptty->ntm);
	ptty->ntm.c_cflag = CLOCAL | CREAD;
	switch(speed) {
		case 300:
			ptty->ntm.c_cflag |= B300;
			break;
		case 1200:
			ptty->ntm.c_cflag |= B1200;
			break;
		case 2400:
			ptty->ntm.c_cflag |= B2400;
			break;
		case 4800:
			ptty->ntm.c_cflag |= B4800;
			break;
		case 9600:
			ptty->ntm.c_cflag |= B9600;
			break;
		case 19200:
			ptty->ntm.c_cflag |= B19200;
			break;
		case 38400:
			ptty->ntm.c_cflag |= B38400;
			break;
		case 115200:
			ptty->ntm.c_cflag |= B115200;
			break;
	}
	ptty->ntm.c_iflag = IGNPAR;
	ptty->ntm.c_oflag = 0;
	tcflush(ptty->fd, TCIFLUSH);
	tcsetattr(ptty->fd,TCSANOW,&ptty->ntm);
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
// 设置串口数据位，停止位和效验位
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针
// databits 参数类型(int), 数据位,取值为7或者8
// stopbits 参数类型(int),停止位,取值为1或者2
// parity 参数类型(int),效验类型 取值为N,E,O,,S
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值
///////////////////////////////////////////////////////////////////////////////
int setTTYParity(TTY_INFO *ptty,int databits,int parity,int stopbits)
{
	// 取得串口设置
	if( tcgetattr(ptty->fd,&ptty->ntm) != 0) {
		printf("SetupSerial [%s]-n",ptty->name);
		return 1;
	}
	bzero(&ptty->ntm, sizeof(ptty->ntm));
	ptty->ntm.c_cflag = CS8 | CLOCAL | CREAD;
	ptty->ntm.c_iflag = IGNPAR;
	ptty->ntm.c_oflag = 0;
	// 设置串口的各种参数
	ptty->ntm.c_cflag &= ~CSIZE;
	switch (databits) {//设置数据位数
		case 7:
			ptty->ntm.c_cflag |= CS7;
			break;
		case 8:
			ptty->ntm.c_cflag |= CS8;
			break;
		default:
			printf("Unsupported data size-n");
			return 5;
	}
	switch (parity) { // 设置奇偶校验位数
		case 'n':
		case 'N':
			ptty->ntm.c_cflag &= ~PARENB;
			ptty->ntm.c_iflag &= ~INPCK;
			break;
		case 'o':
		case 'O':
			ptty->ntm.c_cflag |= (PARODD|PARENB);
			ptty->ntm.c_iflag |= INPCK;
			break;
		case 'e':
		case 'E':
			ptty->ntm.c_cflag |= PARENB;
			ptty->ntm.c_cflag &= ~PARODD;
			ptty->ntm.c_iflag |= INPCK;
			break;
		case 'S':
		case 's':
			ptty->ntm.c_cflag &= ~PARENB;
			ptty->ntm.c_cflag &= ~CSTOPB;
			break;
		default:
			printf("Unsupported parity-n");
			return 2;
	}
	//
	// 设置停止位
	switch (stopbits) {
		case 1:
			ptty->ntm.c_cflag &= ~CSTOPB;
			break;
		case 2:
			ptty->ntm.c_cflag |= CSTOPB;
			break;
		default:
			printf("Unsupported stop bits-n");
			return 3;
	}
	//
	//
	ptty->ntm.c_lflag = 0;
	ptty->ntm.c_cc[VTIME] = 0;// inter-character timer unused
	ptty->ntm.c_cc[VMIN] = 1;// blocking read until 1 chars received
	tcflush(ptty->fd, TCIFLUSH);
	if (tcsetattr(ptty->fd,TCSANOW,&ptty->ntm) != 0) {
		printf("SetupSerial -n");
		return 4;
	}
	return 0;
}

