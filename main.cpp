#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include "gpsParse.cpp"

void onGpsInfo(const pstGpsInfo pInfo)
{
	printf("onGpsInfo,dateTime:%d-%d-%d_%d:%d:%d,valid:%c,latitude:%f,NS:%c,longtitude:%f,EW:%c,speed:%f,high:%f,satelliteNum:%d\n",\
			pInfo->D.year, pInfo->D.month, pInfo->D.day, pInfo->D.hour, pInfo->D.minute, pInfo->D.second,\
		   	pInfo->status, pInfo->latitude, pInfo->NS, pInfo->longitude, pInfo->EW, pInfo->speed, pInfo->high, pInfo->satelliteNum);
}

void onSatelliteInfo(const pstSatelliteInfo pInfo)
{
	printf("onSatelliteInfo,total:%d,PRN:%d,azimuth:%d,elevation:%d,SNR:%d\n", \
			pInfo->total, pInfo->PRN, pInfo->azimuth, pInfo->elevation, pInfo->SNR);
}

int main(int argc, char* argv[])
{
	if ( argc != 2 ) {
		printf("usage method:\n");
		printf("%s serialName\n", argv[0]);
		return -1;
	}
	printf("---gps serial name:%s\n", argv[1]);
	stGpsCallbackInfo cb;
	cb.onGpsInfo = onGpsInfo;
	cb.onSatelliteInfo = onSatelliteInfo;
	init(argv[1], &cb);
	start();
	bool bExit = false;
	int item = 0;
	while ( !bExit ) {
		printf("gpsParseTest item: \n");
		printf("\t1. remind;\n");
		printf("\t2. exit;\n");
		printf(">>");
		std::cin >> item;

		switch(item){
			case 1: break;
			case 2:
				bExit = true;
				break;
			default: break;
		}
	}
	uninit();
	return 0;
}

