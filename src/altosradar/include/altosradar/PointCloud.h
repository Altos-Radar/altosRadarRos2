/*
 * @Copyright (C) 2024 by Altos Radar. All rights reserved.
 * @Author: Fengze Han
 * @Date: 2024-05-29 16:19:39
 * @Description:
 */

#include <stdio.h>
#pragma pack(push, 1)
#define POINTNUM 30
typedef struct DETECTION {
    short rangeIdx;
    short dopplerIdx;
    short aziIdx;
    short eleIdx;
    float range;
    float doppler;
    float azi;
    float ele;
    float snr;
    float rangeVar;
    float dopplerVar;
    float aziVar;
    float eleVar;
} DETECTION;
typedef struct PCKHEADER {
    unsigned int header; // "0xabcd4321",
    unsigned char version;
    unsigned char mode; // long or short distance
    unsigned char reserved[2];
    unsigned int sec;
    unsigned int nsec;
    unsigned int frameId;
    unsigned int objectCount;
    unsigned short curObjInd;
    unsigned short curObjNum;
} PCKHEADER;

typedef struct SECHEADER {
    unsigned int sectionType;
    unsigned int sectionLength;
} SECHEADER;
typedef struct SYSINFO {
    float rangeRes;
    float dopplerRes;
    unsigned short numRangeBins;
    unsigned short numDopplerBins;
    unsigned char numSensors;
    unsigned char numTxAnt;
    unsigned char numRxAnt;
    unsigned char numTxAzimuthAnt;
    unsigned char numTxElevationAnt;
    unsigned char padding[3U];
} SYSINFO;
typedef struct POINTCLOUD {
    PCKHEADER pckHeader;
    SYSINFO sysInfo;
    DETECTION point[POINTNUM];
} POINTCLOUD;
#pragma pack(pop)
