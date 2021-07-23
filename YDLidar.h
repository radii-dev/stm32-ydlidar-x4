/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR STM32F407VG
 *
 *  Copyright 2019 Gachon University, TAKE OUT, Development Group, electric engineering, Seongwook Kang.
 * 
 */

#ifndef YDLIDAR_H
#define YDLIDAR_H

#include "v8stdint.h"
#include <stdbool.h>
#include <time.h>

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x8000

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D
#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1

#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define PackageSampleBytes 2
#define PackageSampleMaxLngth 0x80
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA


static time_t start_time;
static time_t current_time;


typedef enum {
	CT_Normal = 0,
	CT_RingStart  = 1,
	CT_Tail,
}CT;

struct node_info {
	uint8_t    sync_quality;
	uint16_t   angle_q6_checkbit;
	uint16_t   distance_q2;
} __attribute__((packed));
typedef struct node_info node_info;

struct node_package {
	uint16_t  package_Head;
	uint8_t   package_CT;
	uint8_t   nowPackageNum;
	uint16_t  packageFirstSampleAngle;
	uint16_t  packageLastSampleAngle;
	uint16_t  checkSum;
	uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));
typedef struct node_package node_package;


struct device_info{
	uint8_t   model;
	uint16_t  firmware_version;
	uint8_t   hardware_version;
	uint8_t   serialnum[16];
} __attribute__((packed));
typedef struct device_info device_info;

struct device_health {
	uint8_t   status;
	uint16_t  error_code;
} __attribute__((packed));
typedef struct device_health device_health;

struct sampling_rate {
	uint8_t rate;
} __attribute__((packed));
typedef struct sampling_rate sampling_rate;

struct scan_frequency {
	uint32_t frequency;
} __attribute__((packed));
typedef struct scan_frequency scan_frequency;

struct scan_rotation {
	uint8_t rotation;
} __attribute__((packed));
typedef struct scan_rotation scan_rotation;

struct cmd_packet {
	uint8_t syncByte;
	uint8_t cmd_flag;
	uint8_t size;
	uint8_t data;
} __attribute__((packed));
typedef struct cmd_packet cmd_packet;

struct lidar_ans_header {
	uint8_t  syncByte1;
	uint8_t  syncByte2;
	uint32_t size:30;
	uint32_t subType:2;
	uint8_t  type;
} __attribute__((packed));
typedef struct lidar_ans_header lidar_ans_header;

typedef struct {
	uint8_t quality;
	float 	angle;
	float 	distance;
	bool    startBit;
} scanPoint;
static scanPoint point;


#if defined(_WIN32)
#pragma pack(1)
#endif

// ask the YDLIDAR for its health info
result_t getHealth(device_health * health, uint32_t timeout);

// ask the YDLIDAR for its device info like the serial number
result_t getDeviceInfo(device_info * info, uint32_t timeout);

// stop the scanPoint operation
result_t stop(void);

// start the scanPoint operation
result_t startScan(bool force, uint32_t timeout);

// wait for one sample package to arrive
result_t waitScanDot(uint32_t timeout);

// retrieve currently received sample point
static scanPoint getCurrentScanPoint(void)
{
		return point;
}

// send ask commond to YDLIDAR
result_t sendCommand(uint8_t cmd, const void * payload, size_t payloadsize);
//wait for response header to arrive
result_t waitResponseHeader(lidar_ans_header * header, uint32_t timeout);

#endif
