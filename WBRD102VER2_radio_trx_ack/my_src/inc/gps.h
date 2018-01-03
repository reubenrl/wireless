/*
 * gps.h
 *
 *  Created on: Nov 17, 2016
 *      Author: phytech
 */

#ifndef MY_SRC_INC_GPS_H_
#define MY_SRC_INC_GPS_H_

// constants/macros/typdefs
typedef union union_float_u32
{
	float f;
	unsigned long i;
	unsigned char b[4];
} float_u32;

typedef union union_double_u64
{
	double f;
	unsigned long long i;
	unsigned char b[8];
} double_u64;

struct PositionLLA
{
	float_u32 lat;
	float_u32 lon;
	float_u32 alt;
	float_u32 TimeOfFix;
	uint16_t updates;
};

struct VelocityENU
{
	float_u32 east;
	float_u32 north;
	float_u32 up;
	float_u32 TimeOfFix;
	uint16_t updates;
};

struct VelocityHS
{
	float_u32 heading;
	float_u32 speed;
	float_u32 TimeOfFix;
	uint16_t updates;
};

struct PositionECEF
{
	float_u32 x;
	float_u32 y;
	float_u32 z;
	float_u32 TimeOfFix;
	uint16_t updates;
};

struct VelocityECEF
{
	float_u32 x;
	float_u32 y;
	float_u32 z;
	float_u32 TimeOfFix;
	uint16_t updates;
};

typedef struct struct_GpsInfo
{
	float_u32 TimeOfWeek;
	uint16_t WeekNum;
	float_u32 UtcOffset;
	uint8_t numSVs;

	struct PositionLLA PosLLA;
	struct PositionECEF PosECEF;
	struct VelocityECEF VelECEF;
	struct VelocityENU VelENU;
	struct VelocityHS VelHS;

} GpsInfoType;

// functions
void gpsInit(void);
GpsInfoType* gpsGetInfo(void);
void gpsInfoPrint(void);

void gps_bypassUart(void);

#endif /* MY_SRC_INC_GPS_H_ */
