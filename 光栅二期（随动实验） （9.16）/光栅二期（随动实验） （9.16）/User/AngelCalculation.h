#ifndef __ANGELCALCULATION_H
#define __ANGELCALCULATION_H

typedef struct AxisFW_AxisFY
{
	float FW;
	float FY;
}AxisFW_AxisFY;

AxisFW_AxisFY AngelCalculation(float Azimuth, float Roll, float Pitch, float PointToFW, float PointToFY, float NewAngle);


#endif
