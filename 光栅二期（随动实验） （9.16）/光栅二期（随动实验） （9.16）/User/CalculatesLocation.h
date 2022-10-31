#ifndef CALCULATESDATA_H
#define CALCULATESDATA_H

#define M_PI  (3.14159265358979323846)
typedef struct gCalculatesData
{
	float x;
	float y;
	float z;
} CalculatesData;

CalculatesData CalculatesLocation(float azimuth, float pitch, float roll, float AxisPh, float AxisFy, float NewAngle);

#endif