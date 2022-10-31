#ifndef __DY_CONVERTCOORDINATE_H
#define __DY_CONVERTCOORDINATE_H

typedef  struct tagINAngle{
		float Yaw;
		float Pitch;
		float Roll;
} INAngle;

typedef  struct tagAxisAngle{
		float Azimuth;
		float Pitch;
} AxisAngle;

typedef  struct tagXYZCoor{
		float X;
		float Y;
		float Z;
} XYZCoor;

typedef  struct tagSpaceAngle{
		float Azimuth;
		float Pitch;
} SpaceAngle;

typedef  struct tagTwoAngle{
		float Azimuth;
		float Pitch;
} TwoAngle;




TwoAngle ConvertIN2Axis(TwoAngle myTwoAngle);
TwoAngle ConvertAxis2IN(TwoAngle myTwoAngle);
XYZCoor TransCoordinate1(XYZCoor oldXYZ);
XYZCoor TransCoordinate2(XYZCoor oldXYZ);
XYZCoor Angle2XYZ(TwoAngle twoAngle);
TwoAngle XYZ2Angle(XYZCoor myXYZ);


void AskInvMatrix(void);

void askInMatrix(void );
float Rad2dre(float x);
float Dre2rad(float x);
void AskConvMatrix(float angle);
XYZCoor TransleanCoordinate1(XYZCoor oldXYZ);
XYZCoor TransleanCoordinate2(XYZCoor oldXYZ);
#endif
