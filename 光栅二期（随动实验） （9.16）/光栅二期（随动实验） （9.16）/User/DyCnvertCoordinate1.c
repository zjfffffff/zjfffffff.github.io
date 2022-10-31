#include "DyConvertCoordinate1.h"     
#include "math.h"
#include "CalculatesLocation.h"
#include "stm32f4xx.h"	
#define COS_A 0.1
#define COS_B -0.1
#define SIN_A 0.2
#define SIN_B -0.2
#define PI 3.141593
//A位正，B为负
XYZCoor myXYZ;
XYZCoor myXYZ1;
XYZCoor myXYZ2;
XYZCoor myXYZ3;

float yawArray1[3][3]   ={{COS_A,    SIN_A,     (float)0},
                          {SIN_B,    COS_A,     (float)0},
                          {(float)0, (float)0,  (float)1}};
float pitchArray1[3][3] ={{(float)1, (float)0,  (float)0},
                          {(float)0,  COS_A,     SIN_A},
                          {(float)0,  SIN_B,     COS_A}};
float rollArray1[3][3]  ={{COS_A,     (float)0,  SIN_B},
                          {(float)0,  (float)1,  (float)0},
                          {SIN_A,     (float)0,  COS_A}};
float pitchLeanArray[3][3] ={{(float)1, (float)0,  (float)0},
                          {(float)0,  COS_A,     SIN_A},
                          {(float)0,  SIN_B,     COS_A}};
float yawArray2[3][3];
float pitchArray2[3][3];
float rollArray2[3][3];
						  
float inMatrixArray1[3][3];
float inMatrixArray[3][3];
float inMatrixInvArray[3][3];			
						  
float leanMatrixInvArray[3][3];	
						  

	
void askInvMatrix(void)
{
	inMatrixInvArray[0][0]=inMatrixArray[0][0];  inMatrixInvArray[0][1]=inMatrixArray[1][0] ;  inMatrixInvArray[0][2]=inMatrixArray[2][0] ;
	inMatrixInvArray[1][0]=inMatrixArray[0][1];  inMatrixInvArray[1][1]=inMatrixArray[1][1] ;  inMatrixInvArray[1][2]=inMatrixArray[2][1] ;
	inMatrixInvArray[2][0]=inMatrixArray[0][2];  inMatrixInvArray[2][1]=inMatrixArray[1][2] ;  inMatrixInvArray[2][2]=inMatrixArray[2][2] ;
	
	leanMatrixInvArray[0][0]=pitchLeanArray[0][0];  leanMatrixInvArray[0][1]=pitchLeanArray[1][0] ;  leanMatrixInvArray[0][2]=pitchLeanArray[2][0] ;
	leanMatrixInvArray[1][0]=pitchLeanArray[0][1];  leanMatrixInvArray[1][1]=pitchLeanArray[1][1] ;  leanMatrixInvArray[1][2]=pitchLeanArray[2][1] ;
	leanMatrixInvArray[2][0]=pitchLeanArray[0][2];  leanMatrixInvArray[2][1]=pitchLeanArray[1][2] ;  leanMatrixInvArray[2][2]=pitchLeanArray[2][2] ;
	
	
	
}
void askInMatrix(void )
{
	
	
	inMatrixArray1[0][0]=rollArray1[0][0]*pitchArray1[0][0]+rollArray1[0][1]*pitchArray1[1][0]+rollArray1[0][2]*pitchArray1[2][0];
	inMatrixArray1[0][1]=rollArray1[0][0]*pitchArray1[0][1]+rollArray1[0][1]*pitchArray1[1][1]+rollArray1[0][2]*pitchArray1[2][1];
	inMatrixArray1[0][2]=rollArray1[0][0]*pitchArray1[0][2]+rollArray1[0][1]*pitchArray1[1][2]+rollArray1[0][2]*pitchArray1[2][2];
	
	inMatrixArray1[1][0]=rollArray1[1][0]*pitchArray1[0][0]+rollArray1[1][1]*pitchArray1[1][0]+rollArray1[1][2]*pitchArray1[2][0];
	inMatrixArray1[1][1]=rollArray1[1][0]*pitchArray1[0][1]+rollArray1[1][1]*pitchArray1[1][1]+rollArray1[1][2]*pitchArray1[2][1];
	inMatrixArray1[1][2]=rollArray1[1][0]*pitchArray1[0][2]+rollArray1[1][1]*pitchArray1[1][2]+rollArray1[1][2]*pitchArray1[2][2];
	
	inMatrixArray1[2][0]=rollArray1[2][0]*pitchArray1[0][0]+rollArray1[2][1]*pitchArray1[1][0]+rollArray1[2][2]*pitchArray1[2][0];
	inMatrixArray1[2][1]=rollArray1[2][0]*pitchArray1[0][1]+rollArray1[2][1]*pitchArray1[1][1]+rollArray1[2][2]*pitchArray1[2][1];
	inMatrixArray1[2][2]=rollArray1[2][0]*pitchArray1[0][2]+rollArray1[2][1]*pitchArray1[1][2]+rollArray1[2][2]*pitchArray1[2][2];
	
	
	inMatrixArray[0][0]=inMatrixArray1[0][0]*yawArray1[0][0]+inMatrixArray1[0][1]*yawArray1[1][0]+inMatrixArray1[0][2]*yawArray1[2][0];
	inMatrixArray[0][1]=inMatrixArray1[0][0]*yawArray1[0][1]+inMatrixArray1[0][1]*yawArray1[1][1]+inMatrixArray1[0][2]*yawArray1[2][1];
	inMatrixArray[0][2]=inMatrixArray1[0][0]*yawArray1[0][2]+inMatrixArray1[0][1]*yawArray1[1][2]+inMatrixArray1[0][2]*yawArray1[2][2];
	
	inMatrixArray[1][0]=inMatrixArray1[1][0]*yawArray1[0][0]+inMatrixArray1[1][1]*yawArray1[1][0]+inMatrixArray1[1][2]*yawArray1[2][0];
	inMatrixArray[1][1]=inMatrixArray1[1][0]*yawArray1[0][1]+inMatrixArray1[1][1]*yawArray1[1][1]+inMatrixArray1[1][2]*yawArray1[2][1];
	inMatrixArray[1][2]=inMatrixArray1[1][0]*yawArray1[0][2]+inMatrixArray1[1][1]*yawArray1[1][2]+inMatrixArray1[1][2]*yawArray1[2][2];
	
	inMatrixArray[2][0]=inMatrixArray1[2][0]*yawArray1[0][0]+inMatrixArray1[2][1]*yawArray1[1][0]+inMatrixArray1[2][2]*yawArray1[2][0];
	inMatrixArray[2][1]=inMatrixArray1[2][0]*yawArray1[0][1]+inMatrixArray1[2][1]*yawArray1[1][1]+inMatrixArray1[2][2]*yawArray1[2][1];
	inMatrixArray[2][2]=inMatrixArray1[2][0]*yawArray1[0][2]+inMatrixArray1[2][1]*yawArray1[1][2]+inMatrixArray1[2][2]*yawArray1[2][2];
	
	
				
}

float Rad2dre(float x)
{
	float degree;
	float myPI=PI;
	float semiCir=180;	
	degree=(x*semiCir)/myPI;
    return degree;
}
float Dre2rad(float x)
{
	float radian;
	float myPI=PI;
	float semiCir=180;
	radian=(x*myPI)/semiCir;
	return radian;
}

float cosR,sinR,cosP,sinP,cosY,sinY,cosAngle,sinAngle;

/*
AxisAngle ConvertIN2Axis(INAngle inAngle, SpaceAngle spaceAngle, float angle)
{
		
		float myRad;
		float mode;
		
		AxisAngle myAxisAngle;
		INAngle inAngleRad;		
		SpaceAngle spaceAngleRad;
	
		inAngleRad.Yaw=Dre2rad(inAngle.Yaw);
		inAngleRad.Pitch=Dre2rad(inAngle.Pitch);
		inAngleRad.Roll=Dre2rad(inAngle.Roll);
		
		spaceAngleRad.Azimuth=Dre2rad(spaceAngle.Azimuth);
		spaceAngleRad.Pitch=Dre2rad(spaceAngle.Pitch);
		
		myRad=Dre2rad(angle);
		
		cosR=cosf(inAngleRad.Roll);
		sinR=sinf(inAngleRad.Roll);
		cosP=cosf(inAngleRad.Pitch);
		sinP=sinf(inAngleRad.Pitch);
		cosY=cosf(inAngleRad.Yaw);
		sinY=sinf(inAngleRad.Yaw);
	
		
		yawArray1[0][0]=cosY;
		yawArray1[0][1]=sinY;
		yawArray1[1][0]=-sinY;
		yawArray1[1][1]=cosY;
		
		pitchArray1[1][1]=cosP;
		pitchArray1[1][2]=sinP;
		pitchArray1[2][1]=-sinP;
		pitchArray1[2][2]=cosP;

		rollArray1[0][0]=cosR;
		rollArray1[0][2]=-sinR;
		rollArray1[2][0]=sinR;
		rollArray1[2][2]=cosR;				  
		

		
//		spaceAngleRad.Azimuth=Dre2rad(spaceAngle.Azimuth);
//		spaceAngleRad.Pitch=Dre2rad(spaceAngle.Pitch);
		
		
		myXYZ.X=cosf(spaceAngleRad.Pitch)*sinf(-spaceAngleRad.Azimuth);
		myXYZ.Y=cosf(spaceAngleRad.Pitch)*cosf(-spaceAngleRad.Azimuth);
		myXYZ.Z=sinf(spaceAngleRad.Pitch);
		
		
		myXYZ1.X=yawArray1[0][0]*myXYZ.X+yawArray1[0][1]*myXYZ.Y+yawArray1[0][2]*myXYZ.Z;
		myXYZ1.Y=yawArray1[1][0]*myXYZ.X+yawArray1[1][1]*myXYZ.Y+yawArray1[1][2]*myXYZ.Z;
		myXYZ1.Z=yawArray1[2][0]*myXYZ.X+yawArray1[2][1]*myXYZ.Y+yawArray1[2][2]*myXYZ.Z;
		
		myXYZ2.X=pitchArray1[0][0]*myXYZ1.X+pitchArray1[0][1]*myXYZ1.Y+pitchArray1[0][2]*myXYZ1.Z;
		myXYZ2.Y=pitchArray1[1][0]*myXYZ1.X+pitchArray1[1][1]*myXYZ1.Y+pitchArray1[1][2]*myXYZ1.Z;
		myXYZ2.Z=pitchArray1[2][0]*myXYZ1.X+pitchArray1[2][1]*myXYZ1.Y+pitchArray1[2][2]*myXYZ1.Z;
		
		myXYZ3.X=rollArray1[0][0]*myXYZ2.X+rollArray1[0][1]*myXYZ2.Y+rollArray1[0][2]*myXYZ2.Z;
		myXYZ3.Y=rollArray1[1][0]*myXYZ2.X+rollArray1[1][1]*myXYZ2.Y+rollArray1[1][2]*myXYZ2.Z;
		myXYZ3.Z=rollArray1[2][0]*myXYZ2.X+rollArray1[2][1]*myXYZ2.Y+rollArray1[2][2]*myXYZ2.Z;
		
		mode=sqrt(myXYZ3.X*myXYZ3.X+myXYZ3.Y*myXYZ3.Y+myXYZ3.Z*myXYZ3.Z);
		
		return myAxisAngle;	
}
*/
float myWatch1=0;
float myWatch2=0;
float myWatch3=0;
extern INAngle inAngle;
void AskConvMatrix(float angle)
{
	float myRad;
	INAngle inAngleRad;		
	
	inAngleRad.Yaw=Dre2rad(inAngle.Yaw);
	inAngleRad.Pitch=Dre2rad(inAngle.Pitch);
	inAngleRad.Roll=Dre2rad(inAngle.Roll);
	myRad=Dre2rad(angle);
		
	cosR=cosf(inAngleRad.Roll);
	sinR=sinf(inAngleRad.Roll);
	cosP=cosf(inAngleRad.Pitch);
	sinP=sinf(inAngleRad.Pitch);
	cosY=cosf(inAngleRad.Yaw);
	sinY=sinf(inAngleRad.Yaw);
	cosAngle=cosf(myRad);
	sinAngle=sinf(myRad);
	
	
	
	yawArray1[0][0]=cosY;
	yawArray1[0][1]=sinY;
	yawArray1[1][0]=-sinY;
	yawArray1[1][1]=cosY;

	pitchArray1[1][1]=cosP;
	pitchArray1[1][2]=sinP;
	pitchArray1[2][1]=-sinP;
	pitchArray1[2][2]=cosP;

	rollArray1[0][0]=cosR;
	rollArray1[0][2]=-sinR;
	rollArray1[2][0]=sinR;
	rollArray1[2][2]=cosR;	
	
	pitchLeanArray[1][1]=cosAngle;
	pitchLeanArray[1][2]=sinAngle;
	pitchLeanArray[2][1]=-sinAngle;
	pitchLeanArray[2][2]=cosAngle;
	
	askInMatrix();
	askInvMatrix();
	
//	x=rollArray1[0][1];
//	rollArray1[0][1]=rollArray1[1][0];
//	rollArray1[1][0]=x;
//	x=rollArray1[0][2];
//	rollArray1[0][2]=rollArray1[2][0];
//	rollArray1[2][0]=x;
//	x=rollArray1[1][2];
//	rollArray1[1][2]=rollArray1[2][1];
//	rollArray1[2][1]=x;
//	
//	x=pitchArray1[0][1];
//	pitchArray1[0][1]=pitchArray1[1][0];
//	pitchArray1[1][0]=x;
//	x=pitchArray1[0][2];
//	pitchArray1[0][2]=pitchArray1[2][0];
//	pitchArray1[2][0]=x;
//	x=pitchArray1[1][2];
//	pitchArray1[1][2]=pitchArray1[2][1];
//	pitchArray1[2][1]=x;
//	
//	x=yawArray1[0][1];
//	yawArray1[0][1]=yawArray1[1][0];
//	yawArray1[1][0]=x;
//	x=yawArray1[0][2];
//	yawArray1[0][2]=yawArray1[2][0];
//	yawArray1[2][0]=x;
//	x=yawArray1[1][2];
//	yawArray1[1][2]=yawArray1[2][1];
//	yawArray1[2][1]=x;
	
	
	
	
}

XYZCoor TransCoordinate1(XYZCoor oldXYZ) //正矩阵转
{
	XYZCoor newXYZ;

	myXYZ1.X=inMatrixArray[0][0]*oldXYZ.X+inMatrixArray[0][1]*oldXYZ.Y+inMatrixArray[0][2]*oldXYZ.Z;
	myXYZ1.Y=inMatrixArray[1][0]*oldXYZ.X+inMatrixArray[1][1]*oldXYZ.Y+inMatrixArray[1][2]*oldXYZ.Z;
	myXYZ1.Z=inMatrixArray[2][0]*oldXYZ.X+inMatrixArray[2][1]*oldXYZ.Y+inMatrixArray[2][2]*oldXYZ.Z;
	
	newXYZ=myXYZ1;
	
	return newXYZ;
}
XYZCoor TransCoordinate2(XYZCoor oldXYZ) //逆矩阵转
{
	XYZCoor newXYZ;

	myXYZ1.X=inMatrixInvArray[0][0]*oldXYZ.X+inMatrixInvArray[0][1]*oldXYZ.Y+inMatrixInvArray[0][2]*oldXYZ.Z;
	myXYZ1.Y=inMatrixInvArray[1][0]*oldXYZ.X+inMatrixInvArray[1][1]*oldXYZ.Y+inMatrixInvArray[1][2]*oldXYZ.Z;
	myXYZ1.Z=inMatrixInvArray[2][0]*oldXYZ.X+inMatrixInvArray[2][1]*oldXYZ.Y+inMatrixInvArray[2][2]*oldXYZ.Z;
	
	newXYZ=myXYZ1;
	return newXYZ;
}
XYZCoor TransleanCoordinate1(XYZCoor oldXYZ)//正矩阵转
{
	XYZCoor newXYZ;

	myXYZ1.X=pitchLeanArray[0][0]*oldXYZ.X+pitchLeanArray[0][1]*oldXYZ.Y+pitchLeanArray[0][2]*oldXYZ.Z;
	myXYZ1.Y=pitchLeanArray[1][0]*oldXYZ.X+pitchLeanArray[1][1]*oldXYZ.Y+pitchLeanArray[1][2]*oldXYZ.Z;
	myXYZ1.Z=pitchLeanArray[2][0]*oldXYZ.X+pitchLeanArray[2][1]*oldXYZ.Y+pitchLeanArray[2][2]*oldXYZ.Z;

	newXYZ=myXYZ1;
	return newXYZ;
}
XYZCoor TransleanCoordinate2(XYZCoor oldXYZ) //逆矩阵转
{
	XYZCoor newXYZ;

	myXYZ1.X=leanMatrixInvArray[0][0]*oldXYZ.X+leanMatrixInvArray[0][1]*oldXYZ.Y+leanMatrixInvArray[0][2]*oldXYZ.Z;
	myXYZ1.Y=leanMatrixInvArray[1][0]*oldXYZ.X+leanMatrixInvArray[1][1]*oldXYZ.Y+leanMatrixInvArray[1][2]*oldXYZ.Z;
	myXYZ1.Z=leanMatrixInvArray[2][0]*oldXYZ.X+leanMatrixInvArray[2][1]*oldXYZ.Y+leanMatrixInvArray[2][2]*oldXYZ.Z;

	newXYZ=myXYZ1;
	return newXYZ;
}
XYZCoor Angle2XYZ(TwoAngle twoAngle)
{
	 XYZCoor myXYZ;
	TwoAngle twoAngleRad;
	
	twoAngleRad.Azimuth=Dre2rad(twoAngle.Azimuth);
	twoAngleRad.Pitch=Dre2rad(twoAngle.Pitch);

	myXYZ.X=cosf(twoAngleRad.Pitch)*sinf(-twoAngleRad.Azimuth);
	myXYZ.Y=cosf(twoAngleRad.Pitch)*cosf(-twoAngleRad.Azimuth);
	myXYZ.Z=sinf(twoAngleRad.Pitch);
	
	return myXYZ;
}
TwoAngle XYZ2Angle(XYZCoor myXYZ)
{
    TwoAngle myTwoAngle;
	TwoAngle myTwoAngleRad;
	float mode=1;
	float x,y,z;
	myTwoAngleRad.Pitch=asinf(myXYZ.Z/mode);

	x=cosf(myTwoAngleRad.Pitch);
	y=myXYZ.X/x;
	myTwoAngleRad.Azimuth=-asinf(y);

	myTwoAngle.Pitch=Rad2dre(myTwoAngleRad.Pitch);
	myTwoAngle.Azimuth=Rad2dre(myTwoAngleRad.Azimuth);
	return myTwoAngle;
}

TwoAngle ConvertAxis2IN(TwoAngle myTwoAngle)
{
		
	TwoAngle mySpaceAngle;
	XYZCoor newXYZ,myXYZ1,myXYZ2;

	myXYZ1=Angle2XYZ(myTwoAngle);
	myXYZ2= TransleanCoordinate2(myXYZ1);

	newXYZ=TransCoordinate2(myXYZ2);

	mySpaceAngle= XYZ2Angle(newXYZ);

	return mySpaceAngle;	
}
XYZCoor watchXYZ1;
TwoAngle ConvertIN2Axis(TwoAngle myTwoAngle)
{
	TwoAngle myAxisAngle;
	XYZCoor newXYZ,myXYZ1,myXYZ2;

	myXYZ1=Angle2XYZ(myTwoAngle);
	//watchXYZ1=myXYZ1;
	
	myXYZ2=TransCoordinate1(myXYZ1);
	watchXYZ1=myXYZ2;
	
	newXYZ= TransleanCoordinate1(myXYZ2);

	

	myAxisAngle= XYZ2Angle(newXYZ);

	return myAxisAngle;	
}