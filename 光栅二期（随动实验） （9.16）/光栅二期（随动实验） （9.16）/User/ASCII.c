#include "ASCII.h"

/**
  * @brief	ACSII --> 16 hex
  */
unsigned char Char_To_Hex(unsigned char bHex)
{
	if((bHex>=0)&&(bHex<=9))
	{
		bHex += 0x30;
	}
	else if((bHex>=10)&&(bHex<=15))//Capital
	{
		bHex += 0x37;
	}
	else
	{
		bHex = 0xff;
	}
	return bHex;
}

/**
  * @brief	16 hex --> ACSII
  */
unsigned char Hex_To_Char(unsigned char bChar)
{
	if((bChar>=0x30)&&(bChar<=0x39))
	{
		 bChar -= 0x30;
	}
	else if((bChar>=0x41)&&(bChar<=0x46))// Capital
	{
		 bChar -= 0x37;
	}
	else if((bChar>=0x61)&&(bChar<=0x66))//littlecase
	{
		 bChar -= 0x57;
	}
	else
	{
		 bChar = 0xff;
	}
	return bChar;
}
