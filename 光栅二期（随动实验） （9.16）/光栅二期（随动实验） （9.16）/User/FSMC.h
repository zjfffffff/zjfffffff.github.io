#ifndef _FSMC_H
#define _FSMC_H
#include "stm32f4xx.h"	


#define Bank1_SRAM3_ADDR    ((u32)(0x64000000))

#define JingSi_ComWReadADDR        ((u32)(0x64004000))   
#define JingTu_ComWReadADDR        ((u32)(0x64003000))  
#define CuSi_ComWReadADDR          ((u32)(0x64005000))
#define PC_ComWReadADDR            ((u32)(0x6400a000))  
#define PC_ComReadADDRFlag         ((u32)(0x6400F800))    //(0x6400F800)F000
#define PC_TuoBaADDRFlag           ((u32)(0x6400F000))


#define JingSi_ComWriteEnable      ((u32)(0x64004800))
#define JingTu_ComWriteEnable      ((u32)(0x64003800))
#define CuSi_ComWriteEnable        ((u32)(0x64005800))
#define PC_ComWriteEnable          ((u32)(0x6400a800))


#define PFPGA_WRITE_TEMP1          ((u32)(0x6400a000))
#define TEST_ComWriteEnable       ((u32)(0x6400a800))

#define PFPGA_WRITE_USART1        ((u32)(0x64001000))
#define TEST0_ComWriteEnable      ((u32)(0x64001800))

#define PFPGA_WRITE_USART2        ((u32)(0x64002800))		 //读取惯导
#define TEST1_ComWriteEnable      ((u32)(0x64002800))

#define PFPGA_WRITE_USART3        ((u32)(0x64003000))
#define TEST2_ComWriteEnable      ((u32)(0x64003800))

#define PFPGA_WRITE_USART4        ((u32)(0x64004000))
#define TEST2_ComWriteEnable      ((u32)(0x64003800))

#define PFPGA_WRITE_TEST3         ((u32)(0x64009000))
#define TEST3_ComWriteEnable      ((u32)(0x64004800))

#define PFPGA_WRITE_USART5        ((u32)(0x64001800))    //读取相机脱靶量
#define TEST4_ComWriteEnable      ((u32)(0x64005800))

#define PFPGA_WRITE_TEST5         ((u32)(0x64006000))
#define TEST5_ComWriteEnable      ((u32)(0x64006800))
#define PFPGA_WRITE_TEST6         ((u32)(0x64007000))
#define TEST6_ComWriteEnable      ((u32)(0x64007800))
//#define PFPGA_WRITE_TEST7         ((u32)(0x64008000))
#define TEST7_ComWriteEnable      ((u32)(0x64008800))
#define PFPGA_WRITE_TEST8         ((u32)(0x64009000))
#define TEST8_ComWriteEnable      ((u32)(0x64009800))
#define PFPGA_WRITE_TEST9         ((u32)(0x6400a000))
#define TEST9_ComWriteEnable      ((u32)(0x6400a800))

#define PFPGA_WRITE_LVDS          ((u32)(0x6400B000))


#define PFPGA_WRITE_TEMP2          ((u32)(0x6400D000))

#define DA1H      ((u32)(0x64007100))
#define DA1L      ((u32)(0x64007200))
#define DA2H      ((u32)(0x64007300))
#define DA2L      ((u32)(0x64007400))
#define DA1_EN   ((u32)(0x6400BB00))
#define DA21      ((u32)(0x6400B900))		//BA00
#define DA22      ((u32)(0x6400BA00))		//BA00
#define DA2_EN   ((u32)(0x6400BC00))	//BA00H
void bsp_InitFsmc(void);//FSMC存储控制器初始化

#endif
