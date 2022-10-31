

#define PC_Com_Flag			0x8000
#define PC_ADDR             ((u32)(0x64004000))
#define PC_Com_En           ((u32)(0x64004800))

#define PC_TX_Len 34
#define PC_RX_Len 42



void PC_Com_Read();
void PC_Data_TX();
void PC_Com_Write();
