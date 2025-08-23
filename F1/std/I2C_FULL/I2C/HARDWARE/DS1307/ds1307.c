/**	
 * |----------------------------------------------------------------------
 
 * |----------------------------------------------------------------------
 */
#include "ds1307.h"

void DS1307_Init(void)
{
	IIC_Init(I2C1, Pin_PB8PB9, 100000);//IIC_Init();
	//if(I2C_IsDeviceConnected(I2C1, 0xD0)) 
}

u8 RTC_Bcd2Bin(u8 bcd) {
	
	return 10 * (bcd >> 4)+(bcd & 0x0F);
}

u8 RTC_Bin2Bcd(u8 bin) {
		
	return (bin / 10) << 4 | (bin % 10);
}
u8 RTC_CheckMinMax(u8 val, u8 min, u8 max) {
	if (val < min) {
		return min;
	} else if (val > max) {
		return max;
	}
	return val;
}


void RTC_GetDateTime(u8 *gio,u8 *phut,u8 *giay,u8 *thu,u8 *ngay,u8 *thang,u8 *nam) {
	u8 data[7];	u8 res;	
	res = I2C_ReadMulti(I2C1, DS1307_I2C_ADDR, 0x00, data,7);//tu ADD reg=0x00
	if(res==0)
	{
	*giay = RTC_Bcd2Bin(data[0]);
	*phut = RTC_Bcd2Bin(data[1]);
	*gio = RTC_Bcd2Bin(data[2]);
	*thu = RTC_Bcd2Bin(data[3]);
	*ngay = RTC_Bcd2Bin(data[4]);
	*thang = RTC_Bcd2Bin(data[5]);
	*nam = RTC_Bcd2Bin(data[6]);
	}
}
void RTC_SetDateTime(u8 gio,u8 phut,u8 giay,u8 thu,u8 ngay,u8 thang,u8 nam) {
	uint8_t data[7];
	
	data[0] = RTC_Bin2Bcd(RTC_CheckMinMax(giay, 0, 59));
	data[1] = RTC_Bin2Bcd(RTC_CheckMinMax(phut, 0, 59));
	data[2] = RTC_Bin2Bcd(RTC_CheckMinMax(gio, 0, 23));
	data[3] = RTC_Bin2Bcd(RTC_CheckMinMax(thu, 1, 7));
	data[4] = RTC_Bin2Bcd(RTC_CheckMinMax(ngay, 1, 31));
	data[5] = RTC_Bin2Bcd(RTC_CheckMinMax(thang, 1, 12));
	data[6] = RTC_Bin2Bcd(RTC_CheckMinMax(nam, 0, 99));
	
	I2C_WriteMulti(I2C1, DS1307_I2C_ADDR, 0x00, data,7);
}



