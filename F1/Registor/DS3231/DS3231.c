
#include "DS3231.h"


uint8_t BCDtoBIN(uint8_t bcd){
return 10*(bcd>>4) + (bcd&0x0f);

}

uint8_t BINtoBCD(uint8_t bin){
return ((bin/10)<<4) + (bin%10);
}


uint8_t RTC_CheckMinMax(uint8_t val, uint8_t min, uint8_t max) {
	if (val < min) {
		return min;
	} else if (val > max) {
		return max;
	}
	return val;
}

uint8_t Read_SS(){

return BCDtoBIN(I2Cx_ReadData(I2C1, 0x68<<1, 0x00));
}



void DS3231_Init(){
I2Cx_Init(I2C1, PB6PB7 , 100000);

}

void DS3231Read(uint8_t *HH, uint8_t *MM, uint8_t *SS,uint8_t *Date ,uint8_t *D,uint8_t *M,uint8_t *Y){
uint8_t data[7];
	
I2Cx_ReadMulti(I2C1, 0x68<<1, 0x00, data, 7);
	
	*HH=BCDtoBIN(data[2]); *MM=BCDtoBIN(data[1]); *SS=BCDtoBIN(data[0]); *Date =BCDtoBIN(data[3]);*D=BCDtoBIN(data[4]);*M=BCDtoBIN(data[5]);*Y=BCDtoBIN(data[6]);
	
}
void DS3231Set(uint8_t HH, uint8_t MM, uint8_t SS,uint8_t Date ,uint8_t D,uint8_t M,uint8_t Y){

	uint8_t data[7];
	
	data[0] = BINtoBCD(RTC_CheckMinMax(SS, 0, 59)); 
	data[1] = BINtoBCD(RTC_CheckMinMax(MM, 0, 59));
	data[2] = BINtoBCD(RTC_CheckMinMax(HH, 0, 23));
	data[3] = BINtoBCD(RTC_CheckMinMax(Date, 1, 7));
	data[4] = BINtoBCD(RTC_CheckMinMax(D, 1, 31));
	data[5] = BINtoBCD(RTC_CheckMinMax(M, 1, 12));
	data[6] = BINtoBCD(RTC_CheckMinMax(Y, 0, 99));
	
	I2Cx_WriteMulti(I2C1, 0x68<<1, 0x00, data,7);
} 
