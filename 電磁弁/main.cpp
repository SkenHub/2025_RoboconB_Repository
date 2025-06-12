/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

CanData can_data_r;
Gpio solenoid[6];
Gpio LED;
double enc_data[4];
uint8_t candata_r[6];
uint8_t a[6],b[6],c[6],d[6],e[6],f[6],g[6];

void can_rceive(void){
	if(can_data_r.rx_stdid == 0x340){//canIDが0x320なら実行
		for(int m=0;m<6;m++){
		  candata_r[m] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入
		}
	}
	if(can_data_r.rx_stdid == 0x300){//canIDが0x300なら実行
		for(int m=0;m<6;m++){
	      a[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x310){//canIDが0x310なら実行
		for(int m=0;m<6;m++){
		  b[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x330){//canIDが0x330なら実行
		for(int m=0;m<6;m++){
		  c[m] = can_data_r.rx_data[m];
		}
	}
}

int main(void)
{
	sken_system.init();
	solenoid[0].init(B6, OUTPUT);//0x200
	solenoid[1].init(B7, OUTPUT);
	solenoid[2].init(B8, OUTPUT);
	solenoid[3].init(B9, OUTPUT);
	solenoid[4].init(C6, OUTPUT);
	solenoid[5].init(C7, OUTPUT);
	LED.init(A5,OUTPUT);
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	while (1) {
		for (int i = 0; i < 6; ++i) {
			if(candata_r[i] == 1){
			    solenoid[i].write(HIGH);
			}
			else{
				solenoid[i].write(LOW);
			}
		}
		enc_data[0] = (double)(int16_t(candata_r[0] << 8 | candata_r[1]));//double型のenc_dataにuint8_t型のcandata_rを合体して代入
		enc_data[1] = (double)(int16_t(candata_r[2] << 8 | candata_r[3]));
		enc_data[2] = (double)(int16_t(candata_r[4] << 8 | candata_r[5]));
		enc_data[3] = (double)(int16_t(candata_r[6] << 8 | candata_r[7]));
	}

}
