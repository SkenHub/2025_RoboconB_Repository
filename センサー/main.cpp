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

Encoder encoder[4];
Encoder_data e_data[4];
Gpio limit[8];

CanData can_data_r;
uint8_t a[6],b[6],c[6],d[6],e[6],f[6],g[6];
double enc_data[4];
uint8_t send_data_enc[8];   // エンコーダ関連CAN送信データ
uint8_t send_data_limit[8]; // リミットスイッチのCAN送信データ

void main_interrupt(void) {
    // 各エンコーダの割り込み処理
    for (int i = 0; i < 4; ++i) {
        encoder[i].interrupt(&e_data[i]);
        enc_data[i] = e_data[i].deg;
    }
    // 各リミットスイッチの状態を読み取り
    for (int n = 0; n < 8; n++) {
        send_data_limit[n] = limit[n].read();
    }
    int16_t iti = static_cast<int16_t>(enc_data[0]);
    send_data_enc[0] = (iti >> 8) & 0xFF;  // x の上位バイト
    send_data_enc[1] = iti & 0xFF;         // x の下位バイト
    int16_t ni = static_cast<int16_t>(enc_data[1]);
    send_data_enc[2] = (ni >> 8) & 0xFF;  // y の上位バイト
    send_data_enc[3] = ni & 0xFF;         // y の下位バイト
    int16_t san = static_cast<int16_t>(enc_data[2]);
    send_data_enc[4] = (san >> 8) & 0xFF;
    send_data_enc[5] = san & 0xFF;
    int16_t yon = static_cast<int16_t>(enc_data[3]);
    send_data_enc[6] = (yon >> 8) & 0xFF;
    send_data_enc[7] = yon & 0xFF;
}

void can(void){
	if(can_data_r.rx_stdid == 0x320){//canIDが0x320なら実行
		for(int m=0;m<8;m++){
		  a[m] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入
		}
	}
	if(can_data_r.rx_stdid == 0x300){//canIDが0x300なら実行
		for(int m=0;m<8;m++){
		  b[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x310){//canIDが0x310なら実行
		for(int m=0;m<8;m++){
		  c[m] = can_data_r.rx_data[m];
		}
    }
	if(can_data_r.rx_stdid == 0x330){//canIDが0x330なら実行
		for(int m=0;m<8;m++){
		  d[m] = can_data_r.rx_data[m];
		}
	}
}

int main(void) {
    // システム初期化
    sken_system.init();
    sken_system.startCanCommunicate(B13, B12, CAN_2); // CAN通信開始
    sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
    // エンコーダの初期化（各ピンとタイマーを指定）
    encoder[0].init(A0, A1, TIMER5,60.0);
    encoder[1].init(B3, A5, TIMER2,60.0);
    encoder[2].init(B6, B7, TIMER4,60.0);
    encoder[3].init(C6, C7, TIMER8,60.0);

    // リミットスイッチの初期化（各ピン、プルアップ設定）
    limit[0].init(B15, INPUT_PULLUP);
    limit[1].init(B14, INPUT_PULLUP);
    limit[2].init(A11, INPUT_PULLUP);
    limit[3].init(A8, INPUT_PULLUP);
    limit[4].init(A7, INPUT_PULLUP);
    limit[5].init(A6, INPUT_PULLUP);
    limit[6].init(B9, INPUT_PULLUP);
    limit[7].init(B8, INPUT_PULLUP);
    sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);
    sken_system.addTimerInterruptFunc(can, 3, 1);

    // メインループ：CAN通信でエンコーダ・リミットスイッチのデータを送信
    while (1) {
    	sken_system.canTransmit(CAN_2, 0x300, send_data_enc, 8, 1);
    	sken_system.canTransmit(CAN_2, 0x310, send_data_limit, 8, 1);
    }
}
