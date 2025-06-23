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
//色々な変数作成
Gpio SW;
Motor mtr[4];
int botan[13];
int botan_og[13];
int botan_k[13];
int limitdata[8];
int mtr1_sg[20];
int hantei;
Encoder encoder[4];
Encoder_data e_data[4];
double enc_data[4];
double enc_org[4];
double enc_sinti[4];
uint8_t send_data[6] = {0,0,0,0,0,0};
uint8_t send_data_DD[6] = {0,0,0,0,0,0};
CanData can_data_r;
uint8_t candata_r[8];
uint8_t can_enc_data[8];
uint8_t a[8],b[8],c[8],d[8],e[8],f[8],g[8];

void main_interrupt(void){
	for (int i = 0; i < 4; ++i) {
		encoder[i].interrupt(&e_data[i]);//エンコーダのデータをe_dataに代入
		enc_data[i] = e_data[i].deg;//エンコーダのdegデータをenc_dataに入れる
	}

	//ボタンの押された回数をカウント
	for(int i=0;i<13;i++){
	  if(botan_og[i] == 1){
		botan_k[i] +=1;
	  }
	  else if(botan_og[i] == 0){
		botan_k[i] = 0;
	  }
	  if(botan_k[i] == 1){
		botan[i] += 1;
	  }
	}
}
void can_rceive(void){
	if(can_data_r.rx_stdid == 0x400){//canIDが0x400なら実行
		for(int m=0;m<8;m++){
	  	  candata_r[m] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入
	  	  //botan[m] = candata_r[m];
	    }
	}
	if(can_data_r.rx_stdid == 0x310){//canIDが0x310なら実行
		for(int m=0;m<8;m++){
		  limitdata[m] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入
		}
	}
	if(can_data_r.rx_stdid == 0x300){//canIDが0x300なら実行
		for(int m=0;m<8;m++){
		  can_enc_data[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x320){//canIDが0x320なら実行
		for(int m=0;m<8;m++){
		  a[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x330){//canIDが0x330なら実行
		for(int m=0;m<8;m++){
		  b[m] = can_data_r.rx_data[m];
		}
	}
}

void timer(void){//1秒ごとに実行
	//mtr1,2を一秒正転して止める
	if(mtr1_sg[0] == 1){
		mtr[1].write(75);
		mtr[2].write(75);
		mtr1_sg[0] = 2;
	}
	else if(mtr1_sg[0] == 2){
		mtr[1].write(0);
		mtr[2].write(0);
		send_data_DD[0] = 0;
		send_data_DD[1] = 0;
		mtr1_sg[0] = 3;
	}
	//mtr1,2を一秒反転して止める
	if(mtr1_sg[1] == 1){
		mtr[1].write(-75);
		mtr[2].write(-75);
		mtr1_sg[1] = 2;
	}
	else if(mtr1_sg[1] == 2){
		mtr[1].write(0);
		mtr[2].write(0);
		mtr1_sg[1] = 3;
	}
	//mtr1,2を一秒反転して止める
	if(mtr1_sg[2] == 1){
		mtr[1].write(-75);
		mtr[2].write(-75);
		mtr1_sg[2] = 2;
	}
	else if(mtr1_sg[2] == 2){
		mtr[1].write(0);
		mtr[2].write(0);
		mtr1_sg[2] = 3;
	}
	//一秒をカウント
	if(mtr1_sg[3] == 1){
		mtr1_sg[3] = 2;
	}
	else if(mtr1_sg[3] == 2){
		mtr1_sg[3] = 3;
	}
}

int main(void){
	//モーターやエンコーダの設定
	sken_system.init();
	encoder[0].init(A0, A1, TIMER5);
	encoder[1].init(B3, A5, TIMER2);
	encoder[2].init(B6, B7, TIMER4);
	encoder[3].init(C6, C7, TIMER8);
    mtr[0].init(Apin,B8,TIMER10,CH1);
    mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);
	SW.init(C13,INPUT_PULLUP);
	//CAN設定
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	//タイマ関数の使用
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);
	sken_system.addTimerInterruptFunc(timer, 4, 1000);
	while(1){
		//エンコーダの値をリミット１でリセットできるようにする
		for (int i = 0; i < 4; ++i) {
			if(enc_data[i] < 0){
				enc_sinti[i] = enc_org[i] - enc_data[i];
			}
			else if(enc_data[i] > 0){
				enc_sinti[i] = enc_org[i] - enc_data[i];
			}

		}
		if(limitdata[0] == 1){
			enc_org[0] = enc_data[0];
		}
		//マイコンのボタンでの実験用
		if(SW.read() == 0){
			botan_og[1] = 1;
		}
		else{
			botan_og[1] = 0;
		}
		//ボタン１を押したときに実行
	    if(botan[0] == 1){//1押し
            if(limitdata[0] != 1){
            	send_data_DD[0] = 1;
                send_data_DD[1] = 1;
                mtr[0].write(20);
            }
            else if(limitdata[0] == 1 && mtr1_sg[0] == 0){
            	mtr[0].write(0);
            	mtr1_sg[0] = 1;
            }
		}
		else if(botan[0] == 2){//2押し
			if(mtr1_sg[1] == 0){
			    mtr1_sg[1] = 1;
			}
			if(enc_sinti[0]>=-300){

				mtr[0].write(-20);
			}
			else if(enc_sinti[0]<=-300){
				mtr[0].write(0);
			}
		}
	    if(botan[0] > 2){
	        mtr1_sg[0] = 0;
	        mtr1_sg[1] = 0;
	        botan[0] = 0;
	    }
	    //ボタン２を押したときに実行
	    if(botan[1] == 1 && mtr1_sg[2] == 0){//1押し
	    	mtr1_sg[2] = 1;
	    }
	    else if(botan[1] >= 2){//2押し以上
	    	if(((limitdata[1] == 0) || (limitdata[2] == 0)) && (mtr1_sg[3] == 0)){
	    		mtr[1].write(50);
	    		mtr[2].write(50);
	    		mtr1_sg[3] = 1;
	    		hantei = 1;
	    	}
	    	else if(mtr1_sg[3] == 3){
	    		mtr[1].write(0);
	    		mtr[2].write(0);
	    		mtr1_sg[2] = 0;
	    		mtr1_sg[3] = 0;
	    		botan[1] = 0;
	    		hantei = 2;
	    	}
	    	else if((limitdata[1] == 1) && (limitdata[2] == 1)){
	    		mtr[1].write(0);
	    		mtr[2].write(0);
	    	    mtr1_sg[2] = 0;
	    	    mtr1_sg[3] = 0;
	    	    botan[1] = 0;
	    	    hantei = 3;
	    	}
	    }
	    //ボタン３を押したときに実行
	    if(botan[2] == 1){//1押し
	    	send_data_DD[0] = 1;
	    	send_data_DD[1] = 1;
	    }
	    else if(botan[2] == 2){//2押し
	    	send_data_DD[0] = 0;
	    	send_data_DD[1] = 0;
	    	botan[2] = 0;
	    }
	    //ボタン４を押したときに実行
	    if(botan[3] == 1){//1押し
	    	 if(enc_sinti[0]>=-150){
	    	     mtr[0].write(-20);
	    	 }
	    	 else if(enc_sinti[0]<=-150){
	    	     mtr[0].write(0);
	    	 }
	    }
	    else if(botan[3] >= 2){//2押し以上
	    	 if(limitdata[0] != 1){
	    	     mtr[0].write(20);
	    	 }
	    	 else if(limitdata[0] == 1){
	    	     mtr[0].write(0);
	    	     botan[3] = 0;
	    	 }
	    }
	    sken_system.canTransmit(CAN_2, 0x340, send_data_DD,6, 1);//DDモジュールへ送信
	}
}
