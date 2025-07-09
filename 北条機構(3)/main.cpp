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
//色んな変数
Gpio SW;
Motor mtr[4];
int swich,timer_counter[2] = {0,0};
int botan[16],botan_og[16],botan_k[16];
int limitdata[8];
Encoder encoder[4];
Encoder_data e_data[4];
double enc_data[4],enc_org[4],enc_sinti[4];
uint8_t send_data[6] = {0,0,0,0,0,0};
uint8_t send_data_DD[6] = {0,0,0,0,0,0};
CanData can_data_r;
uint8_t candata_r[16],can_enc_data[8];
uint8_t a[8],b[8],c[8],d[8],e[8],f[8],g[8];
double robmas = 0,robmas_kaku = 0,robmas_og = 0,robmas_sinti = 0,robmas_sa = 0,robmas_no = 0,robmas_pla = 0,robmas_c;
Pid pid_mtr,pid_robmtr;
double robmtr_k = 32;//ロボマス機構円周
double mtr_k = 10;//上下機構のモーター円周
double kyori_mtr,kyori_robmtr;
double target = 0,target_robmtr = 0;
uint8_t out_mtr;
int16_t out_robmtr;
int count = 0;

void main_interrupt(void){
	for (int i = 0; i < 4; ++i) {
		encoder[i].interrupt(&e_data[i]);//エンコーダのデータをe_dataに代入
		enc_data[i] = e_data[i].deg;
	}
	for(int m=0;m<16;m++){
		botan_og[m] = candata_r[m];
    }
	//ボタンのカウント
	for(int i=0;i<16;i++){
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
	//移動距離の計算
	kyori_mtr = -(mtr_k*enc_sinti[0]/360);
	kyori_robmtr = robmtr_k*robmas_sinti/360;
	if(target>kyori_mtr){
	    out_mtr = (pid_mtr.control(target,kyori_mtr,1));
	}
	else if(target<kyori_mtr){
		out_mtr = -(pid_mtr.control(target,kyori_mtr,1));
	}
	if(target_robmtr>kyori_robmtr){
	    out_robmtr = pid_robmtr.control(target_robmtr,kyori_robmtr,1);
	}
	else if(target_robmtr<kyori_robmtr){
		out_robmtr = -(pid_robmtr.control(target_robmtr,kyori_robmtr,1));
	}
	if(target_robmtr<kyori_robmtr){
	    out_robmtr = -(out_robmtr);
	}

	send_data[0] = (out_robmtr >> 8);//１バイトでは255までしか送れないので２バイトで送るために解体
	send_data[1] = out_robmtr;
}
void can_rceive(void){
	if(can_data_r.rx_stdid == 0x201){
		robmas = (double)(int16_t(can_data_r.rx_data[0] << 8) | (can_data_r.rx_data[1] & 0xff));
		robmas_kaku = (robmas/8192)*360/19;//３６０度に調整
		//起動時に起動場所をゼロにする＆絶対角をちゃんと使いやすいように調整
		if(robmas_kaku != 0 && count == 0){
			robmas_og = robmas_kaku;
			count = 1;
		}
		robmas_no = robmas_kaku - robmas_og;
		robmas_sa = robmas_c - robmas_no;
		if(robmas_sa > 360/2/19){
			robmas_pla += 360/19;
		}
		else if(robmas_sa < -360/2/19){
			robmas_pla -= 360/19;
		}
		robmas_c = robmas_no;
		robmas_sinti = robmas_pla + robmas_no;
	}

	if(can_data_r.rx_stdid == 0x555){//canIDが0x555なら実行
		for(int m=0;m<8;m++){
	  	  candata_r[m] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入(足回りからの受信)
	    }
	}
	if(can_data_r.rx_stdid == 0x556){//canIDが0x556なら実行
		for(int m=0;m<8;m++){
	  	  candata_r[m+8] = can_data_r.rx_data[m];//CAN受信データをcandata_rに代入(足回りからの受信)
	    }
	}
	if(can_data_r.rx_stdid == 0x310){//canIDが0x310なら実行
		for(int m=0;m<8;m++){
		  limitdata[m] = can_data_r.rx_data[m];
		}
	}
	if(can_data_r.rx_stdid == 0x340){//canIDが0x340なら実行
		for(int m=0;m<8;m++){
		  a[m] = can_data_r.rx_data[m];
		}
	}
}

void timer(void){
	if(swich == 1){
		timer_counter[0]++;
    }
	else if(swich == 2){
		timer_counter[1]++;
	}
	if(timer_counter[0] >= 1000){
		target_robmtr = 0;
		timer_counter[0] = 0;
	}
	else if(timer_counter[1] >= 1000){
		send_data_DD[1] = 1;
		timer_counter[1] = 0;
	}
}

int main(void){
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
	pid_mtr.setGain(2.5,0.7,1.2);
	pid_robmtr.setGain(30,0,0);
	sken_system.startCanCommunicate(B13,B12,CAN_2);//CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_data_r);//CAN受信
	sken_system.addTimerInterruptFunc(can_rceive,3,1);
	sken_system.addTimerInterruptFunc(main_interrupt, 1, 1);
	sken_system.addTimerInterruptFunc(timer, 4, 1);
	while(1){
		for (int i = 0; i < 4; ++i) {
			if(enc_data[i] < 0){
				enc_sinti[i] = enc_org[i] - enc_data[i];
			}
			else if(enc_data[i] > 0){
				enc_sinti[i] = enc_org[i] - enc_data[i];
			}
		}
		if(limitdata[5] == 1){
			enc_org[0] = enc_data[0];
		}
		/*if(SW.read() == 0){
			botan_og[2] = 1;
		}
		else{
			botan_og[2] = 0;
		}*/
		sken_system.canTransmit(CAN_2, 0x350, send_data_DD,6, 1);
		sken_system.canTransmit(CAN_2, 0x200, send_data,6, 1);
	    if(botan[0] == 1){
	    	if(limitdata[1] == 0)
	    	    mtr[0].write(50);
	    	else if(limitdata[1] == 1){
	    		mtr[0].write(0);
	    	}
		}
	    else if(botan[0] == 2){
	    	if(limitdata[2] == 0)
	    	    mtr[1].write(50);
	    	else if(limitdata[2] == 1){
	    	    mtr[1].write(0);
	    	}
	    }
	    else if(botan[0] == 3){
	    	if(limitdata[3] == 0)
	    	    mtr[0].write(-50);
	    	else if(limitdata[3] == 1){
	    	    mtr[0].write(0);
	    	}
	    	if(limitdata[4] == 0)
	    		mtr[1].write(-50);
	    	else if(limitdata[4] == 1){
	    		mtr[1].write(0);
	    	}
	    	if(limitdata[3] == 1 && limitdata[4] == 1){
	    		botan[0] = 0;
	    	}
	    }
	    if(botan[1] == 1){
	    	send_data_DD[0] = 1;
	    }
	    else if(botan[1] == 2){
	    	send_data_DD[0] = 0;
	    	botan[1] = 0;
	    }
	    if(botan[2] == 1){
	    	send_data_DD[1] = 1;
	    	swich = 1;
	    }
	    else if(botan[2] == 2){
	    	target_robmtr = 100;
	    	swich = 2;
	    }
	    else if(botan[2] == 3){
	    	botan[2] = 1;
	    	timer_counter[0] = 0;
	    	timer_counter[1] = 0;
	    }
	    if(botan[3] == 1){
	    	if(limitdata[5] == 0){
	    		mtr[2].write(-50);
	    	}
	    	else if(limitdata[5] == 1){
	    		mtr[2].write(0);
	    		target = 0;
	    		botan[3] = 0;
	    	}
	    }
	    else if(botan[4] == 1){
	    	target = 20;
	    	botan[4] = 0;
	    }
	    else if(botan[5] == 1){
	    	target = 30;
	    	botan[5] = 0;
	    }
	    else if(botan[6] == 1){
	    	target = 40;
	    	botan[6] = 0;
	    }
	    else if(botan[7] == 1){
	    	target = 100;
	    	botan[7] = 0;
	    }
	    if(botan[3] == 0){
	    	if(target>kyori_mtr){
	    	    mtr[2].write(-(out_mtr));
	    	}
	    	else if(target<kyori_mtr){
	    		mtr[2].write(out_mtr);
	    	}
	    }
	    if(botan[8] == 1){
	    	send_data_DD[2] = 1;
	    }
	    else if(botan[8] == 2){
	    	send_data_DD[2] = 0;
	    	botan[8] = 0;
	    }
	}
}
