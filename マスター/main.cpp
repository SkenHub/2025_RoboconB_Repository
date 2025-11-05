#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Gpio transam;

Motor motor[4];

//Encoder encoder[4];
//Encoder_data e_data[4];
//double rps[4];

CanData can_receive;

uint8_t can_ps3a[8]; // ps3送信 {1(1:2:3:4:5:6:7:8:)2(1:2:3:4:5:6:7:8:)} 0x400
uint8_t can_ps3b[7];

uint8_t sen1a_can[8], sen1b_can[8];
uint8_t sen2a_can[8], sen2b_can[8];
uint8_t dd1_can[6], dd2_can[6];
uint8_t rmas1_can[6], rmas2_can[8];

double m0, m1, m2, m3;

//  m3――m0
//  |    |
//  m2――m1

PS4 ps4;
PS4_data ps4_data;

/*
Pid pid_control[4];
double target[4];
double now[4];
double out[4];
*/

void time1()
{
	ps4.Getdata(&ps4_data); // ⊂ニニニ( ^ω^)二⊃
	/*for (int i = 0; i < 4; ++i)
	{
		out[i] = pid_control[i].control(target[i],now[i],10000);
	}
	for (int i = 0; i < 4; ++i)
	{
		encoder[i].interrupt(&e_data[i]);
		rps[i] = e_data[i].rps;
	}*/
}

void can_assign(void)
{
	can_ps3a[0] = (ps4_data.Cross);
	can_ps3a[1] = (ps4_data.Circle);
	can_ps3a[2] = (ps4_data.Triangle);
	can_ps3a[3] = (ps4_data.Square);
	can_ps3a[4] = (ps4_data.Up);
	can_ps3a[5] = (ps4_data.Down);
	can_ps3a[6] = (ps4_data.Left);
	can_ps3a[7] = (ps4_data.Right);

	can_ps3b[0] = (ps4_data.L1);
	can_ps3b[1] = (ps4_data.R1);
	can_ps3b[2] = (ps4_data.L2);
	can_ps3b[3] = (ps4_data.R2);
	can_ps3b[4] = (ps4_data.Share);
	can_ps3b[5] = (ps4_data.Options);
	can_ps3b[6] = (ps4_data.Ps);

	if (can_receive.rx_stdid == 0x300)
	{
		for (int i = 0; i < 8; ++i)
			sen1a_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x270)
	{
		for (int i = 0; i < 8; ++i)
			sen1b_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x320)
	{
		for (int i = 0; i < 8; ++i)
			sen2a_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x330)
	{
		for (int i = 0; i < 8; ++i)
			sen2b_can[i] = can_receive.rx_data[i];
	}

	if (can_receive.rx_stdid == 0x340)
	{
		for (int i = 0; i < 8; ++i)
			dd1_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x350)
	{
		for (int i = 0; i < 8; ++i)
			dd2_can[i] = can_receive.rx_data[i];
	}

	if (can_receive.rx_stdid == 0x200)
	{
		for (int i = 0; i < 8; ++i)
			rmas1_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x201)
	{
		for (int i = 0; i < 8; ++i)
			rmas2_can[i] = can_receive.rx_data[i];
	}
}

int main(void)
{
	sken_system.init();

	transam.init(B7,OUTPUT);

	/*encoder[0].init(A0, A1, TIMER5);
	encoder[1].init(A5, B3, TIMER2);
	encoder[2].init(B6, B7, TIMER4);
	encoder[3].init(C6, C7, TIMER3);*/
	motor[0].init(Apin, B8, TIMER10, CH1);
	motor[0].init(Bpin, B9, TIMER11, CH1);
	motor[1].init(Apin, A6, TIMER13, CH1);
	motor[1].init(Bpin, A7, TIMER14, CH1);
	motor[2].init(Apin, A8, TIMER1, CH1);
	motor[2].init(Bpin, A11, TIMER1, CH4);
	motor[3].init(Apin, B14, TIMER12, CH1);
	motor[3].init(Bpin, B15, TIMER12, CH2);

	ps4.StartRecive(A2, A3, SERIAL2);

/*
	 pid_control[0].setGain(1,0,0);
	 pid_control[1].setGain(1,0,0);
	 pid_control[2].setGain(1,0,0);
	 pid_control[3].setGain(1,0,0);
*/

	sken_system.addTimerInterruptFunc(time1, 0, 1); // お前許さん 100ms

	sken_system.startCanCommunicate(B13, B12, CAN_2);			// CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2, &can_receive); // CAN受信
	sken_system.addTimerInterruptFunc(can_assign, 3, 1);		// CAN代入
	while (1)
	{
		sken_system.canTransmit(CAN_2, 0x555, can_ps3a, 8,0);
		sken_system.canTransmit(CAN_2, 0x556, can_ps3b, 7,0);

		//for (int i = 0; i < 4; ++i)
		//{
			//encoder[i].interrupt(&e_data[i]);
			//rps[i] = e_data[i].rps;
		//}


		if((((ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)>=90) m0=90;//右上タイヤ
		else if((((ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)<=-90) m0=-90;
		else m0=((ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad)*0.9);

		if(((-(ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)>=90) m1=90;//右下タイヤ
		else if(((-(ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)<=-90) m1=-90;
		else m1=(-(ps4_data.LxPad)-(ps4_data.LyPad)+(ps4_data.RxPad))*0.9;

		if(((-(ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)>=90) m2=90;//左下タイヤ
		else if(((-(ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)<=-90) m2=-90;
		else m2=(-(ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9;

		if((((ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)>=90) m3=90;//左上タイヤ
		else if((((ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9)<=-90) m3=-90;
		else m3=((ps4_data.LxPad)+(ps4_data.LyPad)+(ps4_data.RxPad))*0.9;

		if(ps4_data.Cross)
		{
			motor[0].write(m0);
			motor[1].write(m1);
			motor[2].write(m2);
			motor[3].write(m3);
			transam.write(HIGH);
		}
		else
		{
			motor[0].write(m0*0.3);
			motor[1].write(m1*0.3);
			motor[2].write(m2*0.3);
			motor[3].write(m3*0.3);
			transam.write(LOW);
		}

	}
}
