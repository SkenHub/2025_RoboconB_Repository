#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

Motor motor[4];

Encoder encoder;
Encoder_data e_data;
double deg;

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

void time1()
{
	ps4.Getdata(&ps4_data); // ⊂ニニニ( ^ω^)二⊃
}

void can_assign(void)
{
	can_ps3a[0] = (ps4_data.Right);
	can_ps3a[1] = (ps4_data.Down);
	can_ps3a[2] = (ps4_data.Left);
	can_ps3a[3] = (ps4_data.Up);
	can_ps3a[4] = (ps4_data.Square);
	can_ps3a[5] = (ps4_data.Cross);
	can_ps3a[6] = (ps4_data.Circle);
	can_ps3a[7] = (ps4_data.Triangle);

	can_ps3b[0] = (ps4_data.L1);
	can_ps3b[1] = (ps4_data.R1);
	can_ps3b[2] = (ps4_data.L2);
	can_ps3b[3] = (ps4_data.R2);
	can_ps3b[4] = (ps4_data.Share);
	can_ps3b[5] = (ps4_data.Options);
	can_ps3b[6] = (ps4_data.Ps);

	if (can_receive.rx_stdid == 0x300)
	{
		for (int i = 0; i < 7; ++i)
			sen1a_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x310)
	{
		for (int i = 0; i < 7; ++i)
			sen1b_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x320)
	{
		for (int i = 0; i < 7; ++i)
			sen2a_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x330)
	{
		for (int i = 0; i < 7; ++i)
			sen2b_can[i] = can_receive.rx_data[i];
	}

	if (can_receive.rx_stdid == 0x340)
	{
		for (int i = 0; i < 5; ++i)
			dd1_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x350)
	{
		for (int i = 0; i < 5; ++i)
			dd2_can[i] = can_receive.rx_data[i];
	}

	if (can_receive.rx_stdid == 0x200)
	{
		for (int i = 0; i < 5; ++i)
			rmas1_can[i] = can_receive.rx_data[i];
	}
	if (can_receive.rx_stdid == 0x201)
	{
		for (int i = 0; i < 7; ++i)
			rmas2_can[i] = can_receive.rx_data[i];
	}
}

int main(void)
{
	sken_system.init();
	encoder.init(A0, A1, TIMER5);
	motor[0].init(Apin, B8, TIMER10, CH1);
	motor[0].init(Bpin, B9, TIMER11, CH1);
	motor[1].init(Apin, A6, TIMER13, CH1);
	motor[1].init(Bpin, A7, TIMER14, CH1);
	motor[2].init(Apin, A8, TIMER1, CH1);
	motor[2].init(Bpin, A11, TIMER1, CH4);
	motor[3].init(Apin, B14, TIMER12, CH1);
	motor[3].init(Bpin, B15, TIMER12, CH2);

	ps4.StartRecive(A2, A3, SERIAL2);

	sken_system.addTimerInterruptFunc(time1, 0, 1); // お前許さん

	sken_system.startCanCommunicate(B13, B12, CAN_2);			// CAN開始
	sken_system.addCanRceiveInterruptFunc(CAN_2, &can_receive); // CAN受信
	sken_system.addTimerInterruptFunc(can_assign, 3, 1);		// CAN代入
	while (1)
	{
		sken_system.canTransmit(CAN_2, 0x555, can_ps3a, 8, 1);
		sken_system.canTransmit(CAN_2, 0x556, can_ps3b, 7, 1);

		motor[0].write(can_ps3a[0] * 100);
		motor[1].write(can_ps3a[1] * 100);
		motor[2].write(can_ps3a[2] * 100);
		motor[3].write(can_ps3a[3] * 100);
	}
}
