#include "control.h"



PID PID_ROL,PID_PIT,PID_YAW;
u8 ARMED = 0;

void Pid_init(void);

extern short gyro[];
extern short accel[];


void CONTROL(float rol, float pit, float yaw)
{
	u16 moto1=0,moto2=0,moto3=0,moto4=0;
	
	PID_ROL.pout = PID_ROL.P * rol;
	PID_ROL.dout = PID_ROL.D * gyro[0];
	
	PID_PIT.pout = PID_PIT.P * pit;
	PID_PIT.dout = PID_PIT.D * gyro[1];
	
	PID_YAW.dout = PID_YAW.D * gyro[2];
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
	
	#define THROTTLE 1300 //TBD
	if(THROTTLE>1200)
	{
		moto1 = THROTTLE - 1000 - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
		moto2 = THROTTLE - 1000 + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
		moto3 =THROTTLE - 1000 + PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;
		moto4 = THROTTLE - 1000 - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
	if(ARMED)	Moto_PwmRflash(moto1,moto2,moto3,moto4);
	else			Moto_PwmRflash(0,0,0,0);
}

void Pid_init(void)
{
	PID_ROL.P = 12;
	PID_ROL.I = 0;
	PID_ROL.D = 0.2;
	
	PID_PIT.P = 12;
	PID_PIT.I = 0;
	PID_PIT.D = 0.2;
	
	PID_YAW.P = 0;
	PID_YAW.I = 0;
	PID_YAW.D = 0.5;
	
	PID_ROL.pout = 0;
	PID_ROL.iout = 0;
	PID_ROL.dout = 0;
	
	PID_PIT.pout = 0;
	PID_PIT.iout = 0;
	PID_PIT.dout = 0;
	
	PID_YAW.pout = 0;
	PID_YAW.iout = 0;
	PID_YAW.dout = 0;
}
