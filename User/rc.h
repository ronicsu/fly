#ifndef _RC_RC_H_
#define _RC_RC_H_
#include "stm32f10x.h"

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;}RC_GETDATA;

extern RC_GETDATA Rc_Get;//���յ���RC����,1000~2000

void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);

typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;



typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;

extern S_INT16_XYZ ACC_AVG;			//ƽ��ֵ�˲����ACC
extern S_FLOAT_XYZ GYRO_I;				//�����ǻ���
extern S_FLOAT_XYZ EXP_ANGLE;		//�����Ƕ�
extern S_FLOAT_XYZ DIF_ANGLE;		//�����Ƕ���ʵ�ʽǶȲ�
extern S_FLOAT_XYZ Q_ANGLE;			//��Ԫ��������ĽǶ�


#endif
