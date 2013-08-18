/*********************************Copyright (c)*********************************
**                               
**
**--------------File Info-------------------------------------------------------
** File Name:               hmc5883.c
** Last modified Date:      
** Last Version:            
** Description:             ��ά�����Ǵ�����
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
����˵��:
    
******************************************************************************/

/******************************************************************************
*********************************  Ӧ �� �� �� ********************************
******************************************************************************/

/******************************************************************************
********************************* �ļ����ò��� ********************************
******************************************************************************/


/******************************************************************************
******************************* �Զ���������� ********************************
******************************************************************************/


/******************************************************************************
********************************* �� �� �� �� *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:�����ṩ   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:�����ṩ   * 
*----------------------*/
//none

/******************************************************************************
********************************* �� �� �� �� *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:�����ṩ   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:�����ṩ   * 
*----------------------*/
//none

#define HMC5883L_Addr       0x3C            //�ų�������������ַ
#define HMC5883L_INT_PRIO   5

/******************************************************************************
***************************** HMC5883L �궨�� *********************************
******************************************************************************/
/*---------------------* 
*  HMC5883L�ڲ��Ĵ���  * 
*----------------------*/
#define HMC5883L_REGA   0x00
#define HMC5883L_REGB   0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_HX_H   0x03
#define HMC5883L_HX_L   0x04 
#define HMC5883L_HZ_H   0x05
#define HMC5883L_HZ_L   0x06
#define HMC5883L_HY_H   0x07
#define HMC5883L_HY_L   0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_IRA    0x0a    //�����к�ʹ�õļĴ���
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

/*---------------------* 
*   HMC5883 ��������   * 
*----------------------*/
typedef  short int16_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef int int32_t;
#define true 1
#define false 0


typedef struct
{
    int16_t  hx;
    int16_t  hy;
    int16_t  hz;
    uint16_t ha;   
}tg_HMC5883L_TYPE;

/*---------------------* 
*   HMC5883 У������   * 
*----------------------*/
// Ư��ϵ������λ��1��λ�شų�ǿ��
#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)

//��������
#define HMC5883L_GAIN_X     1f
//#define HMC5883L_GAIN_Y   1.04034582
#define HMC5883L_GAIN_Y     10403     //ʵ��1.04034582,����������� 

/******************************************************************************
*********************************  ����ʼ  **********************************
******************************************************************************/
#include <math.h>

char Mult_Read(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t * ptChar,uint8_t size)
{
	if(i2cRead(SlaveAddress,REG_Address,size,ptChar))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

#define Single_Write i2cWrite
uint8_t  Single_Read(uint8_t addr,uint8_t reg)
{
	uint8_t val;
	i2cRead(addr,reg,1,&val);
	return val;
}

/******************************************************************************
/ ��������:��ʼ��HMC5883
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void  HMC5883L_Init(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
}

/******************************************************************************
/ ��������:��ȡHMC5883������
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;
    
    Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
    Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
    delay_ms(10);
    
    tmp[0]=Single_Read(HMC5883L_Addr,HMC5883L_HX_H);//OUT_X_L_A
    tmp[1]=Single_Read(HMC5883L_Addr,HMC5883L_HX_L);//OUT_X_H_A
    
    tmp[2]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_H);//OUT_Z_L_A
    tmp[3]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_L);//OUT_Z_H_A
    
    tmp[4]=Single_Read(HMC5883L_Addr,HMC5883L_HY_H);//OUT_Y_L_A
    tmp[5]=Single_Read(HMC5883L_Addr,HMC5883L_HY_L);//OUT_Y_H_A

    ptResult->hx  = (int16_t)((tmp[0] << 8) | tmp[1])+HMC5883L_OFFSET_X;
    s32Val = (int16_t)((tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y;    
    s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
    ptResult->hy    = (int16_t)s32Val;
    ptResult->hz    = (int16_t)((tmp[2] << 8) | tmp[3]);
}

/******************************************************************************
/ ��������:����HMC5883��ʼת��(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (����)
******************************************************************************/
void HMC5883L_Start(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
}

/******************************************************************************
/ ��������:��ȡHMC5883������(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;

    if(true == Mult_Read(HMC5883L_Addr,HMC5883L_HX_H,tmp,6))   //�����������������
    {
        //��������(����x������y�����)
        ptResult->hx  = (int16_t)((tmp[0] << 8) | tmp[1])+HMC5883L_OFFSET_X;
        s32Val = (int16_t)((tmp[4] << 8) | tmp[5])+HMC5883L_OFFSET_Y;    
        s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
        ptResult->hy    = (int16_t)s32Val;
        ptResult->hz    = (int16_t)((tmp[2] << 8) | tmp[3]);
    }
} 

/******************************************************************************
/ ��������:HMC5883У׼
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_Calibrate(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x15);   //30Hz,�����Լ�ģʽ
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x01);   //��һ����ģʽ
   delay_ms(10);
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //�ص�����ģʽ
}



/******************************************************************************
/ ��������:��ӡHMC�Ĵ���������
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Printf(tg_HMC5883L_TYPE * ptResult)
{
    int16_t x,y;
    float angle;
    
    x = ptResult->hx;
    y = ptResult->hy;
    //�������
    if(x>0x7fff)x-=0xffff;
    if(y>0x7fff)y-=0xffff;
    //LED1_ON();
    angle= atan2(y,x) * (180 / 3.14159265) + 180;   //160us����ʱ��
    //LED1_OFF();
    ptResult->ha = (int16_t)(angle*10);   // �õ�����ȷ��0.1��
   // printf("HMC5883L:\thx: %4d,\thy: %4d,\thz: %4d,\t%4d\n\r",
     //       ptResult->hx, ptResult->hy, ptResult->hz, ptResult->ha/10);
}



