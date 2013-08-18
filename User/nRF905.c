#include "stm32f10x.h"
#include "nRF905.h"
#include "sys.h"
#include "delay.h"

typedef struct RFconfig
{
    u8 n;
    
    u8 buf[10];
    
}RFconfig;
//=====================================================
RFconfig  RxTxconf =
{
	10,
	0x4c,  
	0x0c,     //不重发,433.2MHz,6dBm,正常模式
	0x44,     //收发各4字节地址宽度
	0x20,
	0x20,     //收发有效数据宽度32位
	0xcc,
	0xcc,
	0xcc,
	0xcc,     //接收机地址 
	0x58      //16位CRC校验,允许校验,晶振16MHz,没有外部时钟
};
//=====================================================
u8 TxBuf[32];
u8 RxBuf[32];
u8 DATA_BUF;
u8 TxRxAddress[4]={0xcc,0xcc,0xcc,0xcc};
//四个字节的地址用于检测nRF905是不是可以进行正常的读写
void SPI_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
//	SPI_InitTypeDef   SPI_InitStructure;

	//GPIOA Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//	//SPI1 Periph clock enable
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	//Configure SPI1 pins: SCK, MISO and MOSI
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //输入口
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//Configure PA4 pin: CSN pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);


	// SPI1 Config
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //SPI_NSS_Hard
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_CRCPolynomial = 7;
//	SPI_Init(SPI1,&SPI_InitStructure);
//
//	// SPI1 enable
//	SPI_Cmd(SPI1,ENABLE);
	
}
void nRF905_PortInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	//enable GPIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC,ENABLE);

	//Configure TX_EN , TRX_CE , PWR_UP ,pin as output:  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//Configure AM , DR , CM  pins as input pin to be detected
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	//推挽输出
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	// enable spi
	SPI_Config();
	
	    //NRF905初始化
    CSN = 1 ;				      // Spi disable						

    PWR_UP = 1 ;				      // nRF905 power on
    TRX_CE = 0;				      // Set nRF905 in standby mode
    TX_EN = 0;				      // set radio in Rx mode		
}
//=====================================================
//SPI 写操作代码,向RF905写入一字节的数据
//void SpiWrite(uchar byte)
//=====================================================
//void SpiWrite(u8 byte)
//{
//   //Wait until the transmit buffer is empty
//  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
//  // Send the byte
//  SPI_I2S_SendData(SPI1,byte);                    // Transmit first character
//}
void SpiWrite(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		SCK = 1;
		if(byte & 0x80)
		{
			MOSI = 1;
		}
		else 
		{
			MOSI = 0;
		}
	    SCK = 0;
		byte<<=1;
	}
}
//=====================================================
//SPI 读操作代码,向RF905读出一字节的数据
//uchar SpiRead(void)
//=====================================================
//u8 SpiRead(void)
//{
//	unsigned char data = 0;
//   //Wait until a data is received
//  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
//  // Get the received data
//  data = SPI_I2S_ReceiveData(SPI1);
//  return data;
//}
u8 SpiRead(void)
{
	unsigned char data=0,i;
	for(i=0;i<8;i++)
	{	
		data<<=1;
		SCK=1;
		if(MISO==1)
		{
			data |=0x01;
		}
		else
		{
			data &= 0xfe;
		}
		SCK=0;
		
	}
	return data;
}
//=====================================================
// nRF905 的检测程序,首先写入地址,然后再读出地址,
// 如果读出来的值与原来的一样,那么就返回1 ,否则返回0
//=====================================================
u8 nRF905_Check(void)
{
	u8 i,AddressTemp[4]={0};
	CSN = 0;//SPI valid
	SpiWrite(WTA);//写地址
	for(i=0;i<4;i++)
	{
		SpiWrite(TxRxAddress[i]);
	}
	CSN = 1;
	delay_us(500);
	CSN = 0;
	SpiWrite(RTA);//读地址
	for(i=0;i<4;i++)
	{
		AddressTemp[i]=SpiRead();
	}
	CSN = 1;
	for(i=0;i<4;i++)
	{
		if(AddressTemp[i] != TxRxAddress[i])
		{
			break;
		}
	}
	if(i!=4) return 0;
	return 1;
}
//=====================================================
//主机通过SPI接口向905配置寄存器写入配置信息
//
//=====================================================
void Config905(void)
{
    unsigned char i;
    CSN = 0;
    SpiWrite(WC);
    for(i=0;i<RxTxconf.n;i++)
    {
        SpiWrite(RxTxconf.buf[i]);
    }
    CSN = 1;
    //===================
    
    delay_ms(100);
    /*CSN  = 0;
    SpiWrite(RC);
    for(i=0;i<RxTxconf.n;i++)
    {
        RxBuf[i] = SpiRead();
    }
    CSN = 1;
    */
    //===================
}
//=====================================================
//使用905发送数据
//
//=====================================================
void TxPacket(void)
{
    u16 i;
    CSN = 0;
    SpiWrite(WTP);
    for(i=0;i<32;i++)
    {
        SpiWrite(TxBuf[i]);
    }
    CSN = 1;
    delay_us(100);//
    CSN = 0;
    SpiWrite(WTA);
    for(i=0;i<4;i++)
    {
        SpiWrite(RxTxconf.buf[i+5]);
    }
    CSN = 1;
    TRX_CE  = 1;
    delay_us(100);//
    TRX_CE = 0;
}
//=====================================================
//使用905接收数据
//
//=====================================================
void RxPacket(void)
{
    unsigned char i;
    TRX_CE = 0;                      //设置905进入待机模式
    CSN = 0;
    SpiWrite(RRP);                 //准备读取接收到的数据
    for(i=0;i<32;i++)
    {
        RxBuf[i] = SpiRead();       //通过SPI口从905读取数据
    }
    CSN = 1;
    while(DR||AM);
    TRX_CE = 1;  
    
}
//=====================================================
//设置905为发送模式
//
//=====================================================
void SetTxMode(void)
{
    TX_EN = 1;
    TRX_CE = 0;
    delay_ms(1);                         //delay for mode change(>=650us)
}
//=====================================================
//设置905为接收模式
//
//=====================================================
void SetRxMode(void)
{
    TX_EN = 0;
    TRX_CE = 1;
    delay_ms(1);                         //delay for mode change(>=650us)
}

