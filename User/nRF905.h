#ifndef _NRF905__H_
#define _NRF905__H_

#define 	TX_EN	   PAout(1)
#define 	TRX_CE	   PAout(2)
#define		PWR_UP	   PAout(3)
#define		CSN		   PAout(4)


#define 	AM		   PCin(0)
#define 	DR		   PCin(1)
#define		CD		   PCin(2)
// IO map 
//	PA5    SCK
//  PA6    MISO
//  PA7    MOSI
#define   SCK   PAout(5)
#define   MISO  PAin(6)
#define   MOSI  PAout(7)
//=====================================================
//没有Read RX_ADDRSS command的地址,怎么样来读取????
#define     WC	      0x00		// Write configuration register command
#define     RC	      0x10 		// Read  configuration register command
#define     WTP	      0x20 		// Write TX Payload  command
#define     RTP	      0x21		// Read  TX Payload  command
#define     WTA	      0x22		// Write TX Address  command
#define     RTA	      0x23		// Read  TX Address  command
#define     RRP	      0x24		// Read  RX Payload  command
//=====================================================
//RF-Configuration Register
//配置寄存器共10 bytes
//=====================================================

void SPI_Config(void);
void nRF905_PortInit(void);
//=====================================================
void SpiWrite(u8 byte);
u8 SpiRead(void);
void Config905(void);
void TxPacket(void);
void RxPacket(void);
void SetTxMode(void);
void SetRxMode(void);
u8 nRF905_Check(void);
//=====================================================
#endif

