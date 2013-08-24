
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "STM32_I2C.h"
#include "minus_os.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
void Clock_Enable(void);
void GPIO_Configuration(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0, 1, 0,
                                           0, 0, -1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
//void send_packet(char packet_type, void *data)
//{
//#define MAX_BUF_LENGTH  (18)
//    char buf[MAX_BUF_LENGTH], length;
//
//    memset(buf, 0, MAX_BUF_LENGTH);
//    buf[0] = '$';
//    buf[1] = packet_type;
//
//    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
//        short *sdata = (short*)data;
//        buf[2] = (char)(sdata[0] >> 8);
//        buf[3] = (char)sdata[0];
//        buf[4] = (char)(sdata[1] >> 8);
//        buf[5] = (char)sdata[1];
//        buf[6] = (char)(sdata[2] >> 8);
//        buf[7] = (char)sdata[2];
//        length = 8;
//    } else if (packet_type == PACKET_TYPE_QUAT) {
//        long *ldata = (long*)data;
//        buf[2] = (char)(ldata[0] >> 24);
//        buf[3] = (char)(ldata[0] >> 16);
//        buf[4] = (char)(ldata[0] >> 8);
//        buf[5] = (char)ldata[0];
//        buf[6] = (char)(ldata[1] >> 24);
//        buf[7] = (char)(ldata[1] >> 16);
//        buf[8] = (char)(ldata[1] >> 8);
//        buf[9] = (char)ldata[1];
//        buf[10] = (char)(ldata[2] >> 24);
//        buf[11] = (char)(ldata[2] >> 16);
//        buf[12] = (char)(ldata[2] >> 8);
//        buf[13] = (char)ldata[2];
//        buf[14] = (char)(ldata[3] >> 24);
//        buf[15] = (char)(ldata[3] >> 16);
//        buf[16] = (char)(ldata[3] >> 8);
//        buf[17] = (char)ldata[3];
//        length = 18;
//    } else if (packet_type == PACKET_TYPE_TAP) {
//        buf[2] = ((char*)data)[0];
//        buf[3] = ((char*)data)[1];
//        length = 4;
//    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
//        buf[2] = ((char*)data)[0];
//        length = 3;
//    } else if (packet_type == PACKET_TYPE_PEDO) {
//        long *ldata = (long*)data;
//        buf[2] = (char)(ldata[0] >> 24);
//        buf[3] = (char)(ldata[0] >> 16);
//        buf[4] = (char)(ldata[0] >> 8);
//        buf[5] = (char)ldata[0];
//        buf[6] = (char)(ldata[1] >> 24);
//        buf[7] = (char)(ldata[1] >> 16);
//        buf[8] = (char)(ldata[1] >> 8);
//        buf[9] = (char)ldata[1];
//        length = 10;
//    } else if (packet_type == PACKET_TYPE_MISC) {
//        buf[2] = ((char*)data)[0];
//        buf[3] = ((char*)data)[1];
//        buf[4] = ((char*)data)[2];
//        buf[5] = ((char*)data)[3];
//        length = 6;
//    }
//    cdcSendDataWaitTilDone((BYTE*)buf, length, CDC0_INTFNUM, 100);
//}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Handle sensor on/off combinations. */
//static void setup_gyro(void)
//{
//    unsigned char mask = 0;
//    if (hal.sensors & ACCEL_ON)
//        mask |= INV_XYZ_ACCEL;
//    if (hal.sensors & GYRO_ON)
//        mask |= INV_XYZ_GYRO;
//    /* If you need a power transition, this function should be called with a
//     * mask of the sensors still enabled. The driver turns off any sensors
//     * excluded from this mask.
//     */
//    mpu_set_sensors(mask);
//    if (!hal.dmp_on)
//        mpu_configure_fifo(mask);
//}

//static void tap_cb(unsigned char direction, unsigned char count)
//{
//    char data[2];
//    data[0] = (char)direction;
//    data[1] = (char)count;
//    send_packet(PACKET_TYPE_TAP, data);
//}
//
//static void android_orient_cb(unsigned char orientation)
//{
//    send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
//}
//
//
//static inline void msp430_reset(void)
//{
//    PMMCTL0 |= PMMSWPOR;
//}

static void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		PrintChar("setting bias succesfully ......\n");
    }
	else
	{
		PrintChar("bias has not been modified ......\n");
	}

    /* Report results. */
//    test_packet[0] = 't';
//    test_packet[1] = result;
//    send_packet(PACKET_TYPE_MISC, test_packet);
}

//static void handle_input(void)
//{
//    char c;
//    const unsigned char header[3] = "inv";
//    unsigned long pedo_packet[2];
//
//    /* Read incoming byte and check for header.
//     * Technically, the MSP430 USB stack can handle more than one byte at a
//     * time. This example allows for easily switching to UART if porting to a
//     * different microcontroller.
//     */
//    rx_new = 0;
//    cdcReceiveDataInBuffer((BYTE*)&c, 1, CDC0_INTFNUM);
//    if (hal.rx.header[0] == header[0]) {
//        if (hal.rx.header[1] == header[1]) {
//            if (hal.rx.header[2] == header[2]) {
//                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//                hal.rx.cmd = c;
//            } else if (c == header[2])
//                hal.rx.header[2] = c;
//            else
//                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//        } else if (c == header[1])
//            hal.rx.header[1] = c;
//        else
//            memset(&hal.rx.header, 0, sizeof(hal.rx.header));
//    } else if (c == header[0])
//        hal.rx.header[0] = header[0];
//    if (!hal.rx.cmd)
//        return;
//
//    switch (hal.rx.cmd) {
//    /* These commands turn the hardware sensors on/off. */
//    case '8':
//        if (!hal.dmp_on) {
//            /* Accel and gyro need to be on for the DMP features to work
//             * properly.
//             */
//            hal.sensors ^= ACCEL_ON;
//            setup_gyro();
//        }
//        break;
//    case '9':
//        if (!hal.dmp_on) {
//            hal.sensors ^= GYRO_ON;
//            setup_gyro();
//        }
//        break;
//    /* The commands start/stop sending data to the client. */
//    case 'a':
//        hal.report ^= PRINT_ACCEL;
//        break;
//    case 'g':
//        hal.report ^= PRINT_GYRO;
//        break;
//    case 'q':
//        hal.report ^= PRINT_QUAT;
//        break;
//    /* The hardware self test can be run without any interaction with the
//     * MPL since it's completely localized in the gyro driver. Logging is
//     * assumed to be enabled; otherwise, a couple LEDs could probably be used
//     * here to display the test results.
//     */
//    case 't':
//        run_self_test();
//        break;
//    /* Depending on your application, sensor data may be needed at a faster or
//     * slower rate. These commands can speed up or slow down the rate at which
//     * the sensor data is pushed to the MPL.
//     *
//     * In this example, the compass rate is never changed.
//     */
//    case '1':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(10);
//        else
//            mpu_set_sample_rate(10);
//        break;
//    case '2':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(20);
//        else
//            mpu_set_sample_rate(20);
//        break;
//    case '3':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(40);
//        else
//            mpu_set_sample_rate(40);
//        break;
//    case '4':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(50);
//        else
//            mpu_set_sample_rate(50);
//        break;
//    case '5':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(100);
//        else
//            mpu_set_sample_rate(100);
//        break;
//    case '6':
//        if (hal.dmp_on)
//            dmp_set_fifo_rate(200);
//        else
//            mpu_set_sample_rate(200);
//        break;
//	case ',':
//        /* Set hardware to interrupt on gesture event only. This feature is
//         * useful for keeping the MCU asleep until the DMP detects as a tap or
//         * orientation event.
//         */
//        dmp_set_interrupt_mode(DMP_INT_GESTURE);
//        break;
//    case '.':
//        /* Set hardware to interrupt periodically. */
//        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
//        break;
//    case '7':
//        /* Reset pedometer. */
//        dmp_set_pedometer_step_count(0);
//        dmp_set_pedometer_walk_time(0);
//        break;
//    case 'f':
//        /* Toggle DMP. */
//        if (hal.dmp_on) {
//            unsigned short dmp_rate;
//            hal.dmp_on = 0;
//            mpu_set_dmp_state(0);
//            /* Restore FIFO settings. */
//            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            /* When the DMP is used, the hardware sampling rate is fixed at
//             * 200Hz, and the DMP is configured to downsample the FIFO output
//             * using the function dmp_set_fifo_rate. However, when the DMP is
//             * turned off, the sampling rate remains at 200Hz. This could be
//             * handled in inv_mpu.c, but it would need to know that
//             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
//             * put the extra logic in the application layer.
//             */
//            dmp_get_fifo_rate(&dmp_rate);
//            mpu_set_sample_rate(dmp_rate);
//        } else {
//            unsigned short sample_rate;
//            hal.dmp_on = 1;
//            /* Both gyro and accel must be on. */
//            hal.sensors |= ACCEL_ON | GYRO_ON;
//            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
//            /* Preserve current FIFO rate. */
//            mpu_get_sample_rate(&sample_rate);
//            dmp_set_fifo_rate(sample_rate);
//            mpu_set_dmp_state(1);
//        }
//        break;
//    case 'm':
//        /* Test the motion interrupt hardware feature. */
//        hal.motion_int_mode = 1;
//        break;
//    case 'p':
//        /* Read current pedometer count. */
//        dmp_get_pedometer_step_count(pedo_packet);
//        dmp_get_pedometer_walk_time(pedo_packet + 1);
//        send_packet(PACKET_TYPE_PEDO, pedo_packet);
//        break;
//    case 'x':
//        msp430_reset();
//        break;
//    case 'v':
//        /* Toggle LP quaternion.
//         * The DMP features can be enabled/disabled at runtime. Use this same
//         * approach for other features.
//         */
//        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
//        dmp_enable_feature(hal.dmp_features);
//        break;
//    default:
//        break;
//    }
//    hal.rx.cmd = 0;
//}
/*
#define  UART1_Put_Char UsartSend
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	yaw=yaw*10;
	pitch = pitch*10;
	roll=roll*10;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+2);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}
*/
/**************************????********************************************
*????:		void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
*?  ?:		???????????????
????:
	int16_t ax  ??? X?ADC?? ?? :???????
	int16_t ay  ??? Y?ADC?? ?? :???????
	int16_t az  ??? Z?ADC?? ?? :???????
	int16_t gx  ??? X?ADC?? ?? :???????
	int16_t gy  ??? Y?ADC?? ?? :???????
	int16_t gz  ??? Z?ADC?? ?? :???????
	int16_t hx  ??? X?ADC?? ?? :???????
	int16_t hy  ??? Y?ADC?? ?? :???????
	int16_t hz  ??? Z?ADC?? ?? :???????
	
????:??	
*******************************************************************************
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+8);
	UART1_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}



*/




			
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
//static void gyro_data_ready_cb(void)
//{
//    hal.new_gyro = 1;
//}
#define q30  1073741824.0f
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

typedef struct
{
    int16_t  hx;
    int16_t  hy;
    int16_t  hz;
    uint16_t ha;   
}tg_HMC5883L_TYPE;


tg_HMC5883L_TYPE hmc;




short Yaw,Roll,Pitch;
short gyro[3], accel[3], sensors;

uint8_t Uart1_Put_Char(unsigned char c)
{
	
  UsartSend(c);
	return 1;
}
	
void Uart1_Send_AF(void)
{
	uint8_t sum = 0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAF);
	sum += Uart1_Put_Char(0x1c);
	sum += Uart1_Put_Char(BYTE1(accel[0]));
	sum += Uart1_Put_Char(BYTE0(accel[0]));
	sum += Uart1_Put_Char(BYTE1(accel[1]));
	sum += Uart1_Put_Char(BYTE0(accel[1]));
	sum += Uart1_Put_Char(BYTE1(accel[2]));
	sum += Uart1_Put_Char(BYTE0(accel[2]));
	sum += Uart1_Put_Char(BYTE1(gyro[0]));
	sum += Uart1_Put_Char(BYTE0(gyro[0]));
	sum += Uart1_Put_Char(BYTE1(gyro[1]));
	sum += Uart1_Put_Char(BYTE0(gyro[1]));
	sum += Uart1_Put_Char(BYTE1(gyro[2]));
	sum += Uart1_Put_Char(BYTE0(gyro[2]));
	
	sum += Uart1_Put_Char(BYTE1(hmc.hx));
	sum += Uart1_Put_Char(BYTE0(hmc.hx));
	sum += Uart1_Put_Char(BYTE1(hmc.hy));
	sum += Uart1_Put_Char(BYTE0(hmc.hy));
	sum += Uart1_Put_Char(BYTE1(hmc.hz));
	sum += Uart1_Put_Char(BYTE0(hmc.hz));
	
//	sum += Uart1_Put_Int16(0);
//	sum += Uart1_Put_Int16(0);
//	sum += Uart1_Put_Int16(0);
 
 int _temp;
	_temp = (int)(Roll*100);
	
	
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	_temp = (int)(Pitch*100);
	
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
		_temp = (int)(Yaw*10);
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	
	Uart1_Put_Char(sum);
}
void mpu6050_timer_callback(unsigned long para)
{
	
		
     unsigned long sensor_timestamp;
   //  short gyro[3], accel[3], sensors;
     unsigned char more;
     long quat[4];
	  
	
	   char ready=0;
		
	   dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);	 
		
	//	PrintChar("in Calculating quaternion steps.....\n");
	 /* Gyro and accel data are written to the FIFO by the DMP in chip
     * frame and hardware units. This behavior is convenient because it
     * keeps the gyro and accel outputs of dmp_read_fifo and
     * mpu_read_fifo consistent.
     */
/*     if (sensors & INV_XYZ_GYRO )
       send_packet(PACKET_TYPE_GYRO, gyro);
     if (sensors & INV_XYZ_ACCEL)
        send_packet(PACKET_TYPE_ACCEL, accel); */
     /* Unlike gyro and accel, quaternions are written to the FIFO in
     * the body frame, q30. The orientation is set by the scalar passed
     * to dmp_set_orientation during initialization.
     */
     if (sensors & INV_WXYZ_QUAT )
	 {
	  //   PrintChar("in Calculating quaternion steps.....\n");
	 	 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 
		// PrintInt(quat[0]);PrintChar(" ");
		// PrintInt(quat[1]);PrintChar(" ");
		// PrintInt(quat[2]);PrintChar(" ");
	//	 PrintInt(quat[3]);PrintChar(" \n\n");
		 
		 ready|=0x01;
		 
		 
		 Pitch  =(short)( asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3); // pitch
  	 Roll =(short) (atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3); // roll
		 Yaw = 	(short)(atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3);
		 
		
		 
	 }
	 if(sensors & INV_XYZ_GYRO)
	 {
//		 PrintInt(gyro[0]);PrintChar(" ");
//		 PrintInt(gyro[1]);PrintChar(" ");
//		 PrintInt(gyro[2]);PrintChar(" \n\n");
			ready|=0x02;
   }
	 if(sensors & INV_XYZ_ACCEL)
	 {
//		 PrintInt(accel[0]);PrintChar(" ");
//		 PrintInt(accel[1]);PrintChar(" ");
//		 PrintInt(accel[2]);PrintChar(" \n");
		 ready|=0x04;
		 }
      //send_packet(PACKET_TYPE_QUAT, quat);
	if (ready == 0x07 ){  
		 
		 ready =0;
  //	  UART1_ReportIMU(Yaw,Pitch, Roll,0,0,0,0);
	//    UART1_ReportMotion(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],hmc.hx,hmc.hy,hmc.hz);
	//	Uart1_Send_AF();
		void CONTROL(float rol, float pit, float yaw);
		CONTROL(Roll,Pitch, Yaw);
		
	//   char id=	Single_Read(0x3C,10);
	//	Print(id);
	}
//  HMC5883L_Start();
HMC5883L_MultRead(&hmc);
//		Print(hmc.hx);
//		Print(hmc.hy);
//		Print(hmc.hz);


HMC5883L_Start();
	//   Delayms(50);
	//   HMC5883L_MultRead(&hmc5883l);   //读磁阻仪数据(速度:慢, ms级延时过程)
	 //  BMP085_Read(&bmp085);			 //读气压计数据(速度:慢, ms级延时过程)
	 //  LED1_OFF();
	   
	   //打印数据
	//	 if(true == sw_adxl345)ADXL345_Printf(&adxl345);	   
	//	 if(true == sw_l3g4200d)L3G4200D_Printf(&l3g4200d);
	//   HMC5883L_Printf(&hmc5883l);
   //   unsigned char tmp[6];
	//Mult_Read(0x3c,3,tmp,6);
	/*
	Print(Single_Read(0x3C,3));
	Print(Single_Read(0x3C,4));
	Print(Single_Read(0x3C,5));
	Print(Single_Read(0x3C,6));
	Print(Single_Read(0x3C,7));
	Print(Single_Read(0x3C,8));
	*/

}	


struct minus_timer mpu6050_timer =
{
	.expires = -250,
	.callback = &mpu6050_timer_callback,
	.data=(unsigned long)&mpu6050_timer
};
			

void LED_GPIO_Config(void)
{	
	/*定义一个GPIO_InitTypeDef 类型的结构体，名字叫GPIO_InitStructure*/ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/*开启GPIO的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	/*选择要用的GPIO引脚*/			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	
	/*设置引脚模式*/				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 

	/*设置引脚速度*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*GPIO脚输出高低电平*/
	GPIO_SetBits(GPIOC,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);						 						 
	
}


static void LedLight(unsigned long para)
{
	static unsigned char i=13;
	GPIO_SetBits(GPIOC,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);	
  GPIO_ResetBits(GPIOC,1<<i);	
	if(i==15) i=13;else i++;
	

}

struct minus_timer LedTimer=
{
      .ticks =5000,
	.expires = -2520,
	.callback = &LedLight,
	.data=(unsigned long)&LedTimer
};

void SYSTICK_Init(void)
{		
	while (SysTick_Config(SystemCoreClock/ 100000));
}



 void Timer4_Init(u16 arr,u16 psc)
 {    
 //???????? IO???  
RCC->APB2ENR|=1<<0;    //??  
RCC->APB1ENR|=1<<2;    //TIM4 ????  
RCC->APB2ENR|=1<<3;    //??PORTB?? 
   
  
 
GPIOB->CRH&=0XFFFFFFF0;//PB8 ??  
GPIOB->CRH|=0X0000000B;//??????       
GPIOB->ODR|=1<<8;//PB8 ??  
 
GPIOB->CRH&=0XFFFFFF0F;//PB9 ??  
GPIOB->CRH|=0X000000B0;//??????       
GPIOB->ODR|=1<<9;//PB9 ??   
   
TIM4->ARR=arr;//??????????   
TIM4->PSC=psc;//???????  
 

 
TIM4->CCMR2|=6<<4;  //CH3 PWM1??      
 //TIM4->CCMR2|=1<<3; //CH3 ?????    
TIM4->CCER|=1<<8;  //OC3  ????      
TIM4->CR1=0x0080;   //ARPE?? 
 
TIM4->CCMR2|=6<<12;  //CH4 PWM1??      
 //TIM4->CCMR2|=1<<11; //CH4 ?????       
TIM4->CCER|=1<<12;   //OC4  ????      
TIM4->CR1=0x0080;   //ARPE?? 
 

 TIM4->CR1|=0x01;    //?????4
 

TIM4->CCR3 = 0;   
TIM4->CCR4 = 0;  //CH6?????                 
          
}    
 void Timer3_Init(u16 arr,u16 psc)
 {    
 

RCC->APB2ENR|=1<<0;    //??  
RCC->APB1ENR|=1<<1;    //TIM4 ????  
RCC->APB2ENR|=1<<3;    //??PORTB?? 
//RCC->APB2ENR|=1<<2;    //??PORTA?? 
	 
	 
 
 
GPIOB->CRL&=0XFFFFFFF0;//PB0 ??  
GPIOB->CRL|=0X0000000B;//??????       
GPIOB->ODR|=1<<0;//PB0 ??  
 
GPIOB->CRL&=0XFFFFFF0F;//PB1 ??  
GPIOB->CRL|=0X000000B0;//??????       
GPIOB->ODR|=1<<1;//PB1 ??   
   
TIM3->ARR=arr;//??????????   
TIM3->PSC=psc;//???????  
 

 
TIM3->CCMR2|=6<<4;  //CH3 PWM1??      
 //TIM4->CCMR2|=1<<3; //CH3 ?????    
TIM3->CCER|=1<<8;  //OC3  ????      
TIM3->CR1=0x0080;   //ARPE?? 
 
TIM3->CCMR2|=6<<12;  //CH4 PWM1??      
 //TIM4->CCMR2|=1<<11; //CH4 ?????       
TIM3->CCER|=1<<12;   //OC4  ????      
TIM3->CR1=0x0080;   //ARPE?? 
 

 TIM3->CR1|=0x01;    //?????4
 
//TIM3->CCR1 = 1000;  //CH1?????  
//TIM3->CCR2 = 4000;  //CH2?????
TIM3->CCR3 = 0;   
TIM3->CCR4 = 0;  //CH6?????                 
          
} 
 

void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9);
	
	Timer4_Init(5000,0);
	Timer3_Init(5000,0);
	
}
#define Moto_PwmMax 5000
int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{	

	PrintChar("\n\r\n\r\n\r");
	
	PrintInt(MOTO1_PWM);
  PrintChar("\r\n");
	PrintInt(MOTO2_PWM);
  PrintChar("\r\n");
	PrintInt(MOTO3_PWM);
  PrintChar("\r\n");
	PrintInt(MOTO4_PWM);
	
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
	
  
	TIM4->CCR3 = MOTO1_PWM; //motor1
	TIM4->CCR4 = MOTO2_PWM; //motor2
	TIM3->CCR3 = MOTO3_PWM; //motor3
	TIM3->CCR4 = MOTO4_PWM; //motor4
}

int main(void)
{
	Moto_Init();
	LED_GPIO_Config();
	Pid_init();
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
  int result;     
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
//  Clock_Enable();
//  GPIO_Configuration(); 
  USART_Configuration();
  i2cInit();//IIC总线的初始化，尼玛纠结了这么长时间
	HMC5883L_Init();
	
char id=	Single_Read(0x3C,10);
Print(id);
//	HMC5883L_Start();
  result = mpu_init();
  if(!result)
  {
	  //mpu_init();
//	  PrintChar("mpu initialization complete......\n ");
	  //mpu_set_sensor
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
//	  	 PrintChar("mpu_set_sensor complete ......\n");
	  }
	  else
	  {
	 // 	 PrintChar("mpu_set_sensor come across error ......\n");
	  }
	  //mpu_configure_fifo
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
	//  	 PrintChar("mpu_configure_fifo complete ......\n");
	  }
	  else
	  {
	//  	 PrintChar("mpu_configure_fifo come across error ......\n");
	  }
	  //mpu_set_sample_rate
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  {
	//  	 PrintChar("mpu_set_sample_rate complete ......\n");
	  }
	  else
	  {
	  //	 PrintChar("mpu_set_sample_rate error ......\n");
	  }
	  //dmp_load_motion_driver_firmvare
	  if(!dmp_load_motion_driver_firmware())
	  {
	//  	PrintChar("dmp_load_motion_driver_firmware complete ......\n");
	  }
	  else
	  {
	//  	PrintChar("dmp_load_motion_driver_firmware come across error ......\n");
	  }
	  //dmp_set_orientation
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  {
	 // 	 PrintChar("dmp_set_orientation complete ......\n");
	  }
	  else
	  {
//	  	 PrintChar("dmp_set_orientation come across error ......\n");
	  }
	  //dmp_enable_feature
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))
	  {
	//  	 PrintChar("dmp_enable_feature complete ......\n");
	  }
	  else
	  {
	  //	 PrintChar("dmp_enable_feature come across error ......\n");
	  }
	  //dmp_set_fifo_rate
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  {
	  //	 PrintChar("dmp_set_fifo_rate complete ......\n");
	  }
	  else
	  {
	  //	 PrintChar("dmp_set_fifo_rate come across error ......\n");
	  }
	  run_self_test();
	  if(!mpu_set_dmp_state(1))
	  {
	 // 	 PrintChar("mpu_set_dmp_state complete ......\n");
	  }
	  else
	  {
	 // 	 PrintChar("mpu_set_dmp_state come across error ......\n");
	  }
  }
  SYSTICK_Init();
  minus_add_timer(&LedTimer);		
  minus_add_timer(&mpu6050_timer);	
	
  minus_sched();
  
}
void Clock_Enable(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

//  Led1=1;
//  Led2=1; 
//  Led3=1; 
//  Led4=1;   	
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
