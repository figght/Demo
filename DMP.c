/**
  ******************************************************************************
  * @file    main.c
  * @author  LYC
  * @version V1.0
  * @date    2014-04-22
  * @brief   ±ü»ðMPU6050 DMPÑÝÊ¾Àý³Ì
  ******************************************************************************
  * @attention
  * ÊµÑéÆ½Ì¨:±ü»ð F103-MINI STM32 ¿ª·¢°å 
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "bsp_exti.h"
#include "mltypes.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "log.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "log.h"
#include "packet.h"

#include "oled.h"
#include "mpu6050.h"

#define TASK_ENABLE 0
extern unsigned int Task_Delay[NumOfTask];
char str_OLED[20];

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

#define MOTION (0)
#define NO_MOTION (1)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

///* Platform-specific information. Kinda like a boardfile. */
//struct platform_data_s {
//    signed char orientation[9];
//};

///* The sensors can be mounted onto the board in any orientation. The mounting
// * matrix seen below tells the MPL how to rotate the raw data from the
// * driver(s).
// * TODO: The following matrices refer to the configuration on internal test
// * boards at Invensense. If needed, please modify the matrices to match the
// * chip-to-body matrix for your particular set up.
// */
//static struct platform_data_s gyro_pdata = {
//
//		 .orientation = { 1, 0, 0,
//                     0, 1, 0,
//                     0, 0, 1}
//};

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern struct inv_sensor_cal_t sensors;

void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

int main(void)
{

    //	inv_error_t result;
    //	unsigned char accel_fsr,  new_temp = 0;
    //	unsigned short gyro_rate, gyro_fsr;
    //	unsigned long timestamp;
    //	struct int_param_s int_param;
    struct int_param_s int_param;

    SysTick_Init();
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    /* ´®¿ÚÍ¨ÐÅ³õÊ¼»¯ */
    USART_Config();
    //MPU6050ÖÐ¶ÏÒý½Å
    EXTI_Pxy_Config();
    //I2C³õÊ¼»¯
    I2C_Bus_Init();
    //OLED³õÊ¼»¯
    OLED_Init();
    //OLEDÇåÆÁ
    OLED_Clear();

    printf("mpu 6050 test start");

    mpu_dmp_Init();

    sprintf(str_OLED, "Init success!");
    OLED_ShowString(0, 0, str_OLED);

    while (1)
    {
        unsigned long sensor_timestamp;
        long data[9];
        int8_t accuracy;
        unsigned long timestamp;
        int new_data = 0;

        get_tick_count(&timestamp);

        /* Temperature data doesn't need to be read with every gyro sample.
		 * Let's make them timer-based like the compass reads.
		 */
        if (timestamp > hal.next_temp_ms)
        {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            //new_temp = 1;
        }

        //	if (hal.new_gyro && hal.dmp_on)
        if (hal.new_gyro)
        //else if(0)
        {
            new_data = 1;
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            //			if (!more)
            //					hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO)
            {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                //				if (new_temp) {
                //						new_temp = 0;
                //						/* Temperature only used for gyro temp comp. */
                //						mpu_get_temperature(&temperature, &sensor_timestamp);
                //						inv_build_temp(temperature, sensor_timestamp);
                //				}
            }
            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT)
            {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        }

        if (new_data)
        {
            inv_execute_on_data();

            /* This function reads bias-compensated sensor data and sensor
			 * fusion outputs from the MPL. The outputs are formatted as seen
			 * in eMPL_outputs.c. This function only needs to be called at the
			 * rate requested by the host.
			 */
            read_from_mpl();
        }
    }
}

/*********************************************END OF FILE**********************/
