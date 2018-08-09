/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ADV7280yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.2.3
 *
 * Author:
 * -------
 *   Leo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2012.02.29  kill bugs
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "adv7280mipi_yuv_Sensor.h"
#include "adv7280mipi_yuv_Camera_Sensor_para.h"
#include "adv7280mipi_yuv_CameraCustomized.h"

static DEFINE_SPINLOCK(ADV7280_drv_lock);

#define PFX "adv7280_camera_sensor"
//#define ADV7280YUV_DEBUG

#ifdef ADV7280YUV_DEBUG
#define SENSORDB(format, args...) xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#else
#define SENSORDB(x,...)
#endif

#define ADV7280_TEST_PATTERN_CHECKSUM (0xfd769299)

kal_bool ADV7280_night_mode_enable = KAL_FALSE;
kal_uint16 ADV7280_CurStatus_AWB = 0;


static void ADV7280_awb_enable(kal_bool enalbe);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static void SetPinOutputLevel(uint32_t pin, u8 high)
{
        mt_set_gpio_pull_enable(pin, GPIO_PULL_ENABLE);
        mt_set_gpio_dir(pin, GPIO_DIR_OUT);
        mt_set_gpio_mode(pin, GPIO_MODE_00);
        mt_set_gpio_pull_select(pin, GPIO_PULL_UP);
        if(high)
            mt_set_gpio_out(pin, GPIO_OUT_ONE);
        else
            mt_set_gpio_out(pin, GPIO_OUT_ZERO);
}

int ADV7280_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	int temp=-1;
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	temp = iWriteRegI2C(puSendCmd , 2, ADV7280_WRITE_ID);
	printk("ADV7280_write_cmos_sensor %d \r\n",temp);
	return temp;
}
kal_uint16 ADV7280_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, ADV7280_READ_ID);

    return get_byte;
}

int ADV7280_write_88(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	int temp=-1;
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	temp = iWriteRegI2C(puSendCmd , 2, ADV7280_WRITE_CSI);
	printk("ADV7280_write_88 %d \r\n",temp);
	return temp;
}
kal_uint16 ADV7280_read_88(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, ADV7280_READ_CSI);

    return get_byte;
}
int ADV7280_write_84(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	int temp=-1;
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	temp = iWriteRegI2C(puSendCmd , 2, 0X84);
	printk("ADV7280_write_84 %d \r\n",temp);
	return temp;
}
kal_uint16 ADV7280_read_84(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };
	kdSetI2CBusNum(SUPPORT_I2C_BUS_NUM2);
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, 0X84);

    return get_byte;
}


/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   ADV7280_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 ADV7280_dummy_pixels = 0, ADV7280_dummy_lines = 0;
kal_bool   ADV7280_MODE_CAPTURE = KAL_FALSE;
kal_bool   ADV7280_NIGHT_MODE = KAL_FALSE;

kal_uint32 MINFramerate = 0;
kal_uint32 MAXFramerate = 0;

kal_uint32 ADV7280_isp_master_clock;
static kal_uint32 ADV7280_g_fPV_PCLK = 30 * 1000000;

kal_uint8 ADV7280_sensor_write_I2C_address = ADV7280_WRITE_ID;
kal_uint8 ADV7280_sensor_read_I2C_address = ADV7280_READ_ID;

UINT8 ADV7280PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT ADV7280SensorConfigData;

#define ADV7280_SET_PAGE0 	ADV7280_write_cmos_sensor(0xfe, 0x00)
#define ADV7280_SET_PAGE1 	ADV7280_write_cmos_sensor(0xfe, 0x01)
#define ADV7280_SET_PAGE2 	ADV7280_write_cmos_sensor(0xfe, 0x02)
#define ADV7280_SET_PAGE3 	ADV7280_write_cmos_sensor(0xfe, 0x03)

/*************************************************************************
 * FUNCTION
 *	ADV7280_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of ADV7280 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_ADV7280_Shutter */


/*************************************************************************
 * FUNCTION
 *	ADV7280_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of ADV7280 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 ADV7280_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	//temp_reg1 = ADV7280_read_cmos_sensor(0x04);
	//temp_reg2 = ADV7280_read_cmos_sensor(0x03);

	//shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return 0;//shutter;
} /* ADV7280_read_shutter */

static void ADV7280_Set_Mirrorflip(kal_uint8 image_mirror)
{
	SENSORDB("image_mirror = %d\n", image_mirror);

	switch (image_mirror)
	{
		case IMAGE_NORMAL://IMAGE_NORMAL:
			//ADV7280_write_cmos_sensor(0x17,0x14);//bit[1][0]
	//		write_cmos_sensor(0x92,0x03);
	//		write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
			//ADV7280_write_cmos_sensor(0x17,0x15);
	//		GC2355_write_cmos_sensor(0x92,0x03);
	//		GC2355_write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
			//ADV7280_write_cmos_sensor(0x17,0x16);
	//		GC2355_write_cmos_sensor(0x92,0x02);
	//		GC2355_write_cmos_sensor(0x94,0x0b);
			break;
		case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
			//ADV7280_write_cmos_sensor(0x17,0x17);
	//		GC2355_write_cmos_sensor(0x92,0x02);
	//		GC2355_write_cmos_sensor(0x94,0x0b);
			break;
	}


}

static void ADV7280_set_AE_mode(kal_bool AE_enable)
{

	SENSORDB("[ADV7280]enter ADV7280_set_AE_mode function, AE_enable = %d\n ", AE_enable);
    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
			SENSORDB("[ADV7280]enter ADV7280_set_AE_mode function 1\n ");
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0x4f, 0x01);
    }
    else
    {
        // turn off AEC/AGC
			SENSORDB("[ADV7280]enter ADV7280_set_AE_mode function 2\n ");
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0x4f, 0x00);
    }
}


void ADV7280_set_contrast(UINT16 para)
{
    SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_contrast function:\n ");
#if 1
    switch (para)
    {
        case ISP_CONTRAST_LOW:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd3, 0x30);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			Sleep(200);
			break;
        case ISP_CONTRAST_HIGH:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd3, 0x60);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			Sleep(200);
			break;
        case ISP_CONTRAST_MIDDLE:
        default:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd3, 0x40);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			Sleep(200);
			break;
        //default:
		//	break;
    }
    SENSORDB("[ADV7280]exit ADV7280_set_contrast function:\n ");
    return;
#endif
}

UINT32 ADV7280_MIPI_SetMaxFramerateByScenario(
  MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
{
	SENSORDB("scenarioId = %d\n", scenarioId);
}

UINT32 ADV7280SetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("[ADV7280SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);

	if(bEnable)
	{
		/*ADV7280_write_cmos_sensor(0xfe,0x00);
		ADV7280_write_cmos_sensor(0x40,0x08);
		ADV7280_write_cmos_sensor(0x41,0x00);
		ADV7280_write_cmos_sensor(0x42,0x00);
		ADV7280_write_cmos_sensor(0x4f,0x00);
		ADV7280_write_cmos_sensor(0xfe,0x00);
		ADV7280_write_cmos_sensor(0x03,0x03);
		ADV7280_write_cmos_sensor(0x04,0x9c);
		ADV7280_write_cmos_sensor(0x71,0x20);
		ADV7280_write_cmos_sensor(0x72,0x40);
		ADV7280_write_cmos_sensor(0x73,0x80);
		ADV7280_write_cmos_sensor(0x74,0x80);
		ADV7280_write_cmos_sensor(0x75,0x80);
		ADV7280_write_cmos_sensor(0x76,0x80);
		ADV7280_write_cmos_sensor(0x7a,0x80);
		ADV7280_write_cmos_sensor(0x7b,0x80);
		ADV7280_write_cmos_sensor(0x7c,0x80);
		ADV7280_write_cmos_sensor(0x77,0x40);
		ADV7280_write_cmos_sensor(0x78,0x40);
		ADV7280_write_cmos_sensor(0x79,0x40);
		ADV7280_write_cmos_sensor(0xfe,0x00);
		ADV7280_write_cmos_sensor(0xfe,0x01);
		ADV7280_write_cmos_sensor(0x12,0x00);
		ADV7280_write_cmos_sensor(0x13,0x30);
		ADV7280_write_cmos_sensor(0x44,0x00);
		ADV7280_write_cmos_sensor(0x45,0x00);
		ADV7280_write_cmos_sensor(0xfe,0x00);
		ADV7280_write_cmos_sensor(0xd0,0x40);
		ADV7280_write_cmos_sensor(0xd1,0x20);
		ADV7280_write_cmos_sensor(0xd2,0x20);
		ADV7280_write_cmos_sensor(0xd3,0x40);
		ADV7280_write_cmos_sensor(0xd5,0x00);
		ADV7280_write_cmos_sensor(0xd8,0x00);
		ADV7280_write_cmos_sensor(0xdd,0x00);
		ADV7280_write_cmos_sensor(0xde,0x00);
		ADV7280_write_cmos_sensor(0xe4,0x00);
		ADV7280_write_cmos_sensor(0xeb,0x00);
		ADV7280_write_cmos_sensor(0xa4,0x00);
		ADV7280_write_cmos_sensor(0x4c,0x21);

		ADV7280_write_cmos_sensor(0xfe,0x00);*/


	}
	else
	{
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
		//ADV7280_write_cmos_sensor(0x4c, 0x20);
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
	}

	return ERROR_NONE;
}


void ADV7280_set_brightness(UINT16 para)
{

	SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_brightness function:\n ");
#if 1
	//return;
    switch (para)
    {
        case ISP_BRIGHT_LOW:
		//case AE_EV_COMP_n13:
			//ADV7280_write_cmos_sensor(0xd5, 0xc0);
			Sleep(200);

		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x30);
			//ADV7280_SET_PAGE0;
		break;
        case ISP_BRIGHT_HIGH:
		//case AE_EV_COMP_13:
			//ADV7280_write_cmos_sensor(0xd5, 0x40);
			//Sleep(200);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x90);
		//	ADV7280_SET_PAGE0;
			break;
        case ISP_BRIGHT_MIDDLE:
        default:
		//case AE_EV_COMP_00:
			//ADV7280_write_cmos_sensor(0xd5, 0x00);
			//Sleep(200);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x60);
		//	ADV7280_SET_PAGE0;
		break;
		//	return KAL_FALSE;
		//	break;
    }
    SENSORDB("[ADV7280]exit ADV7280_set_brightness function:\n ");
    return;
#endif
}
void ADV7280_set_saturation(UINT16 para)
{
	SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_saturation function:\n ");

    switch (para)
    {
        case ISP_SAT_HIGH:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd1, 0x45);
			//ADV7280_write_cmos_sensor(0xd2, 0x45);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case ISP_SAT_LOW:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd1, 0x28);
			//ADV7280_write_cmos_sensor(0xd2, 0x28);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case ISP_SAT_MIDDLE:
        default:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0xd1, 0x34);
			//ADV7280_write_cmos_sensor(0xd2, 0x34);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
		//	return KAL_FALSE;
		//	break;
    }
	SENSORDB("[ADV7280]exit ADV7280_set_saturation function:\n ");
     return;

}

void ADV7280_set_iso(UINT16 para)
{

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_iso:\n ");
    switch (para)
	{
        case AE_ISO_100:
             //ISO 100
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x44, 0x00);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case AE_ISO_200:
             //ISO 200
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x44, 0x01);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case AE_ISO_400:
             //ISO 400
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x44, 0x02);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
		case AE_ISO_AUTO:
		default:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x44, 0x02);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
	}
    return;
}



void ADV7280StreamOn(void)
{
	//Sleep(150);
    //ADV7280_write_cmos_sensor(0xfe,0x03);
   // ADV7280_write_cmos_sensor(0x10,0x94);
    //ADV7280_write_cmos_sensor(0xfe,0x00);

	kal_uint16 TEMP=0;

	SENSORDB("ADV7280StreamOn  \r\n");
	/*//:Color Bars 480p MIPI Out:
	//42 0F 00 ; Exit power down mode
	ADV7280_write_cmos_sensor(0x0F,0X00);
	//42 0E 00 ; Enter User Sub Map
	ADV7280_write_cmos_sensor(0x0E,0X00);
	//42 0C 37 ; Force Free-run mode
	ADV7280_write_cmos_sensor(0x0C,0X37);
	//42 02 50 ; Force standard to NTSC-M
	ADV7280_write_cmos_sensor(0x02,0X50);
	//42 14 11 ; Set Free-run pattern to color bars
	ADV7280_write_cmos_sensor(0x14,0X11);
	//42 03 4E ; ADI Required Write
	ADV7280_write_cmos_sensor(0x03,0X4E);
	//42 04 57 ; Enable Intrq pin
        ADV7280_write_cmos_sensor(0x04,0X57);
	//42 13 00 ; Enable INTRQ output driver
	 ADV7280_write_cmos_sensor(0x13,0X00);
	//42 17 41 ; select SH1
	 ADV7280_write_cmos_sensor(0x17,0X41);
	//42 1D C0 ; Tri-State LLC output driver
	ADV7280_write_cmos_sensor(0x1D,0XC0);
	//42 52 CD ; ADI Required Write
	ADV7280_write_cmos_sensor(0x52,0XCD);
	//42 80 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x80,0X51);
	//42 81 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x81,0X51);
	//42 82 68 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x82,0X68);
	//42 FD 84 ; Set VPP Map Address
	ADV7280_write_cmos_sensor(0xFD,0X84);
	//84 A3 00 ; ADI Required Write
	ADV7280_write_84(0xA3,0X00);
	//84 5B 00 ; Advanced Timing Enabled
	ADV7280_write_84(0x5B,0X00);
	//84 55 80 ; Enable I2P
	ADV7280_write_84(0x55,0X80);
	//42 FE 88 ; Set CSI Map Address
	ADV7280_write_cmos_sensor(0xFE,0X88);
	//88 01 20 ; ADI Required Write
	ADV7280_write_88(0x01,0X20);
	//88 02 28 ; ADI Required Write
	ADV7280_write_88(0x02,0X28);
	//88 03 38 ; ADI Required Write
	ADV7280_write_88(0x03,0X38);
	//88 04 30 ; ADI Required Write
	ADV7280_write_88(0x04,0X30);
	//88 05 30 ; ADI Required Write
	ADV7280_write_88(0x05,0X30);
	//88 06 80 ; ADI Required Write
	ADV7280_write_88(0x06,0X80);
	//88 07 70 ; ADI Required Write
	ADV7280_write_88(0x07,0X70);
	//88 08 50 ; ADI Required Write
	ADV7280_write_88(0x08,0X50);
	//88 DE 02 ; Power up D-PHY
	ADV7280_write_88(0xDE,0X02);
	//88 D2 F7 ; ADI Required Write
	ADV7280_write_88(0xD2,0XF7);
	//88 D8 65 ; ADI Required Write
	ADV7280_write_88(0xD8,0X65);
	//88 E0 09 ; ADI Required Write
	ADV7280_write_88(0xE0,0X09);
	//88 2C 00 ; ADI Required Write
	ADV7280_write_88(0x2C,0X00);
	//88 1D 80 ; ADI Required Write
	ADV7280_write_88(0x1D,0X80);
	//88 00 00 ; Power up MIPI CSI-2 Tx
	ADV7280_write_88(0x00,0X00);
	end*/


	/*//##CVBS AUTODETECT##
	//:AUTODETECT CVBS Single Ended In Ain 1, MIPI Out:
	//42 0F 00 ; Exit Power Down Mode
	ADV7280_write_cmos_sensor(0x0F,0X00);
	//42 00 00 ; INSEL = CVBS in on Ain 1
	ADV7280_write_cmos_sensor(0x00,0X00);
	//42 0E 80 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x0E,0X80);
	//42 9C 00 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x9C,0X00);
	//42 9C FF ; ADI Required Write
	ADV7280_write_cmos_sensor(0x9C,0XFF);
	//42 0E 00 ; Enter User Sub Map
	ADV7280_write_cmos_sensor(0x0E,0X00);
	//42 03 4E ; ADI Required Write
	ADV7280_write_cmos_sensor(0x03,0X4E);
	//42 04 57 ; Power-up INTRQ pin
	ADV7280_write_cmos_sensor(0x04,0X57);
	//42 13 00 ; Enable INTRQ output driver
	ADV7280_write_cmos_sensor(0x13,0X00);
	//42 17 41 ; select SH1
	ADV7280_write_cmos_sensor(0x17,0X41);
	//42 1D C0 ; Tri-State LLC output driver
	ADV7280_write_cmos_sensor(0x1D,0XC0);
	//42 52 CD ; ADI Required Write
	ADV7280_write_cmos_sensor(0x52,0XCD);
	//42 80 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x80,0X51);
	//42 81 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x81,0X51);
	//42 82 68 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x82,0X68);
	//42 FE 88 ; Set CSI Map Address
	ADV7280_write_cmos_sensor(0xFE,0X88);
	//88 DE 02 ; Power up MIPI D-PHY
	ADV7280_write_88(0xDE,0X02);
	//88 D2 F7 ; ADI Required Write
	ADV7280_write_88(0xD2,0XF7);
	//88 D8 65 ; ADI Required Write
	ADV7280_write_88(0xD8,0X65);
	//88 E0 09 ; ADI Required Write
	ADV7280_write_88(0xE0,0X09);
	//88 2C 00 ; ADI Required Write
	ADV7280_write_88(0x2C,0X00);
	//88 00 00 ; Power up MIPI CSI-2 Tx
	ADV7280_write_88(0x00,0X00);
	//END*/

//##CVBS Autodetect, Progressive Out##
//:I2P AUTODETECT CVBS Single Ended In Ain 1, 480p/576p MIPI Out:
	//42 0F 00 ; Exit power down mode
	ADV7280_write_cmos_sensor(0x0F,0X00);
	//42 00 00 ; INSEL = CVBS in on Ain 1
	ADV7280_write_cmos_sensor(0x00,0X03);
	//42 0E 80 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x0E,0X80);
	//42 9C 00 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x9C,0X00);
	//42 9C FF ; ADI Required Write
	ADV7280_write_cmos_sensor(0x9C,0XFF);
	//42 0E 00 ; Enter User Sub Map
	ADV7280_write_cmos_sensor(0x0E,0X00);
	//42 03 4E ; ADI Required Write
	ADV7280_write_cmos_sensor(0x03,0X4E);
	//42 04 57 ; Enable Intrq pin
	ADV7280_write_cmos_sensor(0x04,0X57);
	//42 13 00 ; Enable INTRQ output driver
	ADV7280_write_cmos_sensor(0x13,0X00);
	//42 17 41 ; select SH1
	ADV7280_write_cmos_sensor(0x17,0X41);
	//42 1D C0 ; Tri-State LLC output driver
	ADV7280_write_cmos_sensor(0x1D,0XC0);
	//42 52 CD ; ADI Required Write
	ADV7280_write_cmos_sensor(0x52,0XCD);
	//42 80 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x80,0X51);
	//42 81 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x81,0X51);
	//42 82 68 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x82,0X68);
	//42 FD 84 ; Set VPP Map Address
	ADV7280_write_cmos_sensor(0xFD,0X84);
	//84 A3 00 ; ADI Required Write
	ADV7280_write_84(0xA3,0X00);
	//84 5B 00 ; Advanced Timing Enabled
	ADV7280_write_84(0x5B,0X00);
	//84 55 80 ; Enable I2P
	ADV7280_write_84(0x55,0X80);
	//42 FE 88 ; Set CSI Map Address
	ADV7280_write_cmos_sensor(0xFE,0X88);
	//88 01 20 ; ADI Required Write
	ADV7280_write_88(0x01,0X20);
	//88 02 28 ; ADI Required Write
	ADV7280_write_88(0x02,0X28);
	//88 03 38 ; ADI Required Write
	ADV7280_write_88(0x03,0X38);
	//88 04 30 ; ADI Required Write
	ADV7280_write_88(0x04,0X30);
	//88 05 30 ; ADI Required Write
	ADV7280_write_88(0x05,0X30);
	//88 06 80 ; ADI Required Write
	ADV7280_write_88(0x06,0X80);
	//88 07 70 ; ADI Required Write
	ADV7280_write_88(0x07,0X70);
	//88 08 50 ; ADI Required Write
	ADV7280_write_88(0x08,0X50);
	//88 DE 02 ; Power up D-PHY
	ADV7280_write_88(0xDE,0X02);
	//88 D2 F7 ; ADI Required Write
	ADV7280_write_88(0xD2,0XF7);
	//88 D8 65 ; ADI Required Write
	ADV7280_write_88(0xD8,0X65);
	//88 E0 09 ; ADI Required Write
	ADV7280_write_88(0xE0,0X09);
	//88 2C 00 ; ADI Required Write
	ADV7280_write_88(0x2C,0X00);
	//88 1D 80 ; ADI Required Write
	ADV7280_write_88(0x1D,0X80);
	//88 00 00 ; Power up CSI Block
	ADV7280_write_88(0x00,0X00);
	//End

	//88 03 38 ; ADI Required Write

	//88 04 30 ; ADI Required Write

	//88 05 30 ; ADI Required Write

	//88 06 80 ; ADI Required Write

	//88 07 70 ; ADI Required Write

	//88 08 50 ; ADI Required Write

	//88 DE 02 ; Power up D-PHY

	//88 D2 F7 ; ADI Required Write

	//88 D8 65 ; ADI Required Write

	//88 E0 09 ; ADI Required Write

	//88 2C 00 ; ADI Required Write

	//88 1D 80 ; ADI Required Write

	//88 00 00 ; Power up MIPI CSI-2 Tx


    Sleep(50);
}

void ADV7280_MIPI_GetDelayInfo(uintptr_t delayAddr)
{
    SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
    pDelayInfo->InitDelay = 2;
    pDelayInfo->EffectDelay = 2;
    pDelayInfo->AwbDelay = 2;
    pDelayInfo->AFSwitchDelayFrame = 50;
}

UINT32 ADV7280_MIPI_GetDefaultFramerateByScenario(
  MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             *pframeRate = 300;
             break;
        default:
             *pframeRate = 300;
          break;
    }

  return ERROR_NONE;
}

void ADV7280_MIPI_SetMaxMinFps(UINT32 u2MinFrameRate, UINT32 u2MaxFrameRate)
{
	SENSORDB("ADV7280_MIPI_SetMaxMinFps+ :FrameRate= %d %d\n",u2MinFrameRate,u2MaxFrameRate);
	spin_lock(&ADV7280_drv_lock);
	MINFramerate = u2MinFrameRate;
	MAXFramerate = u2MaxFrameRate;
	spin_unlock(&ADV7280_drv_lock);
	return;
}

void ADV7280_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	SENSORDB("[GC0329]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
         // ADV7280_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
         // ADV7280_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          //ADV7280_awb_enable(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
		   if (((AWB_MODE_OFF == ADV7280_CurStatus_AWB) ||
        		(AWB_MODE_AUTO == ADV7280_CurStatus_AWB)))
        	{
         			 //ADV7280_awb_enable(KAL_TRUE);
         	}
      break;
      default:
      	break;
   }
   SENSORDB("[GC0329]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}


void ADV7280_MIPI_GetExifInfo(uintptr_t exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
//    pExifInfo->AEISOSpeed = ADV7280_Driver.isoSpeed;
//    pExifInfo->AWBMode = S5K4ECGX_Driver.awbMode;
//    pExifInfo->CapExposureTime = S5K4ECGX_Driver.capExposureTime;
//    pExifInfo->FlashLightTimeus = 0;
//   pExifInfo->RealISOValue = (S5K4ECGX_MIPI_ReadGain()*57) >> 8;
        //S5K4ECGX_Driver.isoSpeed;
}

#if 0
void ADV7280_MIPI_get_AEAWB_lock(uintptr_t pAElockRet32, uintptr_t pAWBlockRet32)
{
    *pAElockRet32 = 1;
    *pAWBlockRet32 = 1;
    SENSORDB("[ADV7280]GetAEAWBLock,AE=%d ,AWB=%d\n,",(int)*pAElockRet32, (int)*pAWBlockRet32);
}
#endif

/*************************************************************************
 * FUNCTION
 *	ADV7280_write_reg
 *
 * DESCRIPTION
 *	This function set the register of ADV7280.
 *
 * PARAMETERS
 *	addr : the register index of ADV7280
 *  para : setting parameter of the specified register of ADV7280
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_write_reg(kal_uint32 addr, kal_uint32 para)
{
	//ADV7280_write_cmos_sensor(addr, para);
} /* ADV7280_write_reg() */


/*************************************************************************
 * FUNCTION
 *	ADV7280_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from ADV7280.
 *
 * PARAMETERS
 *	addr : the register index of ADV7280
 *
 * RETURNS
 *	the data that read from ADV7280
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 ADV7280_read_reg(kal_uint32 addr)
{
	return 0;//ADV7280_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	ADV7280_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void ADV7280_awb_enable(kal_bool enalbe)
{
	kal_uint16 temp_AWB_reg = 0;

	//ADV7280_write_cmos_sensor(0xfe, 0x00);
	//temp_AWB_reg = ADV7280_read_cmos_sensor(0x42);

	if (enalbe)
	{
	//	ADV7280_write_cmos_sensor(0x42, (temp_AWB_reg |0x02));
	}
	else
	{
	//	ADV7280_write_cmos_sensor(0x42, (temp_AWB_reg & (~0x02)));
	}
	//ADV7280_write_cmos_sensor(0xfe, 0x00);

}


/*************************************************************************
* FUNCTION
*	ADV7280_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ADV7280GammaSelect(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
	{
		case ADV7280_RGB_Gamma_m1:						//smallest gamma curve
			/*ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x06);
			ADV7280_write_cmos_sensor(0xc0, 0x12);
			ADV7280_write_cmos_sensor(0xc1, 0x22);
			ADV7280_write_cmos_sensor(0xc2, 0x35);
			ADV7280_write_cmos_sensor(0xc3, 0x4b);
			ADV7280_write_cmos_sensor(0xc4, 0x5f);
			ADV7280_write_cmos_sensor(0xc5, 0x72);
			ADV7280_write_cmos_sensor(0xc6, 0x8d);
			ADV7280_write_cmos_sensor(0xc7, 0xa4);
			ADV7280_write_cmos_sensor(0xc8, 0xb8);
			ADV7280_write_cmos_sensor(0xc9, 0xc8);
			ADV7280_write_cmos_sensor(0xca, 0xd4);
			ADV7280_write_cmos_sensor(0xcb, 0xde);
			ADV7280_write_cmos_sensor(0xcc, 0xe6);
			ADV7280_write_cmos_sensor(0xcd, 0xf1);
			ADV7280_write_cmos_sensor(0xce, 0xf8);
			ADV7280_write_cmos_sensor(0xcf, 0xfd);*/
			break;
		case ADV7280_RGB_Gamma_m2:
			/*ADV7280_write_cmos_sensor(0xBF, 0x08);
			ADV7280_write_cmos_sensor(0xc0, 0x0F);
			ADV7280_write_cmos_sensor(0xc1, 0x21);
			ADV7280_write_cmos_sensor(0xc2, 0x32);
			ADV7280_write_cmos_sensor(0xc3, 0x43);
			ADV7280_write_cmos_sensor(0xc4, 0x50);
			ADV7280_write_cmos_sensor(0xc5, 0x5E);
			ADV7280_write_cmos_sensor(0xc6, 0x78);
			ADV7280_write_cmos_sensor(0xc7, 0x90);
			ADV7280_write_cmos_sensor(0xc8, 0xA6);
			ADV7280_write_cmos_sensor(0xc9, 0xB9);
			ADV7280_write_cmos_sensor(0xcA, 0xC9);
			ADV7280_write_cmos_sensor(0xcB, 0xD6);
			ADV7280_write_cmos_sensor(0xcC, 0xE0);
			ADV7280_write_cmos_sensor(0xcD, 0xEE);
			ADV7280_write_cmos_sensor(0xcE, 0xF8);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);*/
			break;

		case ADV7280_RGB_Gamma_m3:
			/*ADV7280_write_cmos_sensor(0xbf , 0x0b);
			ADV7280_write_cmos_sensor(0xc0 , 0x17);
			ADV7280_write_cmos_sensor(0xc1 , 0x2a);
			ADV7280_write_cmos_sensor(0xc2 , 0x41);
			ADV7280_write_cmos_sensor(0xc3 , 0x54);
			ADV7280_write_cmos_sensor(0xc4 , 0x66);
			ADV7280_write_cmos_sensor(0xc5 , 0x74);
			ADV7280_write_cmos_sensor(0xc6 , 0x8c);
			ADV7280_write_cmos_sensor(0xc7 , 0xa3);
			ADV7280_write_cmos_sensor(0xc8 , 0xb5);
			ADV7280_write_cmos_sensor(0xc9 , 0xc4);
			ADV7280_write_cmos_sensor(0xca , 0xd0);
			ADV7280_write_cmos_sensor(0xcb , 0xdb);
			ADV7280_write_cmos_sensor(0xcc , 0xe5);
			ADV7280_write_cmos_sensor(0xcd , 0xf0);
			ADV7280_write_cmos_sensor(0xce , 0xf7);
			ADV7280_write_cmos_sensor(0xcf , 0xff);*/
			break;

		case ADV7280_RGB_Gamma_m4:
			/*ADV7280_write_cmos_sensor(0xBF, 0x0E);
			ADV7280_write_cmos_sensor(0xc0, 0x1C);
			ADV7280_write_cmos_sensor(0xc1, 0x34);
			ADV7280_write_cmos_sensor(0xc2, 0x48);
			ADV7280_write_cmos_sensor(0xc3, 0x5A);
			ADV7280_write_cmos_sensor(0xc4, 0x6B);
			ADV7280_write_cmos_sensor(0xc5, 0x7B);
			ADV7280_write_cmos_sensor(0xc6, 0x95);
			ADV7280_write_cmos_sensor(0xc7, 0xAB);
			ADV7280_write_cmos_sensor(0xc8, 0xBF);
			ADV7280_write_cmos_sensor(0xc9, 0xCE);
			ADV7280_write_cmos_sensor(0xcA, 0xD9);
			ADV7280_write_cmos_sensor(0xcB, 0xE4);
			ADV7280_write_cmos_sensor(0xcC, 0xEC);
			ADV7280_write_cmos_sensor(0xcD, 0xF7);
			ADV7280_write_cmos_sensor(0xcE, 0xFD);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);*/
			break;

		case ADV7280_RGB_Gamma_m5:
			/*ADV7280_write_cmos_sensor(0xBF, 0x10);
			ADV7280_write_cmos_sensor(0xc0, 0x20);
			ADV7280_write_cmos_sensor(0xc1, 0x38);
			ADV7280_write_cmos_sensor(0xc2, 0x4E);
			ADV7280_write_cmos_sensor(0xc3, 0x63);
			ADV7280_write_cmos_sensor(0xc4, 0x76);
			ADV7280_write_cmos_sensor(0xc5, 0x87);
			ADV7280_write_cmos_sensor(0xc6, 0xA2);
			ADV7280_write_cmos_sensor(0xc7, 0xB8);
			ADV7280_write_cmos_sensor(0xc8, 0xCA);
			ADV7280_write_cmos_sensor(0xc9, 0xD8);
			ADV7280_write_cmos_sensor(0xcA, 0xE3);
			ADV7280_write_cmos_sensor(0xcB, 0xEB);
			ADV7280_write_cmos_sensor(0xcC, 0xF0);
			ADV7280_write_cmos_sensor(0xcD, 0xF8);
			ADV7280_write_cmos_sensor(0xcE, 0xFD);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);*/
			break;

		case ADV7280_RGB_Gamma_m6:										// largest gamma curve
			/*ADV7280_write_cmos_sensor(0xBF, 0x14);
			ADV7280_write_cmos_sensor(0xc0, 0x28);
			ADV7280_write_cmos_sensor(0xc1, 0x44);
			ADV7280_write_cmos_sensor(0xc2, 0x5D);
			ADV7280_write_cmos_sensor(0xc3, 0x72);
			ADV7280_write_cmos_sensor(0xc4, 0x86);
			ADV7280_write_cmos_sensor(0xc5, 0x95);
			ADV7280_write_cmos_sensor(0xc6, 0xB1);
			ADV7280_write_cmos_sensor(0xc7, 0xC6);
			ADV7280_write_cmos_sensor(0xc8, 0xD5);
			ADV7280_write_cmos_sensor(0xc9, 0xE1);
			ADV7280_write_cmos_sensor(0xcA, 0xEA);
			ADV7280_write_cmos_sensor(0xcB, 0xF1);
			ADV7280_write_cmos_sensor(0xcC, 0xF5);
			ADV7280_write_cmos_sensor(0xcD, 0xFB);
			ADV7280_write_cmos_sensor(0xcE, 0xFE);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);*/
			break;
		case ADV7280_RGB_Gamma_night:									//Gamma for night mode
			/*ADV7280_write_cmos_sensor(0xBF, 0x0B);
			ADV7280_write_cmos_sensor(0xc0, 0x16);
			ADV7280_write_cmos_sensor(0xc1, 0x29);
			ADV7280_write_cmos_sensor(0xc2, 0x3C);
			ADV7280_write_cmos_sensor(0xc3, 0x4F);
			ADV7280_write_cmos_sensor(0xc4, 0x5F);
			ADV7280_write_cmos_sensor(0xc5, 0x6F);
			ADV7280_write_cmos_sensor(0xc6, 0x8A);
			ADV7280_write_cmos_sensor(0xc7, 0x9F);
			ADV7280_write_cmos_sensor(0xc8, 0xB4);
			ADV7280_write_cmos_sensor(0xc9, 0xC6);
			ADV7280_write_cmos_sensor(0xcA, 0xD3);
			ADV7280_write_cmos_sensor(0xcB, 0xDD);
			ADV7280_write_cmos_sensor(0xcC, 0xE5);
			ADV7280_write_cmos_sensor(0xcD, 0xF1);
			ADV7280_write_cmos_sensor(0xcE, 0xFA);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);*/
			break;
		default:
			//ADV7280_RGB_Gamma_m1
			/*ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_write_cmos_sensor(0xbf , 0x0b);
			ADV7280_write_cmos_sensor(0xc0 , 0x17);
			ADV7280_write_cmos_sensor(0xc1 , 0x2a);
			ADV7280_write_cmos_sensor(0xc2 , 0x41);
			ADV7280_write_cmos_sensor(0xc3 , 0x54);
			ADV7280_write_cmos_sensor(0xc4 , 0x66);
			ADV7280_write_cmos_sensor(0xc5 , 0x74);
			ADV7280_write_cmos_sensor(0xc6 , 0x8c);
			ADV7280_write_cmos_sensor(0xc7 , 0xa3);
			ADV7280_write_cmos_sensor(0xc8 , 0xb5);
			ADV7280_write_cmos_sensor(0xc9 , 0xc4);
			ADV7280_write_cmos_sensor(0xca , 0xd0);
			ADV7280_write_cmos_sensor(0xcb , 0xdb);
			ADV7280_write_cmos_sensor(0xcc , 0xe5);
			ADV7280_write_cmos_sensor(0xcd , 0xf0);
			ADV7280_write_cmos_sensor(0xce , 0xf7);
			ADV7280_write_cmos_sensor(0xcf , 0xff);*/
			break;
	}
}


/*************************************************************************
 * FUNCTION
 *	ADV7280_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of ADV7280 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from ADV7280
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* ADV7280_config_window */


/*************************************************************************
 * FUNCTION
 *	ADV7280_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 ADV7280_SetGain(kal_uint16 iGain)
{
	return iGain;
}


/*************************************************************************
 * FUNCTION
 *	ADV7280_NightMode
 *
 * DESCRIPTION
 *	This function night mode of ADV7280.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280NightMode(kal_bool bEnable)
{
	SENSORDB("Enter ADV7280NightMode!, bEnable = %d, ADV7280_MPEG4_encode_mode = %d\n", bEnable, ADV7280_MPEG4_encode_mode);

	spin_lock(&ADV7280_drv_lock);
	ADV7280_night_mode_enable = bEnable;
	spin_unlock(&ADV7280_drv_lock);

	if (ADV7280_night_mode_enable)	// night mode
	{
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
		if(MAXFramerate == 15)	// video
		{
			//ADV7280_write_cmos_sensor(0xf7, 0x33);
			//ADV7280_write_cmos_sensor(0xfa, 0x32);
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x3c, 0x00);
		}
		else
		{
			//ADV7280_write_cmos_sensor(0xf7, 0x1b);
			//ADV7280_write_cmos_sensor(0xfa, 0x11);
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x3c, 0x30);
//			Sleep(50);
		}
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
	}
	else
	{
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
		if(MINFramerate == 15)  // video
		{
			//ADV7280_write_cmos_sensor(0xf7, 0x1b);
			//ADV7280_write_cmos_sensor(0xfa, 0x11);
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x3c, 0x00);
		}
		else
		{
			//ADV7280_write_cmos_sensor(0xf7, 0x1b);
			//ADV7280_write_cmos_sensor(0xfa, 0x11);
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x3c, 0x20);
		}
		//ADV7280_write_cmos_sensor(0xfe, 0x00);
	}

} /* ADV7280_NightMode */

/*************************************************************************
* FUNCTION
*	ADV7280_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void ADV7280_Sensor_Init(void)
{




    /*ADV7280_write_cmos_sensor(0xfe,0xf0);
    ADV7280_write_cmos_sensor(0xfe,0xf0);
    ADV7280_write_cmos_sensor(0xfe,0x00);
    ADV7280_write_cmos_sensor(0xfc,0x0e);
    ADV7280_write_cmos_sensor(0xfc,0x0e);
    ADV7280_write_cmos_sensor(0xf2,0x80);
    ADV7280_write_cmos_sensor(0xf3,0x00);
    ADV7280_write_cmos_sensor(0xf7,0x1b);
    ADV7280_write_cmos_sensor(0xf8,0x04);  // from 03 to 04
    ADV7280_write_cmos_sensor(0xf9,0x8e);
    ADV7280_write_cmos_sensor(0xfa,0x11);
     /////////////////////////////////////////////////
	///////////////////   MIPI   ////////////////////
	/////////////////////////////////////////////////
	ADV7280_write_cmos_sensor(0xfe,0x03);
	ADV7280_write_cmos_sensor(0x40,0x08);
	ADV7280_write_cmos_sensor(0x42,0x00);
	ADV7280_write_cmos_sensor(0x43,0x00);
	ADV7280_write_cmos_sensor(0x01,0x03);
	ADV7280_write_cmos_sensor(0x10,0x84);

	ADV7280_write_cmos_sensor(0x01,0x03);
	ADV7280_write_cmos_sensor(0x02,0x00);
	ADV7280_write_cmos_sensor(0x03,0x94);
	ADV7280_write_cmos_sensor(0x04,0x01);
	ADV7280_write_cmos_sensor(0x05,0x40);  // 40      20
	ADV7280_write_cmos_sensor(0x06,0x80);
	ADV7280_write_cmos_sensor(0x11,0x1e);
	ADV7280_write_cmos_sensor(0x12,0x00);
	ADV7280_write_cmos_sensor(0x13,0x05);
	ADV7280_write_cmos_sensor(0x15,0x10);
	ADV7280_write_cmos_sensor(0x21,0x10);
	ADV7280_write_cmos_sensor(0x22,0x01);
	ADV7280_write_cmos_sensor(0x23,0x10);
	ADV7280_write_cmos_sensor(0x24,0x02);
	ADV7280_write_cmos_sensor(0x25,0x10);
	ADV7280_write_cmos_sensor(0x26,0x03);
	ADV7280_write_cmos_sensor(0x29,0x02); //02
	ADV7280_write_cmos_sensor(0x2a,0x0a);   //0a
	ADV7280_write_cmos_sensor(0x2b,0x04);
	ADV7280_write_cmos_sensor(0xfe,0x00);
        /////////////////////////////////////////////////
        /////////////////   CISCTL reg  /////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x00,0x2f);
    ADV7280_write_cmos_sensor(0x01,0x0f);
    ADV7280_write_cmos_sensor(0x02,0x04);
    ADV7280_write_cmos_sensor(0x03,0x04);
    ADV7280_write_cmos_sensor(0x04,0xd0);
    ADV7280_write_cmos_sensor(0x09,0x00);
    ADV7280_write_cmos_sensor(0x0a,0x00);
    ADV7280_write_cmos_sensor(0x0b,0x00);
    ADV7280_write_cmos_sensor(0x0c,0x06);
    ADV7280_write_cmos_sensor(0x0d,0x01);
    ADV7280_write_cmos_sensor(0x0e,0xe8);
    ADV7280_write_cmos_sensor(0x0f,0x02);
    ADV7280_write_cmos_sensor(0x10,0x88);
    ADV7280_write_cmos_sensor(0x16,0x00);
    ADV7280_write_cmos_sensor(0x17,0x14);
    ADV7280_write_cmos_sensor(0x18,0x1a);
    ADV7280_write_cmos_sensor(0x19,0x14);
    ADV7280_write_cmos_sensor(0x1b,0x48);
    ADV7280_write_cmos_sensor(0x1e,0x6b);
    ADV7280_write_cmos_sensor(0x1f,0x28);
    ADV7280_write_cmos_sensor(0x20,0x8b);  // from 89 to 8b
    ADV7280_write_cmos_sensor(0x21,0x49);
    ADV7280_write_cmos_sensor(0x22,0xb0);
    ADV7280_write_cmos_sensor(0x23,0x04);
    ADV7280_write_cmos_sensor(0x24,0x16);
    ADV7280_write_cmos_sensor(0x34,0x20);

        /////////////////////////////////////////////////
        ////////////////////   BLK   ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x26,0x23);
    ADV7280_write_cmos_sensor(0x28,0xff);
    ADV7280_write_cmos_sensor(0x29,0x00);
    ADV7280_write_cmos_sensor(0x33,0x10);
    ADV7280_write_cmos_sensor(0x37,0x20);
	ADV7280_write_cmos_sensor(0x38,0x10);
    ADV7280_write_cmos_sensor(0x47,0x80);
    ADV7280_write_cmos_sensor(0x4e,0x66);
    ADV7280_write_cmos_sensor(0xa8,0x02);
    ADV7280_write_cmos_sensor(0xa9,0x80);

        /////////////////////////////////////////////////
        //////////////////   ISP reg  ///////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x40,0xff);
    ADV7280_write_cmos_sensor(0x41,0x21);
    ADV7280_write_cmos_sensor(0x42,0xcf);
    ADV7280_write_cmos_sensor(0x44,0x01); // 02 yuv
    ADV7280_write_cmos_sensor(0x45,0xa0); // from a8 - a4 a4-a0
    ADV7280_write_cmos_sensor(0x46,0x03);
    ADV7280_write_cmos_sensor(0x4a,0x11);
    ADV7280_write_cmos_sensor(0x4b,0x01);
    ADV7280_write_cmos_sensor(0x4c,0x20);
    ADV7280_write_cmos_sensor(0x4d,0x05);
    ADV7280_write_cmos_sensor(0x4f,0x01);
    ADV7280_write_cmos_sensor(0x50,0x01);
    ADV7280_write_cmos_sensor(0x55,0x01);
    ADV7280_write_cmos_sensor(0x56,0xe0);
    ADV7280_write_cmos_sensor(0x57,0x02);
    ADV7280_write_cmos_sensor(0x58,0x80);

        /////////////////////////////////////////////////
        ///////////////////   GAIN   ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x70,0x70);
    ADV7280_write_cmos_sensor(0x5a,0x84);
    ADV7280_write_cmos_sensor(0x5b,0xc9);
    ADV7280_write_cmos_sensor(0x5c,0xed);
    ADV7280_write_cmos_sensor(0x77,0x74);
    ADV7280_write_cmos_sensor(0x78,0x40);
    ADV7280_write_cmos_sensor(0x79,0x5f);

        /////////////////////////////////////////////////
        ///////////////////   DNDD  /////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x82,0x1f);
    ADV7280_write_cmos_sensor(0x83,0x0b);


        /////////////////////////////////////////////////
        //////////////////   EEINTP  ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x8f,0xff);
    ADV7280_write_cmos_sensor(0x90,0x9f);
    ADV7280_write_cmos_sensor(0x91,0x90);
    ADV7280_write_cmos_sensor(0x92,0x03);
    ADV7280_write_cmos_sensor(0x93,0x03);
    ADV7280_write_cmos_sensor(0x94,0x05);
    ADV7280_write_cmos_sensor(0x95,0x65);
    ADV7280_write_cmos_sensor(0x96,0xf0);

        /////////////////////////////////////////////////
        /////////////////////  ASDE  ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xfe,0x00);
    ADV7280_write_cmos_sensor(0x9a,0x20);
    ADV7280_write_cmos_sensor(0x9b,0x80);
    ADV7280_write_cmos_sensor(0x9c,0x40);
    ADV7280_write_cmos_sensor(0x9d,0x80);
    ADV7280_write_cmos_sensor(0xa1,0x30);
    ADV7280_write_cmos_sensor(0xa2,0x32);
    ADV7280_write_cmos_sensor(0xa4,0x30);
    ADV7280_write_cmos_sensor(0xa5,0x30);
    ADV7280_write_cmos_sensor(0xaa,0x50);
    ADV7280_write_cmos_sensor(0xac,0x22);

        /////////////////////////////////////////////////
        ///////////////////   GAMMA   ///////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xbf,0x08);
    ADV7280_write_cmos_sensor(0xc0,0x16);
    ADV7280_write_cmos_sensor(0xc1,0x28);
    ADV7280_write_cmos_sensor(0xc2,0x41);
    ADV7280_write_cmos_sensor(0xc3,0x5a);
    ADV7280_write_cmos_sensor(0xc4,0x6c);
    ADV7280_write_cmos_sensor(0xc5,0x7a);
    ADV7280_write_cmos_sensor(0xc6,0x96);
    ADV7280_write_cmos_sensor(0xc7,0xac);
    ADV7280_write_cmos_sensor(0xc8,0xbc);
    ADV7280_write_cmos_sensor(0xc9,0xc9);
    ADV7280_write_cmos_sensor(0xca,0xd3);
    ADV7280_write_cmos_sensor(0xcb,0xdd);
    ADV7280_write_cmos_sensor(0xcc,0xe5);
    ADV7280_write_cmos_sensor(0xcd,0xf1);
    ADV7280_write_cmos_sensor(0xce,0xfa);
    ADV7280_write_cmos_sensor(0xcf,0xff);

        /////////////////////////////////////////////////
        ///////////////////   YCP  //////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xd0,0x40);
    ADV7280_write_cmos_sensor(0xd1,0x34);
    ADV7280_write_cmos_sensor(0xd2,0x34);
    ADV7280_write_cmos_sensor(0xd3,0x3c);
    ADV7280_write_cmos_sensor(0xd6,0xf2);
    ADV7280_write_cmos_sensor(0xd7,0x1b);
    ADV7280_write_cmos_sensor(0xd8,0x18);
    ADV7280_write_cmos_sensor(0xdd,0x03);
        /////////////////////////////////////////////////
        ////////////////////   AEC   ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xfe,0x01);
    ADV7280_write_cmos_sensor(0x05,0x30);
    ADV7280_write_cmos_sensor(0x06,0x75);
    ADV7280_write_cmos_sensor(0x07,0x40);
    ADV7280_write_cmos_sensor(0x08,0xb0);
    ADV7280_write_cmos_sensor(0x0a,0xc5);
    ADV7280_write_cmos_sensor(0x0b,0x11);
    ADV7280_write_cmos_sensor(0x0c,0x00);
    ADV7280_write_cmos_sensor(0x12,0x52);
    ADV7280_write_cmos_sensor(0x13,0x38);
    ADV7280_write_cmos_sensor(0x18,0x95);
    ADV7280_write_cmos_sensor(0x19,0x96);
    ADV7280_write_cmos_sensor(0x1f,0x20);
    ADV7280_write_cmos_sensor(0x20,0xc0);
    ADV7280_write_cmos_sensor(0x3e,0x40);
    ADV7280_write_cmos_sensor(0x3f,0x57);
    ADV7280_write_cmos_sensor(0x40,0x7d);
    ADV7280_write_cmos_sensor(0x03,0x60);
    ADV7280_write_cmos_sensor(0x44,0x02);
    /////////////////////////////////////////////////
    ////////////////////   AWB   ////////////////////
    /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x1c,0x91);
    ADV7280_write_cmos_sensor(0x21,0x15);
    ADV7280_write_cmos_sensor(0x50,0x80);
    ADV7280_write_cmos_sensor(0x56,0x04);
    ADV7280_write_cmos_sensor(0x58,0x08);
    ADV7280_write_cmos_sensor(0x59,0x08);
    ADV7280_write_cmos_sensor(0x5b,0x82);  // 02 to 82 to 02
    ADV7280_write_cmos_sensor(0x61,0x8d);
    ADV7280_write_cmos_sensor(0x62,0xa7);
    ADV7280_write_cmos_sensor(0x63,0x00);   // d0 to  00
    ADV7280_write_cmos_sensor(0x65,0x06);
    ADV7280_write_cmos_sensor(0x66,0x06);   // 06 to 03
    ADV7280_write_cmos_sensor(0x67,0x84);
    ADV7280_write_cmos_sensor(0x69,0x08);   // 08 to 20
    ADV7280_write_cmos_sensor(0x6a,0x25);
    ADV7280_write_cmos_sensor(0x6b,0x01);
    ADV7280_write_cmos_sensor(0x6c,0x00);   // 00 to 0c
    ADV7280_write_cmos_sensor(0x6d,0x02);
    ADV7280_write_cmos_sensor(0x6e,0x00);  // f0 to 00
    ADV7280_write_cmos_sensor(0x6f,0x80);
    ADV7280_write_cmos_sensor(0x76,0x80);
    ADV7280_write_cmos_sensor(0x78,0xaf);
    ADV7280_write_cmos_sensor(0x79,0x75);
    ADV7280_write_cmos_sensor(0x7a,0x40);
    ADV7280_write_cmos_sensor(0x7b,0x50);
    ADV7280_write_cmos_sensor(0x7c,0x08); //0c to 08 8.11

    ADV7280_write_cmos_sensor(0xa4,0xb9);
    ADV7280_write_cmos_sensor(0xa5,0xa0);
    ADV7280_write_cmos_sensor(0x90,0xc9);
    ADV7280_write_cmos_sensor(0x91,0xbe);
    ADV7280_write_cmos_sensor(0xa6,0xb8);
    ADV7280_write_cmos_sensor(0xa7,0x95);
    ADV7280_write_cmos_sensor(0x92,0xe6);
    ADV7280_write_cmos_sensor(0x93,0xca);
    ADV7280_write_cmos_sensor(0xa9,0xb6);
    ADV7280_write_cmos_sensor(0xaa,0x89);
    ADV7280_write_cmos_sensor(0x95,0x23);
    ADV7280_write_cmos_sensor(0x96,0xe7);
    ADV7280_write_cmos_sensor(0xab,0x9d);
    ADV7280_write_cmos_sensor(0xac,0x80);
    ADV7280_write_cmos_sensor(0x97,0x43);
    ADV7280_write_cmos_sensor(0x98,0x24);
    ADV7280_write_cmos_sensor(0xae,0xd0);   // b7 to d0
    ADV7280_write_cmos_sensor(0xaf,0x9e);
    ADV7280_write_cmos_sensor(0x9a,0x43);
    ADV7280_write_cmos_sensor(0x9b,0x24);

    ADV7280_write_cmos_sensor(0xb0,0xc0);  // c8 to c0
    ADV7280_write_cmos_sensor(0xb1,0xa8);   // 97 to a8
    ADV7280_write_cmos_sensor(0x9c,0xc4);
    ADV7280_write_cmos_sensor(0x9d,0x44);
    ADV7280_write_cmos_sensor(0xb3,0xb7);
    ADV7280_write_cmos_sensor(0xb4,0x7f);
    ADV7280_write_cmos_sensor(0x9f,0xc7);
    ADV7280_write_cmos_sensor(0xa0,0xc8);
    ADV7280_write_cmos_sensor(0xb5,0x00);
    ADV7280_write_cmos_sensor(0xb6,0x00);
    ADV7280_write_cmos_sensor(0xa1,0x00);
    ADV7280_write_cmos_sensor(0xa2,0x00);
    ADV7280_write_cmos_sensor(0x86,0x60);
    ADV7280_write_cmos_sensor(0x87,0x08);
    ADV7280_write_cmos_sensor(0x88,0x00);
    ADV7280_write_cmos_sensor(0x89,0x00);
    ADV7280_write_cmos_sensor(0x8b,0xde);
    ADV7280_write_cmos_sensor(0x8c,0x80);
    ADV7280_write_cmos_sensor(0x8d,0x00);
    ADV7280_write_cmos_sensor(0x8e,0x00);
    ADV7280_write_cmos_sensor(0x94,0x55);
    ADV7280_write_cmos_sensor(0x99,0xa6);
    ADV7280_write_cmos_sensor(0x9e,0xaa);
    ADV7280_write_cmos_sensor(0xa3,0x0a);
    ADV7280_write_cmos_sensor(0x8a,0x0a);
    ADV7280_write_cmos_sensor(0xa8,0x55);
    ADV7280_write_cmos_sensor(0xad,0x55);
    ADV7280_write_cmos_sensor(0xb2,0x55);
    ADV7280_write_cmos_sensor(0xb7,0x05);
    ADV7280_write_cmos_sensor(0x8f,0x05);
    ADV7280_write_cmos_sensor(0xb8,0xcc);
    ADV7280_write_cmos_sensor(0xb9,0x9a);

        /////////////////////////////////////
        ////////////////////  CC ////////////
        /////////////////////////////////////
    ADV7280_write_cmos_sensor(0xfe,0x01);
    ADV7280_write_cmos_sensor(0xd0,0x38);
    ADV7280_write_cmos_sensor(0xd1,0xfd);
    ADV7280_write_cmos_sensor(0xd2,0x06);
    ADV7280_write_cmos_sensor(0xd3,0xf0);
    ADV7280_write_cmos_sensor(0xd4,0x40);
    ADV7280_write_cmos_sensor(0xd5,0x08);
    ADV7280_write_cmos_sensor(0xd6,0x30);
    ADV7280_write_cmos_sensor(0xd7,0x00);
    ADV7280_write_cmos_sensor(0xd8,0x0a);
    ADV7280_write_cmos_sensor(0xd9,0x16);
    ADV7280_write_cmos_sensor(0xda,0x39);
    ADV7280_write_cmos_sensor(0xdb,0xf8);

        /////////////////////////////////////////////////
        ////////////////////   LSC   ////////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xfe,0x01);
    ADV7280_write_cmos_sensor(0xc1,0x3c);
    ADV7280_write_cmos_sensor(0xc2,0x50);
    ADV7280_write_cmos_sensor(0xc3,0x00);
    ADV7280_write_cmos_sensor(0xc4,0x40);
    ADV7280_write_cmos_sensor(0xc5,0x30);
    ADV7280_write_cmos_sensor(0xc6,0x30);
    ADV7280_write_cmos_sensor(0xc7,0x10);
    ADV7280_write_cmos_sensor(0xc8,0x00);
    ADV7280_write_cmos_sensor(0xc9,0x00);
    ADV7280_write_cmos_sensor(0xdc,0x20);
    ADV7280_write_cmos_sensor(0xdd,0x10);
    ADV7280_write_cmos_sensor(0xdf,0x00);
    ADV7280_write_cmos_sensor(0xde,0x00);

        /////////////////////////////////////////////////
        ///////////////////  Histogram  /////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x01,0x10);
    ADV7280_write_cmos_sensor(0x0b,0x31);
    ADV7280_write_cmos_sensor(0x0e,0x50);
    ADV7280_write_cmos_sensor(0x0f,0x0f);
    ADV7280_write_cmos_sensor(0x10,0x6e);
    ADV7280_write_cmos_sensor(0x12,0xa0);
    ADV7280_write_cmos_sensor(0x15,0x60);
    ADV7280_write_cmos_sensor(0x16,0x60);
    ADV7280_write_cmos_sensor(0x17,0xe0);

        /////////////////////////////////////////////////
        //////////////   Measure Window   ///////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xcc,0x0c);
    ADV7280_write_cmos_sensor(0xcd,0x10);
    ADV7280_write_cmos_sensor(0xce,0xa0);
    ADV7280_write_cmos_sensor(0xcf,0xe6);

        /////////////////////////////////////////////////
        /////////////////   dark sun   //////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0x45,0xf7);
    ADV7280_write_cmos_sensor(0x46,0xff);
    ADV7280_write_cmos_sensor(0x47,0x15);
    ADV7280_write_cmos_sensor(0x48,0x03);
    ADV7280_write_cmos_sensor(0x4f,0x60);

        /////////////////////////////////////////////////
        ///////////////////  banding  ///////////////////
        /////////////////////////////////////////////////
    ADV7280_write_cmos_sensor(0xfe,0x00);
    ADV7280_write_cmos_sensor(0x05,0x01);
    ADV7280_write_cmos_sensor(0x06,0x18); //HB
#if 1
    ADV7280_write_cmos_sensor(0x07,0x00);
    ADV7280_write_cmos_sensor(0x08,0x10); //VB  from 10 to 50
#else
	ADV7280_write_cmos_sensor(0x07,0x01);
	ADV7280_write_cmos_sensor(0x08,0xe0); //VB
#endif
    ADV7280_write_cmos_sensor(0xfe,0x01);
    ADV7280_write_cmos_sensor(0x25,0x00); //step
    ADV7280_write_cmos_sensor(0x26,0x9a);
    ADV7280_write_cmos_sensor(0x27,0x01); //30fps
    ADV7280_write_cmos_sensor(0x28,0xce);
    ADV7280_write_cmos_sensor(0x29,0x04); //12.5fps
	ADV7280_write_cmos_sensor(0x2a,0x36);
	ADV7280_write_cmos_sensor(0x2b,0x06); //10fps
	ADV7280_write_cmos_sensor(0x2c,0x04);
	ADV7280_write_cmos_sensor(0x2d,0x0c); //5fps
	ADV7280_write_cmos_sensor(0x2e,0x08);
    ADV7280_write_cmos_sensor(0x3c,0x20);

        /////////////////////////////////////////////////
        ///////////////////   MIPI   ////////////////////
        /////////////////////////////////////////////////
   ADV7280_write_cmos_sensor(0xfe,0x03);
   ADV7280_write_cmos_sensor(0x10,0x94);
   ADV7280_write_cmos_sensor(0xfe,0x00); */
}


UINT32 ADV7280GetSensorID(UINT32 *sensorID)
{
    int  temp = -1;

	//if(1==mt_get_gpio_in(GPIO_ACCDET_EINT_PIN))
	//{
	//	 *sensorID = 0xFFFFFFFF;
      // 		 return ERROR_SENSOR_CONNECT_FAIL;
	//}
    // check if sensor ID correct
/*    do {
        *sensorID=((ADV7280_read_cmos_sensor(0xf0)<< 8)|ADV7280_read_cmos_sensor(0xf1));
        if (*sensorID == ADV7280MIPI_YUV_SENSOR_ID)
            break;
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != ADV7280MIPI_YUV_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }*/
  // temp= ADV7280_write_cmos_sensor(0x0e,0x00);
	temp= ADV7280_write_cmos_sensor(0x0F,0X00);

//42 0F 00 ; Exit power down mode
	/*temp=ADV7280_write_cmos_sensor(0x0F,0X00);
	//42 0E 00 ; Enter User Sub Map
	ADV7280_write_cmos_sensor(0x0E,0X00);
	//42 0C 37 ; Force Free-run mode
	ADV7280_write_cmos_sensor(0x0C,0X37);
	//42 02 50 ; Force standard to NTSC-M
	ADV7280_write_cmos_sensor(0x02,0X50);
	//42 14 11 ; Set Free-run pattern to color bars
	ADV7280_write_cmos_sensor(0x14,0X11);
	//42 03 4E ; ADI Required Write
	ADV7280_write_cmos_sensor(0x03,0X4E);
	//42 04 57 ; Enable Intrq pin
        ADV7280_write_cmos_sensor(0x04,0X57);
	//42 13 00 ; Enable INTRQ output driver
	 ADV7280_write_cmos_sensor(0x13,0X00);
	//42 17 41 ; select SH1
	 ADV7280_write_cmos_sensor(0x17,0X41);
	//42 1D C0 ; Tri-State LLC output driver
	ADV7280_write_cmos_sensor(0x1D,0XC0);
	//42 52 CD ; ADI Required Write
	ADV7280_write_cmos_sensor(0x52,0XCD);
	//42 80 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x80,0X51);
	//42 81 51 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x81,0X51);
	//42 82 68 ; ADI Required Write
	ADV7280_write_cmos_sensor(0x82,0X68);
	//42 FD 84 ; Set VPP Map Address
	ADV7280_write_cmos_sensor(0xFD,0X84);
	//84 A3 00 ; ADI Required Write
	ADV7280_write_84(0xA3,0X00);
	//84 5B 00 ; Advanced Timing Enabled
	ADV7280_write_84(0x5B,0X00);
	//84 55 80 ; Enable I2P
	ADV7280_write_84(0x55,0X80);
	//42 FE 88 ; Set CSI Map Address
	ADV7280_write_cmos_sensor(0xFE,0X88);
	//88 01 20 ; ADI Required Write
	ADV7280_write_88(0x01,0X20);
	//88 02 28 ; ADI Required Write
	ADV7280_write_88(0x02,0X28);
	//88 03 38 ; ADI Required Write
	ADV7280_write_88(0x03,0X38);
	//88 04 30 ; ADI Required Write
	ADV7280_write_88(0x04,0X30);
	//88 05 30 ; ADI Required Write
	ADV7280_write_88(0x05,0X30);
	//88 06 80 ; ADI Required Write
	ADV7280_write_88(0x06,0X80);
	//88 07 70 ; ADI Required Write
	ADV7280_write_88(0x07,0X70);
	//88 08 50 ; ADI Required Write
	ADV7280_write_88(0x08,0X50);
	//88 DE 02 ; Power up D-PHY
	ADV7280_write_88(0xDE,0X02);
	//88 D2 F7 ; ADI Required Write
	ADV7280_write_88(0xD2,0XF7);
	//88 D8 65 ; ADI Required Write
	ADV7280_write_88(0xD8,0X65);
	//88 E0 09 ; ADI Required Write
	ADV7280_write_88(0xE0,0X09);
	//88 2C 00 ; ADI Required Write
	ADV7280_write_88(0x2C,0X00);
	//88 1D 80 ; ADI Required Write
	ADV7280_write_88(0x1D,0X80);
	//88 00 00 ; Power up MIPI CSI-2 Tx
	ADV7280_write_88(0x00,0X00);*/

	//*sensorID = ADV7280MIPI_YUV_SENSOR_ID;
	//SENSORDB("mycameraADV7280GetSensorID suc\r\n");
    //return ERROR_NONE;

	if(temp!=0)
	{
		SENSORDB("mycameraADV7280GetSensorID Fail\r\n");
		 *sensorID = 0xFFFFFFFF;
       		 return ERROR_SENSOR_CONNECT_FAIL;
	}
	else
	{
		*sensorID = ADV7280MIPI_SENSOR_ID;
	}

    SENSORDB("mycameraADV7280GetSensorID suc\r\n");
    return ERROR_NONE;
}




/*************************************************************************
* FUNCTION
*	ADV7280_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_ADV7280() directly.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ADV7280_Write_More_Registers(void)
{

}


/*************************************************************************
 * FUNCTION
 *	ADV7280Open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 ADV7280Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	SENSORDB("<Jet> Entry ADV7280Open!!!\r\n");

	//  Read sensor ID to adjust I2C is OK?
/*	for(i=0;i<3;i++)
	{
		sensor_id = ((ADV7280_read_cmos_sensor(0xf0) << 8) | ADV7280_read_cmos_sensor(0xf1));
		if(sensor_id != ADV7280MIPI_YUV_SENSOR_ID)
		{
			SENSORDB("ADV7280mipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id);
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}*/

	SENSORDB("ADV7280mipi_ Sensor Read ID OK \r\n");
	ADV7280_Sensor_Init();
	ADV7280_Write_More_Registers();
	spin_lock(&ADV7280_drv_lock);
	ADV7280_night_mode_enable = KAL_FALSE;
	ADV7280_MPEG4_encode_mode = KAL_FALSE;
	spin_unlock(&ADV7280_drv_lock);

	return ERROR_NONE;
} /* ADV7280Open */


/*************************************************************************
 * FUNCTION
 *	ADV7280Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 ADV7280Close(void)
{
    return ERROR_NONE;
} /* ADV7280Close */


/*************************************************************************
 * FUNCTION
 * ADV7280Preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 ADV7280Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;

	SENSORDB("Enter ADV7280Preview function!!!\r\n");
	ADV7280StreamOn();

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    ADV7280_Set_Mirrorflip(IMAGE_NORMAL);


    // copy sensor_config_data
    memcpy(&ADV7280SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
//    ADV7280NightMode(ADV7280_night_mode_enable);
    return ERROR_NONE;
} /* ADV7280Preview */


/*************************************************************************
 * FUNCTION
 *	ADV7280Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 ADV7280Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
	spin_lock(&ADV7280_drv_lock);
    ADV7280_MODE_CAPTURE=KAL_TRUE;
	spin_unlock(&ADV7280_drv_lock);


    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;

    // copy sensor_config_data
    memcpy(&ADV7280SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* ADV7280_Capture() */



UINT32 ADV7280GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;

    pSensorResolution->SensorHighSpeedVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorHighSpeedVideoHeight=IMAGE_SENSOR_PV_HEIGHT;

    pSensorResolution->SensorSlimVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorSlimVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* ADV7280GetResolution() */


UINT32 ADV7280GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("scenario_id = %d\n", ScenarioId);
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;//SENSOR_OUTPUT_FORMAT_VYUY;//SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 4;
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 0;
       pSensorInfo->YUVAwbDelayFrame = 2;  // add by lanking
	pSensorInfo->YUVEffectDelayFrame = 2;  // add by lanking
	pSensorInfo->SensorMasterClockSwitch = 0;


	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;


	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;

    switch (ScenarioId)
    {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
	//MIPI setting
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x
	pSensorInfo->SensorPacketECCOrder = 1;

        break;
    }
    ADV7280PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &ADV7280SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* ADV7280GetInfo() */


UINT32 ADV7280Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("Entry ADV7280Control, ScenarioId = %d!!!\r\n", ScenarioId);
    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    	SENSORDB("ADV7280 Camera Video Preview!\n");
		spin_lock(&ADV7280_drv_lock);
        ADV7280_MPEG4_encode_mode = KAL_TRUE;
        spin_unlock(&ADV7280_drv_lock);
		ADV7280Preview(pImageWindow, pSensorConfigData);
        break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    default:
        spin_lock(&ADV7280_drv_lock);
        ADV7280_MPEG4_encode_mode = KAL_FALSE;
        spin_unlock(&ADV7280_drv_lock);
		ADV7280Preview(pImageWindow, pSensorConfigData);
        break;
    }

    return ERROR_NONE;
}	/* ADV7280Control() */

BOOL ADV7280_set_param_wb(UINT16 para)
{
	SENSORDB("ADV7280_set_param_wb para = %d\n", para);
    spin_lock(&ADV7280_drv_lock);
    ADV7280_CurStatus_AWB = para;
    spin_unlock(&ADV7280_drv_lock);

	//ADV7280_write_cmos_sensor(0xfe, 0x00);
	switch (para)
	{
		case AWB_MODE_OFF:
			//ADV7280_awb_enable(KAL_FALSE);
		    //ADV7280_write_cmos_sensor(0x77,0x74);
		   // ADV7280_write_cmos_sensor(0x78,0x40);
		   // ADV7280_write_cmos_sensor(0x79,0x5f);
		break;

		case AWB_MODE_AUTO:
		   // ADV7280_write_cmos_sensor(0x77,0x74);
		   // ADV7280_write_cmos_sensor(0x78,0x40);
		   // ADV7280_write_cmos_sensor(0x79,0x5f);
		//	ADV7280_awb_enable(KAL_TRUE);
		break;

		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			//ADV7280_awb_enable(KAL_FALSE);
			//ADV7280_write_cmos_sensor(0x77, 0x40);
			//ADV7280_write_cmos_sensor(0x78, 0x54);
			//ADV7280_write_cmos_sensor(0x79, 0x70);
		break;

		case AWB_MODE_DAYLIGHT: //sunny
			//ADV7280_awb_enable(KAL_FALSE);
			//ADV7280_write_cmos_sensor(0x77, 0x74);
			//ADV7280_write_cmos_sensor(0x78, 0x52);
			//ADV7280_write_cmos_sensor(0x79, 0x40);
		break;

		case AWB_MODE_INCANDESCENT: //office
			//ADV7280_awb_enable(KAL_FALSE);
			//ADV7280_write_cmos_sensor(0x77, 0x48);
			//ADV7280_write_cmos_sensor(0x78, 0x40);
			//ADV7280_write_cmos_sensor(0x79, 0x5c);
		break;

		case AWB_MODE_TUNGSTEN: //home
			//ADV7280_awb_enable(KAL_FALSE);
			//ADV7280_write_cmos_sensor(0x77, 0x8c); //WB_manual_gain
			//ADV7280_write_cmos_sensor(0x78, 0x50);
			//ADV7280_write_cmos_sensor(0x79, 0x40);
		break;

		case AWB_MODE_FLUORESCENT:
			//ADV7280_awb_enable(KAL_FALSE);
			//ADV7280_write_cmos_sensor(0x77, 0x40);
			//ADV7280_write_cmos_sensor(0x78, 0x42);
			//ADV7280_write_cmos_sensor(0x79, 0x50);
		break;

		default:
		return FALSE;
	}
	//ADV7280_write_cmos_sensor(0xfe, 0x00);

	return TRUE;
} /* ADV7280_set_param_wb */


BOOL ADV7280_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;

	switch (para)
	{
		case MEFFECT_OFF:
			//ADV7280_write_cmos_sensor(0x43 , 0x00);
		break;

		case MEFFECT_SEPIA:
			//ADV7280_write_cmos_sensor(0x43 , 0x02);
			//ADV7280_write_cmos_sensor(0xda , 0xd0);
			//ADV7280_write_cmos_sensor(0xdb , 0x28);
		break;

		case MEFFECT_NEGATIVE:
			//ADV7280_write_cmos_sensor(0x43 , 0x01);
		break;

		case MEFFECT_SEPIAGREEN:
			//ADV7280_write_cmos_sensor(0x43 , 0x02);
			//ADV7280_write_cmos_sensor(0xda , 0xc0);
			//ADV7280_write_cmos_sensor(0xdb , 0xc0);
		break;

		case MEFFECT_SEPIABLUE:
			//ADV7280_write_cmos_sensor(0x43 , 0x02);
			//ADV7280_write_cmos_sensor(0xda , 0x50);
			//ADV7280_write_cmos_sensor(0xdb , 0xe0);
		break;

		case MEFFECT_MONO:
			//ADV7280_write_cmos_sensor(0x43 , 0x02);
			//ADV7280_write_cmos_sensor(0xda , 0x00);
			//ADV7280_write_cmos_sensor(0xdb , 0x00);
		break;
		default:
			ret = FALSE;
	}

	return ret;

} /* ADV7280_set_param_effect */


BOOL ADV7280_set_param_banding(UINT16 para)
{

	SENSORDB("Enter ADV7280_set_param_banding!, para = %d\r\n", para);
	switch (para)
	{
		case AE_FLICKER_MODE_AUTO:
		case AE_FLICKER_MODE_OFF:
		case AE_FLICKER_MODE_50HZ:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0x05, 0x01);
			//ADV7280_write_cmos_sensor(0x06, 0x18);
			//ADV7280_write_cmos_sensor(0x07, 0x00);
			//ADV7280_write_cmos_sensor(0x08, 0x10);

			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x25,0x00); //step
			//ADV7280_write_cmos_sensor(0x26,0x9a);

			//ADV7280_write_cmos_sensor(0x27,0x01); //30fps
			//ADV7280_write_cmos_sensor(0x28,0xce);
			//ADV7280_write_cmos_sensor(0x29,0x04); //15fps
			//ADV7280_write_cmos_sensor(0x2a,0x36);
			//ADV7280_write_cmos_sensor(0x2b,0x06); //10fps
			//ADV7280_write_cmos_sensor(0x2c,0x04);
			//ADV7280_write_cmos_sensor(0x2d,0x09); //8fps
			//ADV7280_write_cmos_sensor(0x2e,0xa0);  //
			//ADV7280_write_cmos_sensor(0x3c,0x20);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case AE_FLICKER_MODE_60HZ:
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			//ADV7280_write_cmos_sensor(0x05, 0x01);
			//ADV7280_write_cmos_sensor(0x06, 0x13);
			//ADV7280_write_cmos_sensor(0x07, 0x00);
			//ADV7280_write_cmos_sensor(0x08, 0x10);

			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x25, 0x00);   //anti-flicker step [11:8]
			//ADV7280_write_cmos_sensor(0x26, 0x81);	//anti-flicker step [7:0]

			//ADV7280_write_cmos_sensor(0x27, 0x01);	//exp level 0  30fps
			//ADV7280_write_cmos_sensor(0x28, 0x83);
			//ADV7280_write_cmos_sensor(0x29, 0x04);	//exp level 1  15fps
			//ADV7280_write_cmos_sensor(0x2a, 0x08);
			//ADV7280_write_cmos_sensor(0x2b, 0x06);	//exp level 2  10fps
			//ADV7280_write_cmos_sensor(0x2c, 0x0c);
			//ADV7280_write_cmos_sensor(0x2d, 0x09);	//exp level 3 8fps
			//ADV7280_write_cmos_sensor(0x2e, 0x93);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);

		break;
		default:
		return FALSE;
	}
	//ADV7280_write_cmos_sensor(0xfe, 0x00);

	return TRUE;
} /* ADV7280_set_param_banding */

BOOL ADV7280_set_param_exposure(UINT16 para)
{

	switch (para)
	{

		case AE_EV_COMP_n20:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x13, 0x20);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
		break;

		case AE_EV_COMP_n10:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x13, 0x28);  // 28 to 10
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
		break;

		case AE_EV_COMP_00:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x13, 0x38);//35
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
		break;

		case AE_EV_COMP_15:
		case AE_EV_COMP_10:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x13, 0x48);  // 48 to 60
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
			Sleep(200);
		break;

		case AE_EV_COMP_20:
			//ADV7280_write_cmos_sensor(0xfe, 0x01);
			//ADV7280_write_cmos_sensor(0x13, 0x50);
			//ADV7280_write_cmos_sensor(0xfe, 0x00);
		break;
		default:
		return FALSE;
	}

	return TRUE;
} /* ADV7280_set_param_exposure */


UINT32 ADV7280YUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{

	SENSORDB("Enter ADV7280YUVSetVideoMode, u2FrameRate = %d\n", u2FrameRate);
//	spin_lock(&ADV7280_drv_lock);
//    ADV7280_MPEG4_encode_mode = KAL_TRUE;
//	spin_unlock(&ADV7280_drv_lock);


     if (u2FrameRate == 30)
   	{

   	    /*********video frame ************/

   	}
    else if (u2FrameRate == 15)
    	{

   	    /*********video frame ************/

    	}
    else
   	{

            SENSORDB("Wrong Frame Rate");

   	}

	ADV7280NightMode(ADV7280_night_mode_enable);

    return TRUE;

}


UINT32 ADV7280YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    switch (iCmd) {
    case FID_AWB_MODE:
        ADV7280_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        ADV7280_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        ADV7280_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        ADV7280_set_param_banding(iPara);
		break;
    case FID_SCENE_MODE:
	 	ADV7280NightMode(iPara);
        break;

	case FID_ISP_CONTRAST:
		ADV7280_set_contrast(iPara);
		break;
	case FID_ISP_BRIGHT:
		ADV7280_set_brightness(iPara);
		break;
	case FID_ISP_SAT:
		ADV7280_set_saturation(iPara);
		break;
	case FID_AE_ISO:
		ADV7280_set_iso(iPara);
		break;
	case FID_AE_SCENE_MODE:
		ADV7280_set_AE_mode(iPara);
		break;

    default:
        break;
    }
    return TRUE;
} /* ADV7280YUVSensorSetting */


UINT32 ADV7280FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
        UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{

    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 **ppFeatureData=(UINT32 **) pFeaturePara;
    unsigned long long *feature_data=(unsigned long long *) pFeaturePara;
    unsigned long long *feature_return_para=(unsigned long long *) pFeaturePara;

    UINT32 ADV7280SensorRegNumber;
    UINT32 i;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+ADV7280_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+ADV7280_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *pFeatureReturnPara32 = ADV7280_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        ADV7280NightMode((BOOL) *feature_data);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        ADV7280_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        //ADV7280_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = 0;//ADV7280_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &ADV7280SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        ADV7280YUVSensorSetting((FEATURE_ID)*feature_data, *(feature_data+1));

        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:    //  lanking
	 ADV7280YUVSetVideoMode(*feature_data);
	 break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	ADV7280GetSensorID(pFeatureData32);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*pFeatureReturnPara32=ADV7280_TEST_PATTERN_CHECKSUM;
		*pFeatureParaLen=4;
		break;

	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 SENSORDB("[ADV7280] F_SET_MAX_FRAME_RATE_BY_SCENARIO.\n");
		 ADV7280_MIPI_SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		 break;

//	case SENSOR_CMD_SET_VIDEO_FRAME_RATE:
//		SENSORDB("[ADV7280] Enter SENSOR_CMD_SET_VIDEO_FRAME_RATE\n");
//		//ADV7280_MIPI_SetVideoFrameRate(*pFeatureData32);
//		break;

	case SENSOR_FEATURE_SET_TEST_PATTERN:
		ADV7280SetTestPatternMode((BOOL)*feature_data);
		break;

    case SENSOR_FEATURE_GET_DELAY_INFO:
        SENSORDB("[ADV7280] F_GET_DELAY_INFO\n");
        ADV7280_MIPI_GetDelayInfo((uintptr_t)*feature_data);
    break;

    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
         SENSORDB("[ADV7280] F_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
         ADV7280_MIPI_GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
    break;

	case SENSOR_FEATURE_SET_YUV_3A_CMD:
		 SENSORDB("[ADV7280] SENSOR_FEATURE_SET_YUV_3A_CMD ID:%d\n", *pFeatureData32);
		 ADV7280_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*feature_data);
		 break;


	case SENSOR_FEATURE_GET_EXIF_INFO:
		 //SENSORDB("[4EC] F_GET_EXIF_INFO\n");
		 ADV7280_MIPI_GetExifInfo((uintptr_t)*feature_data);
		 break;


	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
		 SENSORDB("[ADV7280] F_GET_AE_AWB_LOCK_INFO\n");
		 //ADV7280_MIPI_get_AEAWB_lock((uintptr_t)(*feature_data), (uintptr_t)*(feature_data+1));
	break;

	case SENSOR_FEATURE_SET_MIN_MAX_FPS:
		SENSORDB("SENSOR_FEATURE_SET_MIN_MAX_FPS:[%d,%d]\n",*pFeatureData32,*(pFeatureData32+1));
		ADV7280_MIPI_SetMaxMinFps((UINT32)*feature_data, (UINT32)*(feature_data+1));
	break;

    default:
        break;
	}
return ERROR_NONE;

/* UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    //unsigned long long *feature_data=(unsigned long long *) pFeaturePara;
    //unsigned long long *feature_return_para=(unsigned long long *) pFeaturePara;

    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	SENSORDB("feature_id = %d\n", FeatureId);
    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=(VGA_PERIOD_PIXEL_NUMS)+ADV7280_dummy_pixels;
        *pFeatureReturnPara16=(VGA_PERIOD_LINE_NUMS)+ADV7280_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        //*pFeatureReturnPara32 = ADV7280_g_fPV_PCLK;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        ADV7280NightMode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        ADV7280_isp_master_clock=*pFeatureData32;
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        //ADV7280_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = 0;//ADV7280_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &ADV7280SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;
		    break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
	 break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	ADV7280GetSensorID(pFeatureData32);
	break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		      // printk("GC2155MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			ADV7280YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			//GC2155MIPIYUVSensorSetting((FEATURE_ID)*feature_data, *(feature_data+1));
		break;

		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       ADV7280YUVSetVideoMode(*pFeatureData16);
		       break;

        case SENSOR_FEATURE_SET_TEST_PATTERN:
		    ADV7280SetTestPatternMode((BOOL)*pFeatureData32);
		break;

        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*pFeatureReturnPara32=ADV7280_TEST_PATTERN_CHECKSUM;
		*pFeatureParaLen=4;
		break;

		default:
			break;
	}
return ERROR_NONE;
*/
}	/* ADV7280FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncADV7280YUV=
{
	ADV7280Open,
	ADV7280GetInfo,
	ADV7280GetResolution,
	ADV7280FeatureControl,
	ADV7280Control,
	ADV7280Close
};


UINT32 ADV7280_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncADV7280YUV;
	return ERROR_NONE;
} /* SensorInit() */
