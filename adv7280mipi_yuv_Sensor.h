/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ADV7280_yuv_Sensor.h
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver declare and macro define in the header file.
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/10/25 Firsty Released By Mormo;
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
 
#ifndef __ADV7280_SENSOR_H
#define __ADV7280_SENSOR_H


#define VGA_PERIOD_PIXEL_NUMS						720 //694//
#define VGA_PERIOD_LINE_NUMS						507 //488//

#define IMAGE_SENSOR_VGA_GRAB_PIXELS			0
#define IMAGE_SENSOR_VGA_GRAB_LINES			0

#define IMAGE_SENSOR_VGA_WIDTH					(720)  //640
#define IMAGE_SENSOR_VGA_HEIGHT					(480)  //480

#define IMAGE_SENSOR_PV_WIDTH					(IMAGE_SENSOR_VGA_WIDTH)
#define IMAGE_SENSOR_PV_HEIGHT					(IMAGE_SENSOR_VGA_HEIGHT)

#define IMAGE_SENSOR_FULL_WIDTH					(IMAGE_SENSOR_VGA_WIDTH)
#define IMAGE_SENSOR_FULL_HEIGHT					(IMAGE_SENSOR_VGA_HEIGHT)

//#define ADV7280_WRITE_ID							        0x40
//#define ADV7280_READ_ID								0x41
#define ADV7280_WRITE_ID							        0x42
#define ADV7280_READ_ID								0x43

#define ADV7280_WRITE_CSI							        0x88
#define ADV7280_READ_CSI							0x89
// ADV7280 SENSOR Chip ID: 0xd0

typedef enum
{
	ADV7280_RGB_Gamma_m1 = 0,
	ADV7280_RGB_Gamma_m2,
	ADV7280_RGB_Gamma_m3,
	ADV7280_RGB_Gamma_m4,
	ADV7280_RGB_Gamma_m5,
	ADV7280_RGB_Gamma_m6,
	ADV7280_RGB_Gamma_night
}ADV7280_GAMMA_TAG;



UINT32 ADV7280Open(void);
UINT32 ADV7280Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ADV7280FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 ADV7280GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ADV7280GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 ADV7280Close(void);
extern void kdSetI2CSpeed(u16 i2cSpeed);
extern int kdSetI2CBusNum(u32 i2cBusNum); 
#endif /* __SENSOR_H */

