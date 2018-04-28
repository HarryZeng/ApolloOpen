/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "sys.h"
#include "delay.h"
#include "usart.h"			 
#include "sccb.h"	
#include "pcf8574.h"  
#include "ltdc.h"  
#include "mt9v034.h"
#include "lcd.h"

#define FULL_IMAGE_SIZE (640*480)
#define FULL_IMAGE_ROW_SIZE (640)
#define FULL_IMAGE_COLUMN_SIZE (480)
//uint32_t RAW_BUF[65];

/****************************************************************/
const int resolution[][2] = {
			// C/SIF Resolutions
			{88,   72  },    /* QQCIF     0*/
			{176,  144 },    /* QCIF      1*/
			{352,  288 },    /* CIF       2*/
			{88,   60  },    /* QQSIF     3*/
			{176,  120 },    /* QSIF      4*/
			{352,  240 },    /* SIF       5*/
			// VGA Resolutions
			{40,   30  },    /* QQQQVGA   6*/
			{80,   60  },    /* QQQVGA    7*/
			{160,  120 },    /* QQVGA     8*/
			{320,  240 },    /* QVGA      9*/
			{640,  480 },    /* VGA       10*/
			{60,   40  },    /* HQQQVGA   11*/
			{120,  80  },    /* HQQVGA    12*/
			{240,  160 },    /* HQVGA     13*/
			// FFT Resolutions
			{64,   32  },    /* 64x32     14*/
			{64,   64  },    /* 64x64     15*/
			{128,  64  },    /* 128x64    16*/
			{128,  128 },    /* 128x64    17*/
			// Other
			{128,  160 },    /* LCD       18*/
			{128,  160 },    /* QQVGA2    19*/
			{800,  600 },    /* SVGA      20*/
			{1280, 1024},    /* SXGA      21*/
			{1600, 1200},    /* UXGA      22*/
};
///**********************************************************/



void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
		
    if (htim->Instance == DCMI_TIM) {

    }
}
		int tclk;
		int period;
int extclk_config(int frequency)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		TIM_HandleTypeDef  TIMHandle;
		TIM_OC_InitTypeDef TIMOCHandle;

    // Doubles PCLK
    //__HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_ACTIVATED);

				/* Enable DCMI timer clock */
		DCMI_TIM_CLK_ENABLE();

		/* Timer GPIO configuration */
	
		GPIO_InitStructure.Pin       = DCMI_TIM_PIN;
		GPIO_InitStructure.Pull      = GPIO_NOPULL;
		GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Alternate = DCMI_TIM_AF;
		HAL_GPIO_Init(DCMI_TIM_PORT, &GPIO_InitStructure);
	
    /* TCLK (PCLK * 2) */
    tclk  = DCMI_TIM_PCLK_FREQ() * 2;

    /* Period should be even */
    period = (tclk / frequency) - 1;

    /* Timer base configuration */
    TIMHandle.Instance          = DCMI_TIM;
    TIMHandle.Init.Period       = period;
    TIMHandle.Init.Prescaler    = 0;
    TIMHandle.Init.ClockDivision = 0;
    TIMHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

    /* Timer channel configuration */
    
    TIMOCHandle.Pulse       = period/2;
    TIMOCHandle.OCMode      = TIM_OCMODE_PWM1;
    TIMOCHandle.OCPolarity  = TIM_OCPOLARITY_HIGH;
    TIMOCHandle.OCFastMode  = TIM_OCFAST_DISABLE;
    TIMOCHandle.OCIdleState = TIM_OCIDLESTATE_RESET;

    if (HAL_TIM_PWM_Init(&TIMHandle) != HAL_OK
            || HAL_TIM_PWM_ConfigChannel(&TIMHandle, &TIMOCHandle, DCMI_TIM_CHANNEL) != HAL_OK
            || HAL_TIM_PWM_Start(&TIMHandle, DCMI_TIM_CHANNEL) != HAL_OK) {
        // Initialization Error
        return -1;
    }

    return 0;
}


	uint16_t version ;
/**
  * @brief  Configures the mt9v034 camera with two context (binning 4 and binning 2).
  */

uint16_t MT9V034_Width,MT9V034_height;

static int reset(void)
{
    // NOTE: TODO This doesn't reset register configuration.
    mt9v034_WriteReg16(MT9V034_RESET, 1);
		mt9v034_WriteReg16(MT9V034_CHIP_CONTROL, 0x0088);
		mt9v034_WriteReg16(MT9V034_READ_MODE,(MT9V034_READ_MODE_ROW_FLIP | MT9V034_READ_MODE_COL_FLIP));
    mt9v034_WriteReg16(MT9V034_RESET, 1);
    return 0;
}

    uint16_t width  ; 
    uint16_t height ;
		uint16_t MT9V034_Mode=0;

static int set_framesize(uint16_t framesize)
{
    int ret=0;
    uint16_t readmode = 0;

//		uint16_t new_control= 0x0388; // Context A

//	/* image dimentions */
//	uint16_t new_width_context_a  = global_data.param[PARAM_IMAGE_WIDTH] * 4; // windowing off, row + col bin reduce size
//	uint16_t new_height_context_a = global_data.param[PARAM_IMAGE_HEIGHT] * 4;
//	uint16_t new_width_context_b  = FULL_IMAGE_ROW_SIZE * 4; // windowing off, row + col bin reduce size
//	uint16_t new_height_context_b = FULL_IMAGE_COLUMN_SIZE * 4;

//	/* blanking settings */
//	uint16_t new_hor_blanking_context_a = 350 + MINIMUM_HORIZONTAL_BLANKING;// 350 is minimum value without distortions
//	uint16_t new_ver_blanking_context_a = 10; // this value is the first without image errors (dark lines)
//	uint16_t new_hor_blanking_context_b = MAX_IMAGE_WIDTH - new_width_context_b + MINIMUM_HORIZONTAL_BLANKING;
//	uint16_t new_ver_blanking_context_b = 10;


//	/* Read Mode
//	 *
//	 * bits           | ... | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
//	 * -------------------------------------------------------------------
//	 * current values | ... |  0 | 1 | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 0 |
//	 *
//	 * (1:0) Row Bin
//	 * (3:2) Column Bin
//	 * (9:8) Reserved
//	 *
//	 */
//	uint16_t new_readmode_context_a = 0x40;//0x30A ; // row + col bin 4 enable, (9:8) default
//	uint16_t new_readmode_context_b = 0x305 ; // row bin 2 col bin 4 enable, (9:8) default

//	/*
//	 * Settings for both context:
//	 *
//	 * Exposure time should not affect frame time
//	 * so we set max on 64 (lines) = 0x40
//	 */
//	uint16_t min_exposure = 0x0001; // default
//	uint16_t max_exposure = 0x01E0; // default
//	uint16_t new_max_gain = 64; // VALID RANGE: 16-64 (default)
//	uint16_t pixel_count = 65535; //64x64 take all pixels to estimate exposure time // VALID RANGE: 1-65535

//	uint16_t desired_brightness = 58; // default
//	uint16_t resolution_ctrl = 0x0203; // default
//	uint16_t hdr_enabled = 0x0100; // default
//	uint16_t aec_agc_enabled = 0x0003; // default
//	uint16_t coarse_sw1 = 0x01BB; // default from context A
//	uint16_t coarse_sw2 = 0x01D9; // default from context A
//	uint16_t shutter_width_ctrl = 0x0164; // default from context A
//	uint16_t total_shutter_width = 0x01E0; // default from context A
//	uint16_t aec_update_freq = 0x02; // default Number of frames to skip between changes in AEC VALID RANGE: 0-15
//	uint16_t aec_low_pass = 0x01; // default VALID RANGE: 0-2
//	uint16_t agc_update_freq = 0x02; // default Number of frames to skip between changes in AGC VALID RANGE: 0-15
//	uint16_t agc_low_pass = 0x02; // default VALID RANGE: 0-2
//	
//	
//	uint16_t row_noise_correction = 0x0000; // default
//	uint16_t test_data = 0x0000; // default
//	
//	/*****************************************************************************/
////     width  = resolution[framesize][0]; 
////     height = resolution[framesize][1];

	  width  = 40; 
    height = 30;
	
		MT9V034_Width = width;
		MT9V034_height = height;
//	
//	

//		mt9v034_WriteReg16(MTV_CHIP_CONTROL_REG, new_control);

//		/* Context A */
//		mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_A, new_width_context_a);
//		mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_A, new_height_context_a);
//		mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_A, new_hor_blanking_context_a);
//		mt9v034_WriteReg16(MTV_VER_BLANKING_REG_A, new_ver_blanking_context_a);
//		mt9v034_WriteReg16(MTV_READ_MODE_REG_A, new_readmode_context_a);
//		mt9v034_WriteReg16(MTV_COLUMN_START_REG_A, (MAX_IMAGE_WIDTH - new_width_context_a) / 2 + MINIMUM_COLUMN_START); // Set column/row start point for lower resolutions (center window)
//		mt9v034_WriteReg16(MTV_ROW_START_REG_A, (MAX_IMAGE_HEIGHT - new_height_context_a) / 2 + MINIMUM_ROW_START);
//		mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_A, coarse_sw1);
//		mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_A, coarse_sw2);
//		mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_A, shutter_width_ctrl);
//		mt9v034_WriteReg16(MTV_V2_CTRL_REG_A, total_shutter_width);


//		/* Context B */
//		mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_B, new_width_context_b);
//		mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_B, new_height_context_b);
//		mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_B, new_hor_blanking_context_b);
//		mt9v034_WriteReg16(MTV_VER_BLANKING_REG_B, new_ver_blanking_context_b);
//		mt9v034_WriteReg16(MTV_READ_MODE_REG_B, new_readmode_context_b);
//		mt9v034_WriteReg16(MTV_COLUMN_START_REG_B, MINIMUM_COLUMN_START); // default
//		mt9v034_WriteReg16(MTV_ROW_START_REG_B, MINIMUM_ROW_START);
//		mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_B, coarse_sw1);
//		mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_B, coarse_sw2);
//		mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_B, shutter_width_ctrl);
//		mt9v034_WriteReg16(MTV_V2_CTRL_REG_B, total_shutter_width);

//		/* General Settings */
//		mt9v034_WriteReg16(MTV_ROW_NOISE_CORR_CTRL_REG, row_noise_correction);
//		mt9v034_WriteReg16(MTV_AEC_AGC_ENABLE_REG, aec_agc_enabled); // disable AEC/AGC for both contexts
//		mt9v034_WriteReg16(MTV_HDR_ENABLE_REG, hdr_enabled); // disable HDR on both contexts
//		mt9v034_WriteReg16(MTV_MIN_EXPOSURE_REG, min_exposure);
//		mt9v034_WriteReg16(MTV_MAX_EXPOSURE_REG, max_exposure);
//		mt9v034_WriteReg16(MTV_MAX_GAIN_REG, new_max_gain);
//		mt9v034_WriteReg16(MTV_AGC_AEC_PIXEL_COUNT_REG, pixel_count);
//		mt9v034_WriteReg16(MTV_AGC_AEC_DESIRED_BIN_REG, desired_brightness);
//		mt9v034_WriteReg16(MTV_ADC_RES_CTRL_REG, resolution_ctrl); // here is the way to regulate darkness :)

//		mt9v034_WriteReg16(MTV_DIGITAL_TEST_REG, test_data);//enable test pattern

//		mt9v034_WriteReg16(MTV_AEC_UPDATE_REG,aec_update_freq);
//		mt9v034_WriteReg16(MTV_AEC_LOWPASS_REG,aec_low_pass);
//		mt9v034_WriteReg16(MTV_AGC_UPDATE_REG,agc_update_freq);
//		mt9v034_WriteReg16(MTV_AGC_LOWPASS_REG,agc_low_pass);

//		/* Reset */
//		mt9v034_WriteReg16(MTV_SOFT_RESET_REG, 0x01);	
//    if ((width * 4) <= MT9V034_MAX_WIDTH && (height * 4) <= MT9V034_MAX_HEIGHT) 
//		{
//        width  *= 4; 
//				height *= 4;
//        readmode |= MT9V034_READ_MODE_ROW_BIN_4;
//        readmode |= MT9V034_READ_MODE_COL_BIN_4;
//    } 
//		else if ((width * 2) <= MT9V034_MAX_WIDTH && (height * 2) <= MT9V034_MAX_HEIGHT) 
//		{
//        width  *= 2; 
//				height *= 2;
//        readmode |= MT9V034_READ_MODE_ROW_BIN_2;
//        readmode |= MT9V034_READ_MODE_COL_BIN_2;
//    }

//    readmode |= MT9V034_READ_MODE_ROW_FLIP;
//    readmode |= MT9V034_READ_MODE_COL_FLIP;
//	mt9v034_WriteReg16(MT9V034_READ_MODE, readmode);			
//			ret |= mt9v034_WriteReg16( MT9V034_WINDOW_WIDTH, width);
//			ret |= mt9v034_WriteReg16( MT9V034_COL_START, (MT9V034_MAX_WIDTH  - width ) / 2 + MT9V034_COL_START_MIN);
//			ret |= mt9v034_WriteReg16( MT9V034_WINDOW_HEIGHT, height);
//			ret |= mt9v034_WriteReg16( MT9V034_ROW_START, (MT9V034_MAX_HEIGHT - height) / 2 + MT9V034_ROW_START_MIN);

			

//			mt9v034_WriteReg16(MT9V034_READ_MODE,(MT9V034_READ_MODE_COL_FLIP));
//			mt9v034_WriteReg16(MT9V034_READ_MODE,(MT9V034_READ_MODE_ROW_FLIP));
//			mt9v034_WriteReg16(MT9V034_READ_MODE,(MT9V034_READ_MODE_DARK_ROWS));
//			mt9v034_WriteReg16(MT9V034_READ_MODE,(MT9V034_READ_MODE_DARK_COLS));	
			
			mt9v034_WriteReg16(MT9V034_RESET, 1);
			mt9v034_WriteReg16(MT9V034_RESET, 0);
			
			mt9v034_WriteReg16(MT9V034_CHIP_CONTROL, 0x0388);
			mt9v034_WriteReg16(MT9V034_READ_MODE, 0x0330);	
			mt9v034_WriteReg16( MT9V034_COL_START,0x0001);
			mt9v034_WriteReg16( MT9V034_ROW_START,0x0004);			
			mt9v034_WriteReg16( MT9V034_WINDOW_WIDTH, MT9V034_Width);
			mt9v034_WriteReg16( MT9V034_WINDOW_HEIGHT, MT9V034_height);
			mt9v034_WriteReg16( MT9V034_PIXEL_OPERATION_MODE, MT9V034_PIXEL_OPERATION_MODE_COLOR);
			mt9v034_WriteReg16( MT9V034_ANALOG_GAIN_CONTROL, MT9V034_ANALOG_GAIN_MIN);

			mt9v034_WriteReg16(MT9V034_RESET, 1);
			mt9v034_WriteReg16(MT9V034_RESET, 0);



			mt9v034_WriteReg16(0x72, 0x0000); //sync ctrl 
			//i2c_set_register(0x7F, ((1 << 13) | (1 << 10) | (1 << 5))); //test pattern 
			mt9v034_WriteReg16(0x7F, 0x00); 
			
		
		
//			mt9v034_WriteReg16( MT9V034_HORIZONTAL_BLANKING, 0x005E);
//			mt9v034_WriteReg16( MT9V034_VERTICAL_BLANKING, 0x002D);
//			mt9v034_WriteReg16( MT9V034_CHIP_CONTROL, 0x0388);
//			
//			mt9v034_WriteReg16( MT9V034_SHUTTER_WIDTH1, 0x01BB);
//			mt9v034_WriteReg16( MT9V034_SHUTTER_WIDTH2, 0x01D9);
//		
//			mt9v034_WriteReg16(MT9V034_READ_MODE, 0x0300);
//			mt9v034_WriteReg16(MT9V034_PIXEL_OPERATION_MODE, 0x0011);

    return 0;
}

uint16_t temp_w,temp_h;
uint16_t temp_col_s,temp_row_s;
uint16_t temp_chip,temp_read;
uint16_t temp_gain,temp_operation_mode;
void mt9v034_context_configuration(void)
{

		extclk_config(27000000);
		SCCB_Init();			//初始化SCCB 的IO口 
	
	version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);

	if (version == 0x1324)
	{
		/* Reset */
		//reset();
		set_framesize(10);

	}
	temp_w = mt9v034_ReadReg16(MT9V034_WINDOW_WIDTH);
	temp_h = mt9v034_ReadReg16(MT9V034_WINDOW_HEIGHT);
	temp_col_s = mt9v034_ReadReg16(MT9V034_COL_START);
	temp_row_s = mt9v034_ReadReg16(MT9V034_ROW_START);
	temp_chip = mt9v034_ReadReg16(MT9V034_CHIP_CONTROL);
	temp_read = mt9v034_ReadReg16(MT9V034_READ_MODE);
	temp_operation_mode = mt9v034_ReadReg16(MT9V034_PIXEL_OPERATION_MODE);
	temp_gain = mt9v034_ReadReg16(MT9V034_ANALOG_GAIN_CONTROL);
	
	LCD_ShowxNum(640,260,temp_w,5,16,1);
	LCD_ShowxNum(640,280,temp_h,5,16,1);
	
	LCD_ShowxNum(640,300,temp_col_s,5,16,1);
	LCD_ShowxNum(640,320,temp_row_s,5,16,1);
	
	LCD_ShowxNum(680,260,temp_chip,5,16,1);
	LCD_ShowxNum(680,280,temp_read,5,16,1);
	
	LCD_ShowxNum(680,300,temp_operation_mode,5,16,1);
	LCD_ShowxNum(680,320,temp_gain,5,16,1);
}


///**
//  * @brief  Writes a byte at a specific Camera register
//  * @param  Addr: mt9v034 register address.
//  * @param  Data: Data to be written to the specific register
//  * @retval 0x00 if write operation is OK.
//  *       0xFF if timeout condition occured (device not connected or bus error).
//  */
//uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data)
//{
//  uint32_t timeout = TIMEOUT_MAX;

//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);

//  /* Test on I2C2 EV5 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send DCMI selcted device slave Address for write */
//  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send I2C2 location address LSB */
//  I2C_SendData(I2C2, (uint8_t)(Addr));

//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send Data */
//  I2C_SendData(I2C2, Data);

//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send I2C2 STOP Condition */
//  I2C_GenerateSTOP(I2C2, ENABLE);

//  /* If operation is OK, return 0 */
//  return 0;
//}



uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data)
{
	//uint32_t timeout = TIMEOUT_MAX;
	uint8_t 	res=0;
	
	SCCB_Start(); 					//启动SCCB传输
	if(SCCB_WR_Byte(mt9v034_DEVICE_WRITE_ADDRESS))res=1;	//写器件ID	  
	//if(SCCB_WR_Byte(Addr>>8))res=1;	//写寄存器高8位地址
	if(SCCB_WR_Byte(Addr))res=1;		//写寄存器低8位地址		  
	if(SCCB_WR_Byte(Data))res=1; 	//写数据	 
	SCCB_Stop();	  
	return	res;
}

/**
  * @brief  Writes to a specific Camera register
  */
uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data)
{
	uint8_t result = mt9v034_WriteReg(address, (uint8_t)( Data >> 8)); // write upper byte
	result |= mt9v034_WriteReg(0xF0, (uint8_t) Data); // write lower byte
	return result;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: mt9v034 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
//uint8_t mt9v034_ReadReg(uint16_t Addr)
//{
//  uint32_t timeout = TIMEOUT_MAX;
//  uint8_t Data = 0;

//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);

//  /* Test on I2C2 EV5 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send DCMI selcted device slave Address for write */
//  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send I2C2 location address LSB */
//  I2C_SendData(I2C2, (uint8_t)(Addr));

//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Clear AF flag if arised */
//  I2C2->SR1 |= (uint16_t)0x0400;

//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);

//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Send DCMI selcted device slave Address for write */
//  I2C_Send7bitAddress(I2C2, mt9v034_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Prepare an NACK for the next data received */
//  I2C_AcknowledgeConfig(I2C2, DISABLE);

//  /* Test on I2C2 EV7 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
//  {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) return 0xFF;
//  }

//  /* Prepare Stop after receiving data */
//  I2C_GenerateSTOP(I2C2, ENABLE);

//  /* Receive the Data */
//  Data = I2C_ReceiveData(I2C2);

//  /* return the read data */
//  return Data;
//}

uint8_t mt9v034_ReadReg(uint16_t Addr)
{
		uint8_t val=0;
		SCCB_Start(); 				//启动SCCB传输
		SCCB_WR_Byte(mt9v034_DEVICE_WRITE_ADDRESS);	//写器件ID
   	//SCCB_WR_Byte(Addr>>8);	    //写寄存器高8位地址   
  	SCCB_WR_Byte(Addr);			//写寄存器低8位地址	  
		SCCB_Stop(); 
		//设置寄存器地址后，才是读
		SCCB_Start();
		SCCB_WR_Byte(mt9v034_DEVICE_READ_ADDRESS);//发送读命令	  
		//SCCB_WR_Byte(mt9v034_DEVICE_WRITE_ADDRESS|0X01);//发送读命令	  
   	val=SCCB_RD_Byte();		 	//读取数据
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}

/**
  * @brief  Reads from a specific Camera register
  */

uint16_t mt9v034_ReadReg16(uint8_t address)
{
	uint16_t result;
	result = mt9v034_ReadReg(address) << 8; // read upper byte
	result |= mt9v034_ReadReg(0xF0); // read lower byte
	return result;
}

