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

static int set_framesize(uint16_t framesize)
{
    int ret=0;
    uint16_t readmode = 0;
    uint16_t width  = resolution[framesize][0]; 
    uint16_t height = resolution[framesize][1];

	  width  = 320; 
    height = 240;
	
		MT9V034_Width = width;
		MT9V034_height = height;
	
    if ((width * 4) <= MT9V034_MAX_WIDTH && (height * 4) <= MT9V034_MAX_HEIGHT) 
		{
        width  *= 4; 
				height *= 4;
        readmode |= MT9V034_READ_MODE_ROW_BIN_4;
        readmode |= MT9V034_READ_MODE_COL_BIN_4;
    } 
		else if ((width * 2) <= MT9V034_MAX_WIDTH && (height * 2) <= MT9V034_MAX_HEIGHT) 
		{
        width  *= 2; 
				height *= 2;
        readmode |= MT9V034_READ_MODE_ROW_BIN_2;
        readmode |= MT9V034_READ_MODE_COL_BIN_2;
    }

    readmode |= MT9V034_READ_MODE_ROW_FLIP;
    readmode |= MT9V034_READ_MODE_COL_FLIP;
			//mt9v034_WriteReg16(MT9V034_READ_MODE, readmode);
		//ret |= mt9v034_WriteReg16( MT9V034_WINDOW_WIDTH, width);
		//ret |= mt9v034_WriteReg16( MT9V034_COL_START, (MT9V034_MAX_WIDTH  - width ) / 2 + MT9V034_COL_START_MIN);
		//ret |= mt9v034_WriteReg16( MT9V034_WINDOW_HEIGHT, height);
		//ret |= mt9v034_WriteReg16( MT9V034_ROW_START, (MT9V034_MAX_HEIGHT - height) / 2 + MT9V034_ROW_START_MIN);
//			mt9v034_WriteReg16( MT9V034_COL_START,0x0001);
//			mt9v034_WriteReg16( MT9V034_ROW_START,0x0004);
			mt9v034_WriteReg16( MT9V034_WINDOW_HEIGHT, height);
			mt9v034_WriteReg16( MT9V034_WINDOW_WIDTH, width);
//			mt9v034_WriteReg16( MT9V034_HORIZONTAL_BLANKING, 0x005E);
//			mt9v034_WriteReg16( MT9V034_VERTICAL_BLANKING, 0x002D);
//			mt9v034_WriteReg16( MT9V034_CHIP_CONTROL, 0x0388);
//			
//			mt9v034_WriteReg16( MT9V034_SHUTTER_WIDTH1, 0x01BB);
//			mt9v034_WriteReg16( MT9V034_SHUTTER_WIDTH2, 0x01D9);
//		
//			mt9v034_WriteReg16(MT9V034_READ_MODE, 0x0300);
			//mt9v034_WriteReg16(MT9V034_PIXEL_OPERATION_MODE, 0x0011);

    return 0;
}

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

