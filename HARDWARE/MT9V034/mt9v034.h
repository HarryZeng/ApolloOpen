/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

#ifndef MT9V34_H_
#define MT9V34_H_

#include <stdint.h>
#include "settings.h"

/* Constants */
#define TIMEOUT_MAX      				10000

#define mt9v034_DEVICE_WRITE_ADDRESS  0x90
#define mt9v034_DEVICE_READ_ADDRESS  	0x91
#define  MTV_CHIP_VERSION_REG					0x00


#define MT9V034_MAX_HEIGHT				        			(480)
#define MT9V034_MAX_WIDTH					    					(752)
#define MT9V034_CHIP_VERSION                    (0x00)
#define MT9V034_COL_START                       (0x01)
#define MT9V034_COL_START_MIN                   (1)
#define MT9V034_COL_START_MAX                   (752)
#define MT9V034_ROW_START                       (0x02)
#define MT9V034_ROW_START_MIN                   (4)
#define MT9V034_ROW_START_MAX                   (482)
#define MT9V034_WINDOW_HEIGHT                   (0x03)
#define MT9V034_WINDOW_HEIGHT_MIN               (1)
#define MT9V034_WINDOW_HEIGHT_MAX               (480)
#define MT9V034_WINDOW_WIDTH                    (0x04)
#define MT9V034_WINDOW_WIDTH_MIN                (1)
#define MT9V034_WINDOW_WIDTH_MAX                (752)
#define MT9V034_HORIZONTAL_BLANKING             (0x05)
#define MT9V034_HORIZONTAL_BLANKING_MIN         (43)
#define MT9V034_HORIZONTAL_BLANKING_MAX         (1023)
#define MT9V034_VERTICAL_BLANKING               (0x06)
#define MT9V034_VERTICAL_BLANKING_MIN           (4)
#define MT9V034_VERTICAL_BLANKING_MAX           (3000)
#define MT9V034_CHIP_CONTROL                    (0x07)
#define MT9V034_CHIP_CONTROL_MASTER_MODE        (1 << 3)
#define MT9V034_CHIP_CONTROL_DOUT_ENABLE        (1 << 7)
#define MT9V034_CHIP_CONTROL_SEQUENTIAL         (1 << 8)
#define MT9V034_SHUTTER_WIDTH1                  (0x08)
#define MT9V034_SHUTTER_WIDTH2                  (0x09)
#define MT9V034_SHUTTER_WIDTH_CONTROL           (0x0A)
#define MT9V034_TOTAL_SHUTTER_WIDTH             (0x0B)
#define MT9V034_TOTAL_SHUTTER_WIDTH_MIN         (1)
#define MT9V034_TOTAL_SHUTTER_WIDTH_MAX         (32767)
#define MT9V034_RESET                           (0x0C)
#define MT9V034_READ_MODE                       (0x0D)
#define MT9V034_READ_MODE_ROW_BIN_2             (1 << 0)
#define MT9V034_READ_MODE_ROW_BIN_4             (1 << 1)
#define MT9V034_READ_MODE_COL_BIN_2             (1 << 2)
#define MT9V034_READ_MODE_COL_BIN_4             (1 << 3)
#define MT9V034_READ_MODE_ROW_FLIP              (1 << 4)
#define MT9V034_READ_MODE_COL_FLIP              (1 << 5)
#define MT9V034_READ_MODE_DARK_COLS             (1 << 6)
#define MT9V034_READ_MODE_DARK_ROWS             (1 << 7)
#define MT9V034_PIXEL_OPERATION_MODE            (0x0F)
#define MT9V034_PIXEL_OPERATION_MODE_COLOR      (1 << 2)
#define MT9V034_PIXEL_OPERATION_MODE_HDR        (1 << 6)
#define MT9V034_ANALOG_GAIN                     (0x35)
#define MT9V034_ANALOG_GAIN_MIN                 (16)
#define MT9V034_ANALOG_GAIN_MAX                 (64)
#define MT9V034_MAX_ANALOG_GAIN                 (0x36)
#define MT9V034_MAX_ANALOG_GAIN_MAX             (127)
#define MT9V034_FRAME_DARK_AVERAGE              (0x42)
#define MT9V034_DARK_AVG_THRESH                 (0x46)
#define MT9V034_DARK_AVG_LOW_THRESH_MASK        (255 << 0)
#define MT9V034_DARK_AVG_LOW_THRESH_SHIFT       (0)
#define MT9V034_DARK_AVG_HIGH_THRESH_MASK       (255 << 8)
#define MT9V034_DARK_AVG_HIGH_THRESH_SHIFT      (8)
#define MT9V034_ROW_NOISE_CORR_CONTROL          (0x70)
#define MT9V034_ROW_NOISE_CORR_ENABLE           (1 << 5)
#define MT9V034_ROW_NOISE_CORR_USE_BLK_AVG      (1 << 7)
#define MT9V034_PIXEL_CLOCK                     (0x72)
#define MT9V034_PIXEL_CLOCK_INV_LINE            (1 << 0)
#define MT9V034_PIXEL_CLOCK_INV_FRAME           (1 << 1)
#define MT9V034_PIXEL_CLOCK_XOR_LINE            (1 << 2)
#define MT9V034_PIXEL_CLOCK_CONT_LINE           (1 << 3)
#define MT9V034_PIXEL_CLOCK_INV_PXL_CLK         (1 << 4)
#define MT9V034_TEST_PATTERN                    (0x7F)
#define MT9V034_TEST_PATTERN_DATA_MASK          (1023 << 0)
#define MT9V034_TEST_PATTERN_DATA_SHIFT         (0)
#define MT9V034_TEST_PATTERN_USE_DATA           (1 << 10)
#define MT9V034_TEST_PATTERN_GRAY_MASK          (3 << 11)
#define MT9V034_TEST_PATTERN_GRAY_NONE          (0 << 11)
#define MT9V034_TEST_PATTERN_GRAY_VERTICAL      (1 << 11)
#define MT9V034_TEST_PATTERN_GRAY_HORIZONTAL    (2 << 11)
#define MT9V034_TEST_PATTERN_GRAY_DIAGONAL      (3 << 11)
#define MT9V034_TEST_PATTERN_ENABLE             (1 << 13)
#define MT9V034_TEST_PATTERN_FLIP               (1 << 14)
#define MT9V034_AEC_AGC_ENABLE                  (0xAF)
#define MT9V034_AEC_ENABLE                      (1 << 0)
#define MT9V034_AGC_ENABLE                      (1 << 1)
#define MT9V034_THERMAL_INFO                    (0xC1)

#define MT9V034_WINDOW_HEIGHT_A                 (0x03)
#define MT9V034_WINDOW_WIDTH_A                  (0x04)
#define MT9V034_HORIZONTAL_BLANKING_A           (0x05)
#define MT9V034_VERTICAL_BLANKING_A             (0x06)
#define MT9V034_COARSE_SHUTTER_WIDTH_TOTAL_A    (0x0B)
#define MT9V034_ANALOG_GAIN_CONTROL             (0x35)
#define MT9V034_MAX_GAIN                        (0xAB)
#define MT9V034_FINE_SHUTTER_WIDTH_TOTAL_A      (0xD5)



/* DCMI */
#define DCMI_TIM                (TIM1)
#define DCMI_TIM_PIN            (GPIO_PIN_8)
#define DCMI_TIM_PORT           (GPIOA)
#define DCMI_TIM_AF             (GPIO_AF1_TIM1)
#define DCMI_TIM_CHANNEL        (TIM_CHANNEL_1)
#define DCMI_TIM_CLK_ENABLE()   __TIM1_CLK_ENABLE()
#define DCMI_TIM_CLK_DISABLE()  __TIM1_CLK_DISABLE()
#define DCMI_TIM_PCLK_FREQ()    HAL_RCC_GetPCLK2Freq()

/* Functions */

/**
  * @brief  Configures the mt9v034 camera with two context (binning 4 and binning 2).
  */
void mt9v034_context_configuration(void);

/**
  * @brief  Changes sensor context based on settings
  */
void mt9v034_set_context(void);

uint16_t mt9v034_ReadReg16(uint8_t address);
uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data);
uint8_t mt9v034_ReadReg(uint16_t Addr);
uint8_t mt9v034_WriteReg(uint16_t Addr, uint8_t Data);

#endif /* MT9V34_H_ */
