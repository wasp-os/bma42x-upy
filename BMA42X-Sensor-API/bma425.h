/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bma425.h
 * @date       2020-05-08
 * @version    V2.14.13
 *
 */

/**
 * \ingroup bma4xy
 * \defgroup bma425 BMA425
 * @brief Sensor driver for BMA425 sensor
 */

#ifndef BMA425_H
#define BMA425_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA425 sensor */
#define BMA425_CHIP_ID                 UINT8_C(0x13)

/**\ Configuration ID start position of BMA425 sensor */
#define BMA425_CONFIG_ID_START_ADDR    UINT8_C(66)

/**\name Sensor feature size */
#define BMA425_FEATURE_SIZE            UINT8_C(70)
#define BMA425_ANY_MOT_LEN             UINT8_C(4)

/**\name Feature offset address */
#define BMA425_ANY_MOT_OFFSET          UINT8_C(0x00)
#define BMA425_NO_MOT_OFFSET           UINT8_C(0x04)
#define BMA425_STEP_CNTR_PARAM_OFFSET  UINT8_C(0x08)
#define BMA425_STEP_CNTR_OFFSET        UINT8_C(0x3A)
#define BMA425_SINGLE_TAP_OFFSET       UINT8_C(0x3C)
#define BMA425_DOUBLE_TAP_OFFSET       UINT8_C(0x3E)
#define BMA425_WRIST_WEAR_OFFSET       UINT8_C(0x40)
#define BMA425_CONFIG_ID_OFFSET        UINT8_C(0x42)
#define BMA425_AXES_REMAP_OFFSET       UINT8_C(0x44)

/**\name Read/Write Lengths */
#define BMA425_RD_WR_MIN_LEN           UINT8_C(2)
#define BMA425_NO_MOT_RD_WR_LEN        (BMA425_ANY_MOT_LEN + BMA425_NO_MOT_OFFSET)

/*! @name Mask definitions for major and minor config */
#define BMA425_CONFIG_MAJOR_MSK        UINT16_C(0X3C0)
#define BMA425_CONFIG_MINOR_MSK        UINT8_C(0X1F)

/*! @name Bit position for major config */
#define BMA425_CONFIG_MAJOR_POS        UINT8_C(0X06)

/**************************************************************/
/**\name    Re-map Axes */
/**************************************************************/
#define BMA425_X_AXIS_MASK             UINT8_C(0x03)
#define BMA425_X_AXIS_SIGN_MASK        UINT8_C(0x04)
#define BMA425_Y_AXIS_MASK             UINT8_C(0x18)
#define BMA425_Y_AXIS_SIGN_MASK        UINT8_C(0x20)
#define BMA425_Z_AXIS_MASK             UINT8_C(0xC0)
#define BMA425_Z_AXIS_SIGN_MASK        UINT8_C(0x01)

/**************************************************************/
/**\name    Step Counter/Detector/Activity */
/**************************************************************/
/**\name Step counter/activity enable macros */
#define BMA425_STEP_CNTR_EN_MSK        UINT8_C(0x10)
#define BMA425_STEP_ACT_EN_MSK         UINT8_C(0x20)

/**\name Step counter water-mark macros */
#define BMA425_STEP_CNTR_WM_MSK        UINT16_C(0x03FF)

/**\name Step counter reset macros */
#define BMA425_STEP_CNTR_RST_POS       UINT8_C(2)
#define BMA425_STEP_CNTR_RST_MSK       UINT8_C(0x04)

/**\name Step detector enable macros */
#define BMA425_STEP_DETECTOR_EN_POS    UINT8_C(3)
#define BMA425_STEP_DETECTOR_EN_MSK    UINT8_C(0x08)

/**\name Wrist-wear enable macros */
#define BMA425_WRIST_WEAR_EN_MSK       UINT8_C(0x01)

/**\name Step count output length*/
#define BMA425_STEP_CNTR_DATA_SIZE     UINT16_C(4)

/**\name single tap enable macros */
#define BMA425_SINGLE_TAP_EN_MSK       UINT8_C(0x01)

/**\name double tap enable macros */
#define BMA425_DOUBLE_TAP_EN_MSK       UINT8_C(0x01)

/**\name tap sensitivity macros */
#define BMA425_TAP_SENS_POS            UINT8_C(1)
#define BMA425_TAP_SENS_MSK            UINT8_C(0x0E)

/**\name Tap selection macro */
#define BMA425_TAP_SEL_POS             UINT8_C(4)
#define BMA425_TAP_SEL_MSK             UINT8_C(0x10)

/**************************************************************/
/**\name    Any/no Motion */
/**************************************************************/
/**\name Any/No motion threshold macros */
#define BMA425_ANY_NO_MOT_THRES_MSK    UINT16_C(0x07FF)

/**\name Any/No motion duration macros */
#define BMA425_ANY_NO_MOT_DUR_MSK      UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA425_ANY_NO_MOT_AXIS_EN_POS  UINT8_C(0x0D)
#define BMA425_ANY_NO_MOT_AXIS_EN_MSK  UINT16_C(0xE000)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA425_X_AXIS_EN               UINT8_C(0x01)
#define BMA425_Y_AXIS_EN               UINT8_C(0x02)
#define BMA425_Z_AXIS_EN               UINT8_C(0x04)
#define BMA425_EN_ALL_AXIS             UINT8_C(0x07)
#define BMA425_DIS_ALL_AXIS            UINT8_C(0x00)

/**\name Feature enable macros for the sensor */
#define BMA425_STEP_CNTR               UINT8_C(0x01)
#define BMA425_STEP_ACT                UINT8_C(0x02)
#define BMA425_WRIST_WEAR              UINT8_C(0x04)
#define BMA425_SINGLE_TAP              UINT8_C(0x08)
#define BMA425_DOUBLE_TAP              UINT8_C(0x10)

/**\name Interrupt status macros */
#define BMA425_SINGLE_TAP_INT          UINT8_C(0x01)
#define BMA425_STEP_CNTR_INT           UINT8_C(0x02)
#define BMA425_ACTIVITY_INT            UINT8_C(0x04)
#define BMA425_WRIST_WEAR_INT          UINT8_C(0x08)
#define BMA425_DOUBLE_TAP_INT          UINT8_C(0x10)
#define BMA425_ANY_MOT_INT             UINT8_C(0x20)
#define BMA425_NO_MOT_INT              UINT8_C(0x40)
#define BMA425_ERROR_INT               UINT8_C(0x80)

/**\name Activity recognition macros */
#define BMA425_USER_STATIONARY         UINT8_C(0x00)
#define BMA425_USER_WALKING            UINT8_C(0x01)
#define BMA425_USER_RUNNING            UINT8_C(0x02)
#define BMA425_STATE_INVALID           UINT8_C(0x03)

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */
