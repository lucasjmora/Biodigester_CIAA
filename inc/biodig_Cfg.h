/* Copyright 2014, ACSE & CADIEEL
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, Your Name <youremail@domain.com>
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, ACSE & CADIEEL & Your Name <youremail@domain.com
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BIODIG_CFG_H
#define BIODIG_CFG_H
/** \brief General configurations and perform scaled of values */

/*==================[inclusions]=============================================*/
#include "biodigRelays.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/** \brief User setting section
 **
 ** User must be enter the requested parameters for each analog channel
 ** 
 ** \remarks Select analog input range:
 **          ANAn_INPUT_RANGE_A: 0..10V or 0..20mA or edu_ciaa(0..3.3V) 
 **          ANAn_INPUT_RANGE_B: 4..20mA  
 **  Only analog channels 2 and 3 have high and low alarms(relays).
 ** (TODO: check max and min lim inside of range)
 **/
#if (ciaa_nxp == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
#define ANA0_INPUT_RANGE_A 
#define ANA0_PHYSICAL_MAGNITUDE        "temperature"
#define ANA0_PHYSICAL_MAGNITUDE_UNIT   "degree_celsius"
#define ANA0_PHYSICAL_MAGNITUDE_MAX    150 
#define ANA0_PHYSICAL_MAGNITUDE_MIN    -10 
#endif

#define ANA1_INPUT_RANGE_A 
#define ANA1_PHYSICAL_MAGNITUDE        "temperature"
#define ANA1_PHYSICAL_MAGNITUDE_UNIT   "degree_celsius"
#define ANA1_PHYSICAL_MAGNITUDE_MAX    150 
#define ANA1_PHYSICAL_MAGNITUDE_MIN    -10 

#define ANA2_INPUT_RANGE_A 
#define ANA2_PHYSICAL_MAGNITUDE        "pH"
#define ANA2_PHYSICAL_MAGNITUDE_UNIT   "degree_celsius"
#define ANA2_PHYSICAL_MAGNITUDE_MAX    14 
#define ANA2_PHYSICAL_MAGNITUDE_MIN    0 
#define ANA2_HIGH_ALARM_LIM            12 
#define ANA2_LOW_ALARM_LIM             3 
#define ANA2_HIGH_ALARM_RELAY          RELAY_1
#define ANA2_LOW_ALARM_RELAY           RELAY_2

#define ANA3_INPUT_RANGE_A 
#define ANA3_PHYSICAL_MAGNITUDE        "temperature"
#define ANA3_PHYSICAL_MAGNITUDE_UNIT   "degree_celsius"
#define ANA3_PHYSICAL_MAGNITUDE_MAX    150 
#define ANA3_PHYSICAL_MAGNITUDE_MIN    -10 
#define ANA3_HIGH_ALARM_LIM            100 
#define ANA3_LOW_ALARM_LIM             10
#define ANA3_HIGH_ALARM_RELAY          RELAY_3
#define ANA3_LOW_ALARM_RELAY           RELAY_4

/*-------------------------User setting section end -------------------------*/
/** \brief electrical analog input range mapped on analog level range */
#if (ciaa_nxp == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
#ifdef ANA0_INPUT_RANGE_A
#define ANA0_LEVEL_MAX    1023
#define ANA0_LEVEL_MIN    0
#elif ANA0_INPUT_RANGE_B
#define ANA0_LEVEL_MAX    1023
#define ANA0_LEVEL_MIN    205   
#endif
#endif /* ciaa_nxp == BOARD */

#ifdef ANA1_INPUT_RANGE_A
#define ANA1_LEVEL_MAX    1023
#define ANA1_LEVEL_MIN    0
#elif ANA1_INPUT_RANGE_B
#define ANA1_LEVEL_MAX    1023
#define ANA1_LEVEL_MIN    205   
#endif

#ifdef ANA2_INPUT_RANGE_A
#define ANA2_LEVEL_MAX    1023
#define ANA2_LEVEL_MIN    0
#elif ANA2_INPUT_RANGE_B
#define ANA2_LEVEL_MAX    1023
#define ANA2_LEVEL_MIN    205   
#endif

#ifdef ANA3_INPUT_RANGE_A
#define ANA3_LEVEL_MAX    1023
#define ANA3_LEVEL_MIN    0
#elif ANA3_INPUT_RANGE_B
#define ANA3_LEVEL_MAX    1023
#define ANA3_LEVEL_MIN    205   
#endif

/** \brief Scaled analog read values
 ** 
 ** READ MAGNITUDE VALUE = ANA_FACTOR * (anaI level read value) + ANA_OFFSET
 **
 ** \remarks   Use ANAx_FACTOR and ANAx_OFFSET carefull to avoid extra compute.
 **            TIP: Call they only once, maybe in a init task and assig result
 **            to a const variable. 
 **/
#if (ciaa_nxp == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
#define ANA0_FACTOR \
   ((float)ANA0_PHYSICAL_MAGNITUDE_MAX - (float)ANA0_PHYSICAL_MAGNITUDE_MIN) /\
   (float)(ANA0_LEVEL_MAX - ANA0_LEVEL_MIN)

#define ANA0_OFFSET  (float)ANA0_PHYSICAL_MAGNITUDE_MIN + \
   ((-(float)ANA0_LEVEL_MIN * \
   ((float)ANA0_PHYSICAL_MAGNITUDE_MAX - (float)ANA0_PHYSICAL_MAGNITUDE_MIN))/\
   (float)(ANA0_LEVEL_MAX - ANA0_LEVEL_MIN))
#endif

#define ANA1_FACTOR \
   ((float)ANA1_PHYSICAL_MAGNITUDE_MAX - (float)ANA1_PHYSICAL_MAGNITUDE_MIN) /\
   (float)(ANA1_LEVEL_MAX - ANA1_LEVEL_MIN)

#define ANA1_OFFSET  (float)ANA1_PHYSICAL_MAGNITUDE_MIN + \
   ((-(float)ANA1_LEVEL_MIN * \
   ((float)ANA1_PHYSICAL_MAGNITUDE_MAX - (float)ANA1_PHYSICAL_MAGNITUDE_MIN))/\
   (float)(ANA1_LEVEL_MAX - ANA1_LEVEL_MIN))

#define ANA2_FACTOR \
   ((float)ANA2_PHYSICAL_MAGNITUDE_MAX - (float)ANA2_PHYSICAL_MAGNITUDE_MIN) /\
   (float)(ANA2_LEVEL_MAX - ANA2_LEVEL_MIN)

#define ANA2_OFFSET  (float)ANA2_PHYSICAL_MAGNITUDE_MIN + \
   ((-(float)ANA2_LEVEL_MIN * \
   ((float)ANA2_PHYSICAL_MAGNITUDE_MAX - (float)ANA2_PHYSICAL_MAGNITUDE_MIN))/\
   (float)(ANA2_LEVEL_MAX - ANA2_LEVEL_MIN))

#define ANA3_FACTOR \
   ((float)ANA3_PHYSICAL_MAGNITUDE_MAX - (float)ANA3_PHYSICAL_MAGNITUDE_MIN) /\
   (float)(ANA3_LEVEL_MAX - ANA3_LEVEL_MIN)

#define ANA3_OFFSET  (float)ANA3_PHYSICAL_MAGNITUDE_MIN + \
   ((-(float)ANA3_LEVEL_MIN * \
   ((float)ANA3_PHYSICAL_MAGNITUDE_MAX - (float)ANA3_PHYSICAL_MAGNITUDE_MIN))/\
   (float)(ANA3_LEVEL_MAX - ANA3_LEVEL_MIN))

/**Alarm limit from magnitude values to adc levels values convertion
 **
 ** READ MAGNITUDE VALUE = ANA_FACTOR * (anaI level read value) + ANA_OFFSET 
 ** ==> ANAx_HIGH_LIM_LEVEL = (ANAx_HIGH_ALARM_LIM - ANAx_OFFSET) / ANAx_FACTOR
 **/
#define ANA2_HIGH_LIM_LEVEL   ((float)ANA2_HIGH_ALARM_LIM - ANA2_OFFSET) / \
                              (ANA2_FACTOR)

#define ANA2_LOW_LIM_LEVEL    ((float)ANA2_LOW_ALARM_LIM - ANA2_OFFSET) / \
                              (ANA2_FACTOR)

#define ANA3_HIGH_LIM_LEVEL   ((float)ANA3_HIGH_ALARM_LIM - ANA3_OFFSET) / \
                              (ANA3_FACTOR)

#define ANA3_LOW_LIM_LEVEL    ((float)ANA3_LOW_ALARM_LIM - ANA3_OFFSET) / \
                              (ANA3_FACTOR)

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef BIODIG_CFG_H */

