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

#ifndef BIODIGRELAYS_H
#define BIODIGRELAYS_H
/** \brief Relays commands
 **
 ** API for relays operations (and mapping CIAA-NXP relays on leds for 
 ** edu-ciaa-nxp). 
 **/

/*==================[inclusions]=============================================*/
#include "ciaaPOSIX_stdio.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/** \brief ON/OFF are operation actions on relays and ERR is used for errors */
#define  ON    true
#define  OFF   false
#define  ERR   -1

/** \brief Relays id
 *
 ** Relays mapped on output port bits 
 **/
#if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)
#define RELAY_1   4
#define RELAY_2   5
#define RELAY_3   6
#define RELAY_4   7

/** \brief Leds are use in place of relays on edu-ciaa-nxp
 **
 ** \remarks   If you build for edu-ciaa-nxp (BOARD = edu_ciaa_nxp) follow
 **            relays to leds map are implemented:
 **            RELAY_1 --> LED BLUE
 **            RELAY_2 --> LED 1
 **            RELAY_3 --> LED 2
 **            RELAY_4 --> LED 3
 **/
#elif (edu_ciaa_nxp == BOARD)
#define RELAY_1   2  /* LED BLUE */
#define RELAY_2   3  /* LED 1 */
#define RELAY_3   4  /* LED 2 */
#define RELAY_4   5  /* LED 3 */
#endif

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Activate or deactivate a relay.
 **
 ** \param[in]    fildes_out output port's file descriptor 
 ** \param[in]    relay_id relay's id to operate 
 ** \param[in]    oper action to perform over relay(ON/OFF) 
 ** \return 1 if success -1 if an error occurs
 **/
extern int8_t biodigRelays_operate(int32_t fildes_out, uint8_t relay_id, bool oper);

/** \brief Get current relay state 
 **
 ** \param[in]    fildes_out output port's file descriptor 
 ** \param[in]    relay_id relay's id to operate 
 ** \return status of relay_id(1: ON / 0: OFF) -1 if an error occurs
 **/
extern int8_t biodigRelays_getStatus(int32_t fildes_out, uint8_t relay_id);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef BIODIGRELAYS_H */

