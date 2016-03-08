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

/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "ciaaPOSIX_stdio.h"
#include "dio_relay.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static uint8_t OutputPinStatus(int32_t fildes_out,uint8_t pin_to_test);
static void OutpuPinClear(int32_t fildes_out,uint8_t pin_to_clear);
static void OutpuPinSet(int32_t fildes_out,uint8_t pin_to_set);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static uint8_t OutputPinStatus(int32_t fildes_out,uint8_t pin_to_test)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port_status;
   
   one_in_pin <<= pin_to_test;
   ciaaPOSIX_read(fildes_out, &output_port_status, 1);
   output_port_status &= one_in_pin;
   return output_port_status;
}

static void OutpuPinClear(int32_t fildes_out,uint8_t pin_to_clear)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port;
   
   one_in_pin <<= pin_to_clear;
   ciaaPOSIX_read(fildes_out, &output_port, 1);
   output_port &= ~one_in_pin;
   ciaaPOSIX_write(fildes_out, &output_port, 1);
}

static void OutpuPinSet(int32_t fildes_out,uint8_t pin_to_set)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port;
   
   one_in_pin <<= pin_to_set;
   ciaaPOSIX_read(fildes_out, &output_port, 1);
   output_port |= one_in_pin;
   ciaaPOSIX_write(fildes_out, &output_port, 1);
}

/*==================[external functions definition]==========================*/
int8_t ciaaDIO_relay_op(int32_t fildes_out, uint8_t relay_id, bool oper)
{
   int8_t ret = 0;
   /* check relay_id between RELAY_1 and RELAY_4 */
   /* check previous relay state??? */
   if (oper == ON)
   {
      OutpuPinSet(fildes_out, relay_id);
   }
   else if (oper == OFF)
   {
      OutpuPinClear(fildes_out, relay_id);
   }
   else
   {
      ret = ERR;
   }
   
   return ret; 
}

int8_t ciaaDIO_relay_st(int32_t fildes_out, uint8_t relay_id)
{
   /* check relay_id between RELAY_1 and RELAY_4 */
   int8_t ret = 0;
   if (OutputPinStatus(fildes_out, relay_id) == relay_id)
   {
      ret = ON;
   }
   else if (OutputPinStatus(fildes_out, relay_id) == 0)
   {
      ret = OFF;
   }
   else
   {
      ret = ERR;
   }
   
   return ret;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
