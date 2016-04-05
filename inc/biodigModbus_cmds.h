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

#ifndef BIODIGMODBUS_CMDS_H
#define BIODIGMODBUS_CMDS_H
/** \brief Modbus commands 
 **
 ** Modbus functions used and modbus device slave id are here
 **/

/*==================[inclusions]=============================================*/
#include "os.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/** \brief Modbus device id. This is the address which the device is identify
 ** on modbus network. 
 **/   
#define MODBUS_ID    2

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Function 0x04 according to Modbus standard
 **
 ** This one is used to return analog values gets by analog channels to a
 ** master device request. 
 ** Return to master the request value scaled with IEEE754 floating encoding,
 ** 32 bits(2 consecutive register) and, according to modbus standard, in
 ** bigendian.  
 ** Register layout:
 ** Input register 0x0000 --> analog channel 0
 ** Input register 0x0002 --> analog channel 1
 ** Input register 0x0004 --> analog channel 2
 ** Input register 0x0006 --> analog channel 3
 **
 ** \remarks It is a virtual function: prototype is defined by CIAA Modbus
 ** module and implementation must be to do according to specific app
 ** requeriments. 
 **/
uint16_t cmd0x04ReadInputReg(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       );

/** \brief Function 0x01 according to Modbus standard
 **
 ** This one is used to return relays status to a master device request. 
 ** Coil layout:
 ** Coil register: 0x0010 
 ** bit 0 --> Relay 1 
 ** bit 1 --> Relay 2 
 ** bit 2 --> Relay 3 
 ** bit 3 --> Relay 4 
 ** bits 4..15 --> not used
 **
 ** \remarks It is a virtual function: prototype is defined by CIAA Modbus
 ** module and implementation must be to do according to specific app
 ** requeriments. 
 **/
uint16_t cmd0x01ReadCoils(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       );

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/*==================[end of file]============================================*/
#endif /* #ifndef BIODIGMODBUS_CMDS_H */

