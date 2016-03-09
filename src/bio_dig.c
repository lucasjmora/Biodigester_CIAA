/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Gustavo Muro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
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

/** \brief Blinking Modbus example source file
 **
 ** This is a mini example of the CIAA Firmware
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup ADC DAC ADC & DAC example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * GMuro        Gustavo Muro
 * PR           Pablo Ridolfi
 * JCe         Juan Cecconi
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20140805 v0.0.1   GMuro first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "dio_relay.h"

/*==================[macros and definitions]=================================*/

/* high or higher or ??? */
#define  HIGH_LIM_ANAI0    800
#define  LOWER_LIM_ANAI0   300 
#define  HIGH_ALARM_ANAI0    RELAY_1
#define  LOWER_ALARM_ANAI0   RELAY_2
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

void int16ToString(int16_t,char *);
uint8_t OutputPinStatus(int32_t *,uint8_t);
void OutpuPinClear(int32_t *,uint8_t);
void OutpuPinSet(int32_t *,uint8_t);

/*==================[internal data definition]===============================*/

/** \brief File descriptor for ADC
 *
 * Device path /dev/serial/aio/in/0
 */
static int32_t fd_adc;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init the ciaa kernel */
   ciaak_start();

   /* open CIAA digital outputs */
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);
   
   /* open CIAA ADC */
   fd_adc = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_3);
   
   /* Activates the Slave task */
   /* Conflict with Analogic Alarm????? */
   ActivateTask(Analogic);
   /* end InitTask */
   TerminateTask();
}

TASK(Analogic)
{
/*************************************/
/* According to oneVar_twoAlarms.odg */
/*************************************/

   uint16_t anaI0_rv;                  /* analog input 0 read value */
   uint8_t alarm_high_st;              /* high alarm status (1:ON 0:OFF) */
   uint8_t alarm_lower_st;             /* lower alarm status (1:ON 0:OFF) */
   
   /* Read ADC. */
   ciaaPOSIX_read(fd_adc, &anaI0_rv, sizeof(anaI0_rv));


   /* read alarms status*/
   alarm_high_st = ciaaDIO_relay_st(fd_out, HIGH_ALARM_ANAI0);
   alarm_lower_st = ciaaDIO_relay_st(fd_out, LOWER_ALARM_ANAI0);

   /* read value inside of allow range */
   if (anaI0_rv < HIGH_LIM_ANAI0 && anaI0_rv > LOWER_LIM_ANAI0)
   {
      /* clear alarms if it was ON */
      if (alarm_lower_st || alarm_high_st)   
      {
         ciaaDIO_relay_op(fd_out, HIGH_ALARM_ANAI0, OFF);
         ciaaDIO_relay_op(fd_out, LOWER_ALARM_ANAI0, OFF);
      }
   }
   /* read value over high limit */
   else if (anaI0_rv > HIGH_LIM_ANAI0)
   {
      if (!alarm_high_st)
      {
         ciaaDIO_relay_op(fd_out, HIGH_ALARM_ANAI0, ON);
      }
   }
   /* read value under lower limit */
   else if (anaI0_rv < LOWER_LIM_ANAI0)
   {
      if (!alarm_lower_st)
         ciaaDIO_relay_op(fd_out, LOWER_ALARM_ANAI0, ON);
   }
  // else
  // {
      /* ERROR */
  // }
   
   /* end of Blinking */
   TerminateTask();
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

