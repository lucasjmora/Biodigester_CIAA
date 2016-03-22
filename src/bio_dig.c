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
#include "bio_Cfg.h"

/*==================[macros and definitions]=================================*/

//#define  HIGH_LIM_ANAI0    ANA3_HIGH_LIM_LEVEL
//#define  LOW_LIM_ANAI0     ANA3_LOW_LIM_LEVEL  
//#define  HIGH_LIM_ANAI0    800
//#define  LOW_LIM_ANAI0   300 
//#define  HIGH_ALARM_ANAI0    RELAY_1
//#define  LOW_ALARM_ANAI0   RELAY_2

typedef struct {
   uint16_t channel_0;
   uint16_t channel_1;
   uint16_t channel_2;
   uint16_t channel_3;
}anaChReadValues;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/


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

uint16_t ana2_alarm_high_lim;
uint16_t ana2_alarm_low_lim;
uint16_t ana3_alarm_high_lim;
uint16_t ana3_alarm_low_lim;

anaChReadValues anaI_reads;

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
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, (void *)100000);
   
   ana2_alarm_high_lim = (uint16_t)ANA2_HIGH_LIM_LEVEL;//800; 
   ana2_alarm_low_lim = (uint16_t)ANA2_LOW_LIM_LEVEL;//300;
   ana3_alarm_high_lim = (uint16_t)ANA3_HIGH_LIM_LEVEL;//800; 
   ana3_alarm_low_lim = (uint16_t)ANA3_LOW_LIM_LEVEL;//300;

   /* end InitTask */
   TerminateTask();
}

TASK(AnalogReads)
{
#if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, (void *)ciaaCHANNEL_0);
   ciaaPOSIX_read(fd_adc, &anaI_reads.channel_0, sizeof(anaI_reads.channel_0));
#elif (edu_ciaa_nxp == BOARD)
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, (void *)ciaaCHANNEL_1);
   ciaaPOSIX_read(fd_adc, &anaI_reads.channel_1, sizeof(anaI_reads.channel_1));

   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, (void *)ciaaCHANNEL_2);
   ciaaPOSIX_read(fd_adc, &anaI_reads.channel_2, sizeof(anaI_reads.channel_2));

   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, (void *)ciaaCHANNEL_3);
   ciaaPOSIX_read(fd_adc, &anaI_reads.channel_3, sizeof(anaI_reads.channel_3));
#endif

   TerminateTask();
}

TASK(AlarmsCheck)
{
   bool ana2_alarm_high_st;              /* high alarm status (1:ON 0:OFF) */
   bool ana2_alarm_lower_st;             /* lower alarm status (1:ON 0:OFF) */
   bool ana3_alarm_high_st;              /* high alarm status (1:ON 0:OFF) */
   bool ana3_alarm_lower_st;             /* lower alarm status (1:ON 0:OFF) */

/* Analog input channel 2 section */
/*************************************/
/* According to oneVar_twoAlarms.odg */
/*************************************/

   /* get alarms status*/
   ana2_alarm_high_st = ciaaDIO_relay_st(fd_out, ANA2_HIGH_ALARM_RELAY);
   ana2_alarm_lower_st = ciaaDIO_relay_st(fd_out, ANA2_LOW_ALARM_RELAY);

   /* read value inside of allow range */
   if (anaI_reads.channel_2 < ana3_alarm_high_lim && anaI_reads.channel_2 > ana2_alarm_low_lim)
   {
      /* clear alarms if it was ON */
      if (ana2_alarm_lower_st || ana2_alarm_high_st)   
      {
         ciaaDIO_relay_op(fd_out, ANA2_HIGH_ALARM_RELAY, OFF);
         ciaaDIO_relay_op(fd_out, ANA2_LOW_ALARM_RELAY, OFF);
      }
   }
   /* read value over high limit */
   else if (anaI_reads.channel_2 > ana2_alarm_high_lim)
   {
      if (!ana2_alarm_high_st)
      {
         ciaaDIO_relay_op(fd_out, ANA2_HIGH_ALARM_RELAY, ON);
      }
   }
   /* read value under lower limit */
   else if (anaI_reads.channel_2 < ana2_alarm_low_lim)
   {
      if (!ana2_alarm_lower_st)
         ciaaDIO_relay_op(fd_out, ANA2_LOW_ALARM_RELAY, ON);
   }
  // else
  // {
      /* ERROR */
  // }
   
/* Analog input channel 3 section */

/*************************************/
/* According to oneVar_twoAlarms.odg */
/*************************************/

   /* get alarms status*/
   ana3_alarm_high_st = ciaaDIO_relay_st(fd_out, ANA3_HIGH_ALARM_RELAY);
   ana3_alarm_lower_st = ciaaDIO_relay_st(fd_out, ANA3_LOW_ALARM_RELAY);

   /* read value inside of allow range */
   if (anaI_reads.channel_3 < ana3_alarm_high_lim && anaI_reads.channel_3 > ana3_alarm_low_lim)
   {
      /* clear alarms if it was ON */
      if (ana3_alarm_lower_st || ana3_alarm_high_st)   
      {
         ciaaDIO_relay_op(fd_out, ANA3_HIGH_ALARM_RELAY, OFF);
         ciaaDIO_relay_op(fd_out, ANA3_LOW_ALARM_RELAY, OFF);
      }
   }
   /* read value over high limit */
   else if (anaI_reads.channel_3 > ana3_alarm_high_lim)
   {
      if (!ana3_alarm_high_st)
      {
         ciaaDIO_relay_op(fd_out, ANA3_HIGH_ALARM_RELAY, ON);
      }
   }
   /* read value under lower limit */
   else if (anaI_reads.channel_3 < ana3_alarm_low_lim)
   {
      if (!ana3_alarm_lower_st)
         ciaaDIO_relay_op(fd_out, ANA3_LOW_ALARM_RELAY, ON);
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

