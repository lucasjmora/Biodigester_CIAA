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
#include "bio_dig.h"

/*==================[macros and definitions]=================================*/

#define  HIGH_LIM    800
#define  LOWER_LIM   300 
#define  HIGH_ALARM_OUT    5  /* Led 3 */
#define  LOWER_ALARM_OUT   4  /* Led 2 */
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

/** \brief File descriptor for DAC
 *
 * Device path /dev/serial/aio/out/0
 */
static int32_t fd_dac;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void int16ToString(int16_t int16_to_string, char *output_string)
{
   //uint32_t uint32_to_string = (uint32_t) float_to_string;
   int8_t loopi;
   uint8_t get_nibble;
   //uint8_t *array_byte = (uint8_t*) &float_to_string;  
   for (loopi = 3; loopi >= 0; loopi--)
   {
      get_nibble = int16_to_string & 0xF;
      if (get_nibble == 0x0)
         *(output_string + loopi) = '0';      
      else if (get_nibble == 0x1)
         *(output_string + loopi) = '1';      
      else if (get_nibble == 0x2)
         *(output_string + loopi) = '2';      
      else if (get_nibble == 0x3)
         *(output_string + loopi) = '3';      
      else if (get_nibble == 0x4)
         *(output_string + loopi) = '4';      
      else if (get_nibble == 0x5)
         *(output_string + loopi) = '5';      
      else if (get_nibble == 0x6)
         *(output_string + loopi) = '6';      
      else if (get_nibble == 0x7)
         *(output_string + loopi) = '7';      
      else if (get_nibble == 0x8)
         *(output_string + loopi) = '8';      
      else if (get_nibble == 0x9)
         *(output_string + loopi) = '9';      
      else if (get_nibble == 0xA)
         *(output_string + loopi) = 'A';      
      else if (get_nibble == 0xB)
         *(output_string + loopi) = 'B';      
      else if (get_nibble == 0xC)
         *(output_string + loopi) = 'C';      
      else if (get_nibble == 0xD)
         *(output_string + loopi) = 'D';      
      else if (get_nibble == 0xE)
         *(output_string + loopi) = 'E';      
      else if (get_nibble == 0xF)
         *(output_string + loopi) = 'F';      
    //  else
         /* ERROR */
      // in C can not shift a float, only a int !!!
      int16_to_string >>= 4;
   }
}


uint8_t OutputPinStatus(int32_t *fildes_out,uint8_t pin_to_test)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port_status;
   
   one_in_pin <<= pin_to_test;
   ciaaPOSIX_read(*fildes_out, &output_port_status, 1);
   output_port_status &= one_in_pin;
   return output_port_status;
}

void OutpuPinClear(int32_t *fildes_out,uint8_t pin_to_clear)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port;
   
   one_in_pin <<= pin_to_clear;
   ciaaPOSIX_read(*fildes_out, &output_port, 1);
   output_port &= ~one_in_pin;
   ciaaPOSIX_write(fd_out, &output_port, 1);
}

void OutpuPinSet(int32_t *fildes_out,uint8_t pin_to_set)
{
   /* TODO: implement pin_to_test 0...7 validation check */
   uint8_t one_in_pin = 1;
   uint8_t output_port;
   
   one_in_pin <<= pin_to_set;
   ciaaPOSIX_read(*fildes_out, &output_port, 1);
   output_port |= one_in_pin;
   ciaaPOSIX_write(fd_out, &output_port, 1);
}
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
   
   /* open CIAA DAC */
   fd_dac = ciaaPOSIX_open("/dev/serial/aio/out/0", ciaaPOSIX_O_WRONLY);
   ciaaPOSIX_ioctl(fd_dac, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* Activates the ModbusSlave task */
   ActivateTask(Analogic);

   /* end InitTask */
   TerminateTask();
}

TASK(Analogic)
{
/*************************************/
/* According to oneVar_twoAlarms.odg */
/*************************************/

   uint16_t hr_ciaaDac;
   uint8_t outputs;
   uint8_t alarm_high_status;
   uint8_t alarm_lower_status;
   int16_t temp;
   char packet[5]; 

   int8_t message = 2; 
   //uint8_t buf[20];
   //int32_t ret = 0;
   /* Read ADC. */
   ciaaPOSIX_read(fd_adc, &hr_ciaaDac, sizeof(hr_ciaaDac));


   /* read alarms status*/
   alarm_high_status = OutputPinStatus(&fd_out,HIGH_ALARM_OUT);
   alarm_lower_status = OutputPinStatus(&fd_out,LOWER_ALARM_OUT);

   if (hr_ciaaDac < HIGH_LIM && hr_ciaaDac > LOWER_LIM)
   {
      if (alarm_lower_status || alarm_high_status)   
      {
         OutpuPinClear(&fd_out,HIGH_ALARM_OUT);
         OutpuPinClear(&fd_out,LOWER_ALARM_OUT);
      }
   }
   else if (hr_ciaaDac > HIGH_LIM)
   {
      if (!alarm_high_status)
      {
         OutpuPinSet(&fd_out,HIGH_ALARM_OUT);
         ciaaPOSIX_write(fd_uart1, message, sizeof(message));
//         ciaaPOSIX_write(fd_uart1, &hr_ciaaDac, sizeof(hr_ciaaDac));
      }
   }
   else if (hr_ciaaDac < LOWER_LIM)
   {
      if (!alarm_lower_status)
         OutpuPinSet(&fd_out,LOWER_ALARM_OUT);
   }
   else
   {
      /* ERROR */
   }
   
   /* Write DAC */
   ciaaPOSIX_write(fd_dac, &hr_ciaaDac, sizeof(hr_ciaaDac));
   
   /* send adc read value to uart on usb bridge (ft2232) */
      
   /* Signal gain. */
   temp = hr_ciaaDac * 150 / 1024;
   int16ToString(temp, packet);   
   packet[4] = '\n';
   //ciaaPOSIX_printf("%s %i\n",packet,temp);
   ciaaPOSIX_write(fd_uart1, packet, ciaaPOSIX_strlen(packet));
   
   /* end of Blinking */
   TerminateTask();
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

