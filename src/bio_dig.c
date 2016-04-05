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

/*==================[inclusions]=============================================*/
#include "os.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaaModbus.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "biodig_modbus_slave.h"
#include "biodigRelays.h"
#include "biodigAlarms.h"
#include "biodig_Cfg.h"
#include "biodigModbus_cmds.h"

/*==================[macros and definitions]=================================*/
/** \brief number of analog input channels in use */ 
#define ANALOG_CHANNELS_QTY   4

/** \brief number of alarms in use */ 
#define ALARMS_QTY   4

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** \brief callback to execute when alarm is trigger */
static bool anaChAlarmSetAction(void *);

/** \brief callback to execute when alarm is clear */
static bool anaChAlarmClearAction(void *);

/*==================[internal data definition]===============================*/
static int32_t hModbusSlave;
static int32_t hModbusAscii;
static int32_t hModbusGateway;

static const ciaaModbus_slaveCmd_type callbacksStruct =
{
   cmd0x01ReadCoils,
   NULL,
   NULL,
   cmd0x04ReadInputReg,
   NULL,
   NULL,
   NULL,
   NULL,
};

/** \brief arguments for each alarm callback */
static uint8_t alarmArgActions[] = {
   ANA2_HIGH_ALARM_RELAY,
   ANA2_LOW_ALARM_RELAY,
   ANA3_HIGH_ALARM_RELAY,
   ANA3_LOW_ALARM_RELAY
};

/** \brief alarm's parameters */
static alarmParamType alarmParam[ALARMS_QTY] = {
   {(uint16_t)ANA2_HIGH_LIM_LEVEL, 1},    /* alarm limit and set high alarm  */
   {(uint16_t)ANA2_LOW_LIM_LEVEL, 0},     /* alarm limit and set low alarm  */
   {(uint16_t)ANA3_HIGH_LIM_LEVEL, 1},    /* alarm limit and set high alarm  */
   {(uint16_t)ANA3_LOW_LIM_LEVEL, 0}      /* alarm limit and set low alarm  */
};

/** \brief alarms objects */
static alarmType anaCh2_alarm_high = {
   (void *)&alarmArgActions[0],
   (void *)&alarmArgActions[0],
   anaChAlarmSetAction,
   anaChAlarmClearAction,  
   &alarmParam[0]
};

static alarmType anaCh2_alarm_low = {
   (void *)&alarmArgActions[1],
   (void *)&alarmArgActions[1],
   anaChAlarmSetAction,
   anaChAlarmClearAction,  
   &alarmParam[1]
};

static alarmType anaCh3_alarm_high = {
   (void *)&alarmArgActions[2],
   (void *)&alarmArgActions[2],
   anaChAlarmSetAction,
   anaChAlarmClearAction,  
   &alarmParam[2]
};

static alarmType anaCh3_alarm_low = {
   (void *)&alarmArgActions[3],
   (void *)&alarmArgActions[3],
   anaChAlarmSetAction,
   anaChAlarmClearAction,  
   &alarmParam[3]
};

static int32_t fd_adc;  /** <= analog input file descriptor*/

int32_t fd_out;         /** <= digital outputs file descriptor*/

static int32_t fdSerialPort; /** <= uart file descriptor*/

/* used to alocate analog values reads */
uint16_t anaChReadValues[ANALOG_CHANNELS_QTY];

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static bool anaChAlarmSetAction(void * relay)
{
  biodigRelays_operate(fd_out, *((uint8_t *)relay), ON);
  return 0; 
}

static bool anaChAlarmClearAction(void * relay)
{
  biodigRelays_operate(fd_out, *((uint8_t *)relay), OFF);
  return 0; 
}

/*==================[external functions definition]==========================*/
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
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n",
               OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(),
               OSErrorGetParam3(), OSErrorGetRet());
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
   
   fdSerialPort = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR | ciaaPOSIX_O_NONBLOCK);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fdSerialPort, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_9600);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fdSerialPort, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* Open Modbus Slave */
   hModbusSlave = ciaaModbus_slaveOpen(
         &callbacksStruct,
         MODBUS_ID);

   /* Open Transport Modbus Ascii */
   hModbusAscii = ciaaModbus_transportOpen(
         fdSerialPort,
         CIAAMODBUS_TRANSPORT_MODE_ASCII_SLAVE);

   /* Open Gateway Modbus */
   hModbusGateway = ciaaModbus_gatewayOpen();

   /* Add Modbus Slave to gateway */
   ciaaModbus_gatewayAddSlave(
         hModbusGateway,
         hModbusSlave);

   /* Add Modbus Transport to gateway */
   ciaaModbus_gatewayAddTransport(
         hModbusGateway,
         hModbusAscii);

   /* end InitTask */
   TerminateTask();
}

TASK(AnalogReads)
{
   uint8_t i;
   #if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)
   for (i = ciaaCHANNEL_0; i < ANALOG_CHANNELS_QTY; i++)
   #elif (edu_ciaa_nxp == BOARD)
   for (i = ciaaCHANNEL_1; i < ANALOG_CHANNELS_QTY; i++)
   #endif
   {
      ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, (void *)i);
      ciaaPOSIX_read(fd_adc, &anaChReadValues[i], sizeof(anaChReadValues[i]));
   }

   TerminateTask();
}

TASK(AlarmsCheck)
{
   alarmCheck(&anaCh2_alarm_high, &anaChReadValues[ciaaCHANNEL_2]);   
   alarmCheck(&anaCh2_alarm_low, &anaChReadValues[ciaaCHANNEL_2]);   
   alarmCheck(&anaCh3_alarm_high, &anaChReadValues[ciaaCHANNEL_3]);   
   alarmCheck(&anaCh3_alarm_low, &anaChReadValues[ciaaCHANNEL_3]);   
   
   ciaaModbus_gatewayMainTask(hModbusGateway);
   /* end of Blinking */
   TerminateTask();
}

/*==================[end of file]============================================*/

