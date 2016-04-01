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
#include "ciaaModbus.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "bio_dig_modbus_slave.h"
#include "dio_relay.h"
#include "alarms.h"
#include "bio_Cfg.h"
/*==================[macros and definitions]=================================*/
#define MODBUS_ID    2

#define MODBUS_ADDR_ANACH0    0X0000
#define MODBUS_ADDR_ANACH1    0X0002
#define MODBUS_ADDR_ANACH2    0X0004
#define MODBUS_ADDR_ANACH3    0X0006

#define MODBUS_ADDR_ALARMS    0X0010
//#define MODBUS_ADDR_ANACH2_HIGH    0X0008
//#define MODBUS_ADDR_ANACH2_LOW    0X0009
//#define MODBUS_ADDR_ANACH3_HIGH    0X000A
//#define MODBUS_ADDR_ANACH3_LOW    0X000B

#define ANALOG_CHANNELS_QTY   4
#define ALARMS_QTY   4

//#define MSB2LSB(b)   do {  \
//                     b = (((b)&1?128:0)|((b)&2?64:0)|((b)&4?32:0)|((b)&8?16:0)|((b)&16?8:0)|((b)&32?4:0)|((b)&64?2:0)|((b)&128?1:0));             \
//                     }while (0) 
#define NIBBLE_SWAP(b)  do {  \
                        b = ((b<<4) | (b>>4)); \
                        }while (0)
/*==================[internal data declaration]==============================*/


/*==================[internal functions declaration]=========================*/
static uint8_t levelToPhysical (uint8_t ana_channel, const uint16_t *level_value, float *phy_val);
static uint8_t modbus_floatToRegs(const float *var_value, uint16_t * modbus_regs);

static uint16_t cmd0x04ReadInputReg(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       );

//static uint16_t cmd0x03ReadHoldingReg(
//       uint16_t start,
//       uint16_t quantity,
//       uint8_t * exceptioncode,
//       uint8_t * buf
//       );
static uint16_t cmd0x01ReadCoils(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       );

static bool anaChAlarmSetAction(void *);
static bool anaChAlarmClearAction(void *);

/*==================[internal data definition]===============================*/
static int32_t hModbusSlave;
static int32_t hModbusAscii;
static int32_t hModbusGateway;

static const ciaaModbus_slaveCmd_type callbacksStruct =
{
   cmd0x01ReadCoils,
   NULL,
   NULL,//cmd0x03ReadHoldingReg,
   cmd0x04ReadInputReg,
   NULL,
   NULL,
   NULL,
   NULL,
};


static uint8_t alarmArgActions[] = {
   ANA2_HIGH_ALARM_RELAY,
   ANA2_LOW_ALARM_RELAY,
   ANA3_HIGH_ALARM_RELAY,
   ANA3_LOW_ALARM_RELAY
};

static alarmParamType alarmParam[ALARMS_QTY] = {
   {(uint16_t)ANA2_HIGH_LIM_LEVEL, 1},    /* alarm limit and set high alarm  */
   {(uint16_t)ANA2_LOW_LIM_LEVEL, 0},     /* alarm limit and set low alarm  */
   {(uint16_t)ANA3_HIGH_LIM_LEVEL, 1},    /* alarm limit and set high alarm  */
   {(uint16_t)ANA3_LOW_LIM_LEVEL, 0}      /* alarm limit and set low alarm  */
};

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

static int32_t fdSerialPort;

static uint16_t anaChReadValues[ANALOG_CHANNELS_QTY];

/*==================[external data definition]===============================*/
/*==================[internal functions definition]==========================*/
static uint8_t levelToPhysical (uint8_t ana_channel, const uint16_t *level_value, float *phy_val)
{

   switch (ana_channel)
   {
#if (ciaa_nxp == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
      case 0:
         *phy_val = ANA0_FACTOR * (*level_value) + ANA0_OFFSET;
         break;
#endif
      case 1:
         *phy_val = ANA1_FACTOR * (*level_value) + ANA1_OFFSET;
         break;
      case 2:
         *phy_val = ANA2_FACTOR * (*level_value) + ANA2_OFFSET;
         break;
      case 3:
         *phy_val = ANA3_FACTOR * (*level_value) + ANA3_OFFSET;
         break;
//      default:
//         ERROR
   }
   return 0;
}

static uint8_t modbus_floatToRegs(const float *var_value,uint16_t * modbus_regs)
{
   union var_t{
      float flt;
      uint16_t regs[2];
   }var;

   var.flt = *var_value;
   
   *modbus_regs = var.regs[1];
   *(modbus_regs + 1) = var.regs[0];

   return 0;
}
//static uint16_t cmd0x03ReadHoldingReg(
//       uint16_t start,
//       uint16_t quantity,
//       uint8_t * exceptioncode,
//       uint8_t * buf
//       )
//{
//   /* used to indicate quantity of registers processed */
//   int16_t quantityRegProcessed;
///* used to indicate total of registers reads */
//   int8_t ret = 0;
//
//   /* loop to read all registers indicated */
//   do
//   {
//      /* select register address to be read */
//      switch (start)
//      {
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH2_HIGH:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA2_HIGH_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH2_LOW:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA2_LOW_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH3_HIGH:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA3_HIGH_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH3_LOW:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA3_LOW_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* wrong address */
//         default:
//            *exceptioncode = CIAA_MODBUS_E_WRONG_STR_ADDR;
//            quantityRegProcessed = -1;
//            break;
//      }
//
//      /* if quantityRegProcessed > 0, successful operation */
//      if (quantityRegProcessed > 0)
//      {
//         /* update buffer pointer to next register */
//         buf += (quantityRegProcessed*2);
//
//         /* next address to be read */
//         start += quantityRegProcessed;
//
//         /* increment count of registers */
//         ret += quantityRegProcessed;
//      }
//      else
//      {
//         /* an error occurred in reading */
//         ret = -1;
//      }
//      /* repeat until:
//      * - read total registers or
//      * - error occurs
//      */
//   }while ((ret > 0) && (ret < quantity));
//
//   return ret;
//}

static uint16_t cmd0x01ReadCoils(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       )
{
   /* used to indicate quantity of registers processed */
   int16_t quantityRegProcessed;
/* used to indicate total of registers reads */
   int8_t ret = 0;
   
   uint8_t coils_reg = 0;
   ciaaPOSIX_read(fd_out, &coils_reg, 1);
#if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
   NIBBLE_SWAP(coils_reg);
#elif (edu_ciaa_nxp == BOARD)
   coils_reg >>= 2;
#endif
   //MSB2LSB(coils_reg);
   /* loop to read all registers indicated */
  // do
  // {
      /* select register address to be read */
      switch (start)
      {
         /* read inputs of CIAA */
         case MODBUS_ADDR_ALARMS:
            switch (quantity)
            {
               case 1:
                  buf[0] = coils_reg & 1;
                  break;
               case 2:
                  buf[0] = coils_reg & 3;
                  break;
               case 3:
                  buf[0] = coils_reg & 7;
                  break;
               case 4:   
                  buf[0] = coils_reg & 15;
                  break;
               //case 8:   
               default:
                  buf[0] = coils_reg;
                  break;
            }
            //ciaaModbus_writeInt(buf, coils_reg);
//            buf[0] = coils_reg;
            quantityRegProcessed = 1;
            break;

//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH2_LOW:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA2_LOW_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH3_HIGH:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA3_HIGH_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;
//
//         /* read inputs of CIAA */
//         case MODBUS_ADDR_ANACH3_LOW:
//            ciaaModbus_writeInt(buf, ciaaDIO_relay_st(fd_out, ANA3_LOW_ALARM_RELAY));
//            quantityRegProcessed = 1;
//            break;

         /* wrong address */
         default:
            *exceptioncode = CIAA_MODBUS_E_WRONG_STR_ADDR;
            quantityRegProcessed = -1;
            break;
      }

      /* if quantityRegProcessed > 0, successful operation */
      if (quantityRegProcessed > 0)
      {
         /* update buffer pointer to next register */
         buf += (quantityRegProcessed*2);

         /* next address to be read */
         start += quantityRegProcessed;

         /* increment count of registers */
         ret += quantityRegProcessed;
      }
      else
      {
         /* an error occurred in reading */
         ret = -1;
      }
      /* repeat until:
      * - read total registers or
      * - error occurs
      */
  // }while ((ret > 0) && (ret < quantity));

   ret = quantity;//ret * 16;
   return ret;
}

static uint16_t cmd0x04ReadInputReg(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       )
{
   /* used to indicate quantity of registers processed */
   int16_t quantityRegProcessed;
/* used to indicate total of registers reads */
   int8_t ret = 0;
   uint16_t regs[2];

   //float test = 1;
   float phy_value;
   /* loop to read all registers indicated */
   do
   {
      /* select register address to be read */
      switch (start)
      {
#if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)             /* edu_ciaa_nxp do not have anai0 in use */
         /* read inputs of CIAA */
         case MODBUS_ADDR_ANACH0:
            levelToPhysical(0, &anaChReadValues[0], &phy_value);
            modbus_floatToRegs(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;
#endif
         case MODBUS_ADDR_ANACH1:
            levelToPhysical(1, &anaChReadValues[1], &phy_value);
            modbus_floatToRegs(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;

         case MODBUS_ADDR_ANACH2:
            levelToPhysical(2, &anaChReadValues[2], &phy_value);
            modbus_floatToRegs(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;

         case MODBUS_ADDR_ANACH3:
            levelToPhysical(3, &anaChReadValues[3], &phy_value);
            //modbus_floatToRegs(&test, regs);
            modbus_floatToRegs(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;

         /* wrong address */
         default:
            *exceptioncode = CIAA_MODBUS_E_WRONG_STR_ADDR;
            quantityRegProcessed = -1;
            break;
      }

      /* if quantityRegProcessed > 0, successful operation */
      if (quantityRegProcessed > 0)
      {
         /* update buffer pointer to next register */
         buf += (quantityRegProcessed*2);

         /* next address to be read */
         start += quantityRegProcessed;

         /* increment count of registers */
         ret += quantityRegProcessed;
      }
      else
      {
         /* an error occurred in reading */
         ret = -1;
      }
      /* repeat until:
      * - read total registers or
      * - error occurs
      */
   }while ((ret > 0) && (ret < quantity));

   return ret;
}

static bool anaChAlarmSetAction(void * relay)
{
  ciaaDIO_relay_op(fd_out, *((uint8_t *)relay), ON);
  return 0; 
}

static bool anaChAlarmClearAction(void * relay)
{
  ciaaDIO_relay_op(fd_out, *((uint8_t *)relay), OFF);
  return 0; 
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
//   uint8_t test = 15;
//   NIBBLE_SWAP(test);
//   ciaaPOSIX_printf("InitTask: %u\n", test);
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

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

