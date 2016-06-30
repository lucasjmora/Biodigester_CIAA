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

/*==================[inclusions]=============================================*/
#include "os.h"
#include "biodig_Cfg.h"
#include "ciaaModbus.h"

/*==================[macros and definitions]=================================*/
/** \brief analog channels register address */
#define MODBUS_ADDR_ANACH0    0X0000
#define MODBUS_ADDR_ANACH1    0X0002
#define MODBUS_ADDR_ANACH2    0X0004
#define MODBUS_ADDR_ANACH3    0X0006

/** \brief relays register address */
#define MODBUS_ADDR_ALARMS    0X0010

/** \brief swap between MSNibble and LSNibble*/
#define NIBBLE_SWAP(b)  do {                    \
                        b = ((b<<4) | (b>>4));  \
                        }while (0)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** \brief scaled level analog read value to physical magnitud value 
 **
 ** \param[in]    ana_channel analog channel of value to scaled
 ** \param[in]    level_value pointer to value to scaled
 ** \param[out]   phy_val pointer to scaled value
 ** \return 1 if success -1 if an error occurs
 **/
static int8_t levelToPhysical(uint8_t ana_channel,
                              const uint16_t *level_value,
                              float *phy_val);

/** \brief divide a 32bit float in two words(each of 16bits)
 **
 ** \param[in]    var_value pointer to float source value 
 ** \param[out]   modbus_reg pointer to result array of two uint16 
 ** \return 1 if success -1 if an error occurs
 **/
static int8_t floatToWords(const float *var_value,
                                 uint16_t * modbus_regs);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/
extern int32_t fd_out;
extern uint16_t anaChReadValues[];

/*==================[internal functions definition]==========================*/
static int8_t levelToPhysical(uint8_t ana_channel,
                              const uint16_t *level_value,
                              float *phy_val
                              )
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
   return 1;
}

static int8_t floatToWords(const float * var_value,
                                 uint16_t * modbus_regs
                                 )
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

/*==================[external functions definition]==========================*/
uint16_t cmd0x01ReadCoils(
       uint16_t start,
       uint16_t quantity,
       uint8_t * exceptioncode,
       uint8_t * buf
       )
{
   /* used to indicate total of coils reads */
   int8_t ret = 0;
   /* used as LSB of relays coil register 0x0010 */
   uint8_t coils_reg = 0;

   ciaaPOSIX_read(fd_out, &coils_reg, 1);

   #if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)  /* edu_ciaa_nxp do not have
                                                   anai0 in use */
   NIBBLE_SWAP(coils_reg); /* put relay bits in LSNibble */
   #elif (edu_ciaa_nxp == BOARD)
   coils_reg >>= 2;        /* put led bits in LSNibble  */
   #endif
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
               default:
                  buf[0] = coils_reg;
                  break;
            }
            ret = quantity;
            break;

         /* wrong address */
         default:
            *exceptioncode = CIAA_MODBUS_E_WRONG_STR_ADDR;
            ret = -1;
            break;
      }

   return ret;
}

uint16_t cmd0x04ReadInputReg(
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
   float phy_value;
   /* loop to read all registers indicated */
   do
   {
      /* select register address to be read */
      switch (start)
      {
         #if (ciaa_nxp == BOARD || ciaa_sim_ia64 == BOARD)/* edu_ciaa_nxp do
                                                            not have anai0 in
                                                            use */
         case MODBUS_ADDR_ANACH0:
            levelToPhysical(0, &anaChReadValues[0], &phy_value);
          //  modbus_floatToRegs(&phy_value, regs);
            floatToWords(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;
         #endif
         case MODBUS_ADDR_ANACH1:
            levelToPhysical(1, &anaChReadValues[1], &phy_value);
            floatToWords(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;

         case MODBUS_ADDR_ANACH2:
            levelToPhysical(2, &anaChReadValues[2], &phy_value);
            floatToWords(&phy_value, regs);
            ciaaModbus_writeInt(buf, regs[0]);
            ciaaModbus_writeInt(buf+2, regs[1]);
            quantityRegProcessed = 2;
            break;

         case MODBUS_ADDR_ANACH3:
            levelToPhysical(3, &anaChReadValues[3], &phy_value);
            floatToWords(&phy_value, regs);
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

/*==================[end of file]============================================*/
