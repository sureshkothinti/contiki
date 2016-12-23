/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *   - The CC2650 LaunchPad
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"
#include "driverlib/flash.h"
#include "driverlib/cpu.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
#include "VCBinary.h"
#include "aes256.h"
#include "base64.h"
#include "board-i2c.h"
#include "board-spi.h"
#include "ext-flash.h"
#include "ti-lib.h"
#include "rtc.h"
#include <stdio.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/

#define ENCRYPT 1
#define DECRYPT 0

void FlashRead(uint8_t *pui8DataBuffer,uint32_t ui32Address,uint32_t ui32Count)
{
  uint8_t *pui8ReadAdress =(uint8_t *)ui32Address;
  while(ui32Count --)
  {
    *pui8DataBuffer++ = *pui8ReadAdress++;
  }
}

void WriteConfigData(uint32_t addr)
{
  uint8_t buf[4];
  unsigned int res;
  char data[128];
  FlashRead(buf,0x1E000,4);
  strcpy(data,"AA55");
  res = memcmp(buf,data,4);
  if(res != 0 ) {
    strcpy(data+4,"11223344");
    strcpy(data+12,"UNK 180820168872");
    strcpy(data+28,"B374A26A71490437AA024E4FADD5B497FDFF1A8EA6FF12F6FB65AF2720B59CCF");
    FlashProgram((uint8_t *)data,addr,128);
    CPUdelay(10000);
  } 
}
unsigned char key[32] = { 0x3e, 0xc8, 0x7b, 0x95, 0x59, 0xc1, 0x21, 0x1f, 
                          0x6c, 0xa6, 0x55, 0x0a, 0x99, 0xf3, 0xc9, 0xfd, 
                          0x64, 0xb3, 0x3f, 0xa5, 0x57, 0x2a, 0x0f, 0x61, 
                          0xf4, 0x66, 0x67, 0xc0, 0xbb, 0x6d, 0x98, 0x85 };

int cipher(uint8_t type, uint8_t *key, uint8_t *buff)
{
	int i;
	aes256_context ctx;
	aes256_init(&ctx, key);
	if (type == ENCRYPT)
	{
		for (i = 0; i < 96/ 16; i++)
		{

			aes256_encrypt_ecb(&ctx, buff  + i * 16);
		}
	}
	else {
		for (i = 0; i < 96 / 16; i++)
		{
			aes256_decrypt_ecb(&ctx, buff + i * 16);
		}
	}
	aes256_done(&ctx);
	return 0;
}
uint8_t tempData[96];
uint8_t tempData2[192];
uint8_t tempwd[10];
uint8_t pwdRead[10];
uint8_t temp[96];
uint8_t rdata[128];
char edata[270]; //130
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
 
 uint8_t  loop = 0;

 uint8_t  i =0;
 uint32_t res;
 static uint8_t  pflag = 0;
 size_t decodeLen= 264;
 uint32_t configPageAddress = (unsigned int )0x1E000;
 uint8_t buff[4];
 uint8_t time[3];
 uint8_t date,month,year;
  PROCESS_BEGIN();
  
  WriteConfigData(configPageAddress);

  cc26xx_uart_set_input(serial_line_input_byte);
  memset(tempData,0,96);
  memset(tempData2,0,192);
   memset(edata,0,270);
   etimer_set(&et, CLOCK_SECOND);
  while(1) {

    PROCESS_YIELD();
    if(ev ==PROCESS_EVENT_TIMER )
    {
      rtc_get_date(&date,1);
      rtc_get_month(&month,1);
      rtc_get_year(&year,1);
      printf("%x/%x/%x ",date,month,year);
      rtc_get_time(time,3);
       printf("%x:%x:%x\n",time[2],time[1],time[0]);
       etimer_set(&et, CLOCK_SECOND);
    }

    if(ev == serial_line_event_message) {
      /*rtc_read_ids(buff,6);
      printf("ID %x %x %x %x %x %x",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);*/ 
      
      rtc_set_date(0x23,1);
      rtc_set_month(0x12,1);
      rtc_set_year(0x16,1);
      rtc_set_seconds(0x00,1);
      rtc_set_minutes(0x05,1);
      rtc_set_hours(0x18,1);
      rtc_read_status_reg(buff,1);
      printf("%x\n",buff[0]);
             /*printf("success");
       res = ext_flash_open();
       if(res)
       {
         printf("part ok");
       }else{
         printf("part fail");
       }
       res = ext_flash_write(0x00, 4, "test");
       if(res)
       {
         printf("flash write successful\n");
         res = ext_flash_read(0x00,4,buff);
         printf("%s\n",buff);
       }*/
        #if 0
        strcpy(edata,data);
        base64_decode(tempData2, &decodeLen,edata,strlen(edata));
        JoinShares(tempData,96,tempData2,(tempData2+96));
        
        if(tempData[0] == '<')
        {
          switch(tempData[1])
          {
              case 'S':
                memset(rdata,0xFF,128);
                FlashRead(rdata,configPageAddress,128);
                res = FlashSectorErase(configPageAddress);
                if(res == 0)
                {
                  switch(tempData[2])
                  {
                    case 'K':
                    memset(temp,0,96);
                    memset(edata,0,130);
                    if(pflag){
                      i = 28;
                      for(loop = 4; tempData[loop] != '>'; loop++)
                      {
                        
                        rdata[i++] = tempData[loop];
                      }
                      res = FlashProgram(rdata, configPageAddress,128);
                      CPUdelay(1000);
                      if( res== 0)
                      {
                        sprintf((char*)temp,"%s","<SK,0>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                      }else{
                        sprintf((char*)temp,"%s","<SK,2>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                      } 
                    }else{
                        sprintf((char*)temp,"%s","<SK,1>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                    }                  
                    break;
                    case 'U':
                      memset(temp,0,96);
                      memset(edata,0,130);
                      i = 12;
                      for(loop = 4;loop < 21;loop++)
                      {
                        
                        rdata[i++] = tempData[loop];
                      }
                      res = FlashProgram(rdata, configPageAddress,128);
                      CPUdelay(1000);
                      if( res == 0)
                      {
                        sprintf((char*)temp,"%s","<SU,0>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                      }else{
                        sprintf((char*)temp,"%s","<SU,1>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                      }                    
                    break;
                    case 'P':
                      memset(temp,0,96);
                      memset(edata,0,130);
                      memset(tempwd,0x00,10);
                      memset(pwdRead,0x00,10);
                      for(loop = 4;tempData[loop] != ':';loop++)
                      {
                        tempwd[loop -4] = tempData[loop];
                      }
                      FlashRead(pwdRead,(configPageAddress+0x04),8);
                      res = memcmp(tempwd,pwdRead,8);
                      if(res == 0)
                      {
                        i = 4;
                        for(loop = 13;loop < 21;loop++)
                        {
                          rdata[i++] = tempData[loop];
                        } 
                        res =  FlashProgram(rdata,configPageAddress,128);
                        if( res== 0)
                        {
                            sprintf((char*)temp,"%s","<SP,0>");
                            //cipher(ENCRYPT,key,temp);
                            CreateShares(temp,96,tempData2,(tempData2+96));
                            base64_encode(edata,262,tempData2,192);
                            printf("%s",edata);
                            printf("\r\n");
                        }else{
                          FlashProgram(rdata,configPageAddress,128);
                          sprintf((char*)temp,"%s","<SP,1>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                        }
                      }else{
                        FlashProgram(rdata,configPageAddress,128);
                        sprintf((char*)temp,"%s","<SP,1>");
                        //cipher(ENCRYPT,key,temp);
                        CreateShares(temp,96,tempData2,(tempData2+96));
                        base64_encode(edata,262,tempData2,192);
                        printf("%s",edata);
                        printf("\r\n");
                      }    
                                  
                    break;
                    default:
                    //cc26xx_uart_set_input()
                    break;
                  }
                 
                }else{
                  FlashProgram(rdata, configPageAddress,128);
                }
              break;
              case 'G':
                switch(tempData[2])
                {
                  memset(rdata,0,128);
                  memset(edata,0,130);
                  case 'K':
                      rdata[0] = '<';
                      rdata[1] = 'G';
                      rdata[2] = 'K';
                      rdata[3] = ',';
                      FlashRead((rdata+4),(configPageAddress+0x1C),64);
                      rdata[69] = '>';
                      //cipher(ENCRYPT,key,rdata);
                      //base64_encode(edata,130,rdata,96);
                      CreateShares(rdata,96,tempData2,(tempData2+96));
                      base64_encode(edata,262,tempData2,192);
                      printf("%s",edata);
                      printf("\r\n");
                  break;
                  case 'U':
                      rdata[0] = '<';
                      rdata[1] = 'G';
                      rdata[2] = 'U';
                      rdata[3] = ',';
                      FlashRead((rdata+4),(configPageAddress+0x0C),16);
                      rdata[20] = '>';
                      //cipher(ENCRYPT,key,rdata);
                     // base64_encode(edata,130,rdata,96);
                      CreateShares(rdata,96,tempData2,(tempData2+96));
                      base64_encode(edata,262,tempData2,192);
                      printf("%s",edata);
                      printf("\r\n");                      
                break;
                case 'P':
                      rdata[0] = '<';
                      rdata[1] = 'G';
                      rdata[2] = 'P';
                      rdata[3] = ',';
                      FlashRead((rdata+4),(configPageAddress+0x04),8);
                      rdata[12] = '>';
                      //cipher(ENCRYPT,key,rdata);
                      //base64_encode(edata,130,rdata,96);
                      CreateShares(rdata,96,tempData2,(tempData2+96));
                      base64_encode(edata,262,tempData2,192);
                      printf("%s",edata);
                      printf("\r\n");
                break;
                default:
                break;
                }
              break;
              case 'V':

               if(tempData[2] == 'P')
                {
                  i = 0;
                  memset(pwdRead,0x00,10);
                  memset(tempwd,0x00,10);
                  memset(temp,0,96);
                  for(loop = 4;loop < 12;loop++)
                  {
                    tempwd[loop-4] = tempData[loop];
                  }
                  FlashRead(pwdRead,(configPageAddress+0x04),8);
                  if(memcmp(tempwd,pwdRead,8) == 0)
                  {
                    pflag = 1;
                    sprintf((char*)temp,"%s","<VP,0>");
                    CreateShares(temp,96,tempData2,(tempData2+96));
                    //cipher(ENCRYPT,key,temp);
                    base64_encode(edata,262,tempData2,192);
                    printf("%s",edata);
                    printf("\r\n");
                  }
                  else{
                    pflag =0 ;
                    sprintf((char*)temp,"%s","<VP,1>");
                    //cipher(ENCRYPT,key,temp);
                    CreateShares(temp,96,tempData2,(tempData2+96));
                    base64_encode(edata,262,tempData2,192);
                    printf("%s",edata);
                    printf("\r\n");
                  }
                }else{

                }
              break;
              default:
              break;
          }
        }else{

        }
        #endif
    }
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
