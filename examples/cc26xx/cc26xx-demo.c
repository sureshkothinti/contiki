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

struct rtc_time {
  uint8_t tm_sec;
  uint8_t tm_min;
  uint8_t tm_hour;
  uint8_t tm_day;
  uint8_t tm_mon;
  uint8_t year[2];
  uint16_t tm_year;
 
};
/*---------------------------------------------------------------------------*/
static struct etimer et;

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/

#define ENCRYPT 1
#define DECRYPT 0
//time_t time;
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
    strcpy(data+12,"UNK 241220169000");
    strcpy(data+28,"B374A26A71490437AA024E4FADD5B497FDFF1A8EA6FF12F6FB65AF2720B59CCF");
    strcpy(data+92,"26122016204100");
    FlashProgram((uint8_t *)data,addr,128);
    CPUdelay(10000);
  } 
}

unsigned char Ascii_To_Hex(unsigned char *str)
{

	unsigned char res;
	res = (((str[0] - 0x30) << 4) + (str[1] - 0x30));
 // printf("%c,%c,%x",str[0],str[1],res);
	return res;
}
void Hex_To_Ascii(uint16_t byte,uint8_t *buff){

	
	buff[0]= (byte/10)+0x30;
	buff[1]= (byte%10)+0x30;
	return;
}
int16_t month(uint8_t a,uint8_t yy);
uint8_t mon[12]={31,28,31,30,31,30,31,31,30,31,30,31};
int16_t days(uint8_t y1,uint8_t y2,uint8_t m1,uint8_t m2,uint8_t d1,uint8_t d2)
{
  int16_t count=0,i;
  for(i = y1; i < y2;i++)
  {
    if(i%4==0)
    {
      count+=366;
    }else{
      count+=365;
    }
  }
    count-=month(m1,y1);
    count-=d1;
    count+=month(m2,y2);
    count+=d2;
  /*if(count<0)
  count=count*-1;*/
  //printf("The no. of days b/w the 2 dates = %d days",count);
  return count;
}

int16_t month(uint8_t a,uint8_t yy)
{
  int16_t x=0,c;
  for(c = 0;c < (a-1);c++)
  {
    if(c==1)
    {
      if(yy%4 == 0)
      {
        x+=29;
      }
      else{
        x+=28;
      }
    }else{
      x+=mon[c];
    }
  }
  return(x);
}
uint8_t tempData[96];
uint8_t tempData2[192];
uint8_t tempwd[10];
uint8_t pwdRead[10];
uint8_t temp[96];
uint8_t rdata[128];
char edata[270]; //130
 uint8_t time[12];
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
 
 uint8_t  loop = 0;
 struct rtc_time ctime,ltime;
 uint8_t  i =0;
 uint32_t res;
 int16_t ret = 0;
 static uint8_t  pflag = 0;
 size_t decodeLen= 264;
 uint32_t configPageAddress = (unsigned int )0x1E000;
 //uint8_t buff[4];

 uint8_t date,month,year,hour,minutes,seconds;
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
      rtc_get_DateTime(time,8);
      printf("%02x/%02x/20%02x %02x:%02x:%02x\n",time[0],time[1],time[3],time[4],time[5],time[6]);
      etimer_set(&et, CLOCK_SECOND);
    }

    if(ev == serial_line_event_message) {


       //#if 0
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
                    case 'T':
                      memset(temp,0,96);
                      memset(edata,0,130);
                      if(pflag){
                       // i = 92;
                        date = Ascii_To_Hex(&tempData[4]);
                        rtc_set_date(date,1);
                        month = Ascii_To_Hex(&tempData[6]);
                        rtc_set_month(month,1);
                        year = Ascii_To_Hex(&tempData[10]);
                        rtc_set_year(year,1);
                        hour = Ascii_To_Hex(&tempData[12]);
                        rtc_set_hours(hour,1);
                        minutes = Ascii_To_Hex(&tempData[14]);
                        rtc_set_minutes(minutes,1);
                        seconds = Ascii_To_Hex(&tempData[16]);
                        rtc_set_seconds(seconds,1);
                        /*for(loop = 4; tempData[loop] != '>'; loop++)
                        {
                        
                          rdata[i++] = tempData[loop];
                        }*/
                        res = FlashProgram(rdata, configPageAddress,128);
                        CPUdelay(1000);
                        if( res== 0)
                        {
                          sprintf((char*)temp,"%s","<ST,0>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                        }else{
                          sprintf((char*)temp,"%s","<ST,2>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                        } 
                      }else{
                          sprintf((char*)temp,"%s","<ST,1>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                      }                    
                    break;
                    case 'L':
                      memset(temp,0,96);
                      memset(edata,0,130);
                      if(pflag){
                        i = 92;
                        for(loop = 4; tempData[loop] != '>'; loop++)
                        {
                        
                          rdata[i++] = tempData[loop];
                        }
                        res = FlashProgram(rdata, configPageAddress,128);
                        CPUdelay(1000);
                        if( res== 0)
                        {
                          sprintf((char*)temp,"%s","<SL,0>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                        }else{
                          sprintf((char*)temp,"%s","<SL,2>");
                          //cipher(ENCRYPT,key,temp);
                          CreateShares(temp,96,tempData2,(tempData2+96));
                          base64_encode(edata,262,tempData2,192);
                          printf("%s",edata);
                          printf("\r\n");
                        } 
                      }else{
                          sprintf((char*)temp,"%s","<SL,1>");
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
                case 'L':
                      rdata[0] = '<';
                      rdata[1] = 'G';
                      rdata[2] = 'L';
                      rdata[3] = ',';
                      FlashRead((rdata+4),(configPageAddress+0x5C),14);
                      rdata[18] = '>';
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
               switch(tempData[2])
                {
                  i = 0;
                  memset(pwdRead,0x00,14);
                  memset(tempwd,0x00,14);
                  memset(temp,0,96);
                  case 'P':
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
                  break;
                  case 'L':
                    FlashRead(pwdRead,(configPageAddress+0x5C),14);
                    ltime.tm_day = Ascii_To_Hex(&pwdRead[0]);
                    ltime.tm_mon = Ascii_To_Hex(&pwdRead[2]);
                    ltime.tm_year = Ascii_To_Hex(&pwdRead[6]);
                    ltime.tm_hour = Ascii_To_Hex(&pwdRead[8]);
                    ltime.tm_min = Ascii_To_Hex(&pwdRead[10]);
                    ltime.tm_sec = Ascii_To_Hex(&pwdRead[12]);
                    rtc_get_DateTime(time,8);
                    ctime.tm_day = time[0];
                    ctime.tm_mon = time[1];
                    ctime.tm_year = time[3];
                    ctime.tm_hour = time[4];
                    ctime.tm_min = time[5];
                    ctime.tm_sec = time[6];
                    if(ctime.tm_year >= ltime.tm_year)
                    {
                      ret = days(ctime.tm_year,ltime.tm_year,ctime.tm_mon,ltime.tm_mon,ctime.tm_day,ltime.tm_day);
                    }else{
                      ret = days(ltime.tm_year,ctime.tm_year,ltime.tm_mon,ctime.tm_mon,ltime.tm_day,ctime.tm_day);
                    }
                    if(ret >= 0)
                    {
                     
                      sprintf((char*)temp,"%s","<VL,0>");
                      CreateShares(temp,96,tempData2,(tempData2+96));
                      //cipher(ENCRYPT,key,temp);
                      base64_encode(edata,262,tempData2,192);
                      printf("%s",edata);
                      printf("\r\n");
                    }
                    else{
                      
                      sprintf((char*)temp,"%s","<VL,1>");
                      //cipher(ENCRYPT,key,temp);
                      CreateShares(temp,96,tempData2,(tempData2+96));
                      base64_encode(edata,262,tempData2,192);
                      printf("%s",edata);
                      printf("\r\n");
                    }
                  break;
                }
              break;
              default:
              break;
          }
        }else{

        }
 //       #endif
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
