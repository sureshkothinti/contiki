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
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *     A process which communicates over UART 
 *
 *    
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/process.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
#include "com-uart.h"
#include "sys/cc.h"

#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*---------------------------------------------------------------------------*/
#define ADDRESS_CONVERSION_OK       1
#define ADDRESS_CONVERSION_ERROR    0
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(com_uart_process, "Comm UART Process");

/*---------------------------------------------------------------------------*/
static void
release_uart(void)
{
  cc26xx_uart_set_input(NULL);
}
/*---------------------------------------------------------------------------*/
static void
keep_uart_on(void)
{
  cc26xx_uart_set_input(serial_line_input_byte);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(com_uart_process, ev, data)
{
  PROCESS_BEGIN();

  printf("CC26XX Comm UART Process\n");

  set_config_defaults();

  while(1) {

    PROCESS_YIELD();

    if(ev == serial_line_event_message) {
     printf("received line: %s\n", (char *)data);
      
    } 
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
