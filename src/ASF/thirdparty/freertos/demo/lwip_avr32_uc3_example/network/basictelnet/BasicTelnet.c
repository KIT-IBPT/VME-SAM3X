/*****************************************************************************
 *
 * \file
 *
 * \brief Basic TELNET Server for AVR32 UC3.
 *
 * Copyright (c) 2009-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *****************************************************************************/


/*
  Implements a simplistic TELNET server.  Every time a connection is made and
  data is received a dynamic page that shows the current FreeRTOS.org kernel
  statistics is generated and returned.  The connection is then closed.

  This file was adapted from a FreeRTOS lwIP slip demo supplied by a third
  party.
*/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#if (TELNET_USED == 1)


/* Standard includes. */
#include <stdio.h>
#include <string.h>

#include "conf_eth.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "partest.h"

/* Demo includes. */
/* Demo app includes. */
#include "portmacro.h"

/* lwIP includes. */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/stats.h"
#include "lwip/init.h"
#if ( (LWIP_VERSION) == ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)) )
#include "netif/loopif.h"
#endif

/* ethernet includes */
#include "ethernet.h"

#include "evm300.h"

#define LINEBUF_LENGTH 80

/*! The size of the buffer in which the dynamic TELNET page is created. */
#define telnetMAX_PAGE_SIZE	512

/*! The port on which we listen. */
#define TELNET_PORT		( 23 )

/*! Delay on close error. */
#define telnetSHORT_DELAY		( 10 )

portCHAR cTelnetPage[ telnetMAX_PAGE_SIZE ];
portCHAR cTelnetConnections[ 11 ];
portCHAR cStrLen[ 40 ];

/*! Function to process the current connection */
static void prvtelnet_Connection( struct netconn *pxNetCon );


/*! \brief TELNET server main task
 *         check for incoming connection and process it
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
portTASK_FUNCTION( vBasicTelnetServer, pvParameters );
portTASK_FUNCTION( vBasicTelnetServer, pvParameters )
{
struct netconn *pxTELNETListener, *pxNewConnection;

	/* Create a new tcp connection handle */
	pxTELNETListener = netconn_new( NETCONN_TCP );
	netconn_bind(pxTELNETListener, NULL, TELNET_PORT );
	netconn_listen( pxTELNETListener );

	/* Loop forever */
	for( ;; )
	{
#if ( (LWIP_VERSION) == ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)) )
		/* Wait for a first connection. */
	  printf("pxNewConnection = netconn_accept(pxTELNETListener);\r\n");
		pxNewConnection = netconn_accept(pxTELNETListener);
#else
		printf("netconn_accept(pxTELNETListener, &pxNewConnection);\r\n");
        while(netconn_accept(pxTELNETListener, &pxNewConnection) != ERR_OK)
        {
            vTaskDelay( telnetSHORT_DELAY );
		}
#endif

		if(pxNewConnection != NULL)
		{
		  printf("prvtelnet_Connection(pxNewConnection);\r\n");
			prvtelnet_Connection(pxNewConnection);
		}/* end if new connection */


	} /* end infinite loop */
}


int linebuf_set_u32(char *linebuf, int pos, unsigned long *data)
{
  int    st = -1;
  char   ch;
  unsigned long   data_u32 = 0L;

  for (; linebuf[pos] == ' '; pos++);

  for (; (linebuf[pos] >= '0' && linebuf[pos] <= '9') ||
	 (linebuf[pos] >= 'a' && linebuf[pos] <= 'f') ||
	 (linebuf[pos] >= 'A' && linebuf[pos] <= 'F'); pos++)
    {
      st = 0;
      ch = linebuf[pos] - '0';
      if (ch > 9)
	ch -= 'A' - '9' - 1;
      if (ch > 15)
	ch -= 'a' - 'A';
      data_u32 = (data_u32 << 4) + ch;
    }

  *data = data_u32;
  
  if (!st)
    return pos;
  return st;
}

static void prvtelnet_Connection( struct netconn *pxNetCon )
{
  struct netbuf *pxRxBuffer;
  portCHAR *pcRxString;
  unsigned portSHORT usLength;
  static unsigned portLONG ulPageHits = 0;
  int telnet_exit = 0;
  char linebuf[LINEBUF_LENGTH];
  int linebufpos = 0, i;
  long lptr;
  char ch;
  
  do
    {
      strcpy( cTelnetPage, "> ");
      netconn_write( pxNetCon, cTelnetPage, (u16_t) strlen( cTelnetPage ), NETCONN_COPY );
#if ( (LWIP_VERSION) == ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)) )
	/* We expect to immediately get data. */
	pxRxBuffer = netconn_recv( pxNetCon );
#else
    err_t uc_rc = netconn_recv( pxNetCon, &pxRxBuffer );
    while( uc_rc != ERR_OK && uc_rc != ERR_CLSD && uc_rc != ERR_MEM )
    {
		vTaskDelay( telnetSHORT_DELAY );
		uc_rc = netconn_recv( pxNetCon, &pxRxBuffer );
	}
	if ( uc_rc == ERR_CLSD || uc_rc == ERR_MEM )
	{
		break;
	}
#endif
	if( pxRxBuffer != NULL )
	{
		/* Where is the data? */
		netbuf_data( pxRxBuffer, ( void * ) &pcRxString, &usLength );
		
		if(( NULL != pcRxString               ))
		{
		  for (i = 0; i < usLength; i++)
		    {
		      ch = pcRxString[i];
		      if (ch == '\n')
			ch = 0;
		      if (linebufpos < LINEBUF_LENGTH)
			linebuf[linebufpos++] = ch;
		      if (!ch)
			{
			  cTelnetPage[0] = 0;
			  linebufpos = 0;
			  switch(linebuf[0]) {
			  case 'd':
			    {
			      unsigned short test;
			      int st, n;
			      long addr;

			      st = linebuf_set_u32(linebuf, 1, &addr);
			      if (st > 0)
				lptr = addr;
			      sprintf(cStrLen, "Addr %08lx: ", lptr);
			      strcpy( cTelnetPage, cStrLen);
			      for (n = 0; n < 16; n+=2)
				{
				  if (fpga_read_short(lptr, &test)) {
				    sprintf(cStrLen, " failed");
				  } else {
				    sprintf(cStrLen, " %04x", test);
				  }
				  strcat( cTelnetPage, cStrLen);
				  lptr += 2;
				}
			      strcat( cTelnetPage, "\r\n");
			    break;
			    }
			  case 'h':
			  case 'H':
			  case '?':
			    {
			    strcpy( cTelnetPage, "Help\r\n====\r\n");
			    strcat( cTelnetPage, "d <address>                " );
			    strcat( cTelnetPage, "Dump registers\r\n");
			    strcat( cTelnetPage, "m <address>                " );
			    strcat( cTelnetPage, "Read register (short)\r\n");
			    strcat( cTelnetPage, "m <address> <short data>   " );
			    strcat( cTelnetPage, "Write register (short)\r\n");
			    strcat( cTelnetPage, "x                          Exit\r\n\r\n");
			    strcat( cTelnetPage, "Address and data in hexadecimal format\r\n");
			    strcat( cTelnetPage, "Base address for EVR/EVM function is 80000000.\r\n");
			    break;
			    }
			  case 'm':
			  case 'M':
			    {
			      unsigned short test;
			      int st, n;
			      long addr;
			      unsigned long data32;

			      st = linebuf_set_u32(linebuf, 1, &addr);
			      if (st > 0)
				{
				  sprintf(cStrLen, "Addr %08lx: ", addr);
				  strcpy( cTelnetPage, cStrLen);
				  sprintf(cStrLen, "st %d ", st);
				  strcat( cTelnetPage, cStrLen);
				  st = linebuf_set_u32(linebuf, st, &data32);
				  if (st > 0)
				    {
				      sprintf(cStrLen, " write %04lx, read", data32);
				      strcat( cTelnetPage, cStrLen);
				      netconn_write( pxNetCon, cTelnetPage, (u16_t) strlen( cTelnetPage ), NETCONN_COPY );
				      cTelnetPage[0] = 0;
				      fpga_write_short(addr, (uint16_t) data32 & 0xffff);
				    }
				  if (fpga_read_short(addr, &test)) {
				    sprintf(cStrLen, " failed\r\n");
				  } else {
				    sprintf(cStrLen, " %04x\r\n", test);
				  }
				  strcat( cTelnetPage, cStrLen);
				}
			    break;
			    }
			  case 'g':
			  case 'G':
			    {
			      int iteration, iterationWithMismatch = -2, pos = 1;
			      uint32_t addr, data, expected, mode, status;

			      pos = linebuf_set_u32(linebuf, pos, &mode);
			      if (pos <= 0) {
				strcat(cTelnetPage, "Invalid syntax. Use g <mode> <addr> <data>.\r\n");
				break;
			      }
			      pos = linebuf_set_u32(linebuf, pos, &addr);
			      if (pos <= 0) {
				strcat(cTelnetPage, "Invalid syntax. Use g <mode> <addr> <data>.\r\n");
				break;
			      }
			      pos = linebuf_set_u32(linebuf, pos, &expected);
			      if (pos <= 0) {
				strcat(cTelnetPage, "Invalid syntax. Use g <mode> <addr> <data>.\r\n");
				break;
			      }
			      if (mode > 2) {
				strcat(cTelnetPage, "Invalid mode. Must be 0, 1, or 2.\r\n");
				break;
			      }
			      for (iteration = 0; iteration < 200000; ++iteration) {
				status = fpga_read_long(addr, &data);
				switch (mode) {
				case 0:
				  if (status) {
				    char buffer[128];
				    sprintf(buffer, "Read operation in iteration %d from address %08lx failed with status %lu.\r\n", iteration, addr, status);
				    strcat(cTelnetPage, buffer);
				    goto readError;
				  }
				  break;
				case 1:
				  if (data != expected) {
				    char buffer[128];
				    sprintf(buffer, "Read %08lx in iteration %d from address %08lx where %08lx was expected.\r\n", data, iteration, addr, expected);
				    strcat(cTelnetPage, buffer);
				    goto readError;
				  }
				  break;
				case 2:
				  if (status) {
				    char buffer[128];
				    sprintf(buffer, "Read operation in iteration %d from address %08lx failed with status %lu.\r\n", iteration, addr, status);
				    strcat(cTelnetPage, buffer);
				    if (iterationWithMismatch == (iteration - 1)) {
				      goto readError;
				    }
				  }
				  if (data != expected) {
				    char buffer[128];
				    sprintf(buffer, "Read %08lx in iteration %d from address %08lx where %08lx was expected.\r\n", data, iteration, addr, expected);
				    strcat(cTelnetPage, buffer);
				    netconn_write(pxNetCon, cTelnetPage, (u16_t) strlen(cTelnetPage), NETCONN_COPY);
				    cTelnetPage[0] = 0;
				    iterationWithMismatch = iteration;
				  }
				  break;
				}
			      }
			      strcat(cTelnetPage, "No error detected in 200000 iterations.\r\n");
			      break;
			      readError:
			      status = fpga_write_long(addr, expected);
			      if (status) {
				strcat(cTelnetPage, "Subsequent write operation failed.\r\n");
			      } else {
				strcat(cTelnetPage, "Subsequent write operation succeeded.\r\n");
			      }
			      break;
			    }
			  case 's':
			  case 'S':
			    {
			      int pos = 1;
			      uint32_t iterations;
			      pos = linebuf_set_u32(linebuf, pos, &iterations);
			      if (pos <= 0) {
				break;
			      }
			      sprintf(cStrLen, "Waiting for %lu iterations...\r\n", iterations);
			      strcat(cTelnetPage, cStrLen);
			      netconn_write(pxNetCon, cTelnetPage, (u16_t) strlen(cTelnetPage), NETCONN_COPY);
			      cTelnetPage[0] = 0;
			      for (uint32_t i = 0; i < iterations; ++i);
			      strcat(cTelnetPage, "...done.\r\n");
			      break;
			    }
			  default:
			    break;
			  }
			  if (cTelnetPage[0])
			    netconn_write( pxNetCon, cTelnetPage, (u16_t) strlen( cTelnetPage ), NETCONN_COPY );
			  cTelnetPage[0] = 0;
			}
		    }
		}
		if (pcRxString[0] == 'x')
		  telnet_exit = 1;
		netbuf_delete( pxRxBuffer );
	}
	
   }
  while (!telnet_exit);

  netconn_close( pxNetCon );
  netconn_delete( pxNetCon );

}

#endif

