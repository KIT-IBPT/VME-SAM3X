/*****************************************************************************
 *
 * \file
 *
 * \brief Basic TFTP Server for Atmel MCUs.
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
  Implements a simplistic TFTP server.

  In order to put data on the TFTP server (not over 2048 bytes)
  tftp 192.168.0.2 PUT <src_filename>
  this will copy file from your hard drive to the RAM buffer of the application

  tftp 192.168.0.2 GET <dst_filename>
  this will copy file from the RAM buffer of the application to your hard drive
  You can then check that src_filename and dst_filename are identical
*/
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#if (UDP_USED == 1)

#include <string.h>


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"
#include "BasicUDP.h"

#include "evm300.h"
#include "m25p128_spi.h"
#include "ext_flash.h"

/* Demo includes. */
#include "portmacro.h"

/* lwIP includes. */
#include <lwip/api.h>
#include <lwip/netbuf.h>
#include <lwip/netdb.h>
#include <lwip/netifapi.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/udp.h>
#if 0
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/stats.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/arch.h"
#include "lwip/sys.h"
#include "lwip/init.h"
#if ( (LWIP_VERSION) == ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)) )
#include "netif/loopif.h"
#endif
#include "lwip/sockets.h"
#endif

#define O_WRONLY 1
#define O_RDONLY 2


/* The port on which we listen. */
#define UDP_PORT    ( 2000 )

void udp_server_callback(void *arg, struct udp_pcb *upcb,
			 struct pbuf *p, struct ip_addr *addr,
			 u16_t port)
{
  struct udphdr *sHdr;
  int i;
  
#ifdef DEBUG_UDP
  printf("UDP packet received %d bytes.\r\n", p->len);
  for (i = 0; i < p->len; i++)
    printf("%02X ", *((char *) p->payload + i));
#endif // DEBUG_UDP

  sHdr = p->payload;
  sHdr->status = 0;
  if (sHdr->access_type == FPGA_WRITE_ACCESS)
    {
      short data = htons(sHdr->data);
      fpga_write_short(ntohl(sHdr->addr), data);
    }
  if (!sHdr->status && (sHdr->access_type == FPGA_WRITE_ACCESS ||
			sHdr->access_type == FPGA_READ_ACCESS))
    {
      sHdr->data = htons(fpga_read_short(ntohl(sHdr->addr)));
    }

  udp_sendto(upcb, p, addr, port);
  
  pbuf_free(p);
}

portTASK_FUNCTION( vBasicUDPServer, pvParameters )
{
  struct udp_pcb *pcb;
  err_t err;

  printf("Start UDP Server\r\n");
  pcb = udp_new();
  if (pcb == NULL)
    printf("Cannot create UDP PCB.\r\n");

  err = udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
  printf("udp_bind returned %d\r\n", err);
  udp_recv(pcb, udp_server_callback, NULL);

  vTaskDelete(NULL);
 }

#if 0
      do {
        // Create socket
        lSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (lSocket < 0) {
	  printf("Cannot create socket %d\r\n", lSocket);
	  vTaskDelay(1000);
	  /*            return; */
        }
      }
      while (lSocket < 0);
	
        memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
        sLocalAddr.sin_family = AF_INET;
        sLocalAddr.sin_len = sizeof(sLocalAddr);
        sLocalAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        sLocalAddr.sin_port = htons(UDP_PORT);

        if (bind(lSocket, (struct sockaddr *)&sLocalAddr, sizeof(sLocalAddr)) < 0) {
            // Problem setting up my end
	  printf("Cannot bind to UDP port.\r\n");
            close(lSocket);
            return;
        }


        lRecvLen = sizeof(cData);
        lFromLen = sizeof(sFromAddr);
        lDataLen = recvfrom(lSocket, sHdr, lRecvLen, 0,
                            (struct sockaddr *)&sFromAddr, &lFromLen);

	printf("Received UDP packet, size %d\r\n", lDataLen);
	
        if ( lDataLen < 0) {

        } else {
	  sHdr->status = 0;
	  if (sHdr->access_type == FPGA_WRITE_ACCESS)
	    {
	      short data = htons(sHdr->data);
	      fpga_write_short(ntohl(sHdr->addr), &data);
	    }
	  if (!sHdr->status && (sHdr->access_type == FPGA_WRITE_ACCESS ||
				sHdr->access_type == FPGA_READ_ACCESS))
	    {
	      sHdr->data = htons(fpga_read_short(ntohl(sHdr->addr)));
	    }

	  sendto(lSocket, sHdr, sizeof(cData), 0, (struct sockaddr *)&sFromAddr, lFromLen);
	}

	close(lSocket); // so that other servers can bind to the UDP socket

    }
}
#endif
#endif
