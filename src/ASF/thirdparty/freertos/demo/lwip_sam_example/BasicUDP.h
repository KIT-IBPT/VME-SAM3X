/*****************************************************************************
 *
 * \file
 *
 * \brief Basic UDP Server for Atmel MCUs.
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
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


#ifndef BASIC_UDP_SERVER_H
#define BASIC_UDP_SERVER_H

#include <lwip/arch.h>

#include "portmacro.h"

/* netdb.h */
// Internet services
struct servent {
char *s_name; /* official service name */
char **s_aliases; /* alias list */
int s_port; /* port number */
char *s_proto; /* protocol to use */
};

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct udphdr {
  PACK_STRUCT_FIELD(uint8_t access_type);
  PACK_STRUCT_FIELD(int8_t status);
  PACK_STRUCT_FIELD(uint16_t data_v1);
  PACK_STRUCT_FIELD(uint32_t addr);
  PACK_STRUCT_FIELD(uint32_t ref);
  PACK_STRUCT_FIELD(uint32_t data_v2);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define FPGA_READ_ACCESS_16 0x01
#define FPGA_WRITE_ACCESS_16 0x02
#define FPGA_WRITE_ACCESS_16_NO_READBACK 0x03
#define FPGA_READ_ACCESS_32 0x04
#define FPGA_WRITE_ACCESS_32 0x05
#define FPGA_WRITE_ACCESS_32_NO_READBACK 0x06

#define FPGA_STATUS_OK 0
#define FPGA_STATUS_INVALID_ADDR -1
#define FPGA_STATUS_TIMEOUT -2
#define FPGA_STATUS_INVALID_CMD -3

/* The function that implements the UDP server task. */
portTASK_FUNCTION_PROTO( vBasicUDPServer, pvParameters );



#endif

