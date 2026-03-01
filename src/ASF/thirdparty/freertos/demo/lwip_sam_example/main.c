/**
 * \file
 *
 * \brief FreeRTOS with lwIP example.
 *
 * Copyright (c) 2012-2014 Atmel Corporation. All rights reserved.
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
 */

/**
 * \mainpage FreeRTOS with lwIP example
 *
 * \section Purpose
 *
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software for the lwIP basic two-in-one web server and TFTP
 * server demo (with DHCP) example.
 *
 * The given example is an example using freeRTOS, the current lwIP stack and
 * EMAC driver.
 *
 * \section Requirements
 *
 * This package can be used with SAM3X-EK and SAM4E-EK.
 *
 * \section Description
 *
 * The demonstration program create two applications. The web server will
 * feedback the FreeRTOS kernel statistics and the tftp server allows the user
 * to PUT and GET files.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos with lwIP Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

/* Environment include files. */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "status_codes.h"
#include "conf_uart_serial.h"
/* Demo file headers. */
#include "partest.h"
#include "ethernet.h"
#include "netif/etharp.h"
#include "flash.h"
#include "sysclk.h"
#include "ioport.h"
#include "uart_serial.h"
#include "stdio_serial.h"
#include <compiler.h>
#include "led.h"
#include "conf_board.h"
#include "conf_eth.h"

#include "evm300.h"
#include "m25p128_spi.h"
#include "ext_flash.h"
#include "xvc_server.h"

#define TASK_LED_STACK_SIZE                (128 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_SERIAL_STACK_SIZE                (1024 / sizeof(portSTACK_TYPE))
#define TASK_SERIAL_STACK_PRIORITY            (tskIDLE_PRIORITY)

void vAssertCalled( const char * const pcFileName, unsigned long ulLine )
{
  static portBASE_TYPE xPrinted = pdFALSE;
  volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    printf("vAssertCalled: Line %d, File %s\n", ulLine, pcFileName);

    taskENTER_CRITICAL();
    {
        /* You can step out of this function to debug the assertion by using
        the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
        value. */
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
        }
    }
    taskEXIT_CRITICAL();
}
/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

uint8_t gs_uc_mac_address[6];
char serial_number[SN_ASCII_MAXLEN];

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);

extern void xPortSysTickHandler(void);
void task_serial(void *pvParameters);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);

	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

extern void vApplicationMallocFailedHook( void )
{
	printf("Malloc failed\r\n");
	printf("xPortGetFreeHeapSize() %d\r\n", xPortGetFreeHeapSize());
	printf("xPortGetMinimumEverFreeHeapSize() %d\r\n", xPortGetMinimumEverFreeHeapSize());

	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
  LED_Toggle(LED2);
}

/**
 * \brief This task checks handle switch position and controls JTAG chain
 */
static void task_led(void *pvParameters)
{
	for (;;) {
		LED_Toggle(LED0);
		xvc_server_update_jtagsel();
		
                if (gpio_pin_is_high(HSOPEN))
		  {
		    ioport_set_pin_level(PWRDWN, 1);
		    ioport_set_pin_level(LED_GREEN2_OFF, 1);
		  }
                else
		  {
		    ioport_set_pin_level(PWRDWN, 0);
		    ioport_set_pin_level(LED_GREEN2_OFF, 0);
		  }
		vTaskDelay(1000);
	}
}
static void task_led2(void *pvParameters)
{
	for (;;) {
		LED_Toggle(LED2);
		vTaskDelay(500);
	}
}

static void task_simpleserial(void *pvParameters)
{
  Usart *p_usart = CONF_UART;

  uint32_t received_byte;

  printf("CONF_UART = %08lx\r\n", (uint32_t) CONF_UART);

  printf("\r\n> ");

  for (;;) 
    {
      printf("1");
      if (p_usart->US_CSR & US_CSR_RXRDY)
	{
	  printf("2");
	  received_byte = p_usart->US_RHR & US_RHR_RXCHR_Msk;
	  /*
	  putchar(received_byte);
	  */
	  printf("3");
	  switch(received_byte)
	    {
	    case 'T':
	      printf(" Test write TWI");
	      if (fpga_test_write()) {
		printf("...failed");
	      }
	      break;
	    }
	  printf("%c %02X\r\n> ", received_byte, received_byte);
	}
      vTaskDelay(100);
    }
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
#else

	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
  uint32_t flash_dev_id;

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Initilize the SAM system */
	sysclk_init();

	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);

	ioport_set_pin_dir(LED0_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED0_GPIO, LED0_ACTIVE_LEVEL);
	ioport_set_pin_dir(LED1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED1_GPIO, LED1_ACTIVE_LEVEL);

	gpio_configure_group(PINS_UART_PIO, PINS_UART, PINS_UART_FLAGS);

	/* Initialize the XVC server (this does not start the server). */
	xvc_server_init();

	/* Initialize the console uart */
	configure_console();

	/* Configure the pins connected to LEDs as output and set their
	 * default initial state to high (LEDs off).
	 */
	ioport_set_pin_dir(LED2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED2_GPIO, LED2_INACTIVE_LEVEL);

	ioport_set_pin_dir(LED_RED1_ON, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_RED1_ON, 1);
	ioport_set_pin_dir(LED_RED2_ON, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_RED2_ON, 0);
	ioport_set_pin_dir(LED_GREEN1_OFF, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_GREEN1_OFF, 1);
	ioport_set_pin_dir(LED_GREEN2_OFF, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_GREEN2_OFF, 1);
	ioport_set_pin_dir(LED_BLUE1_OFF, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_BLUE1_OFF, 1);
	ioport_set_pin_dir(LED_BLUE2_OFF, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_BLUE2_OFF, 1);
	ioport_set_pin_dir(MAC_PWR_DOWN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(MAC_PWR_DOWN, 1);
	ioport_set_pin_dir(VMERESETOUT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(VMERESETOUT, 0);
	ioport_set_pin_dir(PWRDWN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PWRDWN, 0);
	ioport_set_pin_dir(NVMERESETIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(NPROG, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(NINIT, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(HSOPEN, IOPORT_DIR_INPUT);

	/*
        gpio_configure_pin(LED_RED1_ON, PIO_OUTPUT_1);
        gpio_configure_pin(LED_RED2_ON, PIO_OUTPUT_0);
        gpio_configure_pin(LED_GREEN1_OFF, PIO_OUTPUT_1);
        gpio_configure_pin(LED_GREEN2_OFF, PIO_OUTPUT_1);
        gpio_configure_pin(LED_BLUE1_OFF, PIO_OUTPUT_1);
        gpio_configure_pin(LED_BLUE2_OFF, PIO_OUTPUT_1);
        gpio_configure_pin(MAC_PWR_DOWN, PIO_OUTPUT_1);
        gpio_configure_pin(VMERESETOUT, PIO_OUTPUT_0);
        gpio_configure_pin(NVMERESETIN, PIO_INPUT);
        gpio_configure_pin(PWRDWN, PIO_OUTPUT_0);
        gpio_configure_pin(NPROG, PIO_INPUT);
        gpio_configure_pin(NINIT, PIO_INPUT);
        gpio_configure_pin(HSOPEN, PIO_INPUT);
	*/

	fpga_twi_init();
	m25p_spi_init();

	ext_flash_read_sn(serial_number, sizeof(serial_number));
	if (!isalnum(serial_number[0]))
	  serial_number[0] = 0;
	serial_number[sizeof(serial_number)-1] = 0;

	/* Output demo infomation. */
	printf("-- FreeRTOS with lwIP Example --\n\r");
	printf("-- %s, S/N: %s\n\r", BOARD_NAME, serial_number);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	if (m25p_read_dev_id(&flash_dev_id) != M25P_SUCCESS)
	  printf("Could not read Flash device ID\r\n");
	printf("Flash device ID %06X\r\n", flash_dev_id);

	ext_flash_read_mac_address(gs_uc_mac_address);
	if (gs_uc_mac_address[0] != ETHERNET_CONF_ETHADDR0 ||
	    gs_uc_mac_address[1] != ETHERNET_CONF_ETHADDR1 ||
	    gs_uc_mac_address[2] != ETHERNET_CONF_ETHADDR2)
	  {
	    printf("Invalid MAC address in flash!\r\n");
	    gs_uc_mac_address[0] = ETHERNET_CONF_ETHADDR0;
	    gs_uc_mac_address[1] = ETHERNET_CONF_ETHADDR1;
	    gs_uc_mac_address[2] = ETHERNET_CONF_ETHADDR2;
	    gs_uc_mac_address[3] = ETHERNET_CONF_ETHADDR3;
	    gs_uc_mac_address[4] = ETHERNET_CONF_ETHADDR4;
	    gs_uc_mac_address[5] = ETHERNET_CONF_ETHADDR5;	    
	  }
	printf("Ethernet MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
	       gs_uc_mac_address[0], gs_uc_mac_address[1],
	       gs_uc_mac_address[2], gs_uc_mac_address[3],
	       gs_uc_mac_address[4], gs_uc_mac_address[5]);

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	if (xTaskCreate(task_led2, "Led2", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

#if 1
	/* Create task for serial console */
	if (xTaskCreate(task_serial, "Serial", TASK_SERIAL_STACK_SIZE, NULL,
			TASK_SERIAL_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create serial console task\r\n");
	}
#endif
	
	/* Start the ethernet tasks */
	vStartEthernetTaskLauncher(configMAX_PRIORITIES - 1);
	
	printf("Start Scheduler.\n");
	/* Start FreeRTOS */
	vTaskStartScheduler();
	printf("Insufficient memory.\n");

	/* Will only reach here if there was insufficient memory to create the idle task */

	return 0;
}
