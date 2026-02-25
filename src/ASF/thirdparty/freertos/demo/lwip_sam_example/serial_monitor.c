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

#include "evm300.h"
#include "m25p128_spi.h"
#include "ext_flash.h"

extern uint8_t gs_uc_mac_address[6];
extern char serial_number[SN_ASCII_MAXLEN];
#define TRUE (1)
#define FALSE (0)

int serial_editline(char *s, int len)
{
  Uart *p_uart = CONF_UART;
  char c;
  int i; /* = strlen(s); */

  for (i = 0; s[i]; i++)
    putchar(s[i]);

  do
    {
      c = getchar();
      if (i < len - 1)
	{
	  if (isprint(c))
	    {
	      putchar(c);
	      s[i++] = c;
	    }
	}
      if (c == '\b')
	if (i > 0)
	  {
	    putchar('\b');
	    putchar(' ');
	    putchar('\b');
	    i--;
	  }
    }
  while(c != 3 && c != '\r' && c != '\n'); 
  s[i] = 0;

  if (c == 3)
    return -1;
  return i;
}

int serial_yes()
{
  Uart *p_uart = CONF_UART;
  char c;

  printf("(yes/no) ? ");
  c = getchar();
  if (c == 'y' || c == 'Y')
    {
      putchar(c);
      c = getchar();
      if (c == 'e' || c == 'E')
	{
	  putchar(c);
	  c = getchar();
	  if (c == 's' || c == 'S')
	    {
	      putchar(c);
	      return TRUE;
	    }
	}
    }
  return FALSE;
}

int serial_hexread32(uint32_t *x)
{
  Uart *p_uart = CONF_UART;
  uint32_t d;
  int i;
  char c;

  d = 0;
  i = 0;
  do
    {
      c = getchar();
      putchar(c);
      if (isxdigit(c))
	{
	  c = toupper(c);
	  if (c > '9')
	    c -= 'A' - 10;
	  else
	    c -= '0';
	  d <<= 4;
	  d += c;
	  i++;
	}
      else
	break;
    }
  while(i < 8);

  *x = d;
  if (i == 8 || (i > 0 && (c == '\n' || c == '\r')))
    return true;
  return false;
}

size_t _read(int handle, unsigned char *buffer, size_t size);

/**
 * \brief This listens for the serial port and acts as a serial monitor
 */
void task_serial(void *pvParameters)
{
  volatile Uart *p_uart = CONF_UART;
  int size = 4;
  uint32_t x = 0;
  char c;
  size_t i;

  /*
  printf("CONF_UART = %08lx\r\n", (uint32_t) CONF_UART);
  printf("stdio_base = %08lx\r\n", (uint32_t) stdio_base);

  printf("NXILJTAG %d, JTAGSEL %d, LED_RED1_ON %d, LED_RED2_ON %d\r\n",
  NXILJTAG, JTAGSEL, LED_RED1_ON, LED_RED2_ON);
  */
  
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  for (;;)
    {

      printf("\r\n%08lX> ", x);
      /*      fflush(stdout); */
      
      while(!uart_is_rx_ready(CONF_UART))
	vTaskDelay(10);

      /*      uart_read(CONF_UART, &c); */
      /*      ptr_get(CONF_UART, &c); */
      /*      i = _read(0, &c, 1);
	      printf("_read returned %08lx\r\n", i); */
      c = getchar();
      
      putchar(c);
      c = toupper(c);
      switch(c)
	{
	case '#':
	  {
	    int i;

	    for (i = 100; i; i--)
	      {
		ioport_set_pin_level(LED_RED1_ON, 1);
		ioport_set_pin_level(LED_RED2_ON, 1);
		ioport_set_pin_level(LED_GREEN1_OFF, 0);
		ioport_set_pin_level(LED_GREEN2_OFF, 0);
		ioport_set_pin_level(LED_BLUE1_OFF, 0);
		ioport_set_pin_level(LED_BLUE2_OFF, 0);
		vTaskDelay( 20 );
	      }
	  }
	  break;
	case '1':
	  printf("\r\nAccess size byte.");
	  size = 1;
	  break;
	case '2':
	  printf("\r\nAccess size short.");
	  size = 2;
	  break;
	case '4':
	  printf("\r\nAccess size long word.");
	  size = 4;
	  break;
	case 'X':
	  putchar(' ');
	  serial_hexread32(&x);
	  break;
	case 'C':
	  {
	    uint32_t cmd;
	    putchar(' ');
	    if (serial_hexread32(&cmd))
	      {
		printf("\r\nCommand %02lX", cmd);
		if (fpga_write_command(cmd)) {
		  printf("...failed");
		}
	      }
	    break;
	  }
	case 'W':
	  printf("\r\nWrite OPB Address");
	  if (fpga_write_opb_address(x)) {
	    printf("...failed");
	  }
	  break;
	case 'R':
	  {
	    uint32_t data;
	    fpga_read_data(&data);
	    break;
	  }
	case 'D':
	  {
	    int i;

	    x = x & (-size);
	    printf("\r\n%08lX:", x);
	    for (i = 0; i < 16; i+=size)
	      {
		switch(size)
		  {
		  case 1:
		    {
		      uint8_t data;
		      if (fpga_read_byte(x, &data)) {
			printf(" failed");
		      } else {
		        printf(" %02X", (unsigned int) data);
		      }
		      x++;
		      break;
		    }
		  case 2:
		    {
		      uint16_t data;
		      if (fpga_read_short(x, &data)) {
			printf(" failed");
		      } else {
		        printf(" %04hX", data);
		      }
		      x += 2;
		      break;
		    }
		  case 4:
		    {
		      uint32_t data;
		      if (fpga_read_long(x, &data)) {
			printf(" failed");
		      } else {
		        printf(" %08lX", data);
		      }
		      x += 4;
		      break;
		    }
		  }
	      }
	    break;
	  }
	case 'H':
	  {
	    int i;
	    uint32_t *p = (uint32_t *) x;

	    x = x & (-size);
	    printf("\r\n%08lX:", x);
	    for (i = 0; i < 16; i+=size)
	      {
		p = (uint32_t *) x;
		printf(" %08lX", *p);
		x += 4;
	      }
	    break;
	  }
	case 'M':
	  {
	    uint32_t d;
	    uint32_t *p = (uint32_t *) x;
	    
	    x = x & (-size);
	    printf("\r\n%08lX: ", x);
	    switch(size)
	      {
	      case 1:
		{
		  uint8_t data;
		  if (fpga_read_byte(x, &data)) {
		    printf(" failed");
		  } else {
		    printf(" %02X", (unsigned int) data);
		  }
		  break;
		}
	      case 2:
		{
		  uint16_t data;
		  if (fpga_read_short(x, &data)) {
		    printf(" failed");
		  } else {
		    printf(" %04hX", data);
		  }
		  break;
		}
	      case 4:
		{
		  uint32_t data;
		  if (fpga_read_long(x, &data)) {
		    printf(" failed");
		  } else {
		    printf(" %08lX", data);
		  }
		  break;
		}
	      }
	    printf(" ? ");
	    if (serial_hexread32(&d))
	      {
		switch(size)
		  {
		  case 1:
		    fpga_write_byte(x, (uint8_t) d & 0xff);
		    break;
		  case 2:
		    fpga_write_short(x, (uint16_t) d & 0xffff);
		    break;
		  case 4:
		    fpga_write_long(x, d);
		    break;
		  }
	      }
	  }
	  break;
	case 'N':
	  {
	    uint32_t d;
	    uint32_t *p = (uint32_t *) x;
	    
	    x = x & (-size);
	    printf("\r\n%08lX: %08lX ? ", x, *p);
	    if (serial_hexread32(&d))
	      *p = d;
	  }
	  break;
	case 'T':
	  printf(" Test write TWI");
	  if (fpga_test_write()) {
	    printf("...failed");
	  }
	  break;
	case 'E':
	  printf("Erase Setup Flash ");
	  if (serial_yes())
	    {
	      int i;
	      m25p_status_t status;
	      status = m25p_write_enable();
	      if (status == M25P_SUCCESS)
		status = m25p_sector_erase(MAC_ADDRESS_OFFSET);
	      m25p_write_disable();
	      
	      for (i = 100; i; i--)
		if (ext_flash_wait_for_ready() == M25P_SUCCESS)
		  break;

	      if (!i)
		printf(" READY\r\n");
	      else
		printf(" TIMEOUT\r\n");
	    }
	  break;
	case 'S':
	  {
	    uint32_t d;
	    uint8_t new_mac_address[6];
	    char new_serial_number[SN_ASCII_MAXLEN];

	    printf("\r\nMAC address %02X:%02X:%02X:%02X:%02X:%02X, (enter 3 LSB) ? ",
	       gs_uc_mac_address[0], gs_uc_mac_address[1],
	       gs_uc_mac_address[2], gs_uc_mac_address[3],
	       gs_uc_mac_address[4], gs_uc_mac_address[5]);
	    if (serial_hexread32(&d))
	      {
		new_mac_address[0] = MRF_OUI_0;
		new_mac_address[1] = MRF_OUI_1;
		new_mac_address[2] = MRF_OUI_2;
		new_mac_address[3] = (d >> 16) & 0x00ff;
		new_mac_address[4] = (d >> 8) & 0x00ff;
		new_mac_address[5] = d & 0x00ff;
	      }
	    printf("\r\nSerial number: ");
	    strcpy(new_serial_number, serial_number);
	    if (serial_editline(new_serial_number, SN_ASCII_MAXLEN) < 0)
	      strcpy(new_serial_number, serial_number);
	    printf("\r\n");
	    printf("New MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
		   new_mac_address[0], new_mac_address[1],
		   new_mac_address[2], new_mac_address[3],
		   new_mac_address[4], new_mac_address[5]);
	    printf("New serial number: %s\r\n", new_serial_number);
	    printf("Write to flash ");
	    if (serial_yes())
	      {
		m25p_status_t status;

		printf("Writing MAC address, ");
		status = ext_flash_write_mac_address(new_mac_address);
		ext_flash_print_status(status);
		if (ext_flash_wait_for_ready() == M25P_SUCCESS)
		  printf(" READY\r\n");
		else
		  printf(" TIMEOUT\r\n");
		printf("Writing serial number, ");
		status = ext_flash_write_sn(new_serial_number, strlen(new_serial_number)+1);
		ext_flash_print_status(status);
		if (ext_flash_wait_for_ready() == M25P_SUCCESS)
		  printf(" READY\r\n");
		else
		  printf(" TIMEOUT\r\n");
	      }
	  }
	  break;
	default:
	  break;
	}
      
    }

#if 0
  for (;;) 
    {
      if (p_uart->US_CSR & US_CSR_RXRDY)
	{
	  received_byte = p_uart->US_RHR & US_RHR_RXCHR_Msk;
	  putchar(received_byte);
	  switch(received_byte)
	    {
	    case 'T':
	      printf(" Test write TWI");
	      if (fpga_test_write()) {
		printf("...failed")
	      }
	      break;
	    default:
	      break;
	    }

	  printf("\r\n> ", received_byte);
	}
      vTaskDelay(100);
    }
#endif
}
