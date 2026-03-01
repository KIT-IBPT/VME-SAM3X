#define LED_RED1_ON	   PIO_PA21_IDX
#define LED_GREEN1_OFF PIO_PC30_IDX
#define LED_BLUE2_OFF  PIO_PC29_IDX
#define LED_GREEN2_OFF PIO_PC19_IDX
#define LED_RED2_ON    PIO_PC18_IDX
#define LED_BLUE1_OFF  PIO_PB21_IDX
#define VMERESETOUT    PIO_PA10_IDX
#define NVMERESETIN    PIO_PA11_IDX
#define PWRDWN         PIO_PA14_IDX
#define USBPOWER       PIO_PC4_IDX
#define MAC_PWR_DOWN   PIO_PB10_IDX
#define XILTMS         PIO_PD0_IDX
#define XILTCK         PIO_PD1_IDX
#define JTAGSEL        PIO_PD2_IDX
#define MMCTDO         PIO_PD3_IDX
#define XILTDI         PIO_PD4_IDX
#define NXILJTAG       PIO_PD5_IDX
#define NPROG          PIO_PA0_IDX
#define NINIT          PIO_PA1_IDX
#define HSOPEN         PIO_PA29_IDX

#define MRF_OUI_0      (0x00)
#define MRF_OUI_1      (0x0E)
#define MRF_OUI_2      (0xB2)

void fpga_twi_init(void);
void fpga_twi_reset(void);

uint32_t fpga_test_write(void);

uint32_t fpga_write_command(uint8_t command);
uint32_t fpga_write_opb_address(uint32_t address);
uint32_t fpga_read_data(uint32_t *data);
uint32_t fpga_read_byte(uint32_t address, uint8_t *data);
uint32_t fpga_read_short(uint32_t address, uint16_t *data);
uint32_t fpga_read_long(uint32_t address, uint32_t *data);
uint32_t fpga_write_byte(uint32_t address, uint8_t data);
uint32_t fpga_write_short(uint32_t address, uint16_t data);
uint32_t fpga_write_long(uint32_t address, uint32_t data);

