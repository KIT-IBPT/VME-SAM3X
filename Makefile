RM := rm -rf

# Source file lists
C_SRCS := \
	src/ASF/common/services/clock/sam3x/sysclk.c \
	src/ASF/common/services/serial/usart_serial.c \
	src/ASF/common/services/spi/sam_spi/spi_master.c \
	src/ASF/common/utils/interrupt/interrupt_sam_nvic.c \
	src/ASF/common/utils/stdio/read.c \
	src/ASF/common/utils/stdio/write.c \
	src/ASF/sam/boards/sam3x_evm300/led.c \
	src/ASF/sam/components/ethernet_phy/dm9161a/ethernet_phy.c \
	src/ASF/sam/drivers/emac/emac.c \
	src/ASF/sam/drivers/pio/pio.c \
	src/ASF/sam/drivers/pio/pio_handler.c \
	src/ASF/sam/drivers/pmc/pmc.c \
	src/ASF/sam/drivers/pmc/sleep.c \
	src/ASF/sam/drivers/rstc/rstc.c \
	src/ASF/sam/drivers/spi/spi.c \
	src/ASF/sam/drivers/tc/tc.c \
	src/ASF/sam/drivers/twi/twi.c \
	src/ASF/sam/drivers/uart/uart.c \
	src/ASF/sam/drivers/usart/usart.c \
	src/ASF/sam/utils/cmsis/sam3x/source/templates/exceptions.c \
	src/ASF/sam/utils/cmsis/sam3x/source/templates/gcc/startup_sam3x.c \
	src/ASF/sam/utils/cmsis/sam3x/source/templates/system_sam3x.c \
	src/ASF/sam/utils/syscalls/gcc/syscalls.c \
	src/ASF/thirdparty/freertos/demo/common/minimal/flash.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/BasicUDP.c \
	src/ASF/thirdparty/freertos/demo/lwip_avr32_uc3_example/network/basicweb/BasicWEB.c \
	src/ASF/thirdparty/freertos/demo/lwip_avr32_uc3_example/network/basictelnet/BasicTelnet.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/ext_flash.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/fpga_twi.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/m25p128_spi.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/main.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/network/ethernet.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/partest/ParTest.c \
	src/ASF/thirdparty/freertos/demo/lwip_sam_example/serial_monitor.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/croutine.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/event_groups.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/list.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM3/port.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_4.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/queue.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/stream_buffer.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/tasks.c \
	src/ASF/thirdparty/freertos/freertos-10.0.0/Source/timers.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/api_lib.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/api_msg.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/err.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/netbuf.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/netdb.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/netifapi.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/sockets.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/api/tcpip.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/def.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/dhcp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/dns.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/init.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/autoip.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/icmp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/igmp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/inet.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/inet_chksum.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/ip.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/ip_addr.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/ipv4/ip_frag.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/lwip_timers_140.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/mem.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/memp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/netif.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/pbuf.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/raw.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/stats.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/sys.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/tcp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/tcp_in.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/tcp_out.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/core/udp.c \
	src/ASF/thirdparty/lwip/lwip-1.4.0/src/netif/etharp.c \
	src/ASF/thirdparty/lwip/lwip-port-1.4.0/sam/netif/ethernetif.c \
	src/ASF/thirdparty/lwip/lwip-port-1.4.0/sam/sys_arch.c

LOADER=src/ASF/sam/utils/linker_scripts/sam3x/sam3x8/gcc/flash.ld

BUILD_DIR := build
IMAGE_NAME := EVM_300_UDP


# Automatically generate object files and dependency files from source files
OBJS := $(C_SRCS:src/%.c=$(BUILD_DIR)/%.o)
C_DEPS := $(OBJS:.o=.d)

OUTPUT_FILE_PATH := $(BUILD_DIR)/$(IMAGE_NAME).elf

ADDITIONAL_DEPENDENCIES :=
LIB_DEP := 
LINKER_SCRIPT_DEP := $(LOADER)

# Common compiler flags
COMMON_FLAGS := \
	-x c -mthumb -D__SAM3X8H__ -DDEBUG -DUDP_USED=1 -Dprintf=iprintf \
	-D__FREERTOS__ -DHTTP_USED=0 -DTELNET_USED=1 -DARM_MATH_CM3=true \
	-DBOARD=SAM3X_EVM300 -Dscanf=iscanf -DFREERTOS_USED

# Include paths
INCLUDE_PATHS := \
	-I"src/ASF/thirdparty/freertos/demo/lwip_avr32_uc3_example/network/basictftp" \
	-I"src/ASF/thirdparty/freertos/demo/lwip_avr32_uc3_example/network/basicweb" \
	-I"src/ASF/thirdparty/freertos/demo/lwip_avr32_uc3_example/network/basictelnet" \
	-I"src/ASF/thirdparty/freertos/demo/lwip_sam_example" \
	-I"src/ASF/thirdparty/freertos/demo/common/include" \
	-I"src/ASF/thirdparty/freertos/demo/lwip_sam_example/network" \
	-I"src/ASF/sam/boards" \
	-I"src/ASF/sam/boards/sam3x_evm300" \
	-I"src/ASF/sam/utils/cmsis/sam3x/source/templates" \
	-I"src/ASF/sam/utils/cmsis/sam3x/include" \
	-I"src/ASF/common/boards" \
	-I"src/ASF/sam/utils" \
	-I"src/ASF/sam/utils/header_files" \
	-I"src/ASF/sam/utils/preprocessor" \
	-I"src/ASF/thirdparty/CMSIS/Include" \
	-I"src/ASF/thirdparty/CMSIS/Lib/GCC" \
	-I"src/ASF/common/utils" \
	-I"src/ASF/common/services/gpio" \
	-I"src/ASF/sam/drivers/pio" \
	-I"src/ASF/sam/drivers/pmc" \
	-I"src/ASF/sam/drivers/uart" \
	-I"src/ASF/common/services/ioport" \
	-I"src/ASF/common/services/clock" \
	-I"src/ASF/thirdparty/freertos/freertos-10.0.0/Source/include" \
	-I"src/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM3" \
	-I"src/ASF/thirdparty/lwip/lwip-1.4.0/src/include/lwip" \
	-I"src/ASF/thirdparty/lwip/lwip-1.4.0/src/include" \
	-I"src/ASF/thirdparty/lwip/lwip-1.4.0/src/include/ipv4" \
	-I"src/ASF/thirdparty/lwip/lwip-port-1.4.0/sam/include" \
	-I"src/ASF/sam/drivers/emac" \
	-I"src/ASF/sam/drivers/tc" \
	-I"src/ASF/common/services/twi" \
	-I"src/ASF/sam/drivers/twi" \
	-I"src/ASF/sam/drivers/spi" \
	-I"src/ASF/common/services/spi/sam_spi" \
	-I"src/ASF/common/services/spi" \
	-I"src/ASF/sam/drivers/rstc/example1" \
	-I"src/ASF/sam/drivers/rstc" \
	-I"src/ASF/sam/components/ethernet_phy/dm9161a" \
	-I"src/ASF/common/utils/stdio/stdio_serial" \
	-I"src/ASF/common/services/serial/sam_uart" \
	-I"src/ASF/common/services/serial" \
	-I"src/ASF/sam/drivers/usart" \
	-I"src" \
	-I"src/config"

# Compiler optimization and warning flags
CFLAGS := \
	-O1 -fdata-sections -ffunction-sections -mlong-calls -g3 -Wall \
	-mcpu=cortex-m3 -c -pipe -fno-strict-aliasing -Wall -Wstrict-prototypes \
	-Wmissing-prototypes -Werror-implicit-function-declaration \
	-Wpointer-arith -std=gnu99 -ffunction-sections -fdata-sections \
	-Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int -Wmain \
	-Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs \
	-Wunused -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef \
	-Wshadow -Wbad-function-cast -Wwrite-strings -Wsign-compare \
	-Waggregate-return -Wmissing-declarations -Wformat \
	-Wmissing-format-attribute -Wno-deprecated-declarations -Wpacked \
	-Wredundant-decls -Wnested-externs -Wlong-long -Wunreachable-code \
	-Wcast-align --param max-inline-insns-single=500

# Linker flags
LDFLAGS := \
	-mthumb -Wl,-Map="$(BUILD_DIR)/$(IMAGE_NAME).map" -Wl,--start-group \
	-larm_cortexM3l_math -lm -Wl,--end-group \
	-L"src/ASF/thirdparty/CMSIS/Lib/GCC" -Wl,--gc-sections \
	-mcpu=cortex-m3 -Wl,--entry=Reset_Handler -Wl,--cref -mthumb \
	-T$(LOADER) --specs=nano.specs

# Include dependency files
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# All target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

# Generic rule for compiling C files
$(OBJS): $(BUILD_DIR)/%.o: src/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.3.1
	@mkdir -p $(dir $@)
	arm-none-eabi-gcc $(COMMON_FLAGS) $(INCLUDE_PATHS) $(CFLAGS) \
		-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" \
		-o "$@" "$<"
	@echo Finished building: $<

# Linking
$(OUTPUT_FILE_PATH): $(OBJS) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.3.1
	arm-none-eabi-gcc -o $(OUTPUT_FILE_PATH) $(OBJS) $(LDFLAGS)
	@echo Finished building target: $@
	arm-none-eabi-objcopy \
		-O binary \
		"$(BUILD_DIR)/$(IMAGE_NAME).elf" "$(BUILD_DIR)/$(IMAGE_NAME).bin"
	arm-none-eabi-objcopy \
		-O ihex -R .eeprom -R .fuse -R .lock -R .signature \
		"$(BUILD_DIR)/$(IMAGE_NAME).elf" "$(BUILD_DIR)/$(IMAGE_NAME).hex"
	arm-none-eabi-objcopy \
		-j .eeprom --set-section-flags=.eeprom=alloc,load \
		--change-section-lma .eeprom=0 --no-change-warnings -O binary \
		"$(BUILD_DIR)/$(IMAGE_NAME).elf" "$(BUILD_DIR)/$(IMAGE_NAME).eep" \
		|| exit 0
	arm-none-eabi-objdump \
		-h -S "$(BUILD_DIR)/$(IMAGE_NAME).elf" \
		>"$(BUILD_DIR)/$(IMAGE_NAME).lss"
	arm-none-eabi-objcopy \
		-O srec -R .eeprom -R .fuse -R .lock -R .signature \
		"$(BUILD_DIR)/$(IMAGE_NAME).elf" "$(BUILD_DIR)/$(IMAGE_NAME).srec"
	arm-none-eabi-size "$(BUILD_DIR)/$(IMAGE_NAME).elf"

# Clean target
clean:
	-$(RM) $(OBJS)
	-$(RM) $(C_DEPS)
	rm -rf \
		"$(BUILD_DIR)/$(IMAGE_NAME).a" \
		"$(BUILD_DIR)/$(IMAGE_NAME).bin" \
		"$(BUILD_DIR)/$(IMAGE_NAME).eep" \
		"$(BUILD_DIR)/$(IMAGE_NAME).elf" \
		"$(BUILD_DIR)/$(IMAGE_NAME).hex" \
		"$(BUILD_DIR)/$(IMAGE_NAME).lss" \
		"$(BUILD_DIR)/$(IMAGE_NAME).map" \
		"$(BUILD_DIR)/$(IMAGE_NAME).srec"

.PHONY: all clean
