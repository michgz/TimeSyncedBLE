PROJECT_NAME := TimeSyncedBLE
TARGETS := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := nRF5_SDK_16.0.0_98a08e2
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := time_synced_ble.ld


INC_FOLDERS += ./config
INC_FOLDERS += ./boards/include
INC_FOLDERS += ./time_sync
INC_FOLDERS += ./Services
INC_FOLDERS += ./Hardware
INC_FOLDERS += $(SDK_ROOT)/components
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_advertising
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_db_discovery
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_dtm
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_link_ctx_manager
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_racp
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_ans_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_bas
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_bas_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_cscs
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_cts_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_dfu
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_dis
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_gls
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_hids
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_hrs
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_hts
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_ias
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_ias_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_lbs
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_lls
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_nus
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_nus_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_rscs
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c
INC_FOLDERS += $(SDK_ROOT)/components/ble/ble_services/ble_tps
INC_FOLDERS += $(SDK_ROOT)/components/ble/common
INC_FOLDERS += $(SDK_ROOT)/components/ble/nrf_ble_gatt
INC_FOLDERS += $(SDK_ROOT)/components/ble/nrf_ble_gq
INC_FOLDERS += $(SDK_ROOT)/components/ble/nrf_ble_qwr
INC_FOLDERS += $(SDK_ROOT)/components/ble/nrf_ble_scan
INC_FOLDERS += $(SDK_ROOT)/components/ble/peer_manager
INC_FOLDERS += $(SDK_ROOT)/components/boards
INC_FOLDERS += $(SDK_ROOT)/components/libraries/atomic
INC_FOLDERS += $(SDK_ROOT)/components/libraries/atomic_fifo
INC_FOLDERS += $(SDK_ROOT)/components/libraries/atomic_flags
INC_FOLDERS += $(SDK_ROOT)/components/libraries/balloc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/bootloader/ble_dfu
INC_FOLDERS += $(SDK_ROOT)/components/libraries/bsp
INC_FOLDERS += $(SDK_ROOT)/components/libraries/button
INC_FOLDERS += $(SDK_ROOT)/components/libraries/cli
INC_FOLDERS += $(SDK_ROOT)/components/libraries/crc16
INC_FOLDERS += $(SDK_ROOT)/components/libraries/crc32
INC_FOLDERS += $(SDK_ROOT)/components/libraries/crypto
INC_FOLDERS += $(SDK_ROOT)/components/libraries/csense
INC_FOLDERS += $(SDK_ROOT)/components/libraries/csense_drv
INC_FOLDERS += $(SDK_ROOT)/components/libraries/delay
INC_FOLDERS += $(SDK_ROOT)/components/libraries/ecc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_section_vars
INC_FOLDERS += $(SDK_ROOT)/components/libraries/experimental_task_manager
INC_FOLDERS += $(SDK_ROOT)/components/libraries/fds
INC_FOLDERS += $(SDK_ROOT)/components/libraries/fifo
INC_FOLDERS += $(SDK_ROOT)/components/libraries/fstorage
INC_FOLDERS += $(SDK_ROOT)/components/libraries/gfx
INC_FOLDERS += $(SDK_ROOT)/components/libraries/gpiote
INC_FOLDERS += $(SDK_ROOT)/components/libraries/hardfault
INC_FOLDERS += $(SDK_ROOT)/components/libraries/hci
INC_FOLDERS += $(SDK_ROOT)/components/libraries/led_softblink
INC_FOLDERS += $(SDK_ROOT)/components/libraries/log
INC_FOLDERS += $(SDK_ROOT)/components/libraries/log/src
INC_FOLDERS += $(SDK_ROOT)/components/libraries/low_power_pwm
INC_FOLDERS += $(SDK_ROOT)/components/libraries/mem_manager
INC_FOLDERS += $(SDK_ROOT)/components/libraries/memobj
INC_FOLDERS += $(SDK_ROOT)/components/libraries/mpu
INC_FOLDERS += $(SDK_ROOT)/components/libraries/mutex
INC_FOLDERS += $(SDK_ROOT)/components/libraries/pwm
INC_FOLDERS += $(SDK_ROOT)/components/libraries/pwr_mgmt
INC_FOLDERS += $(SDK_ROOT)/components/libraries/queue
INC_FOLDERS += $(SDK_ROOT)/components/libraries/ringbuf
INC_FOLDERS += $(SDK_ROOT)/components/libraries/scheduler
INC_FOLDERS += $(SDK_ROOT)/components/libraries/sdcard
INC_FOLDERS += $(SDK_ROOT)/components/libraries/slip
INC_FOLDERS += $(SDK_ROOT)/components/libraries/sortlist
INC_FOLDERS += $(SDK_ROOT)/components/libraries/spi_mngr
INC_FOLDERS += $(SDK_ROOT)/components/libraries/stack_guard
INC_FOLDERS += $(SDK_ROOT)/components/libraries/strerror
INC_FOLDERS += $(SDK_ROOT)/components/libraries/svc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/timer
INC_FOLDERS += $(SDK_ROOT)/components/libraries/twi_mngr
INC_FOLDERS += $(SDK_ROOT)/components/libraries/twi_sensor
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/audio
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/cdc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/hid
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/hid/generic
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse
INC_FOLDERS += $(SDK_ROOT)/components/libraries/usbd/class/msc
INC_FOLDERS += $(SDK_ROOT)/components/libraries/util
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/common
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/generic/message
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/generic/record
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/launchapp
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/parser/message
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/parser/record
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/text
INC_FOLDERS += $(SDK_ROOT)/components/nfc/ndef/uri
INC_FOLDERS += $(SDK_ROOT)/components/nfc/platform
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t2t_lib
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t2t_parser
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t4t_lib
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t4t_parser/apdu
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t4t_parser/cc_file
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure
INC_FOLDERS += $(SDK_ROOT)/components/nfc/t4t_parser/tlv
INC_FOLDERS += $(SDK_ROOT)/components/softdevice/common
INC_FOLDERS += $(SDK_ROOT)/components/softdevice/s132/headers
INC_FOLDERS += $(SDK_ROOT)/components/softdevice/s132/headers/nrf52
INC_FOLDERS += $(SDK_ROOT)/components/toolchain/cmsis/include
INC_FOLDERS += $(SDK_ROOT)/external/fprintf
INC_FOLDERS += $(SDK_ROOT)/external/segger_rtt
INC_FOLDERS += $(SDK_ROOT)/external/utf_converter
INC_FOLDERS += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52
INC_FOLDERS += $(SDK_ROOT)/external/freertos/portable/GCC/nrf52
INC_FOLDERS += $(SDK_ROOT)/external/freertos/source/include
INC_FOLDERS += $(SDK_ROOT)/integration/nrfx
INC_FOLDERS += $(SDK_ROOT)/integration/nrfx/legacy
INC_FOLDERS += $(SDK_ROOT)/modules/nrfx
INC_FOLDERS += $(SDK_ROOT)/modules/nrfx/drivers/include
INC_FOLDERS += $(SDK_ROOT)/modules/nrfx/hal
INC_FOLDERS += $(SDK_ROOT)/modules/nrfx/mdk
INC_FOLDERS += $(SDK_ROOT)/components/libraries/log/src


# nRF LOG
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c
SRC_FILES += $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c
#Third Parties">
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/croutine.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/event_groups.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/portable/MemMang/heap_1.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/list.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/portable/GCC/nrf52/port.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/queue.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/stream_buffer.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/tasks.c
#SRC_FILES += $(SDK_ROOT)/external/freertos/source/timers.c
#nRF_Libraries">
SRC_FILES += $(SDK_ROOT)/components/libraries/button/app_button.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_error.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_error_weak.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fifo/app_fifo.c
SRC_FILES += $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c
SRC_FILES += $(SDK_ROOT)/components/libraries/timer/app_timer2.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/app_util_platform.c
SRC_FILES += $(SDK_ROOT)/components/libraries/timer/drv_rtc.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fds/fds.c
SRC_FILES += $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c
SRC_FILES += $(SDK_ROOT)/components/libraries/util/nrf_assert.c
SRC_FILES += $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c
SRC_FILES += $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c
SRC_FILES += $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c
SRC_FILES += $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c
SRC_FILES += $(SDK_ROOT)/external/fprintf/nrf_fprintf.c
SRC_FILES += $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_nvmc.c
SRC_FILES += $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c
SRC_FILES += $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c
SRC_FILES += $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c
SRC_FILES += $(SDK_ROOT)/components/libraries/queue/nrf_queue.c
SRC_FILES += $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c
SRC_FILES += $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c
SRC_FILES += $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c
SRC_FILES += $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c
#None">
SRC_FILES += $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S
#SRC_FILES += $(SDK_ROOT)/modules/nrfx/mdk/ses_startup_nrf_common.s
SRC_FILES += $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c
#Board Definition">
SRC_FILES += $(SDK_ROOT)/components/boards/boards.c
#nRF_Drivers
SRC_FILES += $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_twi.c
SRC_FILES += $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_pwm.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twi.c
SRC_FILES += $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twim.c
#Board Support">
SRC_FILES += $(SDK_ROOT)/components/libraries/bsp/bsp.c
SRC_FILES += $(SDK_ROOT)/components/libraries/bsp/bsp_btn_ble.c
#Application">
SRC_FILES += ./main.c
SRC_FILES += ./time_sync/time_sync.c
SRC_FILES += ./Hardware/HwTimers.c
SRC_FILES += ./Hardware/HwSensor.c
SRC_FILES += ./Hardware/TimedCircBuffer.c
#Services">
SRC_FILES += ./Services/ScanList.c
SRC_FILES += ./Services/ServiceConfig.c
SRC_FILES += ./Services/ServiceDebug.c
SRC_FILES += ./Services/amts.c
SRC_FILES += ./Services/amtc.c
SRC_FILES += ./Services/BroadcastAdvertising.c
#nRF_Segger_RTT">
SRC_FILES += $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c
SRC_FILES += $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_SES.c
SRC_FILES += $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c
#nRF_BLE
SRC_FILES += $(SDK_ROOT)/components/ble/common/ble_advdata.c
SRC_FILES += $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c
SRC_FILES += $(SDK_ROOT)/components/ble/common/ble_conn_params.c
SRC_FILES += $(SDK_ROOT)/components/ble/common/ble_conn_state.c
SRC_FILES += $(SDK_ROOT)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c
SRC_FILES += $(SDK_ROOT)/components/ble/common/ble_srv_common.c
SRC_FILES += $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c
SRC_FILES += $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c
SRC_FILES += $(SDK_ROOT)/external/utf_converter/utf.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/auth_status_tracker.c
SRC_FILES += $(SDK_ROOT)/components/ble/ble_db_discovery/ble_db_discovery.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/id_manager.c
SRC_FILES += $(SDK_ROOT)/components/ble/nrf_ble_gq/nrf_ble_gq.c
SRC_FILES += $(SDK_ROOT)/components/ble/nrf_ble_scan/nrf_ble_scan.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/peer_database.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/peer_id.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/peer_manager_handler.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c
SRC_FILES += $(SDK_ROOT)/components/ble/peer_manager/security_manager.c
#nRF_BLE_Services"
#nRF_SoftDevice
SRC_FILES += $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c
SRC_FILES += $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c
SRC_FILES += $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c


# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto



# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DPERIPHERAL
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# Debug
#CFLAGS += -DDEBUG
#CFLAGS += -DDEBUG_NRF



# C++ flags common to all targets
CXXFLAGS += $(OPT)
# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DPERIPHERAL
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS132
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Debug
#ASMFLAGS += -DDEBUG
#ASMFLAGS += -DDEBUG_NRF


# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52832_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__STACK_SIZE=8192


# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52832_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_7.0.1_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ./config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
