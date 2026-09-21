option(PICO_TNC_USB_BOOTSEL_RESET "Allow USB 1200-baud and vendor BOOTSEL reset for maintenance" ON)

# Also support configuring the pico_tnc submodule directly.
if(NOT DEFINED DIGI_ENABLE)
    set(DIGI_ENABLE OFF)
endif()
if(NOT DEFINED DIGI_MYCALL)
    set(DIGI_MYCALL "")
endif()
if(NOT DEFINED DIGI_PATH)
    set(DIGI_PATH "WIDE1-1")
endif()
if(NOT DEFINED DIGI_HOLDOFF_MS)
    set(DIGI_HOLDOFF_MS 1500)
endif()
string(LENGTH "${DIGI_PATH}" _digi_path_length)
if(_digi_path_length GREATER 31 OR NOT DIGI_HOLDOFF_MS MATCHES "^[0-9]+$" OR DIGI_HOLDOFF_MS GREATER 5000)
    message(FATAL_ERROR "DIGI_PATH must fit 31 bytes and DIGI_HOLDOFF_MS must be 0..5000")
endif()
if(NOT DIGI_MYCALL STREQUAL "" AND NOT DIGI_MYCALL MATCHES "^[A-Z0-9][A-Z0-9]?[A-Z0-9]?[A-Z0-9]?[A-Z0-9]?[A-Z0-9]?(-([0-9]|1[0-5]))?$")
    message(FATAL_ERROR "DIGI_MYCALL must be an AX.25 callsign with optional SSID 0..15")
endif()
if(PICO_TNC_STANDALONE AND (NOT DIGI_ENABLE OR DIGI_MYCALL STREQUAL "" OR DIGI_MYCALL MATCHES "^NOCALL($|-)"))
    message(FATAL_ERROR "Standalone digipeater requires DIGI_ENABLE=ON and a configured DIGI_MYCALL")
endif()

add_executable(pico_tnc
        main.c
        tnc.c
        send.c
        receive.c
        decode.c
        filter.c
        bell202.c
        ax25.c
        test.c
        packet_table.c
        usb_input.c
        usb_output.c
        cmd.c
        serial.c
        tty.c
        flash.c
        gps.c
        unproto.c
        digipeat.c
        beacon.c
        kiss.c
        kiss_stream.c
        )

pico_generate_pio_header(pico_tnc ${CMAKE_CURRENT_LIST_DIR}/pio_dac.pio)

target_include_directories(pico_tnc PRIVATE
        include
)

target_link_libraries(pico_tnc
		pico_stdlib
        #pico_stdio_uart
        #pico_stdio_usb
		hardware_adc
		hardware_dma
        hardware_pwm
        hardware_uart
		# For the dummy output:
		hardware_pio
		#pico_multicore
		)

pico_enable_stdio_usb(pico_tnc 1)
pico_enable_stdio_uart(pico_tnc 0)

include(${CMAKE_CURRENT_LIST_DIR}/pinout.cmake)

target_compile_definitions(pico_tnc PRIVATE
    PICO_TNC_PARENT_INTEGRATION=0
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=$<BOOL:${PICO_TNC_USB_BOOTSEL_RESET}>
    PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=$<BOOL:${PICO_TNC_USB_BOOTSEL_RESET}>
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
    PICO_STDIO_DEADLOCK_TIMEOUT_MS=0
    PICO_STACK_SIZE=0x1000
    DIGI_ENABLE=$<BOOL:${DIGI_ENABLE}>
    DIGI_MYCALL="${DIGI_MYCALL}"
    DIGI_PATH="${DIGI_PATH}"
    DIGI_HOLDOFF_MS=${DIGI_HOLDOFF_MS}
    PICO_TNC_STANDALONE=$<BOOL:${PICO_TNC_STANDALONE}>
    ${PICO_TNC_PIN_COMPILE_DEFINITIONS}
)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_tnc)

# add url via pico_set_program_url
example_auto_set_url(pico_tnc)

target_compile_options(pico_tnc PRIVATE -fstack-usage -Wall -Wextra -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>:-Wno-pointer-sign;-Wno-duplicate-decl-specifier>)
