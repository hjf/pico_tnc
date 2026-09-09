set(PICO_TNC_PINOUT "default" CACHE STRING "pico_tnc pinout profile (default or gps-gp5)")
set_property(CACHE PICO_TNC_PINOUT PROPERTY STRINGS default gps-gp5)

if(PICO_TNC_PINOUT STREQUAL "default")
    set(_pico_tnc_gps_rx_default 5)
    set(_pico_tnc_ptt_default 11)
    set(_pico_tnc_pwm_default 10)
    set(_pico_tnc_terminal_uart_default ON)
elseif(PICO_TNC_PINOUT STREQUAL "gps-gp5")
    set(_pico_tnc_gps_rx_default 5)
    set(_pico_tnc_ptt_default 1)
    set(_pico_tnc_pwm_default 4)
    set(_pico_tnc_terminal_uart_default OFF)
else()
    message(FATAL_ERROR "Unknown PICO_TNC_PINOUT '${PICO_TNC_PINOUT}'")
endif()

set(PICO_TNC_GPS_RX_PIN "${_pico_tnc_gps_rx_default}" CACHE STRING "GPS receive GPIO")
set(PICO_TNC_PTT_PIN "${_pico_tnc_ptt_default}" CACHE STRING "Radio PTT GPIO")
set(PICO_TNC_PWM_PIN "${_pico_tnc_pwm_default}" CACHE STRING "Radio audio output GPIO")
set(PICO_TNC_TERMINAL_UART_ENABLE "${_pico_tnc_terminal_uart_default}" CACHE BOOL
    "Enable the 115200-baud terminal on UART0 GPIO0/GPIO1")

foreach(_pin_var PICO_TNC_GPS_RX_PIN PICO_TNC_PTT_PIN PICO_TNC_PWM_PIN)
    if(NOT ${_pin_var} MATCHES "^[0-9]+$" OR ${${_pin_var}} GREATER 29)
        message(FATAL_ERROR "${_pin_var} must be an RP2040 GPIO number from 0 through 29")
    endif()
endforeach()

set(_pico_tnc_uart0_rx_pins 1 13 17 29)
set(_pico_tnc_uart1_rx_pins 5 9 21 25)
if(PICO_TNC_GPS_RX_PIN IN_LIST _pico_tnc_uart0_rx_pins)
    set(PICO_TNC_GPS_UART_ID 0)
elseif(PICO_TNC_GPS_RX_PIN IN_LIST _pico_tnc_uart1_rx_pins)
    set(PICO_TNC_GPS_UART_ID 1)
else()
    message(FATAL_ERROR
        "PICO_TNC_GPS_RX_PIN=${PICO_TNC_GPS_RX_PIN} is not a hardware UART RX pin; use 1, 5, 9, 13, 17, 21, 25, or 29")
endif()

if(PICO_TNC_GPS_RX_PIN STREQUAL PICO_TNC_PTT_PIN OR
   PICO_TNC_GPS_RX_PIN STREQUAL PICO_TNC_PWM_PIN OR
   PICO_TNC_PTT_PIN STREQUAL PICO_TNC_PWM_PIN)
    message(FATAL_ERROR "GPS RX, PTT, and PWM output must use different GPIOs")
endif()

if(PICO_TNC_TERMINAL_UART_ENABLE AND
   (PICO_TNC_GPS_RX_PIN LESS 2 OR PICO_TNC_PTT_PIN LESS 2 OR PICO_TNC_PWM_PIN LESS 2))
    message(FATAL_ERROR "GPIO0/GPIO1 are reserved by the terminal UART; disable PICO_TNC_TERMINAL_UART_ENABLE for this pinout")
endif()

if(PICO_TNC_TERMINAL_UART_ENABLE AND PICO_TNC_GPS_UART_ID EQUAL 0)
    message(FATAL_ERROR "UART0 is reserved by the terminal; use a UART1 GPS RX pin or disable PICO_TNC_TERMINAL_UART_ENABLE")
endif()

set(PICO_TNC_PIN_COMPILE_DEFINITIONS
    PICO_TNC_GPS_RX_PIN=${PICO_TNC_GPS_RX_PIN}
    PICO_TNC_GPS_UART_ID=${PICO_TNC_GPS_UART_ID}
    PICO_TNC_PTT_PIN=${PICO_TNC_PTT_PIN}
    PICO_TNC_PWM_PIN=${PICO_TNC_PWM_PIN}
    PICO_TNC_TERMINAL_UART_ENABLE=$<BOOL:${PICO_TNC_TERMINAL_UART_ENABLE}>
)