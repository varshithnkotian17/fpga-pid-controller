##=============================================================================
## Constraints: KC705 Board - PID Controller Project
## Target: XC7K325T-2FFG900C
## Board: KC705 Rev 1.1+
##
## NOTE: System clock (200MHz differential) is handled by Vivado board
##       automation (apply_board_connection for sys_diff_clock).
##       Do NOT add clock pin/IOSTANDARD constraints here — they would
##       conflict with the board flow.
##=============================================================================

## --- CPU Reset Button (active high, center of 5-way nav switch) ---
set_property PACKAGE_PIN AB7 [get_ports reset]
set_property IOSTANDARD LVCMOS15 [get_ports reset]

## --- USB UART (CP2103 USB-UART bridge on KC705) ---
## TX from FPGA to PC
set_property PACKAGE_PIN K24 [get_ports usb_uart_txd]
set_property IOSTANDARD LVCMOS25 [get_ports usb_uart_txd]

## RX from PC to FPGA
set_property PACKAGE_PIN M19 [get_ports usb_uart_rxd]
set_property IOSTANDARD LVCMOS25 [get_ports usb_uart_rxd]

## --- GPIO LEDs ---
## LEDs 0-3: Bank 33 (VCCO = 1.5V)
set_property PACKAGE_PIN AB8  [get_ports {gpio_leds_tri_o[0]}]
set_property PACKAGE_PIN AA8  [get_ports {gpio_leds_tri_o[1]}]
set_property PACKAGE_PIN AC9  [get_ports {gpio_leds_tri_o[2]}]
set_property PACKAGE_PIN AB9  [get_ports {gpio_leds_tri_o[3]}]
set_property IOSTANDARD LVCMOS15 [get_ports {gpio_leds_tri_o[0]}]
set_property IOSTANDARD LVCMOS15 [get_ports {gpio_leds_tri_o[1]}]
set_property IOSTANDARD LVCMOS15 [get_ports {gpio_leds_tri_o[2]}]
set_property IOSTANDARD LVCMOS15 [get_ports {gpio_leds_tri_o[3]}]

## LEDs 4-7: Bank 25/26 (VCCO = 2.5V)
set_property PACKAGE_PIN AE26 [get_ports {gpio_leds_tri_o[4]}]
set_property PACKAGE_PIN G19  [get_ports {gpio_leds_tri_o[5]}]
set_property PACKAGE_PIN E18  [get_ports {gpio_leds_tri_o[6]}]
set_property PACKAGE_PIN F16  [get_ports {gpio_leds_tri_o[7]}]
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_leds_tri_o[4]}]
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_leds_tri_o[5]}]
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_leds_tri_o[6]}]
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_leds_tri_o[7]}]

## --- Timing Constraints ---
## UART is slow — no need for tight timing
set_false_path -from [get_ports usb_uart_rxd]
set_false_path -to   [get_ports usb_uart_txd]
set_false_path -to   [get_ports {gpio_leds_tri_o[*]}]

## --- Bitstream Configuration ---
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_VOLTAGE 2.5 [current_design]
set_property CFGBVS VCCO [current_design]
