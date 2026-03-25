#==============================================================================
# Vivado TCL Build Script
# Project: FPGA PID Controller on KC705
# Tool: Vivado 2023.2
#
# Usage:
#   1. Open Vivado 2023.2
#   2. In TCL console: source E:/PID_KC705_RESOURCES/fpga_pid_controller/scripts/build_project.tcl
#   OR from command line:
#     vivado -mode batch -source E:/PID_KC705_RESOURCES/fpga_pid_controller/scripts/build_project.tcl
#
# Creates a complete MicroBlaze SoC with:
#   - Clocking Wizard (200MHz differential input -> 100MHz output)
#   - MicroBlaze processor with 64KB local memory + debug module
#   - AXI UARTLite at 115200 baud (KC705 USB-UART)
#   - AXI GPIO for 8 LEDs
#   - PID controller custom AXI peripheral (pid_axi_wrapper)
#==============================================================================

# ==== Configuration ====
set project_name "pid_kc705"
set project_dir  "E:/Varshith_projects/PID_KC705"
set src_dir      "E:/PID_KC705_RESOURCES/fpga_pid_controller"
set part         "xc7k325tffg900-2"
set board        "xilinx.com:kc705:part0:1.6"
set design_name  "pid_system"

# ==== Clean & Create Project ====
puts "============================================"
puts " Creating project: $project_name"
puts " Location: $project_dir"
puts "============================================"

if {[file exists $project_dir]} {
    file delete -force $project_dir
}
create_project $project_name $project_dir -part $part
set_property board_part $board [current_project]

# ==== Add RTL Sources ====
add_files -norecurse [list \
    $src_dir/rtl/pid_controller.v \
    $src_dir/rtl/tick_generator.v \
    $src_dir/rtl/pid_axi_wrapper.v \
]
update_compile_order -fileset sources_1

# ==== Add Simulation Sources ====
add_files -fileset sim_1 -norecurse $src_dir/tb/tb_pid_controller.v
set_property top tb_pid_controller [get_filesets sim_1]

# ==== Add Constraints ====
add_files -fileset constrs_1 -norecurse $src_dir/constraints/kc705_pid.xdc

# ==== Create Block Design ====
create_bd_design $design_name

#----------------------------------------------------------------------
# Step 1: Clocking Wizard — 200MHz differential input -> 100MHz output
#----------------------------------------------------------------------
puts "Adding Clocking Wizard..."
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clk_wiz_0
set_property -dict [list \
    CONFIG.PRIM_SOURCE {Differential_clock_capable_pin} \
    CONFIG.PRIM_IN_FREQ {200.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {100.000} \
    CONFIG.USE_LOCKED {true} \
    CONFIG.USE_RESET {true} \
    CONFIG.RESET_TYPE {ACTIVE_HIGH} \
    CONFIG.RESET_PORT {reset} \
] [get_bd_cells clk_wiz_0]

# Connect the KC705 200MHz differential clock via board automation
apply_board_connection -board_interface "sys_diff_clock" \
    -ip_intf "clk_wiz_0/CLK_IN1_D" -diagram $design_name

#----------------------------------------------------------------------
# Step 2: MicroBlaze with 64KB local memory + debug module
#----------------------------------------------------------------------
puts "Adding MicroBlaze..."
create_bd_cell -type ip -vlnv xilinx.com:ip:microblaze:11.0 microblaze_0

# Apply board automation: creates local BRAM, AXI interconnect,
# proc_sys_reset (rst_clk_wiz_0_100M), and connects everything
apply_bd_automation -rule xilinx.com:bd_rule:microblaze \
    -config { \
        axi_intc "1" \
        axi_periph "Enabled" \
        cache "None" \
        clk "/clk_wiz_0/clk_out1" \
        debug_module "Debug Only" \
        ecc "None" \
        local_mem "64KB" \
        preset "None" \
    } [get_bd_cells microblaze_0]

#----------------------------------------------------------------------
# Step 3: External reset button
#----------------------------------------------------------------------
puts "Connecting reset..."
create_bd_port -dir I -type rst reset
set_property CONFIG.POLARITY ACTIVE_HIGH [get_bd_ports reset]

# Connect to the proc_sys_reset created by MicroBlaze automation
connect_bd_net [get_bd_ports reset] [get_bd_pins rst_clk_wiz_0_100M/ext_reset_in]

# Also connect to the Clocking Wizard reset (active-high)
connect_bd_net [get_bd_ports reset] [get_bd_pins clk_wiz_0/reset]

#----------------------------------------------------------------------
# Step 4: AXI UARTLite — 115200 baud
#----------------------------------------------------------------------
puts "Adding AXI UARTLite..."
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uartlite:2.0 axi_uartlite_0
set_property -dict [list \
    CONFIG.C_BAUDRATE {115200} \
    CONFIG.C_DATA_BITS {8} \
] [get_bd_cells axi_uartlite_0]

# Connect to MicroBlaze AXI bus
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config { \
        Clk_master "Auto" \
        Clk_slave "Auto" \
        Clk_xbar "Auto" \
        Master "/microblaze_0 (Periph)" \
        Slave "/axi_uartlite_0/S_AXI" \
        intc_ip "Auto" \
        master_apm "0" \
    } [get_bd_intf_pins axi_uartlite_0/S_AXI]

# Make UART pins external and rename to match XDC
make_bd_pins_external [get_bd_pins axi_uartlite_0/rx]
make_bd_pins_external [get_bd_pins axi_uartlite_0/tx]
set_property name usb_uart_rxd [get_bd_ports rx_0]
set_property name usb_uart_txd [get_bd_ports tx_0]

#----------------------------------------------------------------------
# Step 5: AXI GPIO — 8-bit output for LEDs
#----------------------------------------------------------------------
puts "Adding AXI GPIO for LEDs..."
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_leds
set_property -dict [list \
    CONFIG.C_GPIO_WIDTH {8} \
    CONFIG.C_ALL_OUTPUTS {1} \
] [get_bd_cells axi_gpio_leds]

# Connect to MicroBlaze AXI bus
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config { \
        Clk_master "Auto" \
        Clk_slave "Auto" \
        Clk_xbar "Auto" \
        Master "/microblaze_0 (Periph)" \
        Slave "/axi_gpio_leds/S_AXI" \
        intc_ip "Auto" \
        master_apm "0" \
    } [get_bd_intf_pins axi_gpio_leds/S_AXI]

# Make GPIO interface external and rename to match XDC
make_bd_intf_pins_external [get_bd_intf_pins axi_gpio_leds/GPIO]
set_property name gpio_leds [get_bd_intf_ports GPIO_0]

#----------------------------------------------------------------------
# Step 6: PID Controller — custom AXI peripheral (RTL module reference)
#----------------------------------------------------------------------
puts "Adding PID AXI wrapper..."
create_bd_cell -type module -reference pid_axi_wrapper pid_axi_0

# Connect to MicroBlaze AXI bus
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config { \
        Clk_master "Auto" \
        Clk_slave "Auto" \
        Clk_xbar "Auto" \
        Master "/microblaze_0 (Periph)" \
        Slave "/pid_axi_0/s_axi" \
        intc_ip "Auto" \
        master_apm "0" \
    } [get_bd_intf_pins pid_axi_0/s_axi]

# Tie unused input ports to constants
# ext_process_var[31:0] = 0
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_zero_32
set_property -dict [list CONFIG.CONST_WIDTH {32} CONFIG.CONST_VAL {0}] \
    [get_bd_cells const_zero_32]
connect_bd_net [get_bd_pins const_zero_32/dout] \
    [get_bd_pins pid_axi_0/ext_process_var]

# ext_pv_valid = 0
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_zero_1
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] \
    [get_bd_cells const_zero_1]
connect_bd_net [get_bd_pins const_zero_1/dout] \
    [get_bd_pins pid_axi_0/ext_pv_valid]

# ext_pid_out, ext_pid_done, led_out are outputs — left unconnected
# (PID status is read via register and driven to LEDs by firmware through GPIO)

#----------------------------------------------------------------------
# Step 7: Assign addresses
#----------------------------------------------------------------------
puts "Assigning addresses..."
assign_bd_address

#----------------------------------------------------------------------
# Step 8: Validate and save block design
#----------------------------------------------------------------------
puts "Validating block design..."
validate_bd_design
save_bd_design

#----------------------------------------------------------------------
# Step 9: Generate HDL wrapper and set as top
#----------------------------------------------------------------------
puts "Generating HDL wrapper..."
set bd_file [get_files ${design_name}.bd]
make_wrapper -files $bd_file -top

# Add the generated wrapper as a source
set wrapper_file [file normalize \
    "$project_dir/$project_name.gen/sources_1/bd/$design_name/hdl/${design_name}_wrapper.v"]
add_files -norecurse $wrapper_file
set_property top ${design_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

#----------------------------------------------------------------------
# Done
#----------------------------------------------------------------------
puts ""
puts "============================================"
puts " Project created successfully!"
puts "============================================"
puts " Project:  $project_dir"
puts " Design:   $design_name"
puts " Clock:    200MHz diff -> 100MHz (Clocking Wizard)"
puts " CPU:      MicroBlaze, 64KB BRAM, Debug Module"
puts " UART:     115200 baud (USB-UART: TX=K24, RX=M19)"
puts " GPIO:     8 LEDs"
puts " PID:      pid_axi_wrapper (AXI4-Lite slave)"
puts ""
puts " Next steps:"
puts "   1. Open project in Vivado GUI"
puts "   2. Run Synthesis (may take ~5 min)"
puts "   3. Run Implementation"
puts "   4. Generate Bitstream"
puts "   5. File -> Export Hardware (include bitstream)"
puts "   6. Open Vitis IDE:"
puts "      a. Create Platform Project from XSA"
puts "      b. Create Application Project (Empty C)"
puts "      c. Add $src_dir/vivado/pid_firmware.c to src/"
puts "      d. Build, Program FPGA, Run"
puts "   7. Connect serial terminal: 115200 8N1"
puts "      Type 'help' or 'demo' to test"
puts "============================================"
