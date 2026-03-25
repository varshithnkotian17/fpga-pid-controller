//=============================================================================
// Module: pid_axi_wrapper
// Description: AXI4-Lite slave interface to the PID controller.
//              MicroBlaze writes gains, setpoint, limits via AXI registers.
//              MicroBlaze reads error, output, status.
//              Also includes tick generator and a simple test plant model.
//
// Register Map (byte addresses, 32-bit registers):
//   0x00  CTRL      - [0]=enable, [1]=clear_integral, [2]=use_test_plant
//   0x04  STATUS    - [0]=pid_done, [1]=saturated, [31:16]=state
//   0x08  KP        - Proportional gain (Q16.16)
//   0x0C  KI        - Integral gain (Q16.16)
//   0x10  KD        - Derivative gain (Q16.16)
//   0x14  SETPOINT  - Target value (Q16.16)
//   0x18  PROC_VAR  - Process variable input (Q16.16) - write for manual, read for actual
//   0x1C  PID_OUT   - Controller output (Q16.16) [read-only]
//   0x20  ERROR     - Current error (Q16.16) [read-only]
//   0x24  OUT_MIN   - Minimum output limit (Q16.16)
//   0x28  OUT_MAX   - Maximum output limit (Q16.16)
//   0x2C  TICK_DIV  - Tick divider (loop rate = 200MHz / TICK_DIV)
//   0x30  TICK_CNT  - Tick counter [read-only]
//   0x34  PLANT_OUT - Test plant output [read-only] (Q16.16)
//
// Target: Xilinx Kintex-7 (KC705) with Vivado 2023.2
//=============================================================================

module pid_axi_wrapper #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 8
)(
    // AXI4-Lite Slave Interface
    input  wire                                s_axi_aclk,
    input  wire                                s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [2:0]                          s_axi_awprot,
    input  wire                                s_axi_awvalid,
    output wire                                s_axi_awready,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   s_axi_wstrb,
    input  wire                                s_axi_wvalid,
    output wire                                s_axi_wready,

    output wire [1:0]                          s_axi_bresp,
    output wire                                s_axi_bvalid,
    input  wire                                s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [2:0]                          s_axi_arprot,
    input  wire                                s_axi_arvalid,
    output wire                                s_axi_arready,

    output wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                          s_axi_rresp,
    output wire                                s_axi_rvalid,
    input  wire                                s_axi_rready,

    // External process variable input (from real sensor / UART)
    input  wire [31:0]                         ext_process_var,
    input  wire                                ext_pv_valid,

    // PID output for external use (to DAC / UART TX)
    output wire [31:0]                         ext_pid_out,
    output wire                                ext_pid_done,

    // Debug LEDs
    output wire [7:0]                          led_out
);

    //=========================================================================
    // AXI4-Lite write/read logic
    //=========================================================================
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg        axi_awready;
    reg        axi_wready;
    reg [1:0]  axi_bresp;
    reg        axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    reg        axi_arready;
    reg [31:0] axi_rdata;
    reg [1:0]  axi_rresp;
    reg        axi_rvalid;

    assign s_axi_awready = axi_awready;
    assign s_axi_wready  = axi_wready;
    assign s_axi_bresp   = axi_bresp;
    assign s_axi_bvalid  = axi_bvalid;
    assign s_axi_arready = axi_arready;
    assign s_axi_rdata   = axi_rdata;
    assign s_axi_rresp   = axi_rresp;
    assign s_axi_rvalid  = axi_rvalid;

    //=========================================================================
    // Configuration registers
    //=========================================================================
    reg [31:0] reg_ctrl;        // 0x00
    reg [31:0] reg_kp;          // 0x08
    reg [31:0] reg_ki;          // 0x0C
    reg [31:0] reg_kd;          // 0x10
    reg [31:0] reg_setpoint;    // 0x14
    reg [31:0] reg_proc_var;    // 0x18 (manual input mode)
    reg [31:0] reg_out_min;     // 0x24
    reg [31:0] reg_out_max;     // 0x28
    reg [31:0] reg_tick_div;    // 0x2C

    // Derived control signals
    wire pid_enable       = reg_ctrl[0];
    wire clear_integral   = reg_ctrl[1];
    wire use_test_plant   = reg_ctrl[2];

    //=========================================================================
    // PID Controller instantiation
    //=========================================================================
    wire [31:0] pid_out_w;
    wire        pid_done_w;
    wire [31:0] error_out_w;
    wire        saturated_w;
    wire        tick_w;
    wire [31:0] tick_count_w;

    // Forward declaration for test plant output (logic defined below)
    reg signed [31:0] plant_output;

    // Select process variable source
    wire [31:0] actual_pv = use_test_plant ? plant_output :
                            ext_pv_valid   ? ext_process_var :
                                             reg_proc_var;

    tick_generator #(
        .CLK_FREQ(100_000_000)
    ) u_tick (
        .clk          (s_axi_aclk),
        .rst_n        (s_axi_aresetn),
        .enable       (pid_enable),
        .tick_divider (reg_tick_div),
        .tick         (tick_w),
        .tick_count   (tick_count_w)
    );

    pid_controller u_pid (
        .clk         (s_axi_aclk),
        .rst_n       (s_axi_aresetn & ~clear_integral),
        .enable      (pid_enable),
        .tick        (tick_w),
        .setpoint    (reg_setpoint),
        .process_var (actual_pv),
        .kp          (reg_kp),
        .ki          (reg_ki),
        .kd          (reg_kd),
        .out_min     (reg_out_min),
        .out_max     (reg_out_max),
        .pid_out     (pid_out_w),
        .pid_done    (pid_done_w),
        .error_out   (error_out_w),
        .saturated   (saturated_w)
    );

    assign ext_pid_out  = pid_out_w;
    assign ext_pid_done = pid_done_w;

    //=========================================================================
    // Built-in test plant (first-order system)
    // Models: plant_output += (pid_output - plant_output) >> PLANT_SHIFT
    // This is a simple RC-like response for testing the PID without hardware.
    //=========================================================================
    localparam PLANT_SHIFT = 4; // Time constant = 2^4 = 16 ticks

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || !pid_enable) begin
            plant_output <= 0;
        end else if (pid_done_w && use_test_plant) begin
            // First-order lag: y += (u - y) / 16
            plant_output <= plant_output + 
                (($signed(pid_out_w) - plant_output) >>> PLANT_SHIFT);
        end
    end

    //=========================================================================
    // LED output: simple activity indicators
    //=========================================================================
    assign led_out[0] = pid_enable;
    assign led_out[1] = saturated_w;
    assign led_out[2] = (error_out_w[31]) ? 1'b1 : 1'b0; // Negative error
    assign led_out[3] = tick_w;                             // Blinks at loop rate
    assign led_out[7:4] = pid_out_w[19:16];                // Upper bits of output

    //=========================================================================
    // AXI4-Lite Write Logic
    //=========================================================================
    wire aw_en;
    reg  aw_en_r;

    assign aw_en = ~axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en_r;

    // Write address ready
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_awready <= 1'b0;
            aw_en_r     <= 1'b1;
        end else begin
            if (aw_en) begin
                axi_awready <= 1'b1;
                aw_en_r     <= 1'b0;
            end else if (s_axi_bready && axi_bvalid) begin
                axi_awready <= 1'b0;
                aw_en_r     <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    // Latch write address
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            axi_awaddr <= 0;
        else if (aw_en)
            axi_awaddr <= s_axi_awaddr;
    end

    // Write data ready
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            axi_wready <= 1'b0;
        else if (aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end

    // Write to registers
    wire [5:0] wr_addr = axi_awaddr[7:2]; // Word address

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            reg_ctrl     <= 32'h0;
            reg_kp       <= 32'h0001_0000;  // Default Kp = 1.0
            reg_ki       <= 32'h0000_0000;  // Default Ki = 0.0
            reg_kd       <= 32'h0000_0000;  // Default Kd = 0.0
            reg_setpoint <= 32'h0000_0000;  // Default SP = 0.0
            reg_proc_var <= 32'h0000_0000;
            reg_out_min  <= 32'hFFF0_0000;  // -16.0 default min
            reg_out_max  <= 32'h0010_0000;  // +16.0 default max
            reg_tick_div <= 32'd100_000;    // Default 1 kHz loop (100MHz / 100000)
        end else if (axi_wready && s_axi_wvalid && axi_awready && s_axi_awvalid) begin
            case (wr_addr)
                6'd0:  reg_ctrl     <= s_axi_wdata;  // 0x00
                6'd2:  reg_kp       <= s_axi_wdata;  // 0x08
                6'd3:  reg_ki       <= s_axi_wdata;  // 0x0C
                6'd4:  reg_kd       <= s_axi_wdata;  // 0x10
                6'd5:  reg_setpoint <= s_axi_wdata;  // 0x14
                6'd6:  reg_proc_var <= s_axi_wdata;  // 0x18
                6'd9:  reg_out_min  <= s_axi_wdata;  // 0x24
                6'd10: reg_out_max  <= s_axi_wdata;  // 0x28
                6'd11: reg_tick_div <= s_axi_wdata;  // 0x2C
                default: ;
            endcase
        end
    end

    // Write response
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin
            if (axi_awready && s_axi_awvalid && axi_wready && s_axi_wvalid && ~axi_bvalid) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00; // OKAY
            end else if (s_axi_bready && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    //=========================================================================
    // AXI4-Lite Read Logic
    //=========================================================================

    // Read address ready
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 0;
        end else begin
            if (~axi_arready && s_axi_arvalid) begin
                axi_arready <= 1'b1;
                axi_araddr  <= s_axi_araddr;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    // Read data
    wire [5:0] rd_addr = axi_araddr[7:2]; // Word address

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b00;
            axi_rdata  <= 0;
        end else begin
            if (axi_arready && s_axi_arvalid && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
                case (rd_addr)
                    6'd0:  axi_rdata <= reg_ctrl;
                    6'd1:  axi_rdata <= {16'd0, saturated_w, pid_done_w}; // STATUS
                    6'd2:  axi_rdata <= reg_kp;
                    6'd3:  axi_rdata <= reg_ki;
                    6'd4:  axi_rdata <= reg_kd;
                    6'd5:  axi_rdata <= reg_setpoint;
                    6'd6:  axi_rdata <= actual_pv;     // Current process variable
                    6'd7:  axi_rdata <= pid_out_w;     // PID output
                    6'd8:  axi_rdata <= error_out_w;   // Error
                    6'd9:  axi_rdata <= reg_out_min;
                    6'd10: axi_rdata <= reg_out_max;
                    6'd11: axi_rdata <= reg_tick_div;
                    6'd12: axi_rdata <= tick_count_w;  // Tick counter
                    6'd13: axi_rdata <= plant_output;  // Test plant output
                    default: axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end else if (axi_rvalid && s_axi_rready) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
