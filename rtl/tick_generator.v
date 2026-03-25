//=============================================================================
// Module: tick_generator
// Description: Generates a 1-clock-wide pulse at a configurable rate.
//              Used to set the PID loop frequency.
// Target: Xilinx Kintex-7 (KC705)
//
// Example: CLK_FREQ=200MHz, tick_divider=200000 => 1 kHz loop rate
//          CLK_FREQ=200MHz, tick_divider=20000  => 10 kHz loop rate
//          CLK_FREQ=200MHz, tick_divider=4000   => 50 kHz loop rate
//=============================================================================

module tick_generator #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [31:0] tick_divider,  // Number of clk cycles between ticks
    output reg         tick,
    output reg  [31:0] tick_count     // Total ticks since enable (for monitoring)
);

    reg [31:0] counter;

    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            counter    <= 0;
            tick       <= 1'b0;
            tick_count <= 0;
        end else begin
            tick <= 1'b0;
            if (tick_divider == 0) begin
                // Divider of 0 means no ticks (safety)
                tick <= 1'b0;
            end else if (counter >= tick_divider - 1) begin
                counter    <= 0;
                tick       <= 1'b1;
                tick_count <= tick_count + 1;
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule
