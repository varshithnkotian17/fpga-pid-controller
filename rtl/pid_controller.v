//=============================================================================
// Module: pid_controller  (v3 - production)
// Description: Fixed-point PID with anti-windup
// Target: Kintex-7 (KC705), Vivado 2023.2
//
// Key design: integral accumulates Ki*error each tick (not raw error).
//   - If Ki=0, integral is always 0 (no phantom accumulation)
//   - Integral is in output units ? easy to clamp
//   - No giant-number � tiny-Ki overflow
//
// Pipeline: IDLE ? CALC ? DONE  (2 active cycles per tick)
// All state fully cleared on reset or disable.
//
// Q16.16 signed fixed-point: 16-bit integer + 16-bit fraction
//=============================================================================

`timescale 1ns / 1ps

module pid_controller (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        tick,

    input  wire [31:0] setpoint,
    input  wire [31:0] process_var,
    input  wire [31:0] kp,
    input  wire [31:0] ki,
    input  wire [31:0] kd,
    input  wire [31:0] out_min,
    input  wire [31:0] out_max,

    output reg  [31:0] pid_out,
    output reg         pid_done,
    output reg  [31:0] error_out,
    output reg         saturated
);

    // FSM states
    localparam S_IDLE = 2'd0,
               S_CALC = 2'd1,
               S_DONE = 2'd2;
    reg [1:0] state;

    // Signed input aliases (continuous)
    wire signed [31:0] s_sp  = $signed(setpoint);
    wire signed [31:0] s_pv  = $signed(process_var);
    wire signed [31:0] s_kp  = $signed(kp);
    wire signed [31:0] s_ki  = $signed(ki);
    wire signed [31:0] s_kd  = $signed(kd);
    wire signed [31:0] s_min = $signed(out_min);
    wire signed [31:0] s_max = $signed(out_max);

    // Core PID state
    reg signed [31:0] error_prev;
    reg signed [31:0] integral;     // In output units (accumulates Ki*e)

    // Pipeline registers (computed in S_IDLE, used in S_CALC)
    reg signed [31:0] p_term;       // Kp * error
    reg signed [31:0] i_update;     // Ki * error (to be added to integral)
    reg signed [31:0] d_term;       // Kd * delta_error
    reg signed [31:0] cur_error;    // Current error for this tick

    // Sum with headroom
    reg signed [33:0] sum;

    // Signed 34-bit extensions of output limits (for correct signed comparison)
    wire signed [33:0] sum_max = {{2{s_max[31]}}, s_max};
    wire signed [33:0] sum_min = {{2{s_min[31]}}, s_min};

    // Anti-windup signals
    wire sat_high = saturated && ($signed(pid_out) == s_max);
    wire sat_low  = saturated && ($signed(pid_out) == s_min);

    // New integral value (combinational, used in S_CALC)
    reg signed [31:0] new_int;      // Computed combinationally in S_CALC

    //=========================================================================
    // Multiply helper: Q16.16 � Q16.16 ? Q16.16
    // Product is 64-bit (Q32.32), arithmetic right shift by 16 ? Q16.16
    //=========================================================================
    wire signed [31:0] err_now = s_sp - s_pv;

    wire signed [63:0] mul_p = s_kp * err_now;
    wire signed [63:0] mul_i = s_ki * err_now;
    wire signed [63:0] mul_d = s_kd * (err_now - error_prev);

    wire signed [31:0] p_val = mul_p >>> 16;
    wire signed [31:0] i_val = mul_i >>> 16;
    wire signed [31:0] d_val = mul_d >>> 16;

    //=========================================================================
    // Main FSM
    //=========================================================================
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            // Full clear - nothing survives
            state      <= S_IDLE;
            error_prev <= 0;
            integral   <= 0;
            p_term     <= 0;
            i_update   <= 0;
            d_term     <= 0;
            cur_error  <= 0;
            sum        <= 0;
            pid_out    <= 0;
            pid_done   <= 1'b0;
            error_out  <= 0;
            saturated  <= 1'b0;
        end else begin
            pid_done <= 1'b0;

            case (state)

            //--- IDLE: wait for tick, latch multiply results ---
            S_IDLE: begin
                if (tick) begin
                    cur_error  <= err_now;
                    error_out  <= err_now;
                    error_prev <= err_now;    // Save for next tick's derivative

                    p_term   <= p_val;        // Kp * error
                    i_update <= i_val;        // Ki * error (add to integral next)
                    d_term   <= d_val;        // Kd * (error - error_prev)

                    state <= S_CALC;
                end
            end

            //--- CALC: update integral with anti-windup, compute sum ---
            S_CALC: begin
                // Anti-windup logic:
                // Block integral accumulation if output is saturated
                // and the update would push further into saturation.
                if (sat_high && i_update > 0)
                    new_int = integral;             // Saturated high, positive update ? block
                else if (sat_low && i_update < 0)
                    new_int = integral;             // Saturated low, negative update ? block
                else begin
                    // Accumulate and clamp integral to output range
                    if (integral + i_update > s_max)
                        new_int = s_max;
                    else if (integral + i_update < s_min)
                        new_int = s_min;
                    else
                        new_int = integral + i_update;
                end

                integral <= new_int;

                // PID sum = P + I + D
                sum <= {{2{p_term[31]}}, p_term}
                     + {{2{new_int[31]}}, new_int}
                     + {{2{d_term[31]}}, d_term};

                state <= S_DONE;
            end

            //--- DONE: clamp output, assert pid_done ---
            S_DONE: begin
                if (sum > sum_max) begin
                    pid_out   <= out_max;
                    saturated <= 1'b1;
                end else if (sum < sum_min) begin
                    pid_out   <= out_min;
                    saturated <= 1'b1;
                end else begin
                    pid_out   <= sum[31:0];
                    saturated <= 1'b0;
                end
                pid_done <= 1'b1;
                state    <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule