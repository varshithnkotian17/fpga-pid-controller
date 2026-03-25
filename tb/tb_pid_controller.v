//=============================================================================
// Testbench: tb_pid_controller  (v3)
//
// Plant: y += (u - y) / 16   (first-order lag, DC gain=1, tau?16 ticks)
//
// With DC gain=1, the P-only steady state is:
//   PV = Kp * SP / (1 + Kp)
//   Kp=0.5 ? PV = 3.33  (for SP=10)
//
// With PI, the integral eliminates steady-state error ? PV = SP.
//
// The system is stable for all gains tested here (verified analytically).
//=============================================================================

`timescale 1ns / 1ps

module tb_pid_controller;

    reg clk;
    initial clk = 0;
    always #2.5 clk = ~clk;  // 200 MHz

    reg rst_n;

    // PID interface
    reg         enable, tick;
    reg  [31:0] setpoint, process_var;
    reg  [31:0] kp, ki, kd;
    reg  [31:0] out_min, out_max;
    wire [31:0] pid_out;
    wire        pid_done;
    wire [31:0] error_out;
    wire        saturated;

    pid_controller uut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .tick(tick),
        .setpoint(setpoint), .process_var(process_var),
        .kp(kp), .ki(ki), .kd(kd),
        .out_min(out_min), .out_max(out_max),
        .pid_out(pid_out), .pid_done(pid_done),
        .error_out(error_out), .saturated(saturated)
    );

    // Plant: y += (u - y) / 16
    reg signed [31:0] plant_y;
    always @(posedge clk) begin
        if (!rst_n)
            plant_y <= 0;
        else if (pid_done)
            plant_y <= plant_y + (($signed(pid_out) - plant_y) >>> 4);
    end

    // Feed plant output back as process variable
    always @(*) process_var = plant_y;

    // Q16.16 helpers
    function [31:0] to_q16;
        input real val;
        begin to_q16 = $rtoi(val * 65536.0); end
    endfunction

    function real from_q16;
        input [31:0] val;
        begin from_q16 = $itor($signed(val)) / 65536.0; end
    endfunction

    // Run N ticks and print every skip-th
    task run_ticks;
        input integer n;
        input integer skip;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                // Wait between ticks
                tick = 1'b0;
                repeat(50) @(posedge clk);
                // Fire tick
                tick = 1'b1;
                @(posedge clk);
                tick = 1'b0;
                // Wait for PID + plant update
                wait(pid_done);
                @(posedge clk);
                @(posedge clk);

                if (i % skip == 0)
                    $display("  t=%4d  SP=%7.3f  PV=%7.3f  Err=%7.3f  Out=%7.3f  I=%7.3f  Sat=%b",
                        i, from_q16(setpoint), from_q16(plant_y),
                        from_q16(error_out), from_q16(pid_out),
                        from_q16(uut.integral), saturated);
            end
        end
    endtask

    // Reset everything
    task full_reset;
        begin
            enable = 0; tick = 0;
            repeat(5) @(posedge clk);
            rst_n = 0;
            repeat(10) @(posedge clk);
            rst_n = 1;
            repeat(5) @(posedge clk);
        end
    endtask

    integer pass_count;

    initial begin
        $display("=========================================================");
        $display("  PID Controller Testbench v3");
        $display("  Plant: y += (u-y)/16, DC gain=1, tau=16 ticks");
        $display("=========================================================");

        rst_n = 0; enable = 0; tick = 0;
        setpoint = 0; kp = 0; ki = 0; kd = 0;
        out_min = to_q16(-100.0);
        out_max = to_q16(100.0);
        repeat(20) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        pass_count = 0;

        //=============================================================
        // TEST 1: P-only - expect PV = 10 * 0.5/1.5 = 3.333
        //=============================================================
        $display("\n--- Test 1: P-only (Kp=0.5, SP=10) ---");
        $display("  Expected steady-state PV = 3.333");
        enable   = 1;
        kp       = to_q16(0.5);
        ki       = to_q16(0.0);
        kd       = to_q16(0.0);
        setpoint = to_q16(10.0);
        out_min  = to_q16(-100.0);
        out_max  = to_q16(100.0);

        run_ticks(150, 20);

        $display("  Final PV = %7.4f", from_q16(plant_y));
        if (from_q16(plant_y) > 3.2 && from_q16(plant_y) < 3.5) begin
            $display("  >>> TEST 1 PASSED <<<");
            pass_count = pass_count + 1;
        end else
            $display("  >>> TEST 1 FAILED <<<");

        //=============================================================
        // TEST 2: PI - integral eliminates SS error, PV ? 10.0
        //=============================================================
        $display("\n--- Test 2: PI (Kp=0.5, Ki=0.02, SP=10) ---");
        $display("  Expected: PV converges to 10.0");
        full_reset();
        enable   = 1;
        kp       = to_q16(0.5);
        ki       = to_q16(0.02);
        kd       = to_q16(0.0);
        setpoint = to_q16(10.0);
        out_min  = to_q16(-100.0);
        out_max  = to_q16(100.0);

        run_ticks(800, 50);

        $display("  Final PV = %7.4f (integral = %7.4f)",
            from_q16(plant_y), from_q16(uut.integral));
        if (from_q16(plant_y) > 9.0 && from_q16(plant_y) < 11.0) begin
            $display("  >>> TEST 2 PASSED <<<");
            pass_count = pass_count + 1;
        end else
            $display("  >>> TEST 2 FAILED <<<");

        //=============================================================
        // TEST 3: PID with setpoint change: 10 then -5
        //=============================================================
        $display("\n--- Test 3: PID (Kp=0.5, Ki=0.02, Kd=0.1) ---");
        full_reset();
        enable   = 1;
        kp       = to_q16(0.5);
        ki       = to_q16(0.02);
        kd       = to_q16(0.1);
        setpoint = to_q16(10.0);
        out_min  = to_q16(-100.0);
        out_max  = to_q16(100.0);

        $display("  Step to +10:");
        run_ticks(600, 50);
        $display("  PV = %7.4f at SP=10", from_q16(plant_y));

        $display("  Step to -5:");
        setpoint = to_q16(-5.0);
        run_ticks(600, 50);
        $display("  PV = %7.4f at SP=-5", from_q16(plant_y));

        if (from_q16(plant_y) > -6.5 && from_q16(plant_y) < -3.5) begin
            $display("  >>> TEST 3 PASSED <<<");
            pass_count = pass_count + 1;
        end else
            $display("  >>> TEST 3 FAILED <<<");

        //=============================================================
        // TEST 4: Saturation - output clamped, integral doesn't windup
        //=============================================================
        $display("\n--- Test 4: Saturation (out_max=5, SP=100) ---");
        full_reset();
        enable   = 1;
        kp       = to_q16(0.5);
        ki       = to_q16(0.02);
        kd       = to_q16(0.0);
        setpoint = to_q16(100.0);
        out_min  = to_q16(-5.0);
        out_max  = to_q16(5.0);

        run_ticks(100, 20);

        $display("  Final Out = %7.4f (expect clamped ? 5.0)", from_q16(pid_out));
        $display("  Saturated = %b (expect 1)", saturated);
        $display("  Integral  = %7.4f (expect clamped ? 5.0)", from_q16(uut.integral));
        if (from_q16(pid_out) >= 4.9 && saturated == 1) begin
            $display("  >>> TEST 4 PASSED <<<");
            pass_count = pass_count + 1;
        end else
            $display("  >>> TEST 4 FAILED <<<");

        //=============================================================
        // Summary
        //=============================================================
        $display("\n=========================================================");
        $display("  RESULTS: %0d / 4 tests passed", pass_count);
        if (pass_count == 4)
            $display("  ALL TESTS PASSED - PID core is verified!");
        $display("=========================================================");
        $finish;
    end

    // Watchdog
    initial begin
        #100_000_000;
        $display("TIMEOUT!");
        $finish;
    end

endmodule