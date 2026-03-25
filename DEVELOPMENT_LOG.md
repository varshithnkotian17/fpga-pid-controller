# Development Log: FPGA PID Controller

This log documents the bugs found and fixed during development and hardware
bring-up of the PID controller on the KC705 board. Each entry describes the
symptom observed, root-cause analysis, and the fix applied.

---

## Bug 1 — Unsigned concatenation breaks signed output clamping

**File:** `rtl/pid_controller.v`
**Severity:** Critical — all four testbench cases fail

### Symptom

On the very first PID tick with Kp=0.5 and SP=10, the output should be +5.0
but instead saturates to -100.0 (out_min). The P-only test settles at PV=-100
instead of PV=3.333. PI and PID tests oscillate wildly between the saturation
limits.

### Root cause

The output clamping logic in `S_DONE` used inline concatenation for sign
extension:

```verilog
if (sum > {{2{s_max[31]}}, s_max}) begin     // UNSIGNED expression
    pid_out <= out_max;
```

In Verilog, the concatenation operator `{...}` always produces an **unsigned**
result. When the signed reg `sum` (34-bit) is compared against an unsigned
concatenation, both operands are treated as unsigned per IEEE 1364 rules.

For a negative `s_min` (e.g., -100.0 = `0xFF9C_0000`), the sign-extended
34-bit concatenation `{2'b11, 32'hFF9C_0000}` = `0x3_FF9C_0000` which is a
huge unsigned value (~17 billion). The comparison `sum < 0x3_FF9C_0000` is
therefore always true for any normal `sum` value, causing every output to be
clamped to `out_min`.

### Fix

Pre-compute the sign-extended limits as properly-typed signed wires:

```verilog
wire signed [33:0] sum_max = {{2{s_max[31]}}, s_max};
wire signed [33:0] sum_min = {{2{s_min[31]}}, s_min};
```

Then the comparisons `sum > sum_max` and `sum < sum_min` use signed arithmetic
because both operands are `signed`.

### Lesson

Never use raw concatenation in comparisons with signed values. Verilog's
signed/unsigned promotion rules are unintuitive — a single unsigned operand
forces the entire comparison to unsigned. Always assign concatenated
sign-extensions to a `wire signed` before comparing.

---

## Bug 2 — Forward reference and clock alias in AXI wrapper

**File:** `rtl/pid_axi_wrapper.v`
**Severity:** Medium — blocks compilation in Vivado xvlog

### Symptom

After fixing Bug 1, `xvlog` reports:
```
ERROR: [VRFC 10-3380] identifier 'plant_output' is used before its declaration
```

### Root cause

Two issues in the same file:

1. The process variable mux at line 123 references `plant_output`, but the
   `reg signed [31:0] plant_output` declaration is at line 164, inside the
   test plant section. Vivado's Verilog parser (unlike some simulators)
   enforces declaration-before-use for `reg` types.

2. The test plant's always block used `always @(posedge clk)` where `clk` was
   defined as `wire clk = s_axi_aclk;` at line 330 — a forward-reference wire
   alias at the bottom of the module. While technically legal Verilog, it is
   fragile and confusing.

### Fix

- Moved `reg signed [31:0] plant_output;` above the `actual_pv` mux as a
  forward declaration.
- Replaced `always @(posedge clk)` with `always @(posedge s_axi_aclk)`.
- Deleted the `wire clk = s_axi_aclk;` alias.

---

## Bug 3 — LED IOSTANDARD mismatch across I/O banks

**File:** `constraints/kc705_pid.xdc`
**Severity:** Medium — blocks implementation (DRC violation)

### Symptom

Would produce a Vivado DRC error during implementation: IOSTANDARD conflict
for pins in 2.5V I/O banks.

### Root cause

All 8 GPIO LEDs were assigned `LVCMOS15`, but the KC705 routes them across
different voltage banks:

| LEDs  | Pins             | Bank | VCCO  | Correct IOSTANDARD |
|-------|------------------|------|-------|--------------------|
| 0–3   | AB8, AA8, AC9, AB9 | 33 | 1.5V  | LVCMOS15           |
| 4     | AE26             | 25   | 2.5V  | LVCMOS25           |
| 5–7   | G19, E18, F16    | 26   | 2.5V  | LVCMOS25           |

### Fix

Split the IOSTANDARD assignment: LVCMOS15 for LEDs [0:3], LVCMOS25 for
LEDs [4:7]. Also removed the system clock pin constraints (lines 8-12)
because they duplicate what Vivado's board automation applies internally,
which would cause a "conflicting property" error.

---

## Bug 4 — Soft-float library bloats firmware past 64KB BRAM

**File:** `vivado/pid_firmware.c`
**Severity:** Critical — firmware fails to link

### Symptom

Vitis linker error:
```
.text section overflows 64KB BRAM by 33696 bytes
```

### Root cause

The `parse_float()` function called `atof()` from `<stdlib.h>`, which pulls in
the entire MicroBlaze soft-float library (~35KB) since MicroBlaze has no
hardware FPU. Combined with the rest of the firmware code and BSP, this
exceeds the 64KB local memory.

The `float_to_q16()` helper also used `float` types, and `atoi()` from
`<stdlib.h>` contributed additional library code.

### Fix

Eliminated all floating-point code:

- **Replaced `parse_float()` + `float_to_q16()`** with `parse_q16()`: an
  integer-only parser that converts strings like `"2.5"`, `"-0.03"`, `"100"`
  directly to Q16.16 fixed-point using only integer arithmetic and a single
  `uint64_t` multiply for the fractional part.

- **Replaced `atoi()`** with a trivial `parse_int()` function (6 lines).

- **Replaced all compile-time `float_to_q16(literal)` calls** in `cmd_demo()`
  and `main()` with pre-computed Q16.16 integer constants
  (e.g., `float_to_q16(2.0f)` → `(2 << 16)`).

- **Removed** `#include <stdio.h>` and `#include <stdlib.h>`.

This saves approximately 35–40KB of .text, bringing the firmware well within
the 64KB BRAM limit.

---

## Bug 5 — Wrong PID peripheral base address

**File:** `vivado/pid_firmware.c`
**Severity:** Critical — all register reads return garbage

### Symptom

After programming KC705, every `status` command shows the same value for all
registers: `-557785572` (which is `0xDEADBEFC` — offset from the wrapper's
default read value of `0xDEADBEEF` for unmapped addresses).

### Root cause

Two problems in the PID base address definition:

1. The `#ifndef` guard checked for `XPAR_PID_AXI_0_S_AXI_BASEADDR`, but
   Vitis generates `XPAR_PID_AXI_0_BASEADDR` (no `S_AXI_` in the name).
   The guard never matched, so the `#else` branch was never taken.

2. The fallback address was `0x44A00000` (a common Vivado default for custom
   IPs), but `assign_bd_address` in our TCL script placed the PID at
   `0x00010000` (right after the 64KB MicroBlaze local memory).

### Fix

Corrected both the macro name and fallback value:

```c
#ifndef XPAR_PID_AXI_0_BASEADDR
#define PID_BASE 0x00010000
#else
#define PID_BASE XPAR_PID_AXI_0_BASEADDR
#endif
```

The UART base address (`0x40600000`) was confirmed correct in
`xparameters.h` — no change needed.

---

## Additional fixes (non-bugs)

**Clock frequency update:** The Clocking Wizard generates 100MHz (not 200MHz)
for MicroBlaze. Updated `CLK_FREQ` in firmware and `tick_generator` parameter
to 100MHz. Updated default `reg_tick_div` from 200,000 to 100,000 to maintain
1kHz loop rate at the actual clock frequency.

**Stream exit:** `cmd_stream()` had an infinite `while(1)` loop with no exit
mechanism. Added non-blocking UART RX FIFO check so sending any character
stops the stream.
