"""
Generate step_response.png by simulating the PID controller + test plant.

Uses the exact same algorithm and parameters as the FPGA implementation:
  - Q16.16 fixed-point arithmetic (simulated with int32 truncation)
  - Plant: y += (u - y) / 16  (first-order lag, DC gain=1, tau=16 ticks)
  - Demo gains: Kp=2.0, Ki=0.3, Kd=0.05
  - Output limits: -50 to +50
  - Step sequence: SP=0 (settle) -> SP=10 (100 ticks) -> SP=-5 (100 ticks)

Run:  python docs/generate_plot.py
"""

import os
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt


def q16(val):
    """Convert float to Q16.16 integer."""
    return int(val * 65536)


def q16_mul(a, b):
    """Q16.16 multiply: (a * b) >> 16, with 64-bit intermediate."""
    return (a * b) >> 16


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def simulate_pid(sp_sequence, kp, ki, kd, out_min, out_max, ticks_per_step):
    """Simulate the PID controller with the test plant."""
    # State
    integral = 0
    error_prev = 0
    plant_y = 0
    pid_out = 0
    saturated = False

    kp_q = q16(kp)
    ki_q = q16(ki)
    kd_q = q16(kd)
    min_q = q16(out_min)
    max_q = q16(out_max)

    results = {"tick": [], "setpoint": [], "pv": [], "error": [], "output": []}
    tick = 0

    for sp_val in sp_sequence:
        sp_q = q16(sp_val)
        for _ in range(ticks_per_step):
            # Error
            err = sp_q - plant_y

            # P, I, D terms (Q16.16 multiply)
            p_term = q16_mul(kp_q, err)
            i_update = q16_mul(ki_q, err)
            d_term = q16_mul(kd_q, err - error_prev)

            # Anti-windup: block integral if saturated in same direction
            sat_high = saturated and (pid_out == max_q)
            sat_low = saturated and (pid_out == min_q)
            if sat_high and i_update > 0:
                new_int = integral
            elif sat_low and i_update < 0:
                new_int = integral
            else:
                new_int = clamp(integral + i_update, min_q, max_q)
            integral = new_int

            # Sum and clamp
            pid_sum = p_term + new_int + d_term
            if pid_sum > max_q:
                pid_out = max_q
                saturated = True
            elif pid_sum < min_q:
                pid_out = min_q
                saturated = True
            else:
                pid_out = pid_sum
                saturated = False

            error_prev = err

            # Plant: y += (u - y) / 16   (arithmetic right shift by 4)
            plant_y = plant_y + ((pid_out - plant_y) >> 4)

            # Record (convert back to float for plotting)
            results["tick"].append(tick)
            results["setpoint"].append(sp_val)
            results["pv"].append(plant_y / 65536.0)
            results["error"].append(err / 65536.0)
            results["output"].append(pid_out / 65536.0)
            tick += 1

    return results


def main():
    # Settle at 0 (20 ticks), then step to +10 (100 ticks), then -5 (100 ticks)
    data = simulate_pid(
        sp_sequence=[0, 10, -5],
        kp=2.0, ki=0.3, kd=0.05,
        out_min=-50.0, out_max=50.0,
        ticks_per_step=100,
    )

    # Only plot the step portions (skip the initial settle at 0)
    start = 100  # skip first 100 ticks (SP=0 settle)
    x = list(range(len(data["pv"][start:])))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle('FPGA PID Controller \u2014 Step Response (Simulated)', fontsize=13,
                 fontweight='bold')

    # -- Process Variable + Setpoint --
    ax1.plot(x, data["pv"][start:], '#2176FF', linewidth=1.8, label='Process Variable')
    ax1.plot(x, data["setpoint"][start:], 'k--', linewidth=1.0, alpha=0.6, label='Setpoint')
    ax1.set_ylabel('Value')
    ax1.legend(loc='right', fontsize=9)
    ax1.grid(True, alpha=0.25)
    ax1.set_title('PV tracking (Kp=2.0, Ki=0.3, Kd=0.05, Plant: y+=(u\u2212y)/16)',
                   fontsize=10, style='italic')

    # -- Error --
    ax2.plot(x, data["error"][start:], '#E03C31', linewidth=1.0, label='Error')
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.set_ylabel('Error')
    ax2.legend(loc='right', fontsize=9)
    ax2.grid(True, alpha=0.25)

    # -- Output --
    ax3.plot(x, data["output"][start:], '#2CA02C', linewidth=1.0, label='PID Output')
    ax3.set_ylabel('Output')
    ax3.set_xlabel('PID Tick')
    ax3.legend(loc='right', fontsize=9)
    ax3.grid(True, alpha=0.25)

    # Annotate the setpoint change
    ax1.axvline(x=100, color='gray', linestyle=':', alpha=0.5)
    ax2.axvline(x=100, color='gray', linestyle=':', alpha=0.5)
    ax3.axvline(x=100, color='gray', linestyle=':', alpha=0.5)
    ax1.annotate('SP: 10 \u2192 \u22125', xy=(100, 0), fontsize=8, color='gray',
                 ha='center', va='bottom')

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), 'step_response.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
