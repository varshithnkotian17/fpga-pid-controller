"""
FPGA PID Controller - PC Monitor
=================================
Connects to KC705 via USB-UART, sends commands, and plots live data.

Usage:
    python pid_monitor.py COM5          # Just connect and type commands
    python pid_monitor.py COM5 --demo   # Run demo and plot step response
    python pid_monitor.py COM5 --plot   # Stream and plot in real-time

Requirements:
    pip install pyserial matplotlib

The KC705 USB-UART appears as a COM port (Windows) or /dev/ttyUSBx (Linux).
"""

import sys
import time
import serial
import threading
import argparse

# Try to import matplotlib (optional, for plotting)
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Note: matplotlib not installed. Plotting disabled.")
    print("      Install with: pip install matplotlib")


def q16_to_float(raw):
    """Convert Q16.16 fixed-point integer to float."""
    if raw > 0x7FFFFFFF:
        raw -= 0x100000000
    return raw / 65536.0


class PIDMonitor:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)
        # Flush any startup messages
        self.ser.reset_input_buffer()

    def send(self, cmd):
        """Send a command and return the response."""
        self.ser.write((cmd + '\r\n').encode())
        time.sleep(0.1)
        response = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if line and line != '>' and line != cmd:
                response.append(line)
        return response

    def send_raw(self, cmd):
        """Send command, don't wait for response."""
        self.ser.write((cmd + '\r\n').encode())

    def read_line(self):
        """Read one line from UART."""
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            return line
        except:
            return ""

    def run_demo_and_capture(self):
        """Run the 'demo' command and capture all CSV data."""
        print("Sending 'demo' command to FPGA...")
        self.ser.reset_input_buffer()
        self.send_raw("demo")
        
        data = {"tick": [], "setpoint": [], "pv": [], "error": [], "output": []}

        timeout = time.time() + 30  # 30 second timeout

        while time.time() < timeout:
            line = self.read_line()
            if not line:
                continue

            if "Demo complete" in line:
                break

            # Parse CSV: Tick,Setpoint,PV,Error,Output (5 columns)
            parts = line.split(',')
            if len(parts) == 5:
                try:
                    tick = int(parts[0])
                    sp   = q16_to_float(int(parts[1]))
                    pv   = q16_to_float(int(parts[2]))
                    err  = q16_to_float(int(parts[3]))
                    out  = q16_to_float(int(parts[4]))

                    data["tick"].append(tick)
                    data["setpoint"].append(sp)
                    data["pv"].append(pv)
                    data["error"].append(err)
                    data["output"].append(out)

                    # Print progress
                    print(f"  Tick {tick:6d} | SP={sp:8.3f} | PV={pv:8.3f} | Err={err:8.3f} | Out={out:8.3f}")
                except ValueError:
                    pass
            else:
                print(f"  {line}")
        
        return data

    def close(self):
        self.ser.close()


def plot_step_response(data):
    """Plot the step response from demo data."""
    if not HAS_MATPLOTLIB:
        print("matplotlib not installed — can't plot")
        return

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('FPGA PID Controller — Step Response', fontsize=14)
    
    x = range(len(data["pv"]))
    
    # Process Variable + Setpoint
    ax1.plot(x, data["pv"], 'b-', linewidth=1.5, label='Process Variable')
    if data.get("setpoint"):
        ax1.plot(x, data["setpoint"], 'k--', linewidth=1.0, alpha=0.7, label='Setpoint')
    ax1.set_ylabel('PV (Q16.16)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Error
    ax2.plot(x, data["error"], 'r-', linewidth=1.0, label='Error')
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax2.set_ylabel('Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Output
    ax3.plot(x, data["output"], 'g-', linewidth=1.0, label='PID Output')
    ax3.set_ylabel('Output')
    ax3.set_xlabel('Sample')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('pid_step_response.png', dpi=150)
    print("Plot saved to pid_step_response.png")
    plt.show()


def interactive_mode(monitor):
    """Interactive terminal — type commands, see responses."""
    print("\n--- Interactive Mode ---")
    print("Type commands (help, status, demo, etc.)")
    print("Type 'quit' to exit\n")
    
    # Start reader thread
    running = True
    
    def reader():
        while running:
            line = monitor.read_line()
            if line:
                print(line)
    
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    
    while True:
        try:
            cmd = input()
            if cmd.lower() == 'quit':
                break
            monitor.send_raw(cmd)
        except (KeyboardInterrupt, EOFError):
            break
    
    running = False


def main():
    parser = argparse.ArgumentParser(description='FPGA PID Controller Monitor')
    parser.add_argument('port', help='Serial port (e.g., COM5 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--demo', action='store_true', help='Run demo and plot')
    parser.add_argument('--plot', action='store_true', help='Live plot mode')
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    monitor = PIDMonitor(args.port, args.baud)
    print("Connected!\n")

    try:
        if args.demo:
            data = monitor.run_demo_and_capture()
            if data["pv"]:
                plot_step_response(data)
            else:
                print("No data captured — check connection")
        else:
            interactive_mode(monitor)
    finally:
        monitor.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
