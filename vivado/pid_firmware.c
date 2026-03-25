/*=============================================================================
 * MicroBlaze Firmware: PID Controller Command Interface
 * Target: KC705 + Vivado/Vitis 2023.2
 *
 * This firmware runs on MicroBlaze and provides a UART command-line
 * interface to configure, run, and monitor the PID controller.
 *
 * Connect to KC705 USB-UART at 115200 baud, 8N1.
 *
 * Commands:
 *   help                  - Show all commands
 *   status                - Show current PID state
 *   enable                - Start PID loop
 *   disable               - Stop PID loop
 *   plant [on|off]        - Enable/disable test plant
 *   kp <value>            - Set proportional gain
 *   ki <value>            - Set integral gain
 *   kd <value>            - Set derivative gain
 *   sp <value>            - Set setpoint
 *   pv <value>            - Set manual process variable
 *   outmin <value>        - Set output minimum
 *   outmax <value>        - Set output maximum
 *   rate <hz>             - Set loop rate in Hz
 *   clear                 - Clear integral term
 *   monitor [N]           - Print N readings (default 50)
 *   stream                - Continuous output (Ctrl+C to stop)
 *   demo                  - Run a demo step response
 *
 * All gain/value parameters are floating point (e.g., kp 2.5)
 *============================================================================*/

#include <string.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_io.h"
#include "sleep.h"

/*---------------------------------------------------------------------------
 * PID Register Map (base address from xparameters.h)
 * Adjust XPAR_PID_AXI_0_S_AXI_BASEADDR if your block design names differ
 *---------------------------------------------------------------------------*/
#ifndef XPAR_PID_AXI_0_BASEADDR
#define PID_BASE 0x00010000  /* Fallback — must match Vivado address map */
#else
#define PID_BASE XPAR_PID_AXI_0_BASEADDR
#endif

#define PID_CTRL      (PID_BASE + 0x00)
#define PID_STATUS    (PID_BASE + 0x04)
#define PID_KP        (PID_BASE + 0x08)
#define PID_KI        (PID_BASE + 0x0C)
#define PID_KD        (PID_BASE + 0x10)
#define PID_SETPOINT  (PID_BASE + 0x14)
#define PID_PROCVAR   (PID_BASE + 0x18)
#define PID_OUTPUT    (PID_BASE + 0x1C)
#define PID_ERROR     (PID_BASE + 0x20)
#define PID_OUTMIN    (PID_BASE + 0x24)
#define PID_OUTMAX    (PID_BASE + 0x28)
#define PID_TICKDIV   (PID_BASE + 0x2C)
#define PID_TICKCNT   (PID_BASE + 0x30)
#define PID_PLANTOUT  (PID_BASE + 0x34)

#define CLK_FREQ 100000000  /* 100 MHz system clock (after Clocking Wizard) */

/*---------------------------------------------------------------------------
 * UART base address (for non-blocking RX check in stream mode)
 *---------------------------------------------------------------------------*/
#ifndef XPAR_AXI_UARTLITE_0_BASEADDR
#define UART_BASE 0x40600000
#else
#define UART_BASE XPAR_AXI_UARTLITE_0_BASEADDR
#endif
#define UART_STAT_REG  (UART_BASE + 0x08)
#define UART_RX_VALID  0x01

/*---------------------------------------------------------------------------
 * Integer-only string parsers (no float/atof — saves ~35KB soft-float lib)
 *---------------------------------------------------------------------------*/

/* Parse a decimal integer from string: "123", "-5", "0" */
static int parse_int(const char *s) {
    int neg = 0, val = 0;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9')
        val = val * 10 + (*s++ - '0');
    return neg ? -val : val;
}

/*
 * Parse a decimal string directly to Q16.16 fixed-point.
 * Handles: "2.5", "-0.03", "100", "-5", ".25"
 * Returns int32_t in Q16.16 format.
 */
static int32_t parse_q16(const char *s) {
    int neg = 0;
    int32_t int_part = 0;
    uint32_t frac_q16 = 0;

    /* Sign */
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') { s++; }

    /* Integer part */
    while (*s >= '0' && *s <= '9')
        int_part = int_part * 10 + (*s++ - '0');

    /* Fractional part */
    if (*s == '.') {
        s++;
        /*
         * Accumulate fractional digits: frac_num / frac_div
         * Then convert: frac_q16 = frac_num * 65536 / frac_div
         * Use uint32_t to avoid overflow for up to 5 digits.
         */
        uint32_t frac_num = 0;
        uint32_t frac_div = 1;
        int digits = 0;
        while (*s >= '0' && *s <= '9' && digits < 5) {
            frac_num = frac_num * 10 + (*s++ - '0');
            frac_div *= 10;
            digits++;
        }
        if (frac_num > 0)
            frac_q16 = (uint32_t)((uint64_t)frac_num * 65536 / frac_div);
    }

    int32_t result = (int_part << 16) | (frac_q16 & 0xFFFF);
    return neg ? -result : result;
}

/*---------------------------------------------------------------------------
 * Register access helpers
 *---------------------------------------------------------------------------*/
static inline void pid_write(uint32_t addr, uint32_t val) {
    Xil_Out32(addr, val);
}

static inline uint32_t pid_read(uint32_t addr) {
    return Xil_In32(addr);
}

/*---------------------------------------------------------------------------
 * Command handlers
 *---------------------------------------------------------------------------*/

void cmd_help(void) {
    xil_printf("\r\n=== FPGA PID Controller (KC705) ===\r\n");
    xil_printf("Commands:\r\n");
    xil_printf("  help          - This message\r\n");
    xil_printf("  status        - Show PID state\r\n");
    xil_printf("  enable        - Start PID loop\r\n");
    xil_printf("  disable       - Stop PID loop\r\n");
    xil_printf("  plant on/off  - Test plant mode\r\n");
    xil_printf("  kp <val>      - Set Kp (e.g. kp 2.5)\r\n");
    xil_printf("  ki <val>      - Set Ki\r\n");
    xil_printf("  kd <val>      - Set Kd\r\n");
    xil_printf("  sp <val>      - Set setpoint\r\n");
    xil_printf("  pv <val>      - Set manual PV\r\n");
    xil_printf("  outmin <val>  - Set output min\r\n");
    xil_printf("  outmax <val>  - Set output max\r\n");
    xil_printf("  rate <hz>     - Set loop rate\r\n");
    xil_printf("  clear         - Clear integral\r\n");
    xil_printf("  monitor [N]   - Print N readings\r\n");
    xil_printf("  stream        - Continuous output\r\n");
    xil_printf("  demo          - Step response demo\r\n");
    xil_printf("===================================\r\n");
}

void cmd_status(void) {
    uint32_t ctrl   = pid_read(PID_CTRL);
    uint32_t status = pid_read(PID_STATUS);

    xil_printf("\r\n--- PID Status ---\r\n");
    xil_printf("Enabled:    %s\r\n", (ctrl & 1) ? "YES" : "NO");
    xil_printf("Test plant: %s\r\n", (ctrl & 4) ? "ON" : "OFF");
    xil_printf("Saturated:  %s\r\n", (status & 2) ? "YES" : "NO");

    /* Print floating-point values using integer math for printf
     * (MicroBlaze xil_printf doesn't support %f) */
    int32_t sp  = (int32_t)pid_read(PID_SETPOINT);
    int32_t pv  = (int32_t)pid_read(PID_PROCVAR);
    int32_t err = (int32_t)pid_read(PID_ERROR);
    int32_t out = (int32_t)pid_read(PID_OUTPUT);
    int32_t pkp = (int32_t)pid_read(PID_KP);
    int32_t pki = (int32_t)pid_read(PID_KI);
    int32_t pkd = (int32_t)pid_read(PID_KD);

    /* Print as integer.fraction (4 decimal places) */
    #define PRINT_Q16(label, val) do { \
        int32_t _v = (val); \
        int32_t _int = _v >> 16; \
        int32_t _frac = (_v & 0xFFFF) * 10000 / 65536; \
        if (_v < 0 && _int == 0) xil_printf(label "-%d.%04d\r\n", (unsigned)(-_int), (unsigned)((-_frac)>0?-_frac:_frac)); \
        else if (_v < 0) xil_printf(label "%d.%04d\r\n", (int)_int, (unsigned)((_frac<0)?-_frac:_frac)); \
        else xil_printf(label "%d.%04d\r\n", (int)_int, (unsigned)_frac); \
    } while(0)

    PRINT_Q16("Kp:         ", pkp);
    PRINT_Q16("Ki:         ", pki);
    PRINT_Q16("Kd:         ", pkd);
    PRINT_Q16("Setpoint:   ", sp);
    PRINT_Q16("Process var:", pv);
    PRINT_Q16("Error:      ", err);
    PRINT_Q16("Output:     ", out);

    uint32_t tickdiv = pid_read(PID_TICKDIV);
    uint32_t tickcnt = pid_read(PID_TICKCNT);
    uint32_t rate_hz = (tickdiv > 0) ? CLK_FREQ / tickdiv : 0;
    xil_printf("Loop rate:  %u Hz\r\n", (unsigned)rate_hz);
    xil_printf("Tick count: %u\r\n", (unsigned)tickcnt);
    xil_printf("------------------\r\n");
}

void cmd_monitor(int count) {
    xil_printf("\r\nTick,Setpoint,PV,Error,Output\r\n");
    for (int i = 0; i < count; i++) {
        int32_t sp  = (int32_t)pid_read(PID_SETPOINT);
        int32_t pv  = (int32_t)pid_read(PID_PROCVAR);
        int32_t err = (int32_t)pid_read(PID_ERROR);
        int32_t out = (int32_t)pid_read(PID_OUTPUT);
        uint32_t tc = pid_read(PID_TICKCNT);

        /* CSV output: tick,sp,pv,error,output (all as raw Q16.16 integers)
         * The Python monitor script converts these to floats */
        xil_printf("%u,%d,%d,%d,%d\r\n",
            (unsigned)tc, (int)sp, (int)pv, (int)err, (int)out);

        usleep(20000); /* ~50 Hz print rate */
    }
}

void cmd_stream(void) {
    xil_printf("\r\nStreaming... (send any char to stop)\r\n");
    xil_printf("Tick,SP,PV,Error,Output\r\n");

    /* Loop until a character is received on UART */
    while (!(Xil_In32(UART_STAT_REG) & UART_RX_VALID)) {
        int32_t sp  = (int32_t)pid_read(PID_SETPOINT);
        int32_t pv  = (int32_t)pid_read(PID_PROCVAR);
        int32_t err = (int32_t)pid_read(PID_ERROR);
        int32_t out = (int32_t)pid_read(PID_OUTPUT);
        uint32_t tc = pid_read(PID_TICKCNT);

        xil_printf("%u,%d,%d,%d,%d\r\n",
            (unsigned)tc, (int)sp, (int)pv, (int)err, (int)out);

        usleep(10000);
    }
    inbyte(); /* Consume the stop character */
    xil_printf("\r\nStopped.\r\n");
}

void cmd_demo(void) {
    xil_printf("\r\n=== Step Response Demo ===\r\n");
    xil_printf("Setting up: Kp=2.0 Ki=0.3 Kd=0.05 Plant=ON\r\n");

    /* Configure PID — Q16.16 constants (integer<<16 + frac*65536/denom) */
    pid_write(PID_KP, (2 << 16));                  /* Kp = 2.0   */
    pid_write(PID_KI, (0 << 16) | 19661);          /* Ki = 0.3   (0.3*65536=19660.8) */
    pid_write(PID_KD, (0 << 16) | 3277);           /* Kd = 0.05  (0.05*65536=3276.8) */
    pid_write(PID_OUTMIN, (uint32_t)(-(50 << 16))); /* OutMin = -50.0 */
    pid_write(PID_OUTMAX, (50 << 16));              /* OutMax = +50.0 */
    pid_write(PID_TICKDIV, CLK_FREQ / 1000);        /* 1 kHz loop */
    pid_write(PID_SETPOINT, 0);                      /* SP = 0.0   */

    /* Enable with test plant */
    pid_write(PID_CTRL, 0x05); /* enable + test_plant */

    xil_printf("Enabled at SP=0. Waiting 1 sec...\r\n");
    sleep(1);

    /* Step to 10.0 */
    xil_printf("Stepping to SP=10.0\r\n");
    xil_printf("Tick,Setpoint,PV,Error,Output\r\n");
    pid_write(PID_SETPOINT, (10 << 16));  /* SP = 10.0 */

    for (int i = 0; i < 100; i++) {
        int32_t sp  = (int32_t)pid_read(PID_SETPOINT);
        int32_t pv  = (int32_t)pid_read(PID_PROCVAR);
        int32_t err = (int32_t)pid_read(PID_ERROR);
        int32_t out = (int32_t)pid_read(PID_OUTPUT);
        uint32_t tc = pid_read(PID_TICKCNT);

        xil_printf("%u,%d,%d,%d,%d\r\n",
            (unsigned)tc, (int)sp, (int)pv, (int)err, (int)out);
        usleep(20000);
    }

    /* Step to -5.0 */
    xil_printf("\r\nStepping to SP=-5.0\r\n");
    pid_write(PID_SETPOINT, (uint32_t)(-(5 << 16)));  /* SP = -5.0 */

    for (int i = 0; i < 100; i++) {
        int32_t sp  = (int32_t)pid_read(PID_SETPOINT);
        int32_t pv  = (int32_t)pid_read(PID_PROCVAR);
        int32_t err = (int32_t)pid_read(PID_ERROR);
        int32_t out = (int32_t)pid_read(PID_OUTPUT);
        uint32_t tc = pid_read(PID_TICKCNT);

        xil_printf("%u,%d,%d,%d,%d\r\n",
            (unsigned)tc, (int)sp, (int)pv, (int)err, (int)out);
        usleep(20000);
    }

    /* Disable */
    pid_write(PID_CTRL, 0x00);
    xil_printf("\r\n=== Demo complete ===\r\n");
}

/*---------------------------------------------------------------------------
 * Simple command parser
 *---------------------------------------------------------------------------*/

#define CMD_BUF_SIZE 64

static char cmd_buf[CMD_BUF_SIZE];
static int  cmd_len = 0;

void process_command(char *cmd) {
    char *arg = NULL;

    /* Find first space to split command and argument */
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] == ' ') {
            cmd[i] = '\0';
            arg = &cmd[i+1];
            break;
        }
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    }
    else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    }
    else if (strcmp(cmd, "enable") == 0) {
        uint32_t ctrl = pid_read(PID_CTRL);
        pid_write(PID_CTRL, ctrl | 0x01);
        xil_printf("PID enabled\r\n");
    }
    else if (strcmp(cmd, "disable") == 0) {
        uint32_t ctrl = pid_read(PID_CTRL);
        pid_write(PID_CTRL, ctrl & ~0x01);
        xil_printf("PID disabled\r\n");
    }
    else if (strcmp(cmd, "plant") == 0) {
        if (arg && strcmp(arg, "on") == 0) {
            uint32_t ctrl = pid_read(PID_CTRL);
            pid_write(PID_CTRL, ctrl | 0x04);
            xil_printf("Test plant ON\r\n");
        } else {
            uint32_t ctrl = pid_read(PID_CTRL);
            pid_write(PID_CTRL, ctrl & ~0x04);
            xil_printf("Test plant OFF\r\n");
        }
    }
    else if (strcmp(cmd, "kp") == 0 && arg) {
        pid_write(PID_KP, (uint32_t)parse_q16(arg));
        xil_printf("Kp set\r\n");
    }
    else if (strcmp(cmd, "ki") == 0 && arg) {
        pid_write(PID_KI, (uint32_t)parse_q16(arg));
        xil_printf("Ki set\r\n");
    }
    else if (strcmp(cmd, "kd") == 0 && arg) {
        pid_write(PID_KD, (uint32_t)parse_q16(arg));
        xil_printf("Kd set\r\n");
    }
    else if (strcmp(cmd, "sp") == 0 && arg) {
        pid_write(PID_SETPOINT, (uint32_t)parse_q16(arg));
        xil_printf("Setpoint set\r\n");
    }
    else if (strcmp(cmd, "pv") == 0 && arg) {
        pid_write(PID_PROCVAR, (uint32_t)parse_q16(arg));
        xil_printf("PV set\r\n");
    }
    else if (strcmp(cmd, "outmin") == 0 && arg) {
        pid_write(PID_OUTMIN, (uint32_t)parse_q16(arg));
        xil_printf("OutMin set\r\n");
    }
    else if (strcmp(cmd, "outmax") == 0 && arg) {
        pid_write(PID_OUTMAX, (uint32_t)parse_q16(arg));
        xil_printf("OutMax set\r\n");
    }
    else if (strcmp(cmd, "rate") == 0 && arg) {
        int hz = parse_int(arg);
        if (hz > 0 && hz <= 100000) {
            pid_write(PID_TICKDIV, CLK_FREQ / hz);
            xil_printf("Loop rate set to %d Hz\r\n", hz);
        } else {
            xil_printf("Rate must be 1-100000 Hz\r\n");
        }
    }
    else if (strcmp(cmd, "clear") == 0) {
        uint32_t ctrl = pid_read(PID_CTRL);
        pid_write(PID_CTRL, ctrl | 0x02);  /* Set clear bit */
        usleep(1000);
        pid_write(PID_CTRL, ctrl & ~0x02); /* Release clear bit */
        xil_printf("Integral cleared\r\n");
    }
    else if (strcmp(cmd, "monitor") == 0) {
        int n = arg ? parse_int(arg) : 50;
        cmd_monitor(n);
    }
    else if (strcmp(cmd, "stream") == 0) {
        cmd_stream();
    }
    else if (strcmp(cmd, "demo") == 0) {
        cmd_demo();
    }
    else if (strlen(cmd) > 0) {
        xil_printf("Unknown command: %s (type 'help')\r\n", cmd);
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/
int main(void) {
    /* Initialize defaults — Q16.16 integer constants */
    pid_write(PID_CTRL, 0x00);
    pid_write(PID_KP, (1 << 16));                    /* Kp = 1.0    */
    pid_write(PID_KI, 0);                             /* Ki = 0.0    */
    pid_write(PID_KD, 0);                             /* Kd = 0.0    */
    pid_write(PID_SETPOINT, 0);                        /* SP = 0.0    */
    pid_write(PID_OUTMIN, (uint32_t)(-(100 << 16)));   /* Min = -100.0 */
    pid_write(PID_OUTMAX, (100 << 16));                /* Max = +100.0 */
    pid_write(PID_TICKDIV, CLK_FREQ / 1000);           /* 1 kHz default */

    xil_printf("\r\n\r\n");
    xil_printf("=============================================\r\n");
    xil_printf("   FPGA PID Controller - KC705\r\n");
    xil_printf("   Type 'help' for commands\r\n");
    xil_printf("   Type 'demo' for step response test\r\n");
    xil_printf("=============================================\r\n");
    xil_printf("> ");

    /* Command loop */
    cmd_len = 0;
    while (1) {
        char c = inbyte();  /* Blocking read from UART */

        if (c == '\r' || c == '\n') {
            xil_printf("\r\n");
            cmd_buf[cmd_len] = '\0';
            process_command(cmd_buf);
            cmd_len = 0;
            xil_printf("> ");
        }
        else if (c == '\b' || c == 127) {
            /* Backspace */
            if (cmd_len > 0) {
                cmd_len--;
                xil_printf("\b \b");
            }
        }
        else if (cmd_len < CMD_BUF_SIZE - 1) {
            cmd_buf[cmd_len++] = c;
            outbyte(c); /* Echo */
        }
    }

    return 0;
}
