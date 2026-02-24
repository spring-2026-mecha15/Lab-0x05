#!/usr/bin/env python3
"""
Host-side test runner for Romi experiments.

- Auto-connects to a serial device matching DEFAULT_VID/PID
- Iterates a list of velocity/gain configurations
- Drives the Romi UI over serial (presses menu keys and types numeric values)
- Waits for CSV output, saves CSV files in a timestamped subfolder under ./data
- After all runs, calls `plot_csv(filenames)` to plot results

Note: This runs on a host computer (CPython) and is not for MicroPython.
"""
import sys
import os
from serial.tools import list_ports
from UI_Functions import run_step_test, run_circle_log_placeholder

# FSM states
S0_INIT = 0
S1_CMD = 1
S2_STEP_TEST = 2
S3_CIRCLE_LOG = 3

# Device identification (adjust to your target)
DEFAULT_PID = 38912
DEFAULT_VID = 61525

# Reusable UI text
UI_prompt = """\r\n
+-----------------------------------------+\r
| ME 405 Romi Desktop CLI.                |\r
+---+-------------------------------------+\r
| l | Circle Log                          |\r
| s | Run Step Test                       |\r
+---+-------------------------------------+\r\n\r"""

def main():
    """Desktop host CLI entry point (no MCU resource constraints; simple FSM)."""
    try:

        # -------------------------
        # Auto-detect ROMI serial
        # -------------------------
        ports = list_ports.comports()
        # filter for devices that match our VID/PID
        stlink_devices = [p for p in ports if p.vid == DEFAULT_VID and p.pid == DEFAULT_PID]

        first = next(iter(stlink_devices), None)
        if first is None:
            print("\n\rRomi not found. Exiting...\n\r")
            return 1

        com_port = first.device if hasattr(first, "device") else first.name
        print(f"\n\rFound device on {com_port}. Opening serial...", end="")
        state = S0_INIT

        while True:

            if state == S0_INIT:
                print(UI_prompt)
                state = S1_CMD

            elif state == S1_CMD:
                cmd = input(">: ").strip().lower()
                if cmd == "s":
                    state = S2_STEP_TEST
                elif cmd == "l":
                    state = S3_CIRCLE_LOG
                else:
                    print("Unknown command. Use s or l. (Ctrl-C to exit)")
                    state = S1_CMD

            elif state == S2_STEP_TEST:
                run_step_test(com_port)
                state = S0_INIT

            elif state == S3_CIRCLE_LOG:
                run_circle_log_placeholder()
                state = S0_INIT

    except KeyboardInterrupt:
        print("\nExiting host UI.")
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
