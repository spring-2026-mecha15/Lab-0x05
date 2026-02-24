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
import time
import serial
from serial import SerialException
from plot import plot_csv
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from constants import CSV_BEGIN, CSV_END

BAUDRATE = 115200

# Small helper container for setpoint/gain configs
class VelocityConfig:
    def __init__(self, setpoint, kp, ki):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki

# Test configurations to iterate over
configs = [
    # VelocityConfig(50, 0.04, 0.2),
    # VelocityConfig(50, 0.04, 0.4),
    VelocityConfig(75, 0.01, 0.8),
    # VelocityConfig(75, 0.04, 0.6),
    VelocityConfig(75, 0.05, 0.8),
    # VelocityConfig(75, 0.08, 0.6),
    VelocityConfig(75, 0.10, 0.8),
    # VelocityConfig(75, 0.12, 0.6),
    VelocityConfig(75, 0.20, 0.8),
    # VelocityConfig(50, 0.10, 0.6),
    # VelocityConfig(50, 0.04, 0.8),
    # VelocityConfig(50, 0.04, 1.0),
    # VelocityConfig(50, 0.04, 1.2),
    # VelocityConfig(50, 0.04, 1.4),
]

# Generator that reads decoded text lines from serial and yields them.
# Keeps basic error handling and respects KeyboardInterrupt.
def read_csv_data(ser: serial.Serial):
    """Yield decoded, stripped lines from serial until an error or user interrupt."""
    while True:
        try:
            raw = ser.readline()
            if not raw:
                # no data this iteration; continue reading
                continue
            # decode and normalize line endings
            text = raw.decode('ascii', errors='ignore').strip()
            if not text:
                continue
            # yield a line that includes newline at end (keeps original behaviour)
            yield text + '\n'
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Read error: {e}")
            break


def run_step_test(com_port):
    ser = None
    try:
        # -------------------------
        # Open serial connection
        # -------------------------
        try:
            ser = serial.Serial(com_port, baudrate=BAUDRATE, timeout=1.0)
        except SerialException as e:
            # Try to give a helpful message if port is in use
            msg = e.args[0] if e.args else str(e)
            if "PermissionError" in msg or "Access is denied" in msg:
                print("\r\nRomi found, but already in use!")
            else:
                print("\r\nSerial open error:", e)
            return 1

        # Send Ctrl-C + Ctrl-D to soft reset the board
        ser.write(b'\x03\x04')
        print("connected!")

        # -------------------------
        # Create output folder
        # -------------------------
        foldername = time.strftime("%Y%m%d_%H%M%S")
        directory = os.path.join(os.path.dirname(__file__), 'data', foldername)
        os.makedirs(directory, exist_ok=True)

        filenames = []

        # -------------------------
        # Run configurations
        # -------------------------
        for config in configs:
            setpoint = config.setpoint
            kp = config.kp
            ki = config.ki

            filename = f'P{kp:.2f}_I{ki:.2f}_S{setpoint:.1f}.csv'
            filepath = os.path.join(directory, filename)

            # -------------------------
            # Drive ROMI UI to set gains and setpoint
            # -------------------------
            # Enter gains menu (original code used 'k')
            ser.write(b'k')
            # Write Kp and Ki values followed by newline
            ser.write(f'{kp:.3f}\n'.encode('ascii'))
            ser.write(f'{ki:.3f}\n'.encode('ascii'))

            # Enter setpoint menu
            ser.write(b's')
            ser.write(f'{setpoint:.3f}\n'.encode('ascii'))

            # Trigger test run (originally 'g')
            ser.write(b'g')

            print(f'Collecting for {filename}...', end='', flush=True)

            # Wait until device reports "collection complete" (case-insensitive)
            # This loop intentionally discards any intermediate lines.
            while True:
                line = ser.readline()
                if not line:
                    continue
                try:
                    if b'collection complete' in line.lower():
                        break
                except Exception:
                    # ignore decoding/inspection oddities and continue reading
                    pass

            # Wait for CSV START marker before streaming CSV lines
            while True:
                line = ser.readline().decode()
                if not line:
                    continue
                try:
                    if CSV_BEGIN in line:
                        break
                except Exception:
                    pass

            # Stream CSV lines into the file until CSV END marker appears
            with open(filepath, 'w', newline='') as fhand:
                for line in read_csv_data(ser):
                    # Stop when device signals end of CSV
                    if CSV_END in line:
                        break
                    fhand.write(line)

            filenames.append(filepath)
            print('complete')

            # brief pause between runs
            time.sleep(0.5)

        print('All runs complete')
        # Plot the results (plot_csv expects a list of file paths)
        plot_csv(filenames)

        return 0

    except KeyboardInterrupt:
        print("\nUser interrupted. Closing serial and exiting.")
        return 1

    finally:
        if ser is not None and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass

def run_circle_log_placeholder():
    """Placeholder desktop UI state for future circle logging workflow."""
    print("\n[Circle Log]")
    print("Placeholder state only. Fill this in later.")
    input("Press Enter to return to the menu...")
    return 0
