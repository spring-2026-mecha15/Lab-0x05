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
try:
    import utime as time
except ImportError:
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
    # VelocityConfig(75, 0.20, 0.0),
    # VelocityConfig(75, 0.25, 0.0),
    # VelocityConfig(75, 0.30, 0.0),
    # VelocityConfig(75, 0.1, 4.0),
    VelocityConfig(75, 0.14, 4.0),
    VelocityConfig(75, 0.15, 4.0),
    VelocityConfig(75, 0.16, 4.0),
    # VelocityConfig(75, 0.2, 4.0),
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
            raise
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

        time.sleep(3) # Allow romi to wakeup
        print("connected!")

        # -------------------------
        # Create output folder
        # -------------------------
        foldername = time.strftime("%Y%m%d_%H%M%S")
        directory = os.path.join(os.path.dirname(__file__), 'data', foldername)
        os.makedirs(directory, exist_ok=True)
        filenames = []
        filename = "circle_log.csv"
        filepath = os.path.join(directory, filename)

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
            time.sleep(0.05)
            ser.write(f'{ki:.3f}\n'.encode('ascii'))
            time.sleep(0.05)
            ser.write(b'\n') # skip lf kp
            time.sleep(0.05)
            ser.write(b'\n') # skip lf ki
            time.sleep(0.05)
            ser.write(b'\n') # skip lf kff

            time.sleep(0.05)

            # Enter setpoint menu
            ser.write(b's')
            ser.write(f'{setpoint:.3f}\n'.encode('ascii'))
            time.sleep(0.05)
            ser.write(b'\n') # skip lf setpoint
            time.sleep(0.05)

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
        plot_csv(filenames, include_combined=True)

        return 0

    except KeyboardInterrupt:
        print("\nStep test cancelled. Returning to menu.")
        return 1

    finally:
        if ser is not None and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass

def run_circle_log_placeholder(com_port):
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

        print("connected!")

        # -------------------------
        # Create output folder
        # -------------------------
        foldername = time.strftime("%Y%m%d_%H%M%S")
        directory = os.path.join(os.path.dirname(__file__), 'data', foldername)
        os.makedirs(directory, exist_ok=True)
        filenames = []
        filename = "circle_log.csv"
        filepath = os.path.join(directory, filename)

        # Trigger Line Following
        ser.write(b'l')

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
            fhand.write('time_s,centroid_deviation\n')
            for line in read_csv_data(ser):
                parts = line.strip().split(',', 1)
                if len(parts) == 2:
                    try:
                        fhand.write(f'{float(parts[0]) / 1000.0},{parts[1]}\n')
                        continue
                    except ValueError:
                        pass
                fhand.write(line)

        return 0

    except KeyboardInterrupt:
        print("\nCircle log cancelled. Returning to menu.")
        ser.write(b'\x03')
        time.sleep(0.05)
        ser.write(b'\x04')
        filenames.append(filepath)
        plot_csv(filenames, include_combined=False)
        return 1

    finally:
        if ser is not None and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
