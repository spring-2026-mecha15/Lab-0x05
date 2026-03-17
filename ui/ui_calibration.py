"""
Reflectance sensor calibration screen for the Romi robot UI.

Guides the user through a two-phase calibration that establishes white and
black reference levels for the IR reflectance array:

    **Phase 1 – Light surface**: robot placed on a white background.
    **Phase 2 – Dark surface**:  robot placed on the black line.

Communication with the reflectance sensor task uses a shared mode variable:
    - ``2`` → trigger light-surface calibration
    - ``1`` → trigger dark-surface calibration
    - ``0`` ← sensor acknowledges completion (written back by sensor task)

All blocking waits are cooperative (``yield 0``) so the scheduler remains
responsive throughout.
"""


def run(ui):
    """
    Generator that runs the two-phase reflectance sensor calibration workflow.

    First asks the user to confirm (``y``/``n``).  On confirmation it walks
    through light then dark calibration, signalling the sensor task via
    ``ui._reflectanceMode`` and waiting (with ``yield 0``) for each phase
    to complete before proceeding.

    Args:
        ui: UI context object exposing ``_ser`` (serial port) and
            ``_reflectanceMode`` (shared variable used to command and
            synchronise with the reflectance sensor task).

    Yields:
        int: ``0`` on every poll and acknowledgement-wait iteration.
    """
    ui._ser.write("CALIBRATION\r\n")

    # Double check this is the action the user wants to make
    ui._ser.write("Would you like to begin calibration (y/n)?\r\n")

    # Check for a valid keystroke
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"y", "Y", "n", "N"}:
                break

    # If NO selected, return to main menu from this point
    if inChar in {"n", "N"}:
        yield 0
        return

    # Begin with light side calibration
    ui._ser.write("Place Romi on a light surface. Press Enter when ready\r\n")
    # Wait for ENT (cooperative)
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"\r", "\n"}:
                # Clear any remaining input then exit help
                while ui._ser.any():
                    ui._ser.read(1)
                break
        yield 0

    # Request that reflectance sensor goes into light calib. mode
    ui._reflectanceMode.put(2)

    # Wait for 'ack' that calibration is done (mode = 0)
    while ui._reflectanceMode.get() != 0:
        yield 0

    # Then perform dark side calibration
    ui._ser.write("Please Place Romi On a Dark Surface. Press Enter When Complete\r\n")

    # Wait for ENT (cooperative)
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"\r", "\n"}:
                # Clear any remaining input then exit help
                while ui._ser.any():
                    ui._ser.read(1)
                break
        yield 0

    # Request that reflectance sensor goes into light calib. mode
    ui._reflectanceMode.put(1)

    # Wait for 'ack' that calibration is done (mode = 0)
    while ui._reflectanceMode.get() != 0:
        yield 0

    ui._ser.write('Sensor calibration complete.\r\n')
