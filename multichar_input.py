from pyb import USB_VCP, UART
from task_share import BaseShare, Share

def multichar_input(ser, ):

    # A serial port object to use for reading characters
    #ser: stream = USB_VCP()
    # ser: stream = UART(1, 115_200)

    # A share or queue object where the computed number is to be placed
    ##out_share: BaseShare = Share('f', name="A float share")

    # A character buffer used to store incoming characters as they're
    # received by the command processor
    char_buf: str      = ""

    # A set used to quickly check if a character entered by the user is
    # a numerical digit.
    digits:   set(str) = set(map(str,range(10)))

    # A set used to quickly check if a character entered by the user is
    # a terminator (a carriage return or newline)
    term:     set(str) = {"\r", "\n"}

    # A flag used to track whether or not the command processing is
    # still active.
    done = False

    # If the command is not done being processed, wait for characters
    # and then process them individually.
    while not done:
        
        # Only read and process characters if they're available. This
        # is to avoid blocking behavior.
        if ser.any():
            
            # All commands are single characters so only one needs to
            # be read from the serial port. The character coming from
            # the serial port is of type bytes so it is cast as a
            # string.
            char_in = ser.read(1).decode()


            # Digits are simply appended to the incoming character
            # buffer and echoed back to the UI.
            if char_in in digits:
                ser.write(char_in)
                char_buf += char_in
            
            # Decimal points are only valid if they appear once in the
            # string, so the period character is ignored if one is
            # already in the character buffer. The decimal point is
            # echoed if it's added to the buffer.
            elif char_in == "." and "." not in char_buf:
                ser.write(char_in)
                char_buf += char_in
            
            # Dashes are used for negative values but are only valid if
            # they're the first character in the buffer. Valid dashes
            # are echoed.
            elif char_in == "-" and len(char_buf) == 0:
                ser.write(char_in)
                char_buf += char_in
            
            # If a "rubout" character is received, as would come from
            # a serial monitor like PuTTY, and there is at least one
            # character in the character buffer then the last character
            # in the buffer is removed. Echoing the validated "rubout"
            # character deletes the previous digit in the serial
            # monitor and also moves the cursor a unit to the left.
            elif char_in == "\x7f" and len(char_buf) > 0: 
                ser.write("\b \b")
                char_buf = char_buf[:-1]
            
            # If a termination character is received it is interpreted
            # as the end of data entry.
            elif char_in in term:
                
                # If the buffer is empty then the value of the share is
                # left unchanged. That way the user can choose not to
                # enter a new value even if they've already issued the
                # command to change a value.
                if len(char_buf) == 0:
                    ser.write("\r\n")
                    ser.write("Value not changed\r\n")
                    char_buf = ""
                    done = True
                    return None
                    # Same as doing raise StopIteration(None)
                    
                    
                # If the character buffer is not empty the termination
                # character is interpreted as an end to the user input.
                # However if the buffer only contains a single dash or
                # period character the termination key is ignored.
                elif char_buf not in {"-", "."}:
                    ser.write("\r\n")
                    value = float(char_buf)

                    ##out_share.put(value)
                    ##ser.write(f"Value set to {value}\r\n")
                    char_buf = ""
                    done = True
                    return value
                    # Same as doing raise StopIteration(value)
        yield