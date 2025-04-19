import sys
import termios
import tty
import signal
import time
from tps55287 import TSP55287

BB = TSP55287()
vout = 4.6
iout_max = 1.2

brightness_percent = 0  # Brightness as integer percent (0–100)
step = 1  # Step in percent
step_ms = 10  # Step in ms
pulse_duration_ms = 100  # Duration of pulse in ms
mode = "pulse"  # Operating mode: "continuous" or "pulse"

# Исправим ошибку: iout -> iout_max
BB.set_iout_limit_from_current(1, iout_max * brightness_percent / 100)  
BB.set_vout(vout)
BB.set_mode(oe=0)  # Начинаем с выключенного лазера в импульсном режиме

print("Press Ctrl+C to exit.")
print("Use arrow keys to adjust brightness (up to increase, down to decrease).")
print("Use left/right arrow keys to change pulse duration.")
print("Press F/f for continuous mode, I/i for pulse mode, Enter to generate a pulse.")

def get_key():
    """Reads a single key press from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x03':  # Ctrl+C
            raise KeyboardInterrupt
        elif ch == '\x1b':  # Arrow keys and other escape sequences
            # Read the complete escape sequence
            ch += sys.stdin.read(1)
            if ch == '\x1b[':
                ch += sys.stdin.read(1)
        elif ch == '\r':  # Enter key
            ch = 'ENTER'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Consume any additional input that might have accumulated
        termios.tcflush(fd, termios.TCIFLUSH)
    return ch

def handle_exit(signum=None, frame=None):
    """Clean exit on Ctrl+C."""
    print("\nProgram terminated by the user.")
    BB.set_mode(oe=0)
    print("Laser turned off.")
    sys.exit(0)

def generate_pulse(duration_ms):
    """Generate a single pulse with the specified duration."""
    BB.set_mode(oe=1)
    time.sleep(duration_ms / 1000)
    BB.set_mode(oe=0)

# Register signal handler
signal.signal(signal.SIGINT, handle_exit)

try:
    while True:
        # Calculate actual current based on percentage
        current = iout_max * brightness_percent / 100
        
        # Update Laser Power
        BB.set_iout_limit_from_current(1, current)  
        BB.set_vout(vout)
        
        # Control OE based on mode
        if mode == "continuous":
            BB.set_mode(oe=1)
        else:  # "pulse" mode
            BB.set_mode(oe=0)  # Keep off until pulse requested
            
        # Clear the entire line before printing the status
        print("\r\033[K", end="")  # Clear entire line
        print(f"Brightness: {brightness_percent}% | Current: {current:.2f}A | Mode: {mode} | Pulse duration: {pulse_duration_ms}ms", 
              end="", flush=True)

        try:
            key = get_key()
        except KeyboardInterrupt:
            handle_exit()

        if key == '\x1b[A':  # Arrow up
            if brightness_percent < 100:
                brightness_percent += step
            # Removed unnecessary message

        elif key == '\x1b[B':  # Arrow down
            if brightness_percent > 0:
                brightness_percent -= step
            # Removed unnecessary message
                
        elif key == '\x1b[C':  # Arrow right
            pulse_duration_ms += 10
            # Removed unnecessary message
            
        elif key == '\x1b[D':  # Arrow left
            if pulse_duration_ms > 10:
                pulse_duration_ms -= 10
            # Removed unnecessary message
                
        elif key.upper() == 'F':
            mode = "continuous"
            # Removed unnecessary message
            
        elif key.upper() == 'I':
            mode = "pulse"
            BB.set_mode(oe=0)  # Turn off when switching to pulse mode
            # Removed unnecessary message
            
        elif key == 'ENTER':
            if mode == "pulse":
                generate_pulse(pulse_duration_ms)
            else:
                print("\nPulse generation only available in pulse mode")

finally:
    BB.set_mode(oe=0)