#!/usr/bin/env python
"""
I2C Diagnostic Tool for TPS55287 Controller
This script helps diagnose I2C communication issues with the TPS55287 controller.
"""

import os
import sys
import subprocess
import time
import platform
import argparse

def print_header(text, verbose=True):
    """Print a formatted header if in verbose mode."""
    if verbose:
        print("\n" + "="*60)
        print(f"  {text}")
        print("="*60)

def run_command(cmd):
    """Run a command and return output."""
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        output, error = p.communicate()
        return output.decode('utf-8'), error.decode('utf-8'), p.returncode
    except Exception as e:
        return "", str(e), -1

def check_i2c_tools(verbose=True):
    """Check if i2c-tools are installed."""
    if verbose:
        print_header("Checking I2C Tools", verbose)
    
    output, error, rc = run_command("which i2cdetect")
    if rc != 0:
        if verbose:
            print("❌ i2cdetect not found. Please install i2c-tools:")
            if platform.system() == "Linux":
                if os.path.exists("/etc/apt/sources.list"):
                    print("    sudo apt-get install i2c-tools")
                elif os.path.exists("/etc/yum.conf"):
                    print("    sudo yum install i2c-tools")
        return False
    
    if verbose:
        print("✅ i2c-tools installed")
    return True

def check_specific_i2c_device(bus_num, address=0x74, verbose=True):
    """Check for a specific I2C device on a given bus."""
    if verbose:
        print_header(f"Scanning I2C Bus {bus_num} for Device at 0x{address:02x}", verbose)
    
    output, error, rc = run_command(f"i2cdetect -y {bus_num}")
    if rc != 0:
        if verbose:
            print(f"❌ Error scanning bus {bus_num}: {error}")
        return False
    
    addr_hex = f"{address:02x}"
    found = False
    
    # Look for the device address in the output
    if verbose:
        print(output)
    
    for line in output.split('\n'):
        if addr_hex in line.split():
            found = True
            break
    
    if found:
        if verbose:
            print(f"✅ Device found at address 0x{address:02x} on bus {bus_num}")
        return True
    else:
        if verbose:
            print(f"❌ No device found at address 0x{address:02x} on bus {bus_num}")
        return False

def try_direct_access(bus_num, address=0x74, verbose=True):
    """Try to directly access the TPS55287 device on specified bus."""
    if verbose:
        print_header(f"Testing Direct I2C Access on Bus {bus_num}", verbose)
    
    try:
        # First try importing the library
        import smbus
        if verbose:
            print("✅ smbus library is available")
        
        try:
            bus = smbus.SMBus(bus_num)
            time.sleep(0.1)  # Allow time for bus to initialize
            
            # Try reading the first register (REF_LSB)
            try:
                if verbose:
                    print(f"Attempting to read REF_LSB register (0x00)...")
                value = bus.read_byte_data(address, 0x00)
                if verbose:
                    print(f"✅ Success! Read value: 0x{value:02x}")
                bus.close()
                return True
            except Exception as e:
                if verbose:
                    print(f"❌ Failed to read: {e}")
                bus.close()
                return False
            
        except Exception as e:
            if verbose:
                print(f"❌ Failed to open bus: {e}")
            return False
    
    except ImportError:
        if verbose:
            print("❌ smbus library not available. Install with:")
            print("  pip install smbus")
        return False

def provide_recommendations(verbose=True):
    """Provide troubleshooting recommendations."""
    if verbose:
        print_header("Troubleshooting Recommendations", verbose)
        
        print("If you're experiencing 'Remote I/O Error' issues:")
        print()
        print("1. Hardware Checks:")
        print("   - Verify all connections to the TPS55287 device")
        print("   - Check power supply to the device")
        print("   - Make sure pull-up resistors are present on SDA and SCL lines")
        print("   - Use a multimeter to verify proper voltage levels")
        print()
        print("2. Software Configuration:")
        print("   - Ensure I2C is enabled in Raspberry Pi configuration")
        print("   - Try a lower I2C bus speed:")
        print("     sudo sh -c 'echo 10000 > /sys/module/i2c_bcm2708/parameters/baudrate'")
        print("   - Ensure proper permissions for I2C device files")
        print()
        print("3. Try a different I2C address:")
        print("   - The default is 0x74, but try 0x75 if hardware address pin is changed")
        print()
        print("4. Inspect with tools:")
        print("   - Use a logic analyzer to view I2C signals")
        print("   - Try i2cdump to see all registers: sudo i2cdump -y {bus_num} 0x74")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='TPS55287 I2C Diagnostic Tool')
    parser.add_argument('-b', '--bus', type=int, default=1, help='I2C bus number to check (default: 1)')
    parser.add_argument('-a', '--address', type=lambda x: int(x, 0), default=0x74, 
                        help='I2C device address (default: 0x74)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    
    verbose = args.verbose
    
    if verbose:
        print_header("TPS55287 I2C Diagnostic Tool", verbose)
        
        if os.geteuid() != 0:
            print("⚠️  Warning: Some tests might require root privileges")
            print("    Consider running with sudo for complete diagnostics\n")
    
    # Basic check first
    have_tools = check_i2c_tools(verbose)
    if not have_tools:
        print("Error: i2c-tools not installed. Please install them first.")
        return 1
    
    # Check for the specific device
    device_found = check_specific_i2c_device(args.bus, args.address, verbose)
    
    # Try direct access if needed
    if device_found:
        access_ok = try_direct_access(args.bus, args.address, verbose)
        if access_ok:
            if verbose:
                print("\n✅ Device is accessible and responding correctly!")
            else:
                print(f"✅ TPS55287 device at 0x{args.address:02x} on bus {args.bus} is working properly.")
            return 0
    
    # If we get here, there are issues
    provide_recommendations(verbose)
    
    if not verbose:
        print(f"❌ Could not access TPS55287 device at 0x{args.address:02x} on bus {args.bus}.")
        print("Run with --verbose for detailed diagnostics.")
    else:
        print("\nDiagnostic complete. Please review the recommendations above.")
    
    return 1

if __name__ == "__main__":
    sys.exit(main())
