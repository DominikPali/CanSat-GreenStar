"""
CanSat Mission Data Logger
==========================
Logs telemetry packets from Arduino M0 receiver to CSV file for post-flight analysis.

Expected packet format (CSV):
timestamp_ms,tempIn,tempOut,pressure,lat,lon,altitude_AGL,descent

Example:
12340,15.23,14.56,1004.56,52.230145,21.010532,98.543,0

Author: CanSat Team
Date: 2026-02-08
"""

import serial
import serial.tools.list_ports
import time
import os
import sys
from datetime import datetime
from pathlib import Path


class MissionLogger:
    def __init__(self, port=None, baud_rate=115200, log_dir="./mission_logs"):
        """
        Initialize mission logger.
        
        Args:
            port: Serial port (e.g., 'COM3', '/dev/ttyACM0'). If None, will auto-detect.
            baud_rate: Serial baud rate (default 115200 to match receiver)
            log_dir: Directory to save log files
        """
        self.port = port
        self.baud_rate = baud_rate
        self.log_dir = Path(log_dir)
        self.ser = None
        self.log_file = None
        self.backup_file = None
        
        # Statistics
        self.packets_received = 0
        self.packets_invalid = 0
        self.packets_no_gps = 0
        self.start_time = None
        self.last_packet_time = None
        self.gps_locked = False
        
        # Latest data
        self.latest_data = {}
        
        # Create log directory
        self.log_dir.mkdir(exist_ok=True)
        
    def list_available_ports(self):
        """List all available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in ports]
    
    def auto_detect_port(self):
        """Try to auto-detect Arduino M0 port."""
        ports = serial.tools.list_ports.comports()
        
        # Look for Arduino M0 keywords
        keywords = ['Arduino', 'M0', 'USB', 'Serial', 'ACM', 'usbmodem']
        
        for port in ports:
            desc = port.description.lower()
            device = port.device.lower()
            
            for keyword in keywords:
                if keyword.lower() in desc or keyword.lower() in device:
                    print(f"  Found potential Arduino: {port.device} - {port.description}")
                    return port.device
        
        return None
    
    def select_port_interactive(self):
        """Let user select port interactively."""
        ports = self.list_available_ports()
        
        if not ports:
            print("ERROR: No serial ports found!")
            return None
        
        print("\nAvailable serial ports:")
        for i, (device, desc) in enumerate(ports, 1):
            print(f"  {i}. {device} - {desc}")
        
        while True:
            try:
                choice = input(f"\nSelect port (1-{len(ports)}) or 'q' to quit: ").strip()
                if choice.lower() == 'q':
                    return None
                
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    return ports[idx][0]
                else:
                    print(f"Invalid choice. Enter 1-{len(ports)}")
            except (ValueError, KeyboardInterrupt):
                print("\nInvalid input.")
        
    def connect(self):
        """Connect to serial port."""
        # Determine port
        if self.port is None:
            print("No port specified. Attempting auto-detection...")
            self.port = self.auto_detect_port()
            
            if self.port is None:
                print("Auto-detection failed.")
                self.port = self.select_port_interactive()
                
                if self.port is None:
                    print("No port selected. Exiting.")
                    return False
        
        # Connect
        try:
            print(f"\nConnecting to {self.port} at {self.baud_rate} baud...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1
            )
            
            # Wait for Arduino to initialize
            time.sleep(2)
            
            # Flush any old data
            self.ser.reset_input_buffer()
            
            print(f"✓ Connected to {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def create_log_file(self, custom_name=None):
        """Create timestamped log file with header."""
        if custom_name:
            filename = f"{custom_name}.csv"
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"mission_{timestamp}.csv"
        
        filepath = self.log_dir / filename
        
        # Create backup file path
        backup_filepath = self.log_dir / f"{filepath.stem}_backup.csv"
        
        try:
            # Main log file
            self.log_file = open(filepath, 'w', buffering=1)  # Line buffered
            
            # Backup file
            self.backup_file = open(backup_filepath, 'w', buffering=1)
            
            # Write header
            header = "timestamp_ms,tempIn,tempOut,pressure,lat,lon,altitude_AGL,descent\n"
            self.log_file.write(header)
            self.backup_file.write(header)
            
            print(f"✓ Created log file: {filepath}")
            print(f"✓ Created backup file: {backup_filepath}")
            
            return str(filepath)
            
        except IOError as e:
            print(f"✗ Failed to create log file: {e}")
            return None
    
    def parse_packet(self, line):
        """
        Parse CSV packet from Arduino.

        Supported formats:
        1. Received (RSSI = -62): timestamp_ms,tempIn,tempOut,pressure,lat,lon,altitude_AGL,descent
        2. Data: timestamp_ms,tempIn,tempOut,pressure,lat,lon,altitude_AGL,descent
        3. Raw CSV: timestamp_ms,tempIn,tempOut,pressure,lat,lon,altitude_AGL,descent

        Returns:
            dict with parsed data, "skip" for known non-data lines, or None if invalid
        """
        try:
            # Clean line
            line = line.strip()
            
            # Skip empty lines
            if not line:
                return None
            
            # Skip known non-data lines
            skip_patterns = [
                "RX timeout",
                "CRC error",
                "Receive failed",
                "Serial ready",
                "LoRa",
                "Radio init"
            ]

            for pattern in skip_patterns:
                if pattern in line:
                    return "skip"

            # Extract CSV data from "Received (RSSI = -XX): data..." format
            if line.startswith("Received ("):
                colon_idx = line.find("): ")
                if colon_idx != -1:
                    line = line[colon_idx + 3:]
                else:
                    return "skip"
            # Also support old "Data: " prefix format
            elif line.startswith("Data: "):
                line = line[6:]
            # Skip any other non-data lines
            elif line and not line[0].isdigit() and line[0] != '-':
                return "skip"
            
            # Split CSV
            parts = line.split(',')
            
            # Validate field count
            if len(parts) != 8:
                return None
            
            # Parse fields
            data = {
                'timestamp_ms': int(parts[0]),
                'tempIn': float(parts[1]),
                'tempOut': float(parts[2]),
                'pressure': float(parts[3]),
                'lat': float(parts[4]),
                'lon': float(parts[5]),
                'altitude_AGL': float(parts[6]),
                'descent': int(parts[7])
            }
            
            # Validate GPS coordinates
            # Check if GPS has valid fix (not 0.0, 0.0)
            if abs(data['lat']) < 0.001 and abs(data['lon']) < 0.001:
                data['gps_valid'] = False
            else:
                data['gps_valid'] = True
            
            # Convert timestamp to seconds
            data['timestamp_s'] = data['timestamp_ms'] / 1000.0
            
            return data
            
        except (ValueError, IndexError) as e:
            return None
    
    def log_packet(self, data):
        """Write packet to log file."""
        if self.log_file and data:
            # Format CSV line
            line = f"{data['timestamp_ms']},{data['tempIn']:.2f},{data['tempOut']:.2f}," \
                   f"{data['pressure']:.2f},{data['lat']:.6f},{data['lon']:.6f}," \
                   f"{data['altitude_AGL']:.3f},{data['descent']}\n"
            
            # Write to both files
            self.log_file.write(line)
            if self.backup_file:
                self.backup_file.write(line)
    
    def display_status(self, data):
        """Display real-time status on screen."""
        # Clear screen (works on most terminals)
        print("\033[2J\033[H", end='')
        
        # Calculate mission duration
        if self.start_time:
            duration = time.time() - self.start_time
            duration_str = time.strftime("%H:%M:%S", time.gmtime(duration))
        else:
            duration = 0
            duration_str = "00:00:00"

        # Calculate packet rate
        if self.packets_received > 0 and duration > 0:
            packet_rate = self.packets_received / duration
        else:
            packet_rate = 0.0
        
        # GPS lock status
        gps_status = "LOCKED" if self.gps_locked else "NO FIX"
        gps_color = "\033[92m" if self.gps_locked else "\033[91m"  # Green if locked, red if not
        reset_color = "\033[0m"
        
        # Header
        print("=" * 80)
        print("CANSAT MISSION DATA LOGGER")
        print("=" * 80)
        
        # Staleness check
        if self.last_packet_time:
            stale_seconds = time.time() - self.last_packet_time
        else:
            stale_seconds = 0

        if stale_seconds > 5:
            stale_warn = f"\033[91m  ⚠ NO DATA FOR {stale_seconds:.0f}s ⚠\033[0m"
            status_label = f"📡 STATUS: \033[91mNO SIGNAL\033[0m"
        else:
            stale_warn = ""
            status_label = "📡 STATUS: LOGGING"

        # Status
        print(f"\n{status_label}")
        if stale_warn:
            print(stale_warn)
        print(f"   Duration:        {duration_str}")
        print(f"   Packets RX:      {self.packets_received}")
        print(f"   Invalid:         {self.packets_invalid} ({self.packets_invalid/(self.packets_received+self.packets_invalid+0.001)*100:.1f}%)")
        print(f"   No GPS:          {self.packets_no_gps}")
        print(f"   Rate:            {packet_rate:.1f} packets/sec")

        # Latest data
        if data:
            stale_tag = f" \033[91m(STALE)\033[0m" if stale_seconds > 5 else ""
            print(f"\n📊 LATEST DATA:{stale_tag}")
            print(f"   Time:            {data['timestamp_s']:.3f} s")
            print(f"   GPS:             {gps_color}{gps_status}{reset_color}")
            print(f"   Position:        {data['lat']:.6f}°N, {data['lon']:.6f}°E")
            print(f"   Altitude AGL:    {data['altitude_AGL']:.2f} m")
            print(f"   Pressure:        {data['pressure']:.2f} hPa")
            print(f"   Temp (In):       {data['tempIn']:.2f} °C")
            print(f"   Temp (Out):      {data['tempOut']:.2f} °C")
            print(f"   Descent:         {'YES' if data['descent'] else 'NO'}")
        
        # Instructions
        print(f"\n" + "=" * 80)
        print("Press 'q' to stop logging" if sys.platform == 'win32' else "Press 'q' + ENTER to stop logging")
        print("=" * 80)
        print()
    
    def check_for_quit(self):
        """Check if user pressed 'q' to quit (non-blocking)."""
        if sys.platform == 'win32':
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                return key.lower() == 'q'
        else:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                return key == 'q'

        return False
    
    def run(self, custom_name=None, display_interval=1.0):
        """
        Main logging loop.
        
        Args:
            custom_name: Custom name for log file
            display_interval: How often to update display (seconds)
        """
        # Connect to serial port
        if not self.connect():
            return
        
        # Create log file
        log_path = self.create_log_file(custom_name)
        if not log_path:
            return
        
        print("\n" + "=" * 80)
        print("MISSION LOGGING STARTED")
        print("=" * 80)
        print("\nWaiting for packets...")
        print("Press CTRL+C to stop logging\n")
        
        self.start_time = time.time()
        last_display_time = 0
        
        try:
            while True:
                # Check for user quit (CTRL+C or 'q')
                if self.check_for_quit():
                    print("\nUser requested stop.")
                    break
                
                # Read line from serial
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        # Parse packet (parser handles RadioLib format)
                        data = self.parse_packet(line)

                        if data == "skip" or data is None and not line:
                            # Known non-data line or empty line, ignore
                            pass
                        elif isinstance(data, dict):
                            # Valid packet
                            self.packets_received += 1
                            self.last_packet_time = time.time()
                            self.latest_data = data

                            # Check GPS status
                            if data['gps_valid']:
                                self.gps_locked = True
                            else:
                                self.packets_no_gps += 1

                            # Log to file
                            self.log_packet(data)

                        elif line:  # Non-empty but truly invalid
                            self.packets_invalid += 1
                    
                    except UnicodeDecodeError:
                        self.packets_invalid += 1
                        continue
                
                # Update display periodically
                current_time = time.time()
                if current_time - last_display_time >= display_interval:
                    self.display_status(self.latest_data)
                    last_display_time = current_time
                
                # Small delay to prevent CPU spinning
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n\nLogging stopped by user (CTRL+C)")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Close files and serial connection, print summary."""
        print("\n" + "=" * 80)
        print("MISSION LOGGING COMPLETE")
        print("=" * 80)
        
        # Calculate statistics
        if self.start_time:
            duration = time.time() - self.start_time
            duration_str = time.strftime("%H:%M:%S", time.gmtime(duration))
        else:
            duration_str = "00:00:00"
        
        print(f"\n📊 SUMMARY:")
        print(f"   Duration:          {duration_str}")
        print(f"   Packets received:  {self.packets_received}")
        print(f"   Packets invalid:   {self.packets_invalid}")
        print(f"   Packets no GPS:    {self.packets_no_gps}")
        
        if self.packets_received > 0:
            success_rate = (self.packets_received / (self.packets_received + self.packets_invalid)) * 100
            gps_rate = ((self.packets_received - self.packets_no_gps) / self.packets_received) * 100
            print(f"   Success rate:      {success_rate:.1f}%")
            print(f"   GPS lock rate:     {gps_rate:.1f}%")
        
        # Close files
        if self.log_file:
            self.log_file.close()
            print(f"\n✓ Main log file saved")
        
        if self.backup_file:
            self.backup_file.close()
            print(f"✓ Backup file saved")
        
        # Close serial
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"✓ Serial port closed")
        
        print("\n" + "=" * 80)
        print("Log files are ready for post-flight analysis!")
        print("=" * 80 + "\n")


def main():
    """Main entry point."""
    print("\n" + "=" * 80)
    print("CanSat Mission Data Logger v1.0")
    print("=" * 80)
    
    # Configuration
    PORT = None  # Auto-detect
    BAUD_RATE = 115200  # Must match receiver Arduino
    LOG_DIR = "./mission_logs"
    
    # Create logger
    logger = MissionLogger(
        port=PORT,
        baud_rate=BAUD_RATE,
        log_dir=LOG_DIR
    )
    
    # Ask for custom mission name (optional)
    print("\nMission name (press ENTER for auto timestamp): ", end='')
    try:
        custom_name = input().strip()
        if not custom_name:
            custom_name = None
    except KeyboardInterrupt:
        print("\nAborted.")
        return
    
    # Run logger
    logger.run(custom_name=custom_name)



if __name__ == "__main__":
    main()
