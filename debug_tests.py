#!/usr/bin/env python3
"""
Debug test program to exercise all sensors and features and print out a summary report.
"""
import sys
import os
import threading
import time
import argparse
from datetime import datetime

# Ensure raspberry modules are on the import path
script_dir = os.path.dirname(os.path.abspath(__file__))
raspberry_dir = os.path.join(script_dir, 'raspberry')
if raspberry_dir not in sys.path:
    sys.path.insert(0, raspberry_dir)

try:
    from ColorSensorA import ColorSensorA
    from UltrasonicSensor import UltrasonicSensor
    from ServoMotor import ServoMotor
    from robot import Robot, RobotError
    import serial
except ImportError as e:
    print(f"[ERROR] Missing module: {e}")
    sys.exit(1)

def log(msg, level="INFO"):
    ts = datetime.now().strftime('%H:%M:%S')
    print(f"[{level}] {ts}: {msg}")

class TestSuite:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.results = []
        self.ser = None

    def add_result(self, name, passed, message=''):
        status = "PASS" if passed else "FAIL"
        self.results.append((name, passed, message))
        print(f"{status}: {name}" + (f" - {message}" if message else ""))

    def test_connection(self):
        name = "Serial connection to Arduino"
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            ser.close()
            self.add_result(name, True)
            # Reopen for further tests
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.add_result(name, False, str(e))

    def test_handshake(self):
        name = "Handshake with Arduino"
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # Send handshake request
            self.ser.write(b'1\n')
            self.ser.flush()
            start = time.time()
            buffer = bytearray()
            # Await response
            while time.time() - start < 2:
                if self.ser.in_waiting:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    if b'\n' in buffer:
                        break
                time.sleep(0.01)
            lines = buffer.split(b'\n')
            if any(b'SYS|1' in line for line in lines):
                self.add_result(name, True)
            else:
                text = buffer.decode(errors='ignore').strip()
                self.add_result(name, False, f"No SYS|1, got: '{text}'")
        except Exception as e:
            self.add_result(name, False, str(e))

    def test_color_sensor(self):
        name = "Color sensor reading"
        try:
            sensor = ColorSensorA(self.ser, self.lock, "COL1", "5")
            color = sensor.get_color()
            self.add_result(name, True, f"Detected: {color}")
        except Exception as e:
            self.add_result(name, False, str(e))

    def test_ultrasonic_sensor(self, sensor_id, command_code):
        name = f"Ultrasonic sensor {sensor_id} reading"
        try:
            sensor = UltrasonicSensor(self.ser, self.lock, sensor_id, command_code)
            distance = sensor.get_distance()
            self.add_result(name, True, f"Distance: {distance} cm")
        except Exception as e:
            self.add_result(name, False, str(e))

    def test_servo(self, command_code):
        name = f"Servo {command_code} test"
        try:
            servo = ServoMotor(self.ser, self.lock, command_code)
            # Test a few angles
            for angle in (0, 90, 180):
                try:
                    servo.set_angle(angle)
                    time.sleep(0.5)
                except Exception as e:
                    raise RuntimeError(f"Angle {angle}° failed: {e}")
            # Clean up monitor thread
            try:
                servo.shutdown()
            except Exception:
                pass
            self.add_result(name, True)
        except Exception as e:
            self.add_result(name, False, str(e))

    def test_robot(self):
        # Initialization
        name_init = "Robot initialization"
        try:
            robot = Robot('C', 'D')
            self.add_result(name_init, True)
        except Exception as e:
            self.add_result(name_init, False, str(e))
            return
        # Test default_speed setter and getter
        name_speed = "Robot default_speed setter"
        try:
            robot.default_speed = 80
            try:
                robot.default_speed = -10
                self.add_result(name_speed, False, "Invalid speed did not raise")
            except ValueError:
                self.add_result(name_speed, True)
        except Exception as e:
            self.add_result(name_speed, False, str(e))
        # Test context manager
        name_ctx = "Robot context manager"
        try:
            with Robot('C', 'D'):
                pass
            self.add_result(name_ctx, True)
        except Exception as e:
            self.add_result(name_ctx, False, str(e))

    def print_summary(self):
        passed = sum(1 for _, p, _ in self.results if p)
        total = len(self.results)
        print(f"\nSummary: {passed}/{total} tests passed")
        if passed < total:
            print("Failures:")
            for name, p, msg in self.results:
                if not p:
                    print(f"- {name}: {msg}")

    def run_all(self):
        self.test_connection()
        if not self.ser:
            print("Serial connection failed, aborting remaining tests.")
            return
        self.test_handshake()
        self.test_color_sensor()
        self.test_ultrasonic_sensor("DIST", "4")
        self.test_ultrasonic_sensor("DIST2", "6")
        self.test_servo("SERVO1")
        self.test_servo("SERVO2")
        self.test_robot()
        self.print_summary()

def main():
    parser = argparse.ArgumentParser(description="Debug test for sensors and robot features")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port for Arduino")
    parser.add_argument("--baud", default=115200, type=int, help="Baudrate for serial connection")
    args = parser.parse_args()
    ts = TestSuite(args.port, args.baud)
    ts.run_all()

if __name__ == "__main__":
    main()