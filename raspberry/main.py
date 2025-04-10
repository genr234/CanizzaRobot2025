import serial
from buildhat import MotorPair, Motor, BuildHAT

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ruote = MotorPair('A', 'B')
pinza_spike = Motor('C')
print("Sto inviando\n")
arduino.write(b'1')
print("Ho inviato\n")
pinza_spike.run_for_seconds(2)
print("Ciaop")