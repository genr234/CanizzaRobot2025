import serial
import time
from buildhat import MotorPair, Motor
import threading

arudino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

route = MotorPair('A', 'B')
pinza_spike = Motor('C')

colore1 = None
colore2 = None
distanza_pros = None

dati_lock = threading.Lock()

def send_to_arduino(command):
    arudino.write(f"{command}\n".encode())

def send_to_spike(command):
    spike.write(f"{command}\n".encode())

def recive_to_spike():
    global colore1, colore2, distanza_pros
    if spike.in_waiting:
        riga = spike.readline().decode().strip()
        try:
            parti = riga.split("|")
            dati = {}
            for parte in parti:
                chiave, valore = parte.split(":")
                dati[chiave] = valore

            colore1 = dati.get("COL1")
            colore2 = dati.get("COL2")
            distanza_pros = float(dati.get("DIST"))
        except Exception as e:
            print(f"Errore nel parsing dati Spike: {e}")
        time.sleep(0.05)

def main():
    return 1

read_from_spike_thread = threading.Thread(target=recive_to_spike(), daemon=True)
read_from_spike_thread.start()

main()