import signal
import threading
import os
import sys
from time import sleep, time
import serial
from buildhat import MotorPair, Motor

from ColorSensor import ColorSensor
from SensorError import SensorError
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor

#Colori 0 - Sconosciuto | 1 - Bianco | 2 - Rosso | 3 - Scuro | 4 - Verde | 5 - Blu | 6 - Giallo

TEMPO_TIMER = 180
MAX_RETRIES = 10
SERIAL_TIMEOUT = 1.5

serial_lock = threading.Lock()
shutdown_flag = threading.Event()

def safe_serial_connect(port='/dev/cu.usbmodem142201', baud=115200, timeout=SERIAL_TIMEOUT):
    """Connessione seriale con riprova incorporata e gestione generica"""
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            print(f"[Serial] Connesso con successo (tentativo {attempt})")
            return ser
        except serial.SerialException as e:
            print(f"[Serial] Errore connessione (tentativo {attempt}): {e}")
        except Exception as e:
            print(f"[Serial] Errore generico (tentativo {attempt}): {e}")
        sleep(2)
    print("[Serial] Connessione fallita dopo diversi tentativi, esco")
    sys.exit(1)

arduino = safe_serial_connect()
color_sensor_1 = ColorSensor(arduino, serial_lock, "COL1", "5")
color_sensor_2 = ColorSensor(arduino, serial_lock, "COL2", "6")
ultrasonic_sensor = UltrasonicSensor(arduino, serial_lock, "DIST", "4")
servo = ServoMotor(arduino, serial_lock, "SERVO1")

def safe_get_color(sensor):
    try:
        color = sensor.get_color()
        print(f"[DEBUG] {sensor.sensor_id} risposta: {color}")  # Aggiungi debug
        return color
    except SensorError as e:
        print(f"[ERR] {sensor.sensor_id}: {e.message}")
        return "Sconosciuto"

def safe_get_distance(sensor):
    try:
        return sensor.get_distance()
    except SensorError as e:
        print(f"[SAFE] Errore sensore {sensor.sensor_id}: {e.message}")
        return "Sconosciuto"
    except Exception as e:
        print(f"[SAFE] Errore inaspettato get_color: {e}")
        return "Sconosciuto"


def restart_program():
    """Riavvio sicuro con pulizia delle risorse"""
    print("\n[System] Riavvio pulito in corso...")
    shutdown_flag.set()
    with serial_lock:
        try:
            if arduino.is_open:
                arduino.write(b'0')
                arduino.flush()
                arduino.close()
        except Exception:
            pass
    os.execl(sys.executable, sys.executable, *sys.argv)


def check_shutdown():
    """Monitoraggio continuo per comandi di shutdown"""
    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                data = arduino.read(100)
                if b"SYS|3" in data:
                    print("[System] Ricevuto comando di shutdown da Arduino")
                    shutdown_flag.set()
        except Exception as e:
            print(f"[System] Errore monitor seriale: {e}")
        sleep(0.01)
    restart_program()


def termina_programma():
    """Shutdown immediato con priorità al riavvio"""
    print("[System] Iniziando shutdown programmato...")
    shutdown_flag.set()
    with serial_lock:
        try:
            arduino.write(b'3')
        except Exception:
            pass
    restart_program()


def handshake_arduino():
    """Sincronizzazione iniziale con Arduino"""
    print("[System] Handshake con Arduino...")
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            with serial_lock:
                arduino.reset_input_buffer()
                arduino.write(b'1\n')  # Aggiunto newline
                arduino.flush()
                response = arduino.read_until(b'SYS|1\n').decode().strip()

            if response == "SYS|1":
                print(f"[System] Handshake riuscito (tentativo {attempt})")
                return
            else:
                print(f"[System] Risposta inattesa: '{response}' (atteso SYS|1)")

        except Exception as e:
            print(f"[System] Errore handshake (tentativo {attempt}): {e}")
        sleep(0.5)
    print("[System] Handshake fallito, riavvio")
    restart_program()


def wait_for_start():
    """Attesa robusta del comando start con buffer dedicato"""
    print("[System] In attesa comando start...")
    buffer = bytearray()
    start_time = time()
    COMMAND = b'SYS|2'
    TIMEOUT = 15  # secondi

    while not shutdown_flag.is_set():
        try:
            # 1. Lettura chunk con timeout
            with serial_lock:
                chunk = arduino.read(arduino.in_waiting or 1)
                if chunk:
                    buffer.extend(chunk)

                    # 2. Pulizia buffer (mantieni solo gli ultimi 100 bytes)
                    if len(buffer) > 100:
                        buffer = buffer[-100:]

                    # 3. Ricerca efficiente del comando
                    if COMMAND in buffer:
                        print("[SUCCESS] Comando start ricevuto")
                        buffer.clear()
                        return

            # 4. Controllo timeout
            if time() - start_time > TIMEOUT:
                print(f"[ERROR] Timeout dopo {TIMEOUT}s. Buffer finale: {buffer.hex()}")
                restart_program()

            # 5. Sleep ottimizzato

        except Exception as e:
            print(f"[CRITICAL] Errore: {str(e)}")
            restart_program()

def main_execution():
    """Logica operativa principale"""
    print("[Main] Start routine principale")
    # Timer di emergenza
    timer = threading.Timer(TEMPO_TIMER, termina_programma)
    timer.daemon = True
    timer.start()
    # Thread monitor shutdown
    monitor = threading.Thread(target=check_shutdown)
    monitor.daemon = True
    monitor.start()

    servo.set_angle(120)
    while True:
        nome_colore = color_sensor_2.get_color()
        print(f"Rilevato: {nome_colore}")

        print(f"Rilevato: {color_sensor_1.get_color()}")

        print(f"Rilevato: {ultrasonic_sensor.get_distance()}")
    sleep(150)


def main():
    """Funzione principale strutturata"""
    signal.signal(signal.SIGINT, lambda s, f: shutdown_flag.set())
    try:
        handshake_arduino()
        wait_for_start()
        main_execution()
    except Exception as e:
        print(f"[System] Errore critico in main: {e}")
        restart_program()
    finally:
        shutdown_flag.set()
        try:
            color_sensor_2.shutdown()
        except:
            pass
        with serial_lock:
            try:
                if arduino.is_open:
                    arduino.close()
            except:
                pass

if __name__ == "__main__":
    main()
