import signal
import threading
import os
import sys
from time import sleep, time
import serial
from buildhat import MotorPair, Motor

from ColorSensor import ColorSensor
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor

# Configuration
TEMPO_TIMER = 180         # secondi prima di shutdown
SERIAL_PORT = '/dev/cu.usbmodem142201'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 0.5         # secondi tra i tentativi

# Thread-safe serial lock & shutdown flag
serial_lock = threading.Lock()
shutdown_flag = threading.Event()

# Utility: log con timestamp

def log(msg, level="INFO"):
    ts = time()
    print(f"[{level}] {ts:.3f}: {msg}")


def safe_serial_connect(port=SERIAL_PORT, baud=BAUDRATE, timeout=1.5):
    """Apre la connessione seriale, esce al fallimento dopo tentativi."""
    for attempt in range(1, MAX_SERIAL_RETRIES + 1):
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            log(f"Serial connessa su {port} (tentativo {attempt})")
            return ser
        except Exception as e:
            log(f"Impossibile connettersi ({e})", "WARN")
            sleep(SERIAL_DELAY)
    log("Seriale non disponibile, esco", "ERROR")
    sys.exit(1)

# Connessione iniziale
arduino = safe_serial_connect()

# Inizializza sensori e servo
color1 = ColorSensor(arduino, serial_lock, "COL1", "5")
color2 = ColorSensor(arduino, serial_lock, "COL2", "6")
ultrasonic = UltrasonicSensor(arduino, serial_lock)
servo = ServoMotor(arduino, serial_lock, "SERVO1")


def restart_program():
    log("Riavvio pulito in corso...", "SYSTEM")
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


def termina_programma():
    log("Timer scaduto, shutdown programmato", "SYSTEM")
    shutdown_flag.set()
    with serial_lock:
        try:
            arduino.write(b'3')
        except Exception:
            pass
    restart_program()


def check_shutdown():
    """Thread che ascolta comando shutdown da Arduino"""
    buffer = bytearray()
    search_term = b"SYS|3"

    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                # Leggi tutto il dato disponibile in un solo colpo
                chunk = arduino.read(arduino.in_waiting or 1)

            if chunk:
                buffer.extend(chunk)

                # Cerchiamo SYS|3 come messaggio completo
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if search_term in line:
                        log("Ricevuto comando shutdown da Arduino", "SYSTEM")
                        shutdown_flag.set()
                        restart_program()
                        return

                # Limitiamo la dimensione del buffer
                if len(buffer) > 300:
                    buffer = buffer[-300:]
            else:
                sleep(0.005)  # Pausa breve per evitare uso intensivo della CPU
        except Exception as e:
            log(f"Errore monitor shutdown: {e}", "ERROR")

    log("Shutdown attivato da flag, uscita da check_shutdown", "DEBUG")

def read_until_success(callable_fn, *args, **kwargs):
    """Invoca callable_fn finché non restituisce un valore o scatta shutdown."""
    while not shutdown_flag.is_set():
        try:
            return callable_fn(*args, **kwargs)
        except Exception as e:
            name = getattr(callable_fn, '__name__', callable_fn.__class__.__name__)
            log(f"Errore in funzione '{name}': {e}. Riprovo...", "WARN")
            sleep(0.1)
    log("Shutdown attivato, uscita da retry loop", "DEBUG")
    restart_program()


def handshake_arduino():
    log("Handshake con Arduino...", "SYSTEM")
    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                arduino.reset_input_buffer()
                arduino.write(b'1\n')
                arduino.flush()
                response = arduino.read_until(b'SYS|1\n').decode().strip()
            if response == "SYS|1":
                log("Handshake riuscito", "SYSTEM")
                return
            log(f"Handshake risp inesperata: {response}", "WARN")
        except Exception as e:
            log(f"Errore handshake: {e}", "ERROR")
        sleep(0.5)
    restart_program()


def wait_for_start():
    log("In attesa del comando START...", "SYSTEM")
    buffer = bytearray()
    search_term = b"SYS|2"
    start_ts = time()
    TIMEOUT = 15

    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                chunk = arduino.read(arduino.in_waiting or 1)

            if chunk:
                buffer.extend(chunk)

                # Analizza le linee complete
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if search_term in line:
                        log("Comando START ricevuto", "SUCCESS")
                        return

                # Limitiamo il buffer
                if len(buffer) > 300:
                    buffer = buffer[-300:]
            else:
                sleep(0.005)

            if time() - start_ts > TIMEOUT:
                log("Timeout attesa START, riavvio", "ERROR")
                restart_program()
        except Exception as e:
            log(f"Errore wait_for_start: {e}", "ERROR")
            restart_program()


def main_execution():
    log("Avvio routine principale", "SYSTEM")
    # Timer di emergenza
    timer = threading.Timer(TEMPO_TIMER, termina_programma)
    timer.daemon = True
    timer.start()
    # Thread monitor
    monitor = threading.Thread(target=check_shutdown)
    monitor.daemon = True
    monitor.start()

    # Esempio angolo iniziale
    read_until_success(servo.set_angle, 120)

    # Ciclo infinito di lettura sensori
    while not shutdown_flag.is_set():
        """distance = ultrasonic.get_distance()
        log(f"Colore2: {distance}", "DATA")
        
        colore1 = read_until_success(color1.get_color)
        log(f"Colore1: {colore1}", "DATA")
        
        distanza = read_until_success(ultra.get_distance)
        log(f"Distanza: {distanza}", "DATA")

        # Piccola pausa per alleggerire loop
        sleep(0.05)"""



def main():
    signal.signal(signal.SIGINT, lambda s, f: shutdown_flag.set())
    try:
        handshake_arduino()
        wait_for_start()
        main_execution()
    except Exception as e:
        log(f"Errore critico: {e}", "ERROR")
        restart_program()
    finally:
        shutdown_flag.set()
        try:
            color2.shutdown()
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
