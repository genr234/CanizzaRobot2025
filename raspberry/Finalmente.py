#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import signal
import threading
from time import sleep, time
from datetime import datetime
import queue

# Librerie esterne
import serial
import buildhat
from colorama import Fore, Back, Style, init as colorama_init

# Moduli interni (assicurati di averli!)
from ColorSensorA import ColorSensorA
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor
from robot import Robot
from buildhat import Motor

# ========== Costanti configurazione ==========
TEMPO_TIMER = 210  # Durata gara
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 0.5
TIMEOUT_START = 6

# ========== Colori logging ==========
colorama_init(autoreset=True)
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED + Back.WHITE,
    "SYSTEM": Fore.MAGENTA,
    "SUCCESS": Fore.GREEN + Style.BRIGHT,
    "DATA": Fore.WHITE,
    "RESET": Style.RESET_ALL,
    "PARKING": Fore.BLUE + Style.BRIGHT,
}

def log(msg, level="INFO"):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, Fore.WHITE)
    reset = COLORI_LOG["RESET"]
    print(f"{colore}[{level}] {ts}: {msg}{reset}")

# ========== Variabili globali ==========
serial_lock = threading.Lock()
shutdown_flag = threading.Event()
eventi = queue.Queue()
officina_attiva = None

# ========== Mapping colori ==========
COLOR_MAPPING = {
    "rosso": ((150, 255), (0, 80), (0, 80)),
    "verde": ((0, 80), (150, 255), (0, 80)),
    "giallo": ((150, 255), (150, 255), (0, 80)),
    "blu": ((0, 80), (0, 80), (150, 255)),
}
class AutoColor:
    ROSSO = "rosso"
    VERDE = "verde"
    GIALLO = "giallo"
    BLU = "blu"
    SCONOSCIUTO = "sconosciuto"
class OfficineColor:
    VERDE = "verde"
    GIALLO = "giallo"
    SCONOSCIUTO = "sconosciuto"

POSIZIONI = {
    "partenza": (0, 0),
    "officina_verde": (230, 150),
    "officina_gialla": (20, 150),
    "colonnina1": (20, 50),
    "colonnina2": (230, 50),
    "colonnina3": (125, 230),
    "area_parcheggio": (125, 125),
}

# ===================
# Utils hardware e seriale
# ===================
def safe_serial_connect(port=SERIAL_PORT, baud=BAUDRATE, timeout=1.5):
    for attempt in range(1, MAX_SERIAL_RETRIES + 1):
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            log(f"Connessione seriale riuscita su {port} (tentativo {attempt})", "SYSTEM")
            return ser
        except (serial.SerialException, OSError) as e:
            log(f"Errore connessione seriale: {str(e)}", "WARN")
            sleep(SERIAL_DELAY)
    log("Connessione seriale non disponibile", "ERROR")
    sys.exit(1)

def inizializza_sensore(sensore, nome_sensore):
    try:
        log(f"Inizializzazione sensore {nome_sensore}...", "INFO")
        return sensore
    except Exception as e:
        log(f"Errore inizializzazione sensore {nome_sensore}: {str(e)}", "ERROR")
        risposta = input(f"Vuoi continuare senza {nome_sensore}? (y/n): ")
        if risposta.lower() == 'y':
            log(f"Bypass sensore {nome_sensore}.", "WARN")
            return None
        else:
            log(f"Abbandono causa errore {nome_sensore}.", "CRITICAL")
            sys.exit(1)

def handshake_arduino(arduino):
    log("Avvio handshake con Arduino...", "SYSTEM")
    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                arduino.reset_input_buffer()
                arduino.write(b'1\n')
                arduino.flush()
                response = arduino.read_until(b'SYS|1\n').decode().strip()
            if response == "SYS|1":
                log("Handshake completato", "SUCCESS")
                return
            elif response != "":
                log(f"Risposta inattesa handshake: {response}", "WARN")
        except Exception as e:
            log(f"Errore durante handshake: {str(e)}", "ERROR")
        sleep(0.5)

def wait_for_start(arduino):
    log("Attesa comando START...", "SYSTEM")
    buffer = bytearray()
    start_time = time()
    while not shutdown_flag.is_set():
        try:
            with serial_lock:
                data = arduino.read(arduino.in_waiting or 1)
            if data:
                buffer.extend(data)
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    if b"SYS|2" in line:
                        log("Ricevuto comando START", "SUCCESS")
                        return
                if len(buffer) > 300:
                    buffer = buffer[-300:]
            sleep(0.01)
            if time() - start_time > TIMEOUT_START:
                log("Timeout attesa START", "ERROR")
                sys.exit(1)
        except Exception as e:
            log(f"Errore attesa START: {str(e)}", "ERROR")
            sys.exit(1)

def termina_programma():
    """Spegne il sistema in modo controllato"""
    log("Avvio shutdown programmato...", "SYSTEM")
    shutdown_flag.set()
    try:
        robot.stop_movimento()
        with serial_lock:
            arduino.write(b'3\n')
            arduino.flush()
    except Exception as e:
        log(f"Errore invio comando shutdown: {str(e)}", "WARN")
    sys.exit(0)

# ========== Utility colori ==========
def is_color(rgb, target_color):
    if target_color not in COLOR_MAPPING or rgb is None or len(rgb) != 3:
        return False
    r, g, b = rgb
    r_range, g_range, b_range = COLOR_MAPPING[target_color]
    return (r_range <= r <= r_range[1] and 
            g_range <= g <= g_range[1] and 
            b_range <= b <= b_range[1])

# ========== Identificazione e movimento ==========

def identifica_colore_officina():
    """Identifica il colore dell'officina attiva"""
    log("Rilevamento colore officina disponibile...", "PARKING")
    try:
        servo.set_angle(90)
        sleep(0.5)
        rgb = colorLego.get_color()
        log(f"RGB officina rilevato: {rgb}", "DEBUG")
        if is_color(rgb, "verde"):
            log("Officina VERDE attiva", "PARKING")
            return OfficineColor.VERDE
        elif is_color(rgb, "giallo"):
            log("Officina GIALLA attiva", "PARKING")
            return OfficineColor.GIALLO
        else:
            sleep(0.2)
            rgb = colorLego.get_color()
            if is_color(rgb, "verde"):
                return OfficineColor.VERDE
            elif is_color(rgb, "giallo"):
                return OfficineColor.GIALLO
            else:
                log("Impossibile identificare colore officina, default: VERDE", "WARN")
                return OfficineColor.VERDE
    except Exception as e:
        log(f"Errore identificazione officina: {str(e)}", "ERROR")
        return OfficineColor.VERDE

def identifica_colore_auto():
    """Identifica il colore dell'auto di fronte al robot"""
    try:
        servo.set_angle(0)
        sleep(0.3)
        rgb = colorLego.get_color()
        log(f"RGB auto rilevato: {rgb}", "DEBUG")
        for col, enum_val in [("rosso", AutoColor.ROSSO), ("verde", AutoColor.VERDE), ("giallo", AutoColor.GIALLO), ("blu", AutoColor.BLU)]:
            if is_color(rgb, col):
                return enum_val
        # Seconda lettura in caso di dubbio
        sleep(0.2)
        rgb = colorLego.get_color()
        for col, enum_val in [("rosso", AutoColor.ROSSO), ("verde", AutoColor.VERDE), ("giallo", AutoColor.GIALLO), ("blu", AutoColor.BLU)]:
            if is_color(rgb, col):
                return enum_val
        log("Colore auto non identificato", "WARN")
        return AutoColor.SCONOSCIUTO
    except Exception as e:
        log(f"Errore identificazione auto: {str(e)}", "ERROR")
        return AutoColor.SCONOSCIUTO

def muovi_a_posizione(destinazione):
    """Muove il robot verso una posizione specifica (logica semplificata)"""
    log(f"Movimento verso: {destinazione}", "PARKING")
    # Aggiungi qui la logica motion specifica del tuo campo di gara...
    if destinazione == "officina_verde":
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_destra(90, speed=30)
        robot.muovi_avanti_for("seconds", 3, speed=40)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
    elif destinazione == "officina_gialla":
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 3, speed=40)
        robot.gira_destra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
    elif destinazione.startswith("colonnina"):
        robot.muovi_avanti_for("seconds", 2, speed=40)
    elif destinazione == "area_parcheggio":
        robot.gira_destra(180, speed=30)
        robot.muovi_avanti_for("seconds", 2, speed=40)
    elif destinazione == "partenza":
        robot.gira_destra(180, speed=30)
        robot.muovi_avanti_for("seconds", 4, speed=50)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 2, speed=40)
    log(f"Arrivato a {destinazione}", "PARKING")

def evita_ostacolo():
    """Logica per evitare un ostacolo rilevato"""
    log("Rilevato ostacolo, avvio manovra evasiva", "WARN")
    robot.stop_movimento()
    distanza_laterale = ultrasonicLaterale.get_distance()
    if distanza_laterale > 20:
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
    else:
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
    log("Ostacolo evitato", "INFO")

def sensore_ostacoli():
    log("Avvio sensore ostacoli", "SYSTEM")
    while not shutdown_flag.is_set():
        try:
            distanza = ultrasonic.get_distance()
            if distanza < 15:
                eventi.put("ostacolo")
            sleep(0.1)
        except Exception as e:
            log(f"Errore sensore ostacoli: {str(e)}", "ERROR")
            sleep(0.5)

def controlla_ostacoli():
    try:
        evento = eventi.get_nowait()
        if evento == "ostacolo":
            evita_ostacolo()
        return True
    except queue.Empty:
        return False
    except Exception as e:
        log(f"Errore controllo ostacoli: {str(e)}", "ERROR")
        return False

def prendi_auto():
    log("Prelievo auto in corso...", "PARKING")
    servo_alza.set_angle(90)
    sleep(0.5)
    gabbia.run_for_seconds(1, 50)
    log("Auto prelevata con successo", "PARKING")

def rilascia_auto():
    log("Rilascio auto in corso...", "PARKING")
    gabbia.run_for_seconds(1, -50)
    servo_alza.set_angle(0)
    sleep(0.5)
    log("Auto rilasciata con successo", "PARKING")

# ========== MAIN LOGIC ==========
def main_execution():
    global officina_attiva
    log("Avvio missione Smart Parking", "SYSTEM")
    sensor_thread = threading.Thread(target=sensore_ostacoli)
    sensor_thread.daemon = True
    sensor_thread.start()
    safety_timer = threading.Timer(TEMPO_TIMER, termina_programma)
    safety_timer.daemon = True
    safety_timer.start()
    try:
        officina_attiva = identifica_colore_officina()
        log("Movimento verso area di parcheggio centrale", "PARKING")
        robot.muovi_avanti_for("seconds", 2, speed=40)
        auto_gestite = 0
        while auto_gestite < 18 and not shutdown_flag.is_set():
            if controlla_ostacoli():
                continue
            colore_auto = identifica_colore_auto()
            log(f"Auto identificata: {colore_auto}", "PARKING")
            prendi_auto()
            if colore_auto == AutoColor.ROSSO:
                log("Auto ROSSA: destinazione colonnina di ricarica", "PARKING")
                colonnina = f"colonnina{(auto_gestite % 3) + 1}"
                muovi_a_posizione(colonnina)
                rilascia_auto()
            elif colore_auto == AutoColor.BLU:
                log(f"Auto BLU: destinazione officina {officina_attiva}", "PARKING")
                if officina_attiva == OfficineColor.VERDE:
                    muovi_a_posizione("officina_verde")
                else:
                    muovi_a_posizione("officina_gialla")
                rilascia_auto()
            elif colore_auto == AutoColor.GIALLO:
                log("Auto GIALLA: destinazione colonnina di ricarica", "PARKING")
                colonnina = f"colonnina{(auto_gestite % 3) + 1}"
                muovi_a_posizione(colonnina)
                rilascia_auto()
            elif colore_auto == AutoColor.VERDE:
                log("Auto VERDE: resta nel suo stallo", "PARKING")
                rilascia_auto()
            else:
                log("Colore AUTO non identificato, rilascio sul posto", "WARN")
                rilascia_auto()
            auto_gestite += 1
            log(f"Auto gestite: {auto_gestite}/18", "INFO")
            if auto_gestite < 18:
                muovi_a_posizione("area_parcheggio")
        log("Tutte le auto gestite, ritorno all'area di partenza", "PARKING")
        muovi_a_posizione("partenza")
        log("Missione Smart Parking completata con successo!", "SUCCESS")
    except Exception as e:
        log(f"Errore critico durante la missione: {str(e)}", "CRITICAL")
    finally:
        robot.stop_movimento()

# ========== Entry point & chiusura sicura ==========
if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s, f: termina_programma())
    try:
        # ==== Inizializzazione hardware ====
        arduino = safe_serial_connect()
        handshake_arduino(arduino)
        wait_for_start(arduino)
        
        colorLego = inizializza_sensore(buildhat.ColorSensor('A'), "ColorSensorLEGO")
        color1 = inizializza_sensore(ColorSensorA(arduino, serial_lock, "COL1", "5"), "ColorSensorA")
        ultrasonic = inizializza_sensore(UltrasonicSensor(arduino, serial_lock), "UltrasonicSensor")
        ultrasonicLaterale = inizializza_sensore(UltrasonicSensor(arduino, serial_lock, command_code="6", sensor_id="DIST2"), "UltrasonicSensorLaterale")
        servo = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO1"), "ServoMotor Pinza")
        servo_alza = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO2", min_angle=0, max_angle=360), "ServoMotor Alza")
        robot = Robot('C', 'D')
        gabbia = Motor('B')

        # ==== Avvio logica principale ====
        main_execution()

    except Exception as e:
        log(f"Errore critico: {str(e)}", "CRITICAL")
    finally:
        shutdown_flag.set()
        try:
            if 'arduino' in locals() and getattr(arduino, 'is_open', False):
                arduino.close()
        except Exception as e:
            log(f"Errore chiusura seriale: {str(e)}", "WARN")
        log("Applicazione terminata", "SYSTEM")s
