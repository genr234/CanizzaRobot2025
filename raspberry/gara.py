#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Parking - Soluzione per la sfida del robot parcheggiatore
"""

import signal
import threading
import sys
from time import sleep, time
from datetime import datetime
from enum import Enum
import queue

# Importazione librerie esterne
import serial
import buildhat
from colorama import Fore, Style, init as colorama_init

# Importazione moduli interni
from ColorSensorA import ColorSensorA
from ServoMotor import ServoMotor
from UltrasonicSensor import UltrasonicSensor
from buildhat import Motor, ColorSensor
from robot import Robot

# Inizializzazione colorama per colori cross-platform
colorama_init(autoreset=True)

# --------------------------
# COSTANTI DI CONFIGURAZIONE
# --------------------------
TEMPO_TIMER = 210  # Durata della gara (secondi)
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
MAX_SERIAL_RETRIES = 5
SERIAL_DELAY = 0.5  # Pausa tra tentativi (secondi)

# --------------------------
# CONFIGURAZIONE LOGGER
# --------------------------
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED + Style.BRIGHT,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "SYSTEM": Fore.MAGENTA,
    "PARKING": Fore.BLUE + Style.BRIGHT,
}

# --------------------------
# DEFINIZIONE COSTANTI
# --------------------------
class AutoColor(Enum):
    ROSSO = "rosso"
    VERDE = "verde"
    GIALLO = "giallo"
    BLU = "blu"
    SCONOSCIUTO = "sconosciuto"

class OfficineColor(Enum):
    VERDE = "verde"
    GIALLO = "giallo"
    SCONOSCIUTO = "sconosciuto"

# Mappatura dei valori RGB ai colori delle auto
COLOR_MAPPING = {
    "rosso": ((150, 255), (0, 80), (0, 80)),     # Range RGB per rosso
    "verde": ((0, 80), (150, 255), (0, 80)),     # Range RGB per verde
    "giallo": ((150, 255), (150, 255), (0, 80)), # Range RGB per giallo
    "blu": ((0, 80), (0, 80), (150, 255)),       # Range RGB per blu
    "fucsia": ((150, 255), (0, 80), (150, 255)), # Range RGB per fucsia (ostacoli)
}

# Posizioni nel campo di gioco
POSIZIONI = {
    "partenza": (0, 0),
    "officina_verde": (230, 150),
    "officina_gialla": (20, 150),
    "colonnina1": (20, 50),
    "colonnina2": (230, 50),
    "colonnina3": (125, 230),
    "area_parcheggio": (125, 125),
}

# --------------------------
# VARIABILI GLOBALI
# --------------------------
serial_lock = threading.Lock()
shutdown_flag = threading.Event()
officina_attiva = OfficineColor.SCONOSCIUTO
eventi = queue.Queue()

# --------------------------
# FUNZIONI DI UTILITY
# --------------------------
def log(msg, level="INFO"):
    """Registra messaggi con timestamp e colori

    Args:
        msg (str): Messaggio da registrare
        level (str): Livello di gravità
    """
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    reset = Style.RESET_ALL
    print(f"{colore}[{level}] {ts}: {msg}{reset}")

def safe_serial_connect(port=SERIAL_PORT, baud=BAUDRATE, timeout=1.5):
    """Crea connessione seriale con sistema di retry

    Returns:
        serial.Serial: Oggetto seriale connesso

    Raises:
        SystemExit: Se fallisce dopo MAX_SERIAL_RETRIES tentativi
    """
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

def termina_programma():
    """Spegne il sistema in modo controllato"""
    log("Avvio shutdown programmato...", "SYSTEM")
    shutdown_flag.set()
    
    try:
        robot.stop_movimento()
        
        with serial_lock:
            arduino.write(b'3\n')  # Comando spegnimento Arduino
            arduino.flush()
    except Exception as e:
        log(f"Errore invio comando shutdown: {str(e)}", "WARN")
    
    sys.exit(0)

# --------------------------
# FUNZIONI DI RILEVAMENTO E MOVIMENTO
# --------------------------
def identifica_colore_officina():
    """Identifica il colore dell'officina disponibile
    
    Returns:
        OfficineColor: Colore dell'officina attiva
    """
    log("Rilevamento colore officina disponibile...", "PARKING")
    
    # Utilizziamo il sensore di colore Arduino per leggere il colore della parete
    try:
        # Posiziona il sensore verso la parete
        servo.set_angle(90)  # Orienta il servo verso la parete
        sleep(0.5)  # Attendi stabilizzazione
        
        # Leggi il colore con il sensore
        rgb = colorLego.get_color()
        log(f"RGB officina rilevato: {rgb}", "DEBUG")
        
        # Identifica il colore
        if is_color(rgb, "verde"):
            log("Officina VERDE attiva", "PARKING")
            return OfficineColor.VERDE
        elif is_color(rgb, "giallo"):
            log("Officina GIALLA attiva", "PARKING")
            return OfficineColor.GIALLO
        else:
            # Seconda lettura in caso di errore
            sleep(0.2)
            rgb = colorLego.get_color()
            
            if is_color(rgb, "verde"):
                log("Officina VERDE attiva (seconda lettura)", "PARKING")
                return OfficineColor.VERDE
            elif is_color(rgb, "giallo"):
                log("Officina GIALLA attiva (seconda lettura)", "PARKING")
                return OfficineColor.GIALLO
            else:
                log("Impossibile identificare colore officina, default: VERDE", "WARN")
                return OfficineColor.VERDE
    
    except Exception as e:
        log(f"Errore identificazione officina: {str(e)}", "ERROR")
        return OfficineColor.VERDE  # Default in caso di errore

def is_color(rgb, target_color):
    """Verifica se un valore RGB corrisponde a un determinato colore
    
    Args:
        rgb (tuple): Valori RGB (r, g, b)
        target_color (str): Nome del colore da verificare
    
    Returns:
        bool: True se il colore corrisponde, False altrimenti
    """
    if target_color not in COLOR_MAPPING:
        return False
    
    if rgb is None or len(rgb) != 3:
        return False
    
    r, g, b = rgb
    r_range, g_range, b_range = COLOR_MAPPING[target_color]
    
    return (r_range[0] <= r <= r_range[1] and 
            g_range[0] <= g <= g_range[1] and 
            b_range[0] <= b <= b_range[1])

def identifica_colore_auto():
    """Identifica il colore dell'auto di fronte al robot
    
    Returns:
        AutoColor: Colore dell'auto identificata
    """
    try:
        # Posiziona il sensore per leggere il colore dell'auto
        servo.set_angle(0)  # Orienta il servo verso l'auto
        sleep(0.3)  # Attendi stabilizzazione
        
        # Leggi il colore con il sensore
        rgb = colorLego.get_color()
        log(f"RGB auto rilevato: {rgb}", "DEBUG")
        
        # Identifica il colore
        if is_color(rgb, "rosso"):
            return AutoColor.ROSSO
        elif is_color(rgb, "verde"):
            return AutoColor.VERDE
        elif is_color(rgb, "giallo"):
            return AutoColor.GIALLO
        elif is_color(rgb, "blu"):
            return AutoColor.BLU
        else:
            # Seconda lettura in caso di errore
            sleep(0.2)
            rgb = colorLego.get_color()
            
            if is_color(rgb, "rosso"):
                return AutoColor.ROSSO
            elif is_color(rgb, "verde"):
                return AutoColor.VERDE
            elif is_color(rgb, "giallo"):
                return AutoColor.GIALLO
            elif is_color(rgb, "blu"):
                return AutoColor.BLU
            
        log("Colore auto non identificato", "WARN")
        return AutoColor.SCONOSCIUTO
    
    except Exception as e:
        log(f"Errore identificazione auto: {str(e)}", "ERROR")
        return AutoColor.SCONOSCIUTO

def muovi_a_posizione(destinazione):
    """Muove il robot verso una posizione specifica
    
    Args:
        destinazione (str): Nome della posizione di destinazione
    """
    log(f"Movimento verso: {destinazione}", "PARKING")
    
    # Implementazione semplificata - in un robot reale utilizzeremmo
    # algoritmi di navigazione con sensori di distanza per evitare ostacoli
    
    if destinazione == "officina_verde":
        # Sequenza di movimenti per raggiungere l'officina verde
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_destra(90, speed=30)
        robot.muovi_avanti_for("seconds", 3, speed=40)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
    
    elif destinazione == "officina_gialla":
        # Sequenza di movimenti per raggiungere l'officina gialla
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 3, speed=40)
        robot.gira_destra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
    
    elif destinazione == "colonnina1":
        # Sequenza per colonnina 1
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1.5, speed=40)
    
    elif destinazione == "colonnina2":
        # Sequenza per colonnina 2
        robot.muovi_avanti_for("seconds", 2, speed=40)
        robot.gira_destra(90, speed=30)
        robot.muovi_avanti_for("seconds", 1.5, speed=40)
    
    elif destinazione == "colonnina3":
        # Sequenza per colonnina 3
        robot.muovi_avanti_for("seconds", 3, speed=40)
    
    elif destinazione == "area_parcheggio":
        # Torna all'area di parcheggio per prendere un'altra auto
        robot.gira_destra(180, speed=30)
        robot.muovi_avanti_for("seconds", 2, speed=40)
    
    elif destinazione == "partenza":
        # Torna alla posizione di partenza
        robot.gira_destra(180, speed=30)
        robot.muovi_avanti_for("seconds", 4, speed=50)
        robot.gira_sinistra(90, speed=30)
        robot.muovi_avanti_for("seconds", 2, speed=40)
    
    log(f"Arrivato a {destinazione}", "PARKING")

def evita_ostacolo():
    """Implementa la logica per evitare un ostacolo rilevato"""
    log("Rilevato ostacolo, avvio manovra evasiva", "WARN")
    
    # Ferma il robot
    robot.stop_movimento()
    
    # Verifica la distanza laterale
    distanza_laterale = ultrasonicLaterale.get_distance()
    
    if distanza_laterale > 20:  # Se c'è spazio a sinistra
        # Manovra di aggiramento a sinistra
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
    else:
        # Manovra di aggiramento a destra
        robot.gira_destra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_sinistra(45, speed=30)
        robot.muovi_avanti_for("seconds", 1, speed=30)
        robot.gira_destra(45, speed=30)
    
    log("Ostacolo evitato", "INFO")

def prendi_auto():
    """Effettua la manovra per prendere un'auto"""
    log("Prelievo auto in corso...", "PARKING")
    
    # Abbassiamo la gabbia/pinza per prendere l'auto
    servo_alza.set_angle(90)  # Posizione abbassata
    sleep(0.5)
    
    # Chiusura pinza/gabbia 
    gabbia.run_for_seconds(1, 50)
    
    log("Auto prelevata con successo", "PARKING")

def rilascia_auto():
    """Effettua la manovra per rilasciare un'auto"""
    log("Rilascio auto in corso...", "PARKING")
    
    # Apertura pinza/gabbia
    gabbia.run_for_seconds(1, -50)
    
    # Alziamo la gabbia/pinza
    servo_alza.set_angle(0)  # Posizione alzata
    sleep(0.5)
    
    log("Auto rilasciata con successo", "PARKING")

def sensore_ostacoli():
    """Thread di monitoraggio degli ostacoli"""
    log("Avvio sensore ostacoli", "SYSTEM")
    
    while not shutdown_flag.is_set():
        try:
            # Leggi la distanza frontale
            distanza = ultrasonic.get_distance()
            
            # Se la distanza è inferiore a una soglia, segnala un ostacolo
            if distanza < 15:  # 15 cm come soglia di sicurezza
                eventi.put("ostacolo")
            
            # Pausa breve per non saturare il sistema
            sleep(0.1)
            
        except Exception as e:
            log(f"Errore sensore ostacoli: {str(e)}", "ERROR")
            sleep(0.5)

def controlla_ostacoli():
    """Verifica la presenza di ostacoli nella coda eventi"""
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

# --------------------------
# LOGICA PRINCIPALE
# --------------------------
def main_execution():
    """Funzione principale di esecuzione"""
    global officina_attiva
    
    # Configurazione iniziale
    log("Avvio missione Smart Parking", "SYSTEM")
    
    # Avvia il thread di monitoraggio ostacoli
    sensor_thread = threading.Thread(target=sensore_ostacoli)
    sensor_thread.daemon = True
    sensor_thread.start()
    
    # Configura timer sicurezza
    safety_timer = threading.Timer(TEMPO_TIMER, termina_programma)
    safety_timer.daemon = True
    safety_timer.start()
    
    try:
        # 1. Identificazione dell'officina attiva
        officina_attiva = identifica_colore_officina()
        
        # 2. Movimento verso la zona centrale del parcheggio
        log("Movimento verso area di parcheggio centrale", "PARKING")
        robot.muovi_avanti_for("seconds", 2, speed=40)
        
        # 3. Ciclo principale per gestire le auto
        auto_gestite = 0
        
        while auto_gestite < 18 and not shutdown_flag.is_set():
            # Controlla se ci sono ostacoli
            if controlla_ostacoli():
                continue
                
            # Identifica il colore dell'auto
            colore_auto = identifica_colore_auto()
            log(f"Auto identificata: {colore_auto.value}", "PARKING")
            
            # Prendi l'auto
            prendi_auto()
            
            # Gestisci l'auto in base al colore
            if colore_auto == AutoColor.ROSSO:
                # Auto rosse -> colonnine di ricarica
                log("Auto ROSSA: destinazione colonnina di ricarica", "PARKING")
                
                # Scegliamo una colonnina a rotazione
                colonnina = f"colonnina{(auto_gestite % 3) + 1}"
                muovi_a_posizione(colonnina)
                rilascia_auto()
                
            elif colore_auto == AutoColor.BLU:
                # Auto blu -> officina attiva
                log(f"Auto BLU: destinazione officina {officina_attiva.value}", "PARKING")
                
                if officina_attiva == OfficineColor.VERDE:
                    muovi_a_posizione("officina_verde")
                else:
                    muovi_a_posizione("officina_gialla")
                    
                rilascia_auto()
                
            elif colore_auto == AutoColor.GIALLO:
                # Auto gialle -> opzionalmente alle colonnine
                # Per semplicità, le portiamo tutte alle colonnine
                log("Auto GIALLA: destinazione colonnina di ricarica", "PARKING")
                
                colonnina = f"colonnina{(auto_gestite % 3) + 1}"
                muovi_a_posizione(colonnina)
                rilascia_auto()
                
            elif colore_auto == AutoColor.VERDE:
                # Auto verdi -> lasciarle ferme
                log("Auto VERDE: resta nel suo stallo", "PARKING")
                
                # Rilascia l'auto dove si trova
                rilascia_auto()
            
            else:
                # Colore sconosciuto
                log("Colore AUTO non identificato, rilascio sul posto", "WARN")
                rilascia_auto()
            
            # Incrementa contatore auto gestite
            auto_gestite += 1
            log(f"Auto gestite: {auto_gestite}/18", "INFO")
            
            # Torna all'area di parcheggio per la prossima auto
            if auto_gestite < 18:
                muovi_a_posizione("area_parcheggio")
        
        # 4. Ritorno all'area di partenza
        log("Tutte le auto gestite, ritorno all'area di partenza", "PARKING")
        muovi_a_posizione("partenza")
        
        log("Missione Smart Parking completata con successo!", "SUCCESS")
        
    except Exception as e:
        log(f"Errore critico durante la missione: {str(e)}", "CRITICAL")
    
    finally:
        # Fermata sicura dei motori
        robot.stop_movimento()

def inizializza_sensore(sensore, nome_sensore):
    """Funzione per inizializzare un sensore e gestire eventuali errori.
    
    Args:
        sensore: Il sensore da inizializzare.
        nome_sensore (str): Il nome del sensore per il log.
        
    Returns:
        sensore: Il sensore inizializzato o None se non riuscito.
    """
    try:
        log(f"Inizializzazione sensore {nome_sensore}...", "INFO")
        return sensore
    except Exception as e:
        log(f"Errore inizializzazione sensore {nome_sensore}: {str(e)}", "ERROR")
        risposta = input(f"Il sensore {nome_sensore} non è stato rilevato. Vuoi continuare senza di esso? (y/n): ")
        if risposta.lower() == 'y':
            log(f"Bypassato sensore {nome_sensore}.", "WARN")
            return None
        else:
            log(f"Interruzione a causa della mancata inizializzazione di {nome_sensore}.", "CRITICAL")
            sys.exit(1)


# --------------------------
# ENTRY POINT
# --------------------------
if __name__ == "__main__":
    # Cattura il segnale di interruzione
    signal.signal(signal.SIGINT, lambda s, f: termina_programma())
    
    try:
        # Inizializza la connessione seriale
        arduino = safe_serial_connect()
        
        # Inizializzazione dei sensori
        colorLego = inizializza_sensore(buildhat.ColorSensor('A'), "ColorSensor")
        color1 = inizializza_sensore(ColorSensorA(arduino, serial_lock, "COL1", "5"), "ColorSensorA")
        ultrasonic = inizializza_sensore(UltrasonicSensor(arduino, serial_lock), "UltrasonicSensor")
        ultrasonicLaterale = inizializza_sensore(UltrasonicSensor(arduino, serial_lock, command_code="6", sensor_id="DIST2"), "UltrasonicSensor Laterale")
        servo = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO1"), "ServoMotor 1")
        servo_alza = inizializza_sensore(ServoMotor(arduino, serial_lock, "SERVO2", min_angle=0, max_angle=360), "ServoMotor 2")
        robot = Robot('C', 'D')
        gabbia = Motor('A')

        
        # Esegui la missione principale
        main_execution()
        
    except Exception as e:
        log(f"Errore critico: {str(e)}", "CRITICAL")
        
    finally:
        # Chiudi le connessioni e rilascia le risorse
        shutdown_flag.set()
        
        try:
            if 'arduino' in locals() and arduino.is_open:
                arduino.close()
        except Exception as e:
            log(f"Errore chiusura seriale: {str(e)}", "WARN")
        
        log("Applicazione terminata", "SYSTEM")
