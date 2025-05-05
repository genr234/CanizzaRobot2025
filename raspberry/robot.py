from time import sleep

from buildhat import MotorPair
from typing import Union
from colorama import Fore, Style, init as colorama_init
from datetime import datetime

# Inizializzazione colorama per colori cross-platform
colorama_init(autoreset=True)

# Configurazione colori per logging
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "ROBOT": Fore.MAGENTA,  # Colore speciale per operazioni robot
    "CALIB": Fore.BLUE  # Colore per la calibrazione
}


def log(msg: str, level: str = "INFO"):
    """Registra messaggi formattati con colori completi

    Args:
        msg (str): Messaggio da loggare
        level (str): Livello di gravità (DEBUG/INFO/WARN/ERROR/CRITICAL/SERVO)
    """
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    print(f"{colore}[{level}] {ts}: {msg}{Style.RESET_ALL}")


class RobotError(Exception):
    """Eccezione base per gli errori del robot con logging integrato"""

    def __init__(self, message: str):
        log(message, "ERROR")
        super().__init__(message)


class Robot:
    """Classe per il controllo avanzato di un robot differenziale"""

    def __init__(self, left_port: str, right_port: str):
        """
        Inizializza il robot con configurazione base

        Args:
            left_port (str): Porta motore sinistro (es. 'A')
            right_port (str): Porta motore destro (es. 'B')

        Raises:
            RobotError: In caso di errore inizializzazione motori
        """
        try:
            log(f"Inizializzazione robot - Motori: {left_port}/{right_port}", "ROBOT")
            self.motor_ruote = MotorPair(left_port, right_port)
            self._default_speed = 50
            self._is_moving = False
            log("Robot inizializzato con successo", "ROBOT")
        except Exception as e:
            raise RobotError(f"Errore inizializzazione motori: {str(e)}") from e

    @property
    def default_speed(self) -> int:
        """Restituisce la velocità predefinita (0-100)"""
        return self._default_speed

    @default_speed.setter
    def default_speed(self, value: int):
        """Imposta la velocità predefinita con validazione"""
        if not 0 <= value <= 100:
            log(f"Velocità {value}% non valida", "ERROR")
            raise ValueError("La velocità deve essere tra 0 e 100")
        self._default_speed = value
        log(f"Velocità predefinita impostata a {value}%", "DEBUG")

    def muovi_avanti(self, speed: Union[int, None] = None):
        """
        Avvia il movimento in avanti

        Args:
            speed (int|None): Velocità opzionale (0-100)
        """
        actual_speed = speed if speed is not None else self.default_speed
        if not 0 <= actual_speed <= 100:
            log(f"Velocità {actual_speed}% non valida", "ERROR")
            raise ValueError("La velocità dev'essere tra 0 e 100")
        log(f"Avvio movimento in avanti a {actual_speed}%", "ROBOT")
        self.motor_ruote.start(actual_speed, actual_speed)
        self._is_moving = True

    def muovi_indietro(self, speed: Union[int, None] = None):
        """
        Avvia il movimento all'indietro

        Args:
            speed (int|None): Velocità opzionale (0-100)
        """
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Avvio movimento all'indietro a {actual_speed}%", "ROBOT")
        self.motor_ruote.start(actual_speed, actual_speed)
        self._is_moving = True

    def stop_movimento(self):
        """Ferma il movimento con frenata attiva"""
        log("Arresto movimento", "ROBOT")
        self.motor_ruote.stop()
        self._is_moving = False

    def muovi_avanti_for(self, unit: str, value: float, speed: Union[int, None] = None):
        """
        Movimento controllato in avanti

        Args:
            unit (str): Unità di misura ('degrees'=gradi, 'seconds'=secondi, 'rotations'=giri)
            value (float): Valore dell'unità
            speed (int|None): Velocità opzionale (0-100)
        """
        #Così sappiamo quali unità di misura
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Movimento in avanti di {value} {unit} a {actual_speed}%", "DEBUG")
        self._run_movement(value, actual_speed, unit, direction=1)

    def muovi_indietro_for(self, unit: str, value: float, speed: Union[int, None] = None):
        """
        Movimento controllato all'indietro

        Args:
            unit (str): Unità di misura ('degrees', 'seconds', 'rotations')
            value (float): Valore dell'unità
            speed (int|None): Velocità opzionale (0-100)
        """
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Movimento all'indietro di {value} {unit} a {actual_speed}%", "DEBUG")
        self._run_movement(value, -actual_speed, unit, direction=-1)

    def gira_destra(self, degrees: int = 90, speed: Union[int, None] = None):
        """
        Rotazione controllata a destra

        Args:
            degrees (int): Gradi di rotazione (default: 90)
            speed (int|None): Velocità opzionale (0-100)
        """
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Rotazione di {degrees}° a destra a {actual_speed}%", "ROBOT")
        self._turn(degrees, actual_speed, right=True)

    def gira_sinistra(self, degrees: int = 90, speed: Union[int, None] = None):
        """
        Rotazione controllata a sinistra

        Args:
            degrees (int): Gradi di rotazione (default: 90)
            speed (int|None): Velocità opzionale (0-100)
        """
        actual_speed = speed if speed is not None else self.default_speed
        log(f"Rotazione di {degrees}° a sinistra a {actual_speed}%", "ROBOT")
        self._turn(degrees, actual_speed, right=False)

    def _run_movement(self, value: float, speed: int, unit: str, direction: int):
        """Gestore interno per movimenti controllati"""
        valid_units = ['degrees', 'seconds', 'rotations']
        if unit not in valid_units:
            log(f"Unità {unit} non supportata", "ERROR")
            raise RobotError(f"Unità non valida: {unit}. Usare {valid_units}")

        try:
            method_name = f'run_for_{unit}'
            log(f"Esecuzione metodo {method_name}", "DEBUG")
            method = getattr(self.motor_ruote, method_name)
            method(value, speed * direction, speed * direction)
            self._is_moving = False
            log(f"Movimento completato: {value} {unit}", "DEBUG")
        except AttributeError:
            error_msg = f"Metodo {method_name} non disponibile"
            log(error_msg, "CRITICAL")
            raise RobotError(error_msg)
        except Exception as e:
            error_msg = f"Errore movimento: {str(e)}"
            log(error_msg, "ERROR")
            raise RobotError(error_msg) from e

    def _turn(self, degrees: int, speed: int, right: bool = True):
        """Gestore interno per rotazioni sul posto"""
        try:
            log(f"Avvio rotazione: {degrees}°", "DEBUG")

            # Calcolo empirico del tempo (ad esempio: 0.01s per ogni grado)
            duration = degrees * 0.01  # Puoi regolare questo valore

            if right:
                self.motor_ruote.start(speed, -speed)
            else:
                self.motor_ruote.start(-speed, speed)

            sleep(duration)
            self.motor_ruote.stop()
            self._is_moving = False
            log("Rotazione completata", "DEBUG")
        except Exception as e:
            error_msg = f"Errore rotazione: {str(e)}"
            log(error_msg, "ERROR")
            raise RobotError(error_msg) from e

    @property
    def is_moving(self) -> bool:
        """Stato del movimento"""
        return self._is_moving

    def __enter__(self):
        """Gestione contesto with"""
        log("Avvio contesto robot", "DEBUG")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Pulizia contesto with"""
        log("Uscita contesto robot", "DEBUG")
        self.stop_movimento()

    def calibra_motori(self):
        """Procedura automatica di calibrazione"""
        log("Avvio procedura di calibrazione", "CALIB")
        try:
            for i in range(2):
                log(f"Ciclo calibrazione {i + 1}/2", "CALIB")
                self.muovi_avanti_for('seconds', 1, speed=30)
                self.muovi_indietro_for('seconds', 1, speed=30)
                self.gira_destra(90, speed=30)
                self.gira_sinistra(90, speed=30)
            log("Calibrazione completata con successo", "CALIB")
        except Exception as e:
            error_msg = f"Calibrazione fallita: {str(e)}"
            log(error_msg, "CRITICAL")
            raise RobotError(error_msg) from e