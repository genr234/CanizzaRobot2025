import queue
import threading
from time import time, sleep
import serial
from colorama import Fore, Style, init as colorama_init
from SensorErrorType import SensorErrorType
from SensorError import SensorError
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
    "SERVO": Fore.MAGENTA  # Colore speciale per operazioni servo
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


class ServoMotor:
    """Classe per il controllo avanzato di servomotori con gestione thread-safe"""

    def __init__(
            self,
            arduino_connection: serial.Serial,
            shared_lock: threading.Lock,
            command_code: str,
            min_angle: int = 0,
            max_angle: int = 180,
            timeout: float = 0.2,
            retries: int = 2
    ):
        """
        Inizializza il controller del servomotore.

        Args:
            arduino_connection (serial.Serial): Connessione seriale ad Arduino
            shared_lock (threading.Lock): Mutex per operazioni thread-safe
            command_code (str): Codice comando per il servomotore
            min_angle (int): Angolo minimo consentito (default: 0)
            max_angle (int): Angolo massimo consentito (default: 180)
            timeout (float): Timeout risposta (secondi, default: 0.2)
            retries (int): Tentativi prima di fallire (default: 2)
        """
        self.arduino = arduino_connection
        self.lock = shared_lock
        self.command_code = command_code
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.timeout = timeout
        self.retries = retries
        self._current_angle = None  # Ultimo angolo confermato
        self._response_queue = queue.Queue()  # Coda messaggi non servo
        self._running = threading.Event()  # Flag stato operativo

        # Configurazione iniziale
        self._start_monitor()
        log(f"Inizializzato servomotore {self.command_code} ({min_angle}-{max_angle}°)", "SERVO")

    def _send_command(self, angle: int):
        if not self.min_angle <= angle <= self.max_angle:
            err_msg = f"Angolo {angle}° fuori range consentito ({self.min_angle}-{self.max_angle}°)"
            log(err_msg, "ERROR")
            raise SensorError(SensorErrorType.INVALID_DATA, err_msg)

        full_cmd = f"{self.command_code}|{angle}\n"

        for attempt in range(1, self.retries + 1):
            with self.lock:
                try:
                    self.arduino.reset_output_buffer()
                    start_write = time()
                    self.arduino.write(full_cmd.encode())
                    write_time = time() - start_write
                    if write_time > 0.02:
                        log(f"Latenza scrittura elevata: {write_time:.3f}s", "WARN")

                    response_buffer = bytearray()
                    start_time = time()
                    response_received = False
                    servo_status = None

                    while (time() - start_time) < self.timeout:
                        chunk = self.arduino.read(self.arduino.in_waiting or 1)
                        if chunk:
                            response_buffer.extend(chunk)
                            lines = response_buffer.split(b'\n')
                            response_buffer = lines[-1]

                            for line in lines[:-1]:
                                decoded_line = line.decode(errors='ignore').strip()
                                if decoded_line.startswith("SERVO|"):
                                    response_received = True
                                    parts = decoded_line.split('|')
                                    servo_status = parts[1]
                                    break
                                elif decoded_line:
                                    self._response_queue.put_nowait(decoded_line)

                        if response_received:
                            break

                        sleep(max(0.001, self.timeout / 100))

                    print(response_received, response_buffer, servo_status)
                    if servo_status == "OK":
                        self._current_angle = angle
                        log(f"Angolo {angle}° confermato", "SERVO")
                        return
                    elif servo_status == "-1":
                        err_msg = f"Errore firmware per angolo {angle}°"
                        log(err_msg, "ERROR")
                        raise SensorError(SensorErrorType.INVALID_DATA, err_msg)
                    else:
                        log(f"Risposta non valida: {servo_status}", "WARN")

                except serial.SerialException as e:
                    log(f"Errore comunicazione: {str(e)}", "ERROR")
                    if attempt == self.retries:
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Fallo dopo {self.retries} tentativi: {str(e)}"
                        ) from e
                    sleep(0.1 * attempt)
                except Exception as e:
                    log(f"Errore generico: {str(e)}", "CRITICAL")
                    raise

            log(f"Tentativo {attempt} fallito, retry...", "WARN")

        raise SensorError(SensorErrorType.TIMEOUT, f"Timeout dopo {self.retries} tentativi")

    def set_angle(self, angle: int, blocking: bool = True):
        """
        Imposta l'angolo del servomotore.

        Args:
            angle (int): Angolo target (0-180)
            blocking (bool): Se True attende conferma movimento (default: True)
        """
        log(f"Richiesta impostazione angolo {angle}° (blocking={blocking})", "INFO")

        if blocking:
            self._send_command(angle)
        else:
            threading.Thread(
                target=self._send_command,
                args=(angle,),
                daemon=True
            ).start()

    def get_current_angle(self) -> int:
        """Restituisce l'ultimo angolo confermato dal servomotore"""
        if self._current_angle is not None:
            return self._current_angle
        else:
            self.min_angle

    def _start_monitor(self):
        """Avvia il thread di monitoraggio messaggi in background"""
        self._running.set()
        self._monitor_thread = threading.Thread(
            target=self._monitor_responses,
            daemon=True
        )
        self._monitor_thread.start()

    def _monitor_responses(self):
        """Monitoraggio continuo per messaggi non destinati al servo"""
        while self._running.is_set():
            try:
                with self.lock:
                    if self.arduino.in_waiting > 0:
                        data = self.arduino.read_all().decode(errors='ignore')
                        for line in data.split('\n'):
                            clean_line = line.strip()
                            if clean_line and not clean_line.startswith("SERVO|"):
                                self._response_queue.put(clean_line)
                sleep(0.01)
            except Exception as e:
                log(f"Errore monitoraggio: {str(e)}", "ERROR")
                break

    def shutdown(self):
        """Arresto sicuro del componente"""
        log("Avvio procedura shutdown...", "SERVO")
        self._running.clear()
        if self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=0.5)
        log("Servomotore disattivato", "INFO")

    def get_queued_messages(self) -> list:
        """Restituisce i messaggi non processati"""
        msgs = list(self._response_queue.queue)
        return msgs