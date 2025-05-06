import queue
import threading
from time import time, sleep
import serial
from colorama import Fore, Style, init as colorama_init
from SensorErrorType import SensorErrorType
from SensorError import SensorError
from datetime import datetime

colorama_init(autoreset=True)

COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "SERVO": Fore.MAGENTA
}


def log(msg: str, level: str = "INFO"):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    print(f"{colore}[{level}] {ts}: {msg}{Style.RESET_ALL}")


class ServoMotor:
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
        self.arduino = arduino_connection
        self.lock = shared_lock
        self.command_code = command_code
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.timeout = timeout
        self.retries = retries
        self._current_angle = None
        self._response_queue = queue.Queue()
        self._running = threading.Event()

        self._start_monitor()
        log(f"Inizializzato servomotore {self.command_code} ({min_angle}-{max_angle}°)", "SERVO")

    def _send_command(self, angle: int):
        log(f"DEBUG VARIABILI: angle={angle}, min_angle={self.min_angle}, max_angle={self.max_angle}, "
            f"command_code={self.command_code}, timeout={self.timeout}, retries={self.retries}", "DEBUG")

        if not self.min_angle <= angle <= self.max_angle:
            msg = f"Angolo {angle}° fuori range ({self.min_angle}-{self.max_angle}°)"
            log(msg, "ERROR")
            raise SensorError(SensorErrorType.INVALID_DATA, msg)

        full_cmd = f"{self.command_code}|{angle}\n"
        log(f"DEBUG VARIABILI: full_cmd={repr(full_cmd)}", "DEBUG")

        for attempt in range(1, self.retries + 1):
            with self.lock:
                try:
                    self.arduino.reset_input_buffer()
                    self.arduino.reset_output_buffer()
                    self.arduino.write(full_cmd.encode())
                    log(f"Comando inviato: {full_cmd.strip()}", "DEBUG")

                    response = b""
                    start_time = time()
                    while (time() - start_time) < self.timeout:
                        if self.arduino.in_waiting:
                            chunk = self.arduino.read(self.arduino.in_waiting)
                            log(f"DEBUG VARIABILI: chunk ricevuto={repr(chunk)}", "DEBUG")
                            response += chunk
                            if b'\n' in response:
                                break
                        sleep(0.005)

                    decoded_lines = response.decode(errors="ignore").split('\n')
                    log(f"DEBUG VARIABILI: decoded_lines={decoded_lines}", "DEBUG")

                    for line in decoded_lines:
                        line = line.strip()
                        if not line:
                            continue
                        log(f"Risposta ricevuta: {line}", "DEBUG")

                        if line.startswith("SERVO|"):
                            status = line.split("|")[1]
                            log(f"DEBUG VARIABILI: status={status}", "DEBUG")

                            if status == "OK":
                                self._current_angle = angle
                                log(f"Angolo {angle}° confermato", "SERVO")
                                return
                            elif status == "-1":
                                err_msg = f"Errore firmware per angolo {angle}°"
                                log(err_msg, "ERROR")
                                raise SensorError(SensorErrorType.INVALID_DATA, err_msg)
                            else:
                                log(f"Stato sconosciuto: {status}", "WARN")
                        else:
                            self._response_queue.put_nowait(line)
                            log(f"DEBUG VARIABILI: risposta messa in coda: {line}", "DEBUG")

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
        return self._current_angle if self._current_angle is not None else self.min_angle

    def _start_monitor(self):
        self._running.set()
        self._monitor_thread = threading.Thread(
            target=self._monitor_responses,
            daemon=True
        )
        self._monitor_thread.start()

    def _monitor_responses(self):
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
        log("Avvio procedura shutdown...", "SERVO")
        self._running.clear()
        if self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=0.5)
        log("Servomotore disattivato", "INFO")

    def get_queued_messages(self) -> list:
        return list(self._response_queue.queue)
