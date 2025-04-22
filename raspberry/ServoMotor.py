import queue
import threading
from enum import Enum
from time import time, sleep
import serial


class SensorErrorType(Enum):
    TIMEOUT = 1
    COMMUNICATION = 2
    INVALID_DATA = 3


class SensorError(Exception):
    def __init__(self, error_type: SensorErrorType, message: str):
        self.error_type = error_type
        self.message = message
        super().__init__(message)


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

    def _send_command(self, angle: int):
        """Invio comando con validazione angolo e gestione risposta"""
        if not self.min_angle <= angle <= self.max_angle:
            raise SensorError(
                SensorErrorType.INVALID_DATA,
                f"Angolo {angle} fuori range ({self.min_angle}-{self.max_angle})"
            )

        full_cmd = f"{self.command_code}|{angle}\n"

        for attempt in range(self.retries):
            with self.lock:
                try:
                    self.arduino.reset_output_buffer()
                    self.arduino.write(full_cmd.encode())

                    start_time = time()
                    response_buffer = bytearray()
                    servo_response_received = False

                    while (time() - start_time) < self.timeout:
                        chunk = self.arduino.read(self.arduino.in_waiting or 1)
                        if chunk:
                            response_buffer.extend(chunk)
                            lines = response_buffer.split(b'\n')

                            # Mantieni dati parziali nel buffer
                            response_buffer = lines[-1]

                            for line in lines[:-1]:
                                decoded_line = line.decode(errors='ignore').strip()
                                if decoded_line.startswith("SERVO|"):
                                    servo_response_received = True
                                    status = decoded_line.split('|')[1]
                                    if status == "OK":
                                        self._current_angle = angle
                                        return
                                    elif status == "-1":
                                        raise SensorError(
                                            SensorErrorType.INVALID_DATA,
                                            "Angolo non valido dal firmware"
                                        )
                                elif decoded_line:
                                    self._response_queue.put_nowait(decoded_line)

                        if servo_response_received:
                            break

                        # Gestione attesa adattiva
                        elapsed = time() - start_time
                        sleep(max(0.001, min(0.01, self.timeout - elapsed)))

                    if not servo_response_received:
                        raise SensorError(
                            SensorErrorType.TIMEOUT,
                            f"Timeout risposta servo (tentativo {attempt + 1})"
                        )

                except serial.SerialException as e:
                    if attempt == self.retries - 1:
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Errore comunicazione: {str(e)}"
                        ) from e
                    sleep(0.05 * (attempt + 1))

    def set_angle(self, angle: int, blocking: bool = True):
        """
        Imposta angolo del servo
        :param angle: Angolo target (0-180)
        :param blocking: Se True attende completamento movimento
        """
        if blocking:
            self._send_command(angle)
        else:
            threading.Thread(
                target=self._send_command,
                args=(angle,),
                daemon=True
            ).start()

    def get_current_angle(self) -> int:
        """Restituisce ultimo angolo confermato"""
        return self._current_angle

    def _start_monitor(self):
        """Avvia thread monitoraggio messaggi"""
        self._running.set()
        self._monitor_thread = threading.Thread(target=self._monitor_responses)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def _monitor_responses(self):
        """Gestione messaggi non destinati al servo"""
        while self._running.is_set():
            try:
                with self.lock:
                    if self.arduino.in_waiting > 0:
                        data = self.arduino.read_all().decode(errors='ignore')
                        for line in data.split('\n'):
                            line = line.strip()
                            if line and not line.startswith("SERVO|"):
                                self._response_queue.put(line)
                sleep(0.01)
            except:
                break

    def shutdown(self):
        """Arresto sicuro del componente"""
        self._running.clear()
        if self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=0.5)

    def get_queued_messages(self) -> list:
        """Recupera messaggi non processati"""
        return list(self._response_queue.queue)