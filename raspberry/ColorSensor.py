import threading
import serial
from time import perf_counter, sleep
from enum import Enum
from typing import Optional


class SensorErrorType(Enum):
    TIMEOUT = 1
    COMMUNICATION = 2
    INVALID_DATA = 3


class SensorError(Exception):
    def __init__(self, error_type: SensorErrorType, message: str):
        self.error_type = error_type
        self.message = message
        super().__init__(message)


class ColorSensor:
    # Mappatura colori con codici esadecimali per debug
    COLOR_MAP = {
        '0': ('Sconosciuto'),
        '1': ('Bianco'),
        '2': ('Rosso'),
        '3': ('Scuro'),
        '4': ('Verde'),
        '5': ('Blu'),
        '6': ('Giallo')
    }

    def __init__(
            self,
            arduino_conn: serial.Serial,
            lock: threading.Lock,
            sensor_id: str,
            command_code: str,
            timeout: float = 0.8,
            retries: int = 2,
            baudrate: int = 115200
    ):
        self.arduino = arduino_conn
        self.lock = lock
        self.sensor_id = sensor_id
        self.command_code = f"{command_code}\n"  # Comando con terminatore
        self.timeout = timeout
        self.retries = retries
        self.baudrate = baudrate
        self._response_pattern = f"{self.sensor_id}|".encode()
        self._line_buffer = bytearray()
        self._last_color = (None, None)

        # Ottimizzazione parametri seriali
        self.arduino.timeout = 0.01
        self.arduino.write_timeout = 0.5

    def _send_command(self) -> str:
        """Core ad alta velocità per l'invio comandi e lettura risposte"""
        for retry in range(self.retries):
            with self.lock:
                try:
                    self.arduino.reset_input_buffer()

                    # 1. Ottimizzazione invio comando
                    command = self.command_code.encode()
                    start_write = perf_counter()
                    self.arduino.write(command)
                    write_time = perf_counter() - start_write

                    # Controllo prestazioni scrittura
                    if write_time > 0.05:  # Soglia più stretta (50ms)
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Scrittura troppo lenta: {write_time:.3f}s"
                        )

                    # 2. Calcolo timeout corretto
                    bytes_expected = len(self._response_pattern) + 2  # Pattern + valore + \n
                    byte_time = (10 * bytes_expected) / self.baudrate  # 10 bit/byte (8N1)
                    timeout = self.timeout + byte_time

                    # 3. Lettura risposta con timeout dinamico
                    response = self._read_response(timeout)

                    # 4. Validazione risposta
                    if response and response in self.COLOR_MAP:
                        return response
                    elif response:
                        raise SensorError(
                            SensorErrorType.INVALID_DATA,
                            f"Valore non valido: {response}"
                        )

                except serial.SerialException as e:
                    if retry == self.retries - 1:
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Errore seriale: {str(e)}"
                        ) from e
                    sleep(0.03 * (retry + 1))  # Backoff esponenziale

        # 5. Reset del colore cache in caso di fallimento
        self._last_color = (None, None)
        raise SensorError(SensorErrorType.TIMEOUT, "Tentativi esauriti")

    def _read_response(self, timeout: float) -> Optional[str]:
        """Lettura ad alta efficienza con timeout preciso"""
        start = perf_counter()
        self._line_buffer.clear()

        while (perf_counter() - start) < timeout:
            # Lettura chunk ottimizzato per dimensione
            chunk = self.arduino.read(size=64)
            if chunk:
                self._line_buffer.extend(chunk)
                if b'\n' in self._line_buffer:
                    lines = self._line_buffer.split(b'\n')
                    self._line_buffer = lines[-1]  # Salva dati parziali

                    for line in lines[:-1]:
                        line = line.strip()
                        if line.startswith(self._response_pattern):
                            return line[len(self._response_pattern):].decode()

        return None

    def get_color(self) -> str:
        """Restituisce (nome colore, esadecimale) con caching"""
        try:
            raw = self._send_command()
            if raw and raw in self.COLOR_MAP:
                self._last_color = self.COLOR_MAP[raw]
                return self._last_color
            return 'Sconosciuto'
        except SensorError as e:
            print(f"[{self.sensor_id}] Errore: {e.message}")
            return self._last_color if self._last_color[0] else ('Sconosciuto', '#808080')

    def benchmark(self, samples: int = 10) -> dict:
        """Test prestazioni con statistiche dettagliate"""
        results = {
            'success': 0,
            'timeouts': 0,
            'errors': 0,
            'avg_time': 0,
            'min_time': float('inf'),
            'max_time': 0
        }

        total_time = 0

        for _ in range(samples):
            try:
                start = perf_counter()
                self.get_color()
                elapsed = perf_counter() - start

                results['success'] += 1
                total_time += elapsed
                results['min_time'] = min(results['min_time'], elapsed)
                results['max_time'] = max(results['max_time'], elapsed)

            except SensorError as e:
                if e.error_type == SensorErrorType.TIMEOUT:
                    results['timeouts'] += 1
                else:
                    results['errors'] += 1

        if results['success'] > 0:
            results['avg_time'] = total_time / results['success']

        return results

    def adaptive_timeout(self, window_size: int = 20):
        """Regolazione automatica timeout basata su risposte recenti"""
        # Implementazione algoritmo adattivo (es. media mobile)
        pass