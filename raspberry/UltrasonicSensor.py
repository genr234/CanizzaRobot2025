import threading
import serial
from time import perf_counter, sleep
from typing import Optional

from ColorSensor import SensorErrorType, SensorError


class UltrasonicSensor:
    def __init__(
            self,
            arduino_conn: serial.Serial,
            lock: threading.Lock,
            sensor_id: str = "DIST",
            command_code: str = "4",
            timeout: float = 0.6,
            retries: int = 3,
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
        self._last_valid_distance = None

        # Ottimizzazione parametri seriali
        self.arduino.timeout = 0.02
        self.arduino.write_timeout = 0.3
        self.byte_time = 11 / self.baudrate  # 10 bit/byte + margine

    def _send_command(self) -> int:
        """Core ad alta velocità con gestione protocollo robusta"""
        for retry in range(self.retries):
            with self.lock:
                try:
                    self.arduino.reset_input_buffer()

                    # 1. Invio comando ottimizzato
                    cmd = self.command_code.encode()
                    self.arduino.write(cmd)

                    # 2. Calcolo timeout dinamico
                    expected_bytes = len(self._response_pattern) + 4  # VALORE + \n
                    timeout = self.timeout + (expected_bytes * self.byte_time)

                    # 3. Lettura risposta mirata
                    response = self._read_response(timeout)
                    if response is not None:
                        return int(response)

                except serial.SerialException as e:
                    if retry == self.retries - 1:
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Errore comunicazione: {str(e)}"
                        ) from e
                    sleep(0.05 * (retry + 1))  # Backoff progressivo

        raise SensorError(SensorErrorType.TIMEOUT, "Tentativi esauriti")

    def _read_response(self, timeout: float) -> Optional[str]:
        """Lettura ad alta efficienza con buffer circolare"""
        start = perf_counter()
        buffer = bytearray()

        while (perf_counter() - start) < timeout:
            # Lettura chunk ottimizzata
            chunk = self.arduino.read_until(b'\n')
            if chunk:
                buffer.extend(chunk)

                # Ricerca pattern senza decodifica
                if self._response_pattern in buffer:
                    # Estrazione valore diretto
                    try:
                        line = buffer.split(b'\n')[0].decode().strip()
                        return line.split('|')[1]
                    except (IndexError, UnicodeDecodeError):
                        pass

                # Mantieni solo ultimi 128 bytes
                if len(buffer) > 128:
                    buffer = buffer[-128:]

            # Controllo timeout adattivo
            elapsed = perf_counter() - start
            if elapsed < 0.01:
                sleep(0.001)
            else:
                sleep(0.005)

        return None

    def get_distance(self) -> int:
        """Restituisce (distanza, validità) con cache intelligente"""
        try:
            distance = self._send_command()
            self._last_valid_distance = distance
            return distance
        except SensorError as e:
            print(f"[{self.sensor_id}] Warn: {e.message}")
            return -1

    def adaptive_timeout(self, success_rate: float):
        """Regolazione automatica timeout basata sullo storico"""
        if success_rate > 0.8:
            self.timeout = max(0.3, self.timeout * 0.9)
        else:
            self.timeout = min(1.5, self.timeout * 1.1)

    def benchmark(self, samples: int = 20) -> dict:
        """Test prestazioni completo con statistiche avanzate"""
        stats = {
            'success': 0,
            'timeouts': 0,
            'errors': 0,
            'avg_time': 0.0,
            'throughput': 0.0,
            'jitter': 0.0
        }

        total_time = 0.0
        last_time = 0.0
        times = []

        for _ in range(samples):
            try:
                start = perf_counter()
                self.get_distance()
                elapsed = perf_counter() - start

                stats['success'] += 1
                total_time += elapsed
                times.append(elapsed)

                # Calcolo jitter
                if last_time > 0:
                    stats['jitter'] += abs(elapsed - last_time)
                last_time = elapsed

            except SensorError as e:
                if e.error_type == SensorErrorType.TIMEOUT:
                    stats['timeouts'] += 1
                else:
                    stats['errors'] += 1

        if stats['success'] > 0:
            stats['avg_time'] = total_time / stats['success']
            stats['throughput'] = 1 / stats['avg_time']
            stats['jitter'] /= stats['success'] - 1 if stats['success'] > 1 else 1

        return stats