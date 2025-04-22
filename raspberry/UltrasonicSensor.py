import threading
import serial
from time import perf_counter, sleep, time
from typing import Optional
from SensorError import SensorError
from SensorErrorType import SensorErrorType

# Logger minimale

def log(msg: str, level: str = "INFO"):
    ts = time()
    print(f"[{level}] {ts:.3f}: {msg}")

shutdown_flag = threading.Event()

def restart_program():
    log("Restart del programma richiesto!", "CRITICAL")
    raise SystemExit("Restart programmato")

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

class UltrasonicSensor:
    """
    Lettura distanza ultrarapida con retry continuo.
    """
    def __init__(
        self,
        arduino_conn: serial.Serial,
        lock: threading.Lock,
        sensor_id: str = "DIST",
        command_code: str = "4",
        timeout: float = 0.2,
        retries: int = 2,
        baudrate: int = 115200
    ):
        self.arduino = arduino_conn
        self.lock = lock
        self.sensor_id = sensor_id
        self.command = f"{command_code}\n"
        self.timeout = timeout
        self.retries = retries
        self.baudrate = baudrate
        self._pattern = f"{self.sensor_id}|".encode()
        self._buffer = bytearray()
        self._last = -1

        # Parametri seriali ultraveloci
        self.arduino.timeout = 0.005
        self.arduino.write_timeout = 0.2
        self._byte_time = 11 / self.baudrate

    def _send(self) -> int:
        """Invia comando e prova un numero limitato di volte."""
        for attempt in range(1, self.retries + 1):
            with self.lock:
                start = perf_counter()
                self.arduino.reset_input_buffer()
                self.arduino.write(self.command.encode())
                write_dur = perf_counter() - start
                if write_dur > 0.02:
                    raise SensorError(SensorErrorType.COMMUNICATION,
                                      f"Write lento: {write_dur:.3f}s")

                # Calcolo timeout dinamico
                expect = len(self._pattern) + 6  # max digits + \n
                to = self.timeout + expect * self._byte_time
                resp = self._read(to)
                if resp is None:
                    continue

                try:
                    value = int(resp)
                    return value
                except ValueError:
                    raise SensorError(SensorErrorType.INVALID_DATA,
                                      f"Output non intero: '{resp}'")

            sleep(0.005 * attempt)
        raise SensorError(SensorErrorType.TIMEOUT, "Tentativi esauriti")

    def _read(self, timeout: float) -> Optional[str]:
        start = perf_counter()
        buf = self._buffer
        buf.clear()
        while perf_counter() - start < timeout:
            chunk = self.arduino.read(32)
            if chunk:
                buf.extend(chunk)
                if b'\n' in buf:
                    parts = buf.split(b'\n')
                    buf[:] = parts[-1]
                    for line in parts[:-1]:
                        if line.startswith(self._pattern):
                            try:
                                return line.split(b'|',1)[1].decode().strip()
                            except Exception as e:
                                raise SensorError(SensorErrorType.INVALID_DATA, f"Parsing fallito: {e}")
        return None

    def get_distance(self) -> int:
        """
        Richiama _send in loop infinito finché non ottiene un valore valido.
        """
        while True:
            try:
                dist = self._send()
                self._last = dist
                log(f"{self.sensor_id}: {dist}", "INFO")
                return dist
            except SensorError as e:
                log(f"{self.sensor_id}: {e.message}. Riprovo...", "ERROR")
                sleep(0.005)

    def benchmark(self, samples: int = 20) -> dict:
        stats = { 'success':0, 'timeouts':0, 'errors':0, 'avg':0, 'min':float('inf'), 'max':0 }
        tot = 0.0
        for _ in range(samples):
            t0 = perf_counter()
            try:
                self.get_distance()
                d = perf_counter() - t0
                stats['success'] += 1
                tot += d
                stats['min'] = min(stats['min'], d)
                stats['max'] = max(stats['max'], d)
            except SensorError as e:
                if e.error_type == SensorErrorType.TIMEOUT:
                    stats['timeouts'] += 1
                else:
                    stats['errors'] += 1
        if stats['success']:
            stats['avg'] = tot / stats['success']
        return stats

    def adaptive_timeout(self, window_size: int = 20):
        # placeholder per algoritmo adattivo
        pass
