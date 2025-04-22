import threading
import serial
from time import perf_counter, sleep, time
from enum import Enum
from typing import Optional
from SensorError import SensorError
from SensorErrorType import SensorErrorType

# Logger minimale

def log(msg: str, level: str = "INFO"):
    ts = time()
    print(f"[{level}] {ts:.3f}: {msg}")

class ColorSensor:
    """
    Lettura colore ultrarapida con retry continuo.
    """
    COLOR_MAP = {
        '0': 'sconosciuto',
        '1': 'bianco',
        '2': 'rosso',
        '3': 'scuro',
        '4': 'verde',
        '5': 'blu',
        '6': 'giallo'
    }

    def __init__(
        self,
        arduino_conn: serial.Serial,
        lock: threading.Lock,
        sensor_id: str,
        command_code: str,
        timeout: float = 0.2,
        retries: int = 2,
        baudrate: int = 115200
    ):
        self.arduino = arduino_conn
        self.lock = lock
        self.sensor_id = sensor_id
        self.command_code = f"{command_code}\n"
        self.timeout = timeout
        self.retries = retries
        self.baudrate = baudrate
        self._pattern = f"{self.sensor_id}|".encode()
        self._buffer = bytearray()
        self._last = self.COLOR_MAP['0']

        # Parametri seriali ultra-veloci
        self.arduino.timeout = 0.005
        self.arduino.write_timeout = 0.2

    def _send(self) -> str:
        for attempt in range(1, self.retries + 1):
            with self.lock:
                start = perf_counter()
                self.arduino.reset_input_buffer()
                self.arduino.write(self.command_code.encode())
                write_dur = perf_counter() - start
                if write_dur > 0.02:
                    raise SensorError(SensorErrorType.COMMUNICATION, f"Write lento: {write_dur:.3f}s")

                # timeout dinamico
                expect = len(self._pattern) + 2
                dt = (10 * expect) / self.baudrate
                to = self.timeout + dt
                resp = self._read(to)
                if resp and resp in self.COLOR_MAP:
                    return resp
                if resp:
                    raise SensorError(SensorErrorType.INVALID_DATA, f"Invalido: {resp}")

            # backoff minimo
            sleep(0.005 * attempt)
        raise SensorError(SensorErrorType.TIMEOUT, "Timeout")

    def _read(self, timeout: float) -> Optional[str]:
        start = perf_counter()
        self._buffer.clear()
        while perf_counter() - start < timeout:
            chunk = self.arduino.read(32)
            if chunk:
                self._buffer.extend(chunk)
                if b'\n' in self._buffer:
                    parts = self._buffer.split(b'\n')
                    self._buffer = parts[-1]
                    for line in parts[:-1]:
                        if line.startswith(self._pattern):
                            return line[len(self._pattern):].decode().strip()
        return None

    def get_color(self) -> str:
        while True:
            try:
                code = self._send()
                col = self.COLOR_MAP.get(code, self.COLOR_MAP['0'])
                self._last = col
                log(f"{self.sensor_id}: {col}", "INFO")
                return col
            except SensorError as e:
                # retry immediato
                sleep(0.005)

    def benchmark(self, samples: int = 10) -> dict:
        stats = { 'success':0, 'timeouts':0, 'errors':0, 'avg':0, 'min':float('inf'), 'max':0 }
        tot = 0
        for _ in range(samples):
            st = perf_counter()
            try:
                self.get_color()
                d = perf_counter() - st
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
        pass
