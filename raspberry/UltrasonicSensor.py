import threading
import serial
from time import perf_counter, sleep, time
from typing import Optional
from colorama import Fore, Style, init as colorama_init  # Aggiunti per i colori
from SensorError import SensorError
from SensorErrorType import SensorErrorType
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
    "PERF": Fore.MAGENTA  # Per metriche prestazionali
}


def log(msg: str, level: str = "INFO"):
    """Registra messaggi formattati con colori completi

    Args:
        msg (str): Messaggio da loggare
        level (str): Livello di gravità (DEBUG/INFO/WARN/ERROR/CRITICAL/PERF)
    """
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    print(f"{colore}[{level}] {ts}: {msg}{Style.RESET_ALL}")


class UltrasonicSensor:
    """
    Classe per la lettura ad alta velocità di distanze con sensori a ultrasuoni.
    Implementa meccanismi di retry e gestione errori avanzata.

    Attributi:
        sensor_id (str): Identificativo univoco del sensore
        _last (int): Ultimo valore valido registrato
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
        """
        Inizializza il sensore ultrasuoni.

        Args:
            arduino_conn (serial.Serial): Connessione seriale ad Arduino
            lock (threading.Lock): Mutex per accesso thread-safe
            sensor_id (str): Identificativo del sensore (default: "DIST")
            command_code (str): Codice comando per il sensore (default: "4")
            timeout (float): Timeout lettura in secondi (default: 0.2)
            retries (int): Tentativi prima di fallire (default: 2)
            baudrate (int): Velocità comunicazione seriale (default: 115200)
        """
        self.arduino = arduino_conn
        self.lock = lock
        self.sensor_id = sensor_id
        self.command = f"{command_code}\n"
        self.timeout = timeout
        self.retries = retries
        self.baudrate = baudrate
        self._pattern = f"{self.sensor_id}|".encode()
        self._buffer = bytearray()
        self._last = -1  # Ultima lettura valida

        # Ottimizzazione parametri seriali
        self.arduino.timeout = 0.005  # Timeout non bloccante
        self.arduino.write_timeout = 0.2  # Timeout scrittura aggressivo
        self._byte_time = 11 / self.baudrate  # Tempo per byte trasmesso

    def _send(self) -> int:
        """Esegue l'invio del comando e gestisce la risposta con retry.

        Returns:
            int: Valore della distanza in unità configurate

        Raises:
            SensorError: In caso di errori di comunicazione o dati invalidi
        """
        for attempt in range(1, self.retries + 1):
            with self.lock:
                try:
                    # Reset buffer e invio comando
                    start = perf_counter()
                    self.arduino.reset_input_buffer()
                    self.arduino.write(self.command.encode())

                    # Monitoraggio prestazioni scrittura
                    write_dur = perf_counter() - start
                    if write_dur > 0.02:
                        log(f"Latenza scrittura elevata: {write_dur:.3f}s", "WARN")
                        raise SensorError(
                            SensorErrorType.COMMUNICATION,
                            f"Write lento: {write_dur:.3f}s"
                        )

                    # Calcolo timeout dinamico basato su baudrate
                    max_digits = 6  # Massimo previsto per valori distanza
                    timeout = self.timeout + (max_digits * self._byte_time)

                    # Lettura e validazione risposta
                    resp = self._read(timeout)
                    if resp is None:
                        continue  # Prossimo tentativo

                    value = int(resp)
                    return value

                except ValueError as ve:
                    raise SensorError(
                        SensorErrorType.INVALID_DATA,
                        f"Valore non numerico: '{resp}'"
                    )
                except Exception as e:
                    raise SensorError(
                        SensorErrorType.COMMUNICATION,
                        f"Errore generico: {str(e)}"
                    )

            # Backoff progressivo tra i tentativi
            sleep(0.005 * attempt)
            log(f"Tentativo {attempt} fallito, retry...", "WARN")

        raise SensorError(SensorErrorType.TIMEOUT, "Tentativi esauriti")

    def _read(self, timeout: float) -> Optional[str]:
        """Legge e interpreta la risposta dal sensore.

        Args:
            timeout (float): Tempo massimo attesa risposta

        Returns:
            Optional[str]: Valore letto come stringa o None
        """
        start = perf_counter()
        self._buffer.clear()

        while perf_counter() - start < timeout:
            try:
                # Lettura chunk non bloccante
                chunk = self.arduino.read(32)
                if chunk:
                    self._buffer.extend(chunk)

                    # Processamento linee complete
                    if b'\n' in self._buffer:
                        lines = self._buffer.split(b'\n')
                        self._buffer = lines[-1]  # Conserva dati incompleti

                        for line in lines[:-1]:
                            if not line:
                                pass
                            if line.startswith(self._pattern):
                                # Estrazione e pulizia valore
                                parts = line.split(b'|', 1)
                                return parts[1].decode().strip()
            except Exception as e:
                log(f"Errore durante la lettura: {str(e)}", "ERROR")
                return None

        log("Timeout lettura risposta", "WARN")
        return None

    def get_distance(self, max_attempts: int = 10) -> int:
        """Ottiene la distanza corrente con gestione errori trasparente.

        Args:
            max_attempts (int): Numero massimo di tentativi (default: 10, 0 = infinito)

        Returns:
            int: Distanza misurata in unità configurate

        Raises:
            SensorError: Se max_attempts è raggiunto e ci sono ancora errori
        """
        attempts = 0
        last_error = None

        while max_attempts == 0 or attempts < max_attempts:
            try:
                start_time = perf_counter()
                distance = self._send()
                latency = perf_counter() - start_time

                self._last = distance
                log(
                    f"{self.sensor_id} ➔ {distance}cm "
                    f"(latency: {latency * 1000:.1f}ms)",
                    "INFO"
                )
                return distance
            except SensorError as e:
                last_error = e
                log(
                    f"Errore sensore {self.sensor_id}: {e.message}",
                    "ERROR"
                )
                attempts += 1
                sleep(0.005)  # Prevenzione busy loop

        # Se arrivassimo qui, vorrebbe dire che abbiamo superato max_attempts
        if last_error:
            raise last_error
        # Caso molto improbabile, nessun errore && nessun valore
        return self._last if self._last >= 0 else 0

    def benchmark(self, samples: int = 20) -> dict:
        """Esegue test prestazionali sul sensore.

        Args:
            samples (int): Numero di campioni da raccogliere

        Returns:
            dict: Statistiche dettagliate con:
                - success: letture riuscite
                - timeouts: timeout comunicazione
                - errors: altri errori
                - latency metrics
        """
        stats = {
            'success': 0,
            'timeouts': 0,
            'errors': 0,
            'avg_latency': 0.0,
            'min_latency': float('inf'),
            'max_latency': 0.0,
            'total_time': 0.0
        }

        for _ in range(samples):
            start = perf_counter()
            try:
                self.get_distance()
                latency = perf_counter() - start

                stats['success'] += 1
                stats['total_time'] += latency
                stats['min_latency'] = min(stats['min_latency'], latency)
                stats['max_latency'] = max(stats['max_latency'], latency)

                log(
                    f"Benchmark sample: {latency * 1000:.2f}ms",
                    "PERF"
                )
            except SensorError as e:
                if e.error_type == SensorErrorType.TIMEOUT:
                    stats['timeouts'] += 1
                else:
                    stats['errors'] += 1
                log(f"Errore benchmark: {e.message}", "WARN")

        if stats['success'] > 0:
            stats['avg_latency'] = stats['total_time'] / stats['success']

        log(
            f"Risultati benchmark: "
            f"Successi={stats['success']} "
            f"Timeout={stats['timeouts']} "
            f"Errori={stats['errors']} "
            f"Latency avg={stats['avg_latency'] * 1000:.1f}ms",
            "INFO"
        )

        return stats

    def adaptive_timeout(self, window_size: int = 20):
        """Algoritmo adattivo per impostare il timeout sulla base delle latenze storiche."""
        # Array delle ultime N latenze
        latencies = []
        for _ in range(window_size):
            try:
                start = perf_counter()
                self._send()
                latency = perf_counter() - start
                latencies.append(latency)
                if len(latencies) > window_size:
                    latencies.pop(0)
            except SensorError:
                pass  # Ignora fallimenti per questa stima

        if latencies:
            max_latency = max(latencies)
            avg_latency = sum(latencies) / len(latencies)
            suggested_timeout = max_latency + 0.02 #ms
            log(
                f"Timeout adattivo suggerito: "
                f"{suggested_timeout * 1000:.1f} ms "
                f"(avg: {avg_latency * 1000:.1f} ms, max: {max_latency * 1000:.1f} ms)",
                "PERF"
            )
            self.timeout = suggested_timeout
            return suggested_timeout
        else:
            log("Impossibile calcolare timeout adattivo: nessuna lettura valida.", "WARN")
            return self.timeout