import threading
import serial
from time import perf_counter, sleep, time
from enum import Enum
from typing import Optional
from colorama import Fore, Style, init as colorama_init  # Aggiunti per i colori
from SensorError import SensorError
from SensorErrorType import SensorErrorType

# Inizializzazione colorama per colori cross-platform
colorama_init(autoreset=True)

# Configurazione colori per logging
COLORI_LOG = {
    "DEBUG": Fore.CYAN,
    "INFO": Fore.GREEN,
    "WARN": Fore.YELLOW,
    "ERROR": Fore.RED,
    "CRITICAL": Fore.RED + Style.BRIGHT,
    "PERF": Fore.MAGENTA  # Aggiunto per metriche prestazionali
}


def log(msg: str, level: str = "INFO"):
    """Registra messaggi formattati con colori completi

    Args:
        msg (str): Messaggio da loggare
        level (str): Livello di gravità (DEBUG/INFO/WARN/ERROR/CRITICAL/PERF)
    """
    ts = time()
    colore = COLORI_LOG.get(level, COLORI_LOG["INFO"])
    print(f"{colore}[{level}] {ts:.4f}: {msg}{Style.RESET_ALL}")


class ColorSensor:
    """
    Classe per la lettura avanzata dei colori con gestione errori robusta,
    ottimizzata per bassa latenza e alta affidabilità.

    Attributi:
        COLOR_MAP (dict): Mappatura codici colore Arduino -> valori leggibili
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
        """
        Inizializza il sensore colore.

        Args:
            arduino_conn (serial.Serial): Connessione seriale ad Arduino
            lock (threading.Lock): Mutex per accesso thread-safe
            sensor_id (str): Identificativo univoco del sensore
            command_code (str): Codice comando per il sensore
            timeout (float): Timeout lettura (default: 0.2s)
            retries (int): Tentativi prima di fallire (default: 2)
            baudrate (int): Velocità comunicazione (default: 115200)
        """
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

        # Ottimizzazione parametri seriali
        self.arduino.timeout = 0.005  # Timeout non bloccante
        self.arduino.write_timeout = 0.2  # Timeout scrittura aggressivo

    def _send(self) -> str:
        """Invio comando e lettura risposta con retry integrato.

        Returns:
            str: Codice colore ricevuto

        Raises:
            SensorError: In caso di errori di comunicazione o dati invalidi
        """
        for attempt in range(1, self.retries + 1):
            with self.lock:
                start = perf_counter()
                try:
                    self.arduino.reset_input_buffer()
                    self.arduino.write(self.command_code.encode())
                except Exception as e:
                    raise SensorError(
                        SensorErrorType.COMMUNICATION,
                        f"Errore scrittura: {str(e)}"
                    )

                # Monitoraggio prestazioni scrittura
                write_dur = perf_counter() - start
                if write_dur > 0.02:
                    log(f"Latenza scrittura elevata: {write_dur:.3f}s", "WARN")
                    raise SensorError(
                        SensorErrorType.COMMUNICATION,
                        f"Write lento: {write_dur:.3f}s"
                    )

                # Calcolo timeout dinamico basato su baudrate
                expect_bytes = len(self._pattern) + 2
                transmission_time = (10 * expect_bytes) / self.baudrate
                timeout = self.timeout + transmission_time

                try:
                    resp = self._read(timeout)
                except Exception as e:
                    raise SensorError(
                        SensorErrorType.COMMUNICATION,
                        f"Errore lettura: {str(e)}"
                    )

                if resp:
                    if resp in self.COLOR_MAP:
                        log(f"Lettura valida: {resp}", "DEBUG")
                        return resp
                    raise SensorError(
                        SensorErrorType.INVALID_DATA,
                        f"Codice colore invalido: {resp}"
                    )

            # Backoff progressivo per tentativi falliti
            sleep(0.005 * attempt)
            log(f"Tentativo {attempt} fallito, retry...", "WARN")

        raise SensorError(SensorErrorType.TIMEOUT, "Timeout dopo tutti i tentativi")

    def _read(self, timeout: float) -> Optional[str]:
        """Lettura e parsing della risposta seriale.

        Args:
            timeout (float): Tempo massimo attesa risposta

        Returns:
            Optional[str]: Codice colore letto o None
        """
        start = perf_counter()
        self._buffer.clear()

        while perf_counter() - start < timeout:
            try:
                chunk = self.arduino.read(32)
                if chunk:
                    self._buffer.extend(chunk)

                    if b'\n' in self._buffer:
                        lines = self._buffer.split(b'\n')
                        self._buffer = lines[-1]  # Conserva dati incompleti

                        for line in lines[:-1]:
                            if line.startswith(self._pattern):
                                decoded = line[len(self._pattern):].decode().strip()
                                log(f"Raw response: {decoded}", "DEBUG")
                                return decoded
            except Exception as e:
                log(f"Errore durante la lettura: {str(e)}", "ERROR")
                return None

        log("Timeout lettura risposta", "WARN")
        return None

    def get_color(self) -> str:
        """Ottiene il colore corrente con gestione errori trasparente.

        Returns:
            str: Nome del colore rilevato
        """
        while True:
            try:
                start_time = perf_counter()
                code = self._send()
                color = self.COLOR_MAP.get(code, self.COLOR_MAP['0'])
                latency = perf_counter() - start_time

                self._last = color
                log(
                    f"{self.sensor_id} ➔ {color} "
                    f"(latency: {latency * 1000:.1f}ms)",
                    "INFO"
                )
                return color
            except SensorError as e:
                log(f"Errore sensore {self.sensor_id}: {str(e)}", "ERROR")
                sleep(0.005)  # Prevenzione busy loop

    def benchmark(self, samples: int = 10) -> dict:
        """Esegue test prestazionali sul sensore.

        Args:
            samples (int): Numero di campioni da raccogliere

        Returns:
            dict: Statistiche prestazionali dettagliate
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
                self.get_color()
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
                log(f"Errore benchmark: {str(e)}", "WARN")

        if stats['success'] > 0:
            stats['avg_latency'] = stats['total_time'] / stats['success']

        log(
            f"Risultati benchmark: "
            f"Successi={stats['success']}, "
            f"Timeout={stats['timeouts']}, "
            f"Errori={stats['errors']}, "
            f"Latency avg={stats['avg_latency'] * 1000:.1f}ms",
            "INFO"
        )

        return stats

    def adaptive_timeout(self, window_size: int = 20):
        """TODO: Implementare adattamento dinamico timeout"""
        # Implementazione futura basata su dati storici
        pass