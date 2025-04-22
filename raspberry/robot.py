from buildhat import MotorPair

from typing import Union


class RobotError(Exception):
    """Eccezione base per gli errori del robot"""
    pass


class Robot:
    def __init__(self, left_port: str, right_port: str):
        """
        Inizializza il robot con i motori specificati

        :param left_port: Porta del motore sinistro (es. 'A')
        :param right_port: Porta del motore destro (es. 'B')
        """
        try:
            self.motor_ruote = MotorPair(left_port, right_port)
            self._default_speed = 50
            self._is_moving = False
        except Exception as e:
            raise RobotError(f"Errore inizializzazione motori: {str(e)}") from e

    @property
    def default_speed(self) -> int:
        """Restituisce la velocità predefinita (0-100)"""
        return self._default_speed

    @default_speed.setter
    def default_speed(self, value: int):
        """Imposta la velocità predefinita (0-100)"""
        if not 0 <= value <= 100:
            raise ValueError("La velocità deve essere tra 0 e 100")
        self._default_speed = value

    def muovi_avanti(self, speed: Union[int, None] = None):
        """
        Avvia il movimento in avanti

        :param speed: Velocità opzionale (0-100), usa default se None
        """
        speed = speed if speed is not None else self.default_speed
        self.motor_ruote.start(speed, speed)
        self._is_moving = True

    def muovi_indietro(self, speed: Union[int, None] = None):
        """
        Avvia il movimento all'indietro

        :param speed: Velocità opzionale (0-100), usa default se None
        """
        speed = speed if speed is not None else self.default_speed
        self.motor_ruote.start(-speed, -speed)
        self._is_moving = True

    def stop_movimento(self):
        """
        Ferma il movimento

        :param brake: Se True, frena attivamente invece di lasciar rotolare
        """
        self.motor_ruote.stop()
        self._is_moving = False

    def muovi_avanti_for(self, unit: str, value: float, speed: Union[int, None] = None):
        """
        Movimento in avanti per unità specificate

        :param unit: Unità di misura ('degrees', 'seconds', 'rotations')
        :param value: Valore dell'unità
        :param speed: Velocità opzionale (0-100)
        """
        speed = speed if speed is not None else self.default_speed
        self._run_movement(value, speed, unit, direction=1)

    def muovi_indietro_for(self, unit: str, value: float, speed: Union[int, None] = None):
        """
        Movimento all'indietro per unità specificate

        :param unit: Unità di misura ('degrees', 'seconds', 'rotations')
        :param value: Valore dell'unità
        :param speed: Velocità opzionale (0-100)
        """
        speed = speed if speed is not None else self.default_speed
        self._run_movement(value, -speed, unit, direction=-1)

    def gira_destra(self, degrees: int = 90, speed: Union[int, None] = None):
        """
        Gira a destra di un certo numero di gradi

        :param degrees: Gradi di rotazione (default: 90)
        :param speed: Velocità opzionale (0-100)
        """
        speed = speed if speed is not None else self.default_speed
        self._turn(degrees, speed, right=True)

    def gira_sinistra(self, degrees: int = 90, speed: Union[int, None] = None):
        """
        Gira a sinistra di un certo numero di gradi

        :param degrees: Gradi di rotazione (default: 90)
        :param speed: Velocità opzionale (0-100)
        """
        speed = speed if speed is not None else self.default_speed
        self._turn(degrees, speed, right=False)

    def _run_movement(self, value: float, speed: int, unit: str, direction: int):
        """Funzione interna per gestire il movimento"""
        valid_units = ['degrees', 'seconds', 'rotations']
        if unit not in valid_units:
            raise RobotError(f"Unità non valida: {unit}. Usare {valid_units}")

        try:
            method = getattr(self.motor_ruote, f'run_for_{unit}')
            method(value, speed * direction, speed * direction)
            self._is_moving = False
        except AttributeError:
            raise RobotError(f"Metodo per unità {unit} non trovato")
        except Exception as e:
            raise RobotError(f"Errore durante il movimento: {str(e)}") from e

    def _turn(self, degrees: int, speed: int, right: bool = True):
        """Funzione interna per gestire le rotazioni"""
        try:
            if right:
                self.motor_ruote.run_for_degrees(degrees, speed, -speed)
            else:
                self.motor_ruote.run_for_degrees(degrees, -speed, speed)
            self._is_moving = False
        except Exception as e:
            raise RobotError(f"Errore durante la rotazione: {str(e)}") from e

    @property
    def is_moving(self) -> bool:
        """Restituisce True se il robot è in movimento"""
        return self._is_moving

    def __enter__(self):
        """Permette l'uso del contesto with"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Garantisce l'arresto dei motori all'uscita del contesto"""
        self.stop_movimento()

    def calibra_motori(self):
        """Calibrazione automatica dei motori"""
        try:
            print("Calibrazione in corso...")
            for _ in range(2):
                self.muovi_avanti_for('seconds', 1, speed=30)
                self.muovi_indietro_for('seconds', 1, speed=30)
                self.gira_destra(90, speed=30)
                self.gira_sinistra(90, speed=30)
            print("Calibrazione completata!")
        except Exception as e:
            raise RobotError(f"Calibrazione fallita: {str(e)}") from e