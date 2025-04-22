import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import queue
import time
import json
import os
from serial.tools import list_ports

# File di configurazione per impostazioni persistenti
CONFIG_FILE = 'config.json'

# Mappatura colori da codici a nomi e valori esadecimali
COLOR_MAP = {
    '0': {'name': 'sconosciuto', 'hex': '#000000'},
    '1': {'name': 'bianco', 'hex': '#FFFFFF'},
    '2': {'name': 'rosso', 'hex': '#FF0000'},
    '3': {'name': 'scuro', 'hex': '#333333'},
    '4': {'name': 'verde', 'hex': '#00FF00'},
    '5': {'name': 'blu', 'hex': '#0000FF'},
    '6': {'name': 'giallo', 'hex': '#FFFF00'}
}


class SensorDashboard:
    def __init__(self, root):
        """Inizializza la dashboard principale"""
        self.root = root
        self.root.title("Robot Sensor Dashboard")
        self.root.geometry("900x600")
        self.running = False  # Flag per lo stato della connessione
        self.data_queue = queue.Queue()  # Coda per i dati dai sensori

        # Schema colori dell'interfaccia
        self.colors = {
            'background': '#2E2E2E',
            'frame': '#404040',
            'text': '#FFFFFF',
            'proximity': '#96CEB4',
            'warning': '#FFD700'
        }

        self.load_config()  # Carica la configurazione
        self.configure_styles()  # Configura lo stile grafico
        self.create_widgets()  # Crea i componenti dell'interfaccia
        self.init_sensor_data()  # Inizializza i dati dei sensori
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)  # Gestione chiusura

    def load_config(self):
        """Carica la configurazione da file o usa valori di default"""
        default = {
            'serial_port': '/dev/cu.usbmodem142201',
            'baudrate': 115200,
            'command': '7',
            'sensors': [
                {'keyword': 'COL1', 'type': 'color'},
                {'keyword': 'COL2', 'type': 'color'},
                {'keyword': 'DISTANZA', 'type': 'distance'}
            ]
        }
        # Tentativo di caricamento configurazione esistente
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    self.config = json.load(f)
            except Exception as e:
                messagebox.showerror("Errore caricamento config", str(e))
                self.config = default
        else:
            self.config = default

    def save_config(self):
        """Salva la configurazione corrente su file"""
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            messagebox.showinfo("Impostazioni salvate", "Configurazione salvata. Riavvia l'app.")
        except Exception as e:
            messagebox.showerror("Errore salvataggio config", str(e))

    def configure_styles(self):
        """Configura gli stili grafici per i widget"""
        style = ttk.Style()
        style.theme_use('clam')  # Tema scuro
        # Configurazione stili per diversi componenti
        style.configure('TFrame', background=self.colors['background'])
        style.configure('TLabel', background=self.colors['frame'], foreground=self.colors['text'])
        style.configure('TButton', font=('Helvetica', 12), padding=5)
        style.configure('Title.TLabel', font=('Helvetica', 16, 'bold'), foreground=self.colors['text'])

    def create_widgets(self):
        """Crea i componenti grafici principali"""
        # Area superiore con titolo e pulsante impostazioni
        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, pady=5)
        ttk.Label(top, text="Robot Sensor Dashboard", style='Title.TLabel').pack(side=tk.LEFT, padx=10)
        ttk.Button(top, text="Impostazioni", command=self.open_settings).pack(side=tk.RIGHT, padx=10)

        # Area principale per la visualizzazione dei sensori
        self.display = ttk.Frame(self.root)
        self.display.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.sensor_frames = {}
        self.build_sensor_views()

        # Area di controllo inferiore
        ctrl = ttk.Frame(self.root)
        ctrl.pack(pady=5)
        self.btn_connect = ttk.Button(ctrl, text="Connetti", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl, text="Emergency Stop", command=self.emergency_stop).pack(side=tk.LEFT, padx=5)

    def build_sensor_views(self):
        """Costruisce le visualizzazioni per i sensori in base alla configurazione"""
        # Pulisce la vista precedente
        for child in self.display.winfo_children():
            child.destroy()
        self.sensor_frames.clear()

        # Crea un frame per ogni sensore configurato
        for sensor in self.config['sensors']:
            key, typ = sensor['keyword'], sensor['type']
            frame = ttk.Frame(self.display)
            frame.pack(side=tk.LEFT, padx=15)

            # Vista per sensori di colore
            if typ == 'color':
                canvas = tk.Canvas(frame, width=150, height=150, bg=self.colors['frame'], highlightthickness=0)
                canvas.pack()
                label = ttk.Label(frame, text=f"{key}: sconosciuto")
                label.pack()
                self.sensor_frames[key] = {'type': typ, 'canvas': canvas, 'label': label}

            # Vista per sensori di distanza
            elif typ == 'distance':
                canvas = tk.Canvas(frame, width=300, height=120, bg=self.colors['frame'], highlightthickness=0)
                canvas.pack()
                label = ttk.Label(frame, text="Distanza: 0.0 cm")
                label.pack()
                self.sensor_frames[key] = {'type': typ, 'canvas': canvas, 'label': label}

            # Vista generica per altri tipi
            else:
                label = ttk.Label(frame, text=f"{key}: -")
                label.pack()
                self.sensor_frames[key] = {'type': typ, 'label': label}

    def init_sensor_data(self):
        """Inizializza la struttura dati per i valori dei sensori"""
        self.sensor_data = {}
        for sensor in self.config['sensors']:
            if sensor['type'] == 'color':
                self.sensor_data[sensor['keyword']] = COLOR_MAP['0']
            else:
                self.sensor_data[sensor['keyword']] = 0.0

    def open_settings(self):
        """Apre la finestra di configurazione delle impostazioni"""
        win = tk.Toplevel(self.root)
        win.title("Impostazioni")
        win.grab_set()

        # Widget per la selezione della porta seriale
        ttk.Label(win, text="Porta Serial:").grid(row=0, column=0, sticky=tk.W)
        ports = [p.device for p in list_ports.comports()]
        port_var = tk.StringVar(value=self.config['serial_port'])
        if port_var.get() not in ports:
            ports.append(port_var.get())
        cb_port = ttk.Combobox(win, textvariable=port_var, values=ports, state='readonly')
        cb_port.grid(row=0, column=1, padx=5, pady=2)

        # Widget per il baudrate
        ttk.Label(win, text="Baudrate:").grid(row=1, column=0, sticky=tk.W)
        e_baud = ttk.Entry(win)
        e_baud.insert(0, str(self.config['baudrate']))
        e_baud.grid(row=1, column=1)

        # Widget per il comando
        ttk.Label(win, text="Comando:").grid(row=2, column=0, sticky=tk.W)
        e_cmd = ttk.Entry(win)
        e_cmd.insert(0, self.config['command'])
        e_cmd.grid(row=2, column=1)

        # Widget per il numero di sensori
        ttk.Label(win, text="Numero sensori:").grid(row=3, column=0, sticky=tk.W)
        num_var = tk.IntVar(value=len(self.config['sensors']))

        # Validazione input numerico
        def validate_spin(new_val):
            if not new_val:
                num_var.set(1)
                return False
            return new_val.isdigit() and 1 <= int(new_val) <= 10

        spin = ttk.Spinbox(win, from_=1, to=10, textvariable=num_var, width=5,
                           validate='key', validatecommand=(win.register(validate_spin), '%P'))
        spin.grid(row=3, column=1, sticky=tk.W)

        # Area per la configurazione dei sensori
        sensors_frame = ttk.Frame(win)
        sensors_frame.grid(row=4, column=0, columnspan=2, pady=10)
        kw_vars = []
        typ_vars = []

        def rebuild(*args):
            """Ricostruisce l'interfaccia per la configurazione dei sensori"""
            try:
                num = max(1, min(num_var.get(), 10))
            except:
                num = 1
                num_var.set(1)

            for widget in sensors_frame.winfo_children():
                widget.destroy()

            kw_vars.clear()
            typ_vars.clear()

            # Crea campi per ogni sensore
            for i in range(num):
                ttk.Label(sensors_frame, text=f"Sensore {i + 1} keyword:").grid(row=i, column=0, padx=2)
                kw = tk.StringVar()
                typ = tk.StringVar(value='color' if i < 2 else 'distance')

                if i < len(self.config['sensors']):
                    kw.set(self.config['sensors'][i]['keyword'])
                    typ.set(self.config['sensors'][i]['type'])

                ttk.Entry(sensors_frame, textvariable=kw).grid(row=i, column=1, padx=2)
                cb = ttk.Combobox(sensors_frame, textvariable=typ,
                                  values=['color', 'distance', 'numeric'], state='readonly')
                cb.grid(row=i, column=2, padx=2)
                kw_vars.append(kw)
                typ_vars.append(typ)

        num_var.trace_add('write', rebuild)
        rebuild()

        def save():
            """Salva le nuove impostazioni"""
            self.config['serial_port'] = port_var.get()

            try:
                self.config['baudrate'] = int(e_baud.get())
            except ValueError:
                messagebox.showerror("Errore", "Baudrate deve essere un numero valido")
                return

            self.config['command'] = e_cmd.get()
            self.config['sensors'] = []

            for kw, typ in zip(kw_vars, typ_vars):
                keyword = kw.get().strip()
                sensor_type = typ.get()
                if keyword:
                    self.config['sensors'].append({
                        'keyword': keyword,
                        'type': sensor_type
                    })

            self.save_config()
            self.build_sensor_views()
            self.init_sensor_data()
            win.destroy()

        ttk.Button(win, text="Salva", command=save).grid(row=5, column=0, columnspan=2, pady=10)

    def toggle_connection(self):
        """Attiva/Disattiva la connessione seriale"""
        if not self.running:
            self.start_serial()
        else:
            self.stop_serial()

    def start_serial(self):
        """Avvia la connessione seriale e il thread di lettura"""
        try:
            self.serial = serial.Serial(
                self.config['serial_port'],
                self.config['baudrate'],
                timeout=1
            )
            self.running = True
            threading.Thread(target=self.read_loop, daemon=True).start()
            self.btn_connect.config(text="Disconnetti")
            self.root.after(100, self.update_display)
        except Exception as e:
            messagebox.showerror("Errore connessione", str(e))

    def stop_serial(self):
        """Ferma la connessione seriale"""
        self.running = False
        try:
            self.serial.close()
        except:
            pass
        self.btn_connect.config(text="Connetti")

    def read_loop(self):
        """Thread per la lettura continua dalla porta seriale"""
        buf = ''
        while self.running:
            try:
                data = self.serial.read_all().decode('utf-8', errors='replace')
                buf += data
                # Elabora i dati ricevuti linea per linea
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    self.data_queue.put(line.strip())
                # Invia comando di polling
                self.serial.write((self.config['command'] + '\n').encode())
                time.sleep(0.1)
            except Exception as e:
                if self.running:  # Evita messaggi dopo chiusura
                    messagebox.showerror("Errore seriale", str(e))
                break

    def update_display(self):
        """Aggiorna l'interfaccia con i nuovi dati dalla coda"""
        while not self.data_queue.empty():
            raw = self.data_queue.get()
            raw = raw.replace('CCOL1', 'COL1').replace('CCOL2', 'COL2')
            parts = raw.split('|')

            # Estrae i valori dai dati grezzi
            vals = {}
            for i, p in enumerate(parts):
                if p in self.sensor_frames and i + 1 < len(parts):
                    vals[p] = parts[i + 1]

            # Aggiorna i dati dei sensori
            for sensor, value in vals.items():
                if self.sensor_frames[sensor]['type'] == 'color':
                    self.sensor_data[sensor] = COLOR_MAP.get(value.strip(), COLOR_MAP['0'])
                else:
                    try:
                        self.sensor_data[sensor] = float(value)
                    except ValueError:
                        pass

            # Aggiorna le visualizzazioni
            for sensor, widget in self.sensor_frames.items():
                if widget['type'] == 'color':
                    color = self.sensor_data[sensor]
                    widget['canvas'].delete('all')
                    widget['canvas'].create_rectangle(10, 10, 140, 140, fill=color['hex'], outline='')
                    contrast = self.get_contrast(color['hex'])
                    widget['canvas'].create_text(75, 75,
                                                 text=f"{color['name']}\n{color['hex']}",
                                                 fill=contrast,
                                                 font=('Helvetica', 10)
                                                 )
                    widget['label'].config(text=f"{sensor}: {color['name']}")

                elif widget['type'] == 'distance':
                    dist = self.sensor_data[sensor]
                    widget['label'].config(text=f"Distanza: {dist:.1f} cm")
                    widget['canvas'].delete('all')
                    x = 50 + min(dist, 100) / 100 * 200
                    col = self.colors['warning'] if dist < 30 else self.colors['proximity']
                    widget['canvas'].create_line(x, 20, x, 100, fill=col, width=3)
                    widget['canvas'].create_oval(50, 20, 250, 100, outline=self.colors['proximity'], width=2)

                else:
                    widget['label'].config(text=f"{sensor}: {self.sensor_data[sensor]}")

        if self.running:
            self.root.after(100, self.update_display)

    def get_contrast(self, hexcol):
        """Calcola il colore del testo in base allo sfondo per massimo contrasto"""
        h = hexcol.lstrip('#')
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        lum = 0.299 * r + 0.587 * g + 0.114 * b
        return '#FFFFFF' if lum < 128 else '#000000'

    def emergency_stop(self):
        """Invoca l'arresto di emergenza del robot"""
        try:
            self.serial.write(b'!STOP\n')
        except:
            pass
        self.stop_serial()

    def on_close(self):
        """Gestisce la chiusura corretta dell'applicazione"""
        self.stop_serial()
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    app = SensorDashboard(root)
    root.mainloop()