import os
import sys
import time
import subprocess
import toml
import logging
import argparse
import platform

if platform.system() == "Linux":
    import daemon

def read_config(path):
    with open(path, 'r') as f:
        config = toml.load(f)
    return config

def setup_logging(log_file):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )

def git_pull(local_path):
    logging.info(f"Eseguo git pull in {local_path}")
    result = subprocess.run(
        ["git", "pull"],
        cwd=local_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    if result.returncode != 0:
        logging.error(f"Errore in git pull: {result.stderr.decode()}")
        raise Exception(f"Git pull fallito con codice {result.returncode}")
    else:
        logging.info("Eseguito con successo")

def updater_loop(config):
    local_path = config['local_path']
    interval = config['interval_seconds']

    while True:
        if os.path.exists(os.path.join(local_path, ".git")):
            try:
                git_pull(local_path)
            except Exception as e:
                logging.error(f"Errore nel pull: {e}")
        else:
            logging.error(f"Cartella non trovata o non è un repository Git: {local_path}")
        time.sleep(interval)

def main():
    parser = argparse.ArgumentParser(description="Daemon per aggiornare un repo GitHub")
    parser.add_argument("config", help="Path al file di configurazione TOML")
    args = parser.parse_args()

    config = read_config(args.config)
    setup_logging(config['log_file'])

    if platform.system() == "Linux":
        logging.info("Demonizzazione del processo su Linux...")
        with daemon.DaemonContext(
            stdout=open("/tmp/git_updater.out", "w+"),
            stderr=open("/tmp/git_updater.err", "w+")
        ):
            # NON cambiamo working dir, ma passiamo il path in ogni comando
            updater_loop(config)
    else:
        logging.info("Avvio in modalità normale su macOS...")
        updater_loop(config)

if __name__ == "__main__":
    main()
