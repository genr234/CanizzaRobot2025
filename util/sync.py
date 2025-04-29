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

def git_clone(repo_url, local_path, branch):
    logging.info(f"Clonazione iniziale di {repo_url} in {local_path}")
    result = subprocess.run(
        ["git", "clone", "-b", branch, repo_url, local_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    logging.info(f"Clonazione iniziale di {repo_url} in {local_path}")
    result = subprocess.run(
        ["git", "clone", "-b", branch, repo_url, local_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    if result.returncode != 0:
        logging.error(f"Errore in git clone: {result.stderr.decode()}")
        raise Exception(f"Git clone fallito con codice {result.returncode}")
    else:
        logging.info(f"Clonazione eseguita con successo in {local_path}")

def git_pull(local_path):
    logging.info(f"Eseguo git pull in {local_path}")
    result = subprocess.run(
        ["git", "pull"],
        cwd=local_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
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
    repo_url = config['repo_url']
    local_path = config['local_path']
    interval = config['interval_seconds']
    branch = config.get('branch', 'main')

    while True:
        if not os.path.exists(os.path.join(local_path, ".git")):
            try:
                git_clone(repo_url, local_path, branch)
            except Exception as e:
                logging.error(f"Errore nella clonazione: {e}")
        else:
            try:
                git_pull(local_path)
            except Exception as e:
                logging.error(f"Errore nel pull: {e}")
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
    parser = argparse.ArgumentParser(description="Daemon per aggiornare un repo GitHub")
    parser.add_argument("config", help="Path al file di configurazione TOML")
    args = parser.parse_args()

    config = read_config(args.config)
    setup_logging(config['log_file'])

    if platform.system() == "Linux":
        logging.info("Demonizzazione del processo su Linux...")
        with daemon.DaemonContext(
            working_directory=config['local_path'],
            stdout=open("/tmp/git_updater.out", "w+"),
            stderr=open("/tmp/git_updater.err", "w+")
        ):
            updater_loop(config)
    else:
        logging.info("Avvio in modalità normale su macOS...")
        updater_loop(config)

if __name__ == "__main__":
    main()
