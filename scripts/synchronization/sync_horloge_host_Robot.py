import subprocess

def sync_time():
    try:
        # Activer et démarrer les services de synchronisation temporelle
        subprocess.run(["sudo", "systemctl", "enable", "ntp"], check=True)
        subprocess.run(["sudo", "systemctl", "start", "ntp"], check=True)
        subprocess.run(["sudo", "systemctl", "enable", "systemd-timesyncd"], check=True)
        subprocess.run(["sudo", "systemctl", "start", "systemd-timesyncd"], check=True)
        
        # Activer la synchronisation NTP
        subprocess.run(["sudo", "timedatectl", "set-ntp", "true"], check=True)
        
        # Exécuter la commande SSH avec mot de passe
        ssh_command = "sshpass -p 'hiwonder' ssh -o StrictHostKeyChecking=no jetauto@192.168.149.1 'date -u'"
        result = subprocess.run(ssh_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        
        if result.returncode == 0:
            remote_date = result.stdout.strip()
            subprocess.run(["sudo", "date", "-s", remote_date], check=True)
            print(f"Heure synchronisée : {remote_date}")
        else:
            print("Non connecté au robot")
    
    except Exception as e:
        print(f"Erreur : {e}")

if __name__ == "__main__":
    sync_time()
