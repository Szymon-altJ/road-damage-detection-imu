import serial
import pynmea2
import time

# do mapy
import csv
import os
from datetime import datetime

import folium #nieużywane póki co, będzie do mapowania

class GPSParser:
    def __init__(self, port='/dev/serial0', baudrate=115200):
        # Inicjalizacja połączenia szeregowego
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to GPS on {port} at {baudrate} baud.")

            # nagłówki tylko jeśli plik pusty
            self.csv_file = open("gps_log.csv", "a", newline="", buffering=1)
            self.csv_writer = csv.writer(self.csv_file)

            if os.stat("gps_log.csv").st_size == 0:
                self.csv_writer.writerow(["timestamp", "lat", "lon", "satellites", "altitude"])

        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.serial_port = None

    def get_data(self):
        if not self.serial_port:
            return None

        try:
            # Odczyt jednej linii z GPS
            line = self.serial_port.readline().decode('ascii', errors='replace')
            # Interesuje nas głównie ramka GGA (pozycja i czas) lub RMC (pozycja, czas i prędkość)
            if line.startswith(('$GNGGA', '$GNRMC', '$GPGGA', '$GPRMC')):
                try:
                    msg = pynmea2.parse(line)
                    
                    # Sprawdzenie czy mamy "Fix" (czy dane są poprawne)
                    # W pynmea2 dla GGA: msg.gps_qual > 0 oznacza, że mamy sygnał
                    if hasattr(msg, 'gps_qual') and int(msg.gps_qual) > 0:
                        data = {
                            "time": msg.timestamp,
                            "lat": round(msg.latitude, 6),
                            "lon": round(msg.longitude, 6),
                            "satellites": getattr(msg, 'num_sats', 'N/A'),
                            "altitude": getattr(msg, 'altitude', 'N/A')
                        }

                        self.log_to_csv(data)

                        return data
                except pynmea2.ParseError:
                    return None
        except Exception as e:
            print(f"Read error: {e}")
            return None
        return None
    
    def log_to_csv(self, data):
        self.csv_writer.writerow([
            datetime.now().isoformat(),
            data["lat"],
            data["lon"],
            data["satellites"],
            data["altitude"]
        ])
        self.csv_file.flush()
        os.fsync(self.csv_file.fileno())  # gwarantuje zapis na kartę SD

# --- Testowanie ---
if __name__ == "__main__":
    # Bo mamy 115200 - sprawdziałem
    gps = GPSParser(baudrate=115200) 
    
    print("Oczekiwanie na dane z satelitów (może to potrwać kilka minut)...")
    try:
        while True:
            data = gps.get_data()
            if data:
                print("-" * 30)
                print(f"Godzina: {data['time']}")
                print(f"Szerokość (Lat): {data['lat']}")
                print(f"Długość (Lon):  {data['lon']}")
                print(f"Satelity:        {data['satellites']}")
            time.sleep(0.1) # Mała przerwa dla procesora
    except KeyboardInterrupt:
        print("\nZatrzymano odczyt GPS.")
        if gps.csv_file:
            gps.csv_file.close()