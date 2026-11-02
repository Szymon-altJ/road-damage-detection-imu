import serial
import pynmea2
import time
import signal
import sys
import csv
import os
from datetime import datetime
from collections import deque
import adafruit_mpu6050
from adafruit_extended_bus import ExtendedI2C as I2C
import board
import glob
import math
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import threading

manual_event = False


def wait_for_input():
    global manual_event
    while True:
        try:
            input()
            manual_event = True
        except EOFError:
            # Ignoruj błąd jeśli skrypt zostanie odpięty od terminala (np. nohup)
            time.sleep(1)


class GPSParser:
    def __init__(self, log_dir="logs", port='/dev/serial0', baudrate=115200, run_id=1):
        os.makedirs(log_dir, exist_ok=True)
        date = datetime.now().strftime("%d_%m_%y")

        self.gps_path = os.path.join(log_dir, f"GPS_{date}_{run_id}.csv")
        self.csv_file = None
        self.csv_writer = None
        self.last_fix = None

        try:
            self.csv_file = open(self.gps_path, "w", newline="", buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp", "Lat", "Lon", "Satellites", "Altitude"])
        except Exception as e:
            print(f"Błąd otwarcia pliku CSV GPS: {e}")

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            print(f"Połączono z GPS na port {port} ({baudrate} baud).")
        except Exception as e:
            print(f"Błąd otwarcia portu szeregowego GPS: {e}")
            self.serial_port = None

    def get_data(self):
        if not self.serial_port or not self.serial_port.is_open:
            try:
                if self.serial_port:
                    self.serial_port.close()
                self.serial_port = serial.Serial('/dev/serial0', 115200, timeout=1)
                time.sleep(2)
            except Exception:
                time.sleep(5)
                return None

        start = time.time()
        while time.time() - start < 1.0:
            try:
                line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                if not line:
                    continue

                msg = pynmea2.parse(line)

                if isinstance(msg, pynmea2.types.talker.GGA):
                    if int(msg.gps_qual) > 0:
                        data = {
                            "time": msg.timestamp,
                            "lat": round(msg.latitude, 6),
                            "lon": round(msg.longitude, 6),
                            "satellites": int(msg.num_sats),
                            "altitude": float(msg.altitude) if msg.altitude else None
                        }
                        self.last_fix = data
                        self.log_to_csv(data)
                        return data

                elif isinstance(msg, pynmea2.types.talker.RMC):
                    if getattr(msg, 'status', 'V') == "A":
                        data = {
                            "time": msg.timestamp,
                            "lat": round(msg.latitude, 6),
                            "lon": round(msg.longitude, 6),
                            "satellites": "N/A",
                            "altitude": "N/A"
                        }
                        self.last_fix = data
                        self.log_to_csv(data)
                        return data

            except pynmea2.ParseError:
                continue
            except Exception as e:
                # Ignorujemy błędy odczytu pojedynczych zniekształconych linii
                pass

        return None

    def log_to_csv(self, data):
        if not self.csv_writer:
            return
        self.csv_writer.writerow([
            datetime.now().isoformat(),
            data["lat"],
            data["lon"],
            data["satellites"],
            data["altitude"]
        ])

    def get_last_fix(self):
        return self.last_fix

    def close(self):
        if self.csv_file:
            self.csv_file.close()


class MPUParser:
    def __init__(self, log_dir="logs", run_id=1, calibration_time=3.0, filter_window=5.0, threshold=5.0, cooldown=0.5):

        os.makedirs(log_dir, exist_ok=True)
        date = datetime.now().strftime("%d_%m_%y")

        self.imu_path = os.path.join(log_dir, f"AKCELEROMETR_{date}_{run_id}.csv")
        self.event_path = os.path.join(log_dir, f"ZDARZENIA_{date}_{run_id}.csv")

        self.csv = open(self.imu_path, "w", newline="", buffering=1)
        self.writer = csv.writer(self.csv)
        self.writer.writerow([
            "Timestamp", "AccX", "AccY", "AccZ",
            "GyroX", "GyroY", "GyroZ",
            "Roll", "Pitch", "VerticalAcc", "Event"
        ])

        self.events = open(self.event_path, "w", newline="", buffering=1)
        self.event_writer = csv.writer(self.events)
        self.event_writer.writerow([
            "Timestamp", "VerticalAcc", "Roll", "Pitch",
            "GPS_Lat", "GPS_Lon", "Video"
        ])

        self.filter = deque(maxlen=filter_window)
        self.threshold = threshold
        self.cooldown = cooldown
        self.last_event = 0

        i2c_bus = I2C(3)
        self.mpu = adafruit_mpu6050.MPU6050(i2c_bus)

        print("Kalibracja MPU...")
        ax = ay = az = gx = gy = gz = 0.0
        n = 0
        start = time.time()
        while time.time() - start < calibration_time:
            a = self.mpu.acceleration
            g = self.mpu.gyro
            ax += a[0];
            ay += a[1];
            az += a[2]
            gx += g[0];
            gy += g[1];
            gz += g[2]
            n += 1
            time.sleep(0.01)

        self.ax0 = ax / n
        self.ay0 = ay / n
        self.az0 = az / n

        self.roll0 = math.degrees(math.atan2(self.ay0, self.az0))
        self.pitch0 = math.degrees(math.atan2(-self.ax0, math.sqrt(self.ay0 ** 2 + self.az0 ** 2)))

        print(f"Roll0={self.roll0:.2f} Pitch0={self.pitch0:.2f}")

    def get_data(self):
        # Pobieranie danych z czujnika I2C (może rzucić błąd OSError przy wstrząsie)
        ax, ay, az = self.mpu.acceleration
        gx, gy, gz = self.mpu.gyro

        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        roll_r = math.radians(roll)
        pitch_r = math.radians(pitch)

        vertical = (
                -ax * math.sin(pitch_r)
                + ay * math.sin(roll_r) * math.cos(pitch_r)
                + az * math.cos(roll_r) * math.cos(pitch_r)
        )
        vertical -= 9.81

        self.filter.append(vertical)
        filt = sum(self.filter) / len(self.filter)

        event = 0
        if abs(filt) > self.threshold:
            if time.time() - self.last_event > self.cooldown:
                event = 1
                self.last_event = time.time()

        self.writer.writerow([
            datetime.now().isoformat(),
            round(ax, 3), round(ay, 3), round(az, 3),
            round(gx, 3), round(gy, 3), round(gz, 3),
            round(roll, 2), round(pitch, 2),
            round(filt, 3), event
        ])

        return {
            "event": event,
            "vertical_acc": filt,
            "roll": roll,
            "pitch": pitch,
            "accel": (ax, ay, az),
            "gyro": (gx, gy, gz),
            "timestamp": datetime.now().isoformat()
        }

    def record_event(self, vertical_acc, roll, pitch, gps, video):
        lat = gps["lat"] if gps else ""
        lon = gps["lon"] if gps else ""
        self.event_writer.writerow([
            datetime.now().isoformat(),
            round(vertical_acc, 3),
            round(roll, 2),
            round(pitch, 2),
            lat,
            lon,
            video
        ])

    def close(self):
        self.csv.close()
        self.events.close()


class CameraBuffer:
    def __init__(self, output_dir="videos", fps=20, width=1280, height=720, seconds_before=2, seconds_after=2):
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.fps = fps
        self.seconds_before = seconds_before
        self.seconds_after = seconds_after
        self.buffer = deque(maxlen=fps * (seconds_before + seconds_after + 2))
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cfg)
        self.running = False
        self.lock = threading.Lock()
        self.frame_counter = 0
        self.event_counter = 0

    def start(self):
        self.running = True
        self.picam2.start()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        period = 1.0 / self.fps
        while self.running:
            try:
                start = time.time()
                frame = self.picam2.capture_array()
                with self.lock:
                    self.frame_counter += 1
                    self.buffer.append((self.frame_counter, time.time(), frame.copy()))
                dt = time.time() - start
                if dt < period:
                    time.sleep(period - dt)
            except Exception as e:
                # Ochrona przed zatrzymaniem pętli jeśli tasiemka kamery zgubi styk
                with open(os.path.join("logs", "error_log.txt"), "a") as err_f:
                    err_f.write(f"{datetime.now().isoformat()} - CAMERA LOOP ERROR:\n{traceback.format_exc()}\n")
                time.sleep(0.5)

    def trigger_save_event(self):
        filename = f"DZIURA_{datetime.now().strftime('%d_%m_%y_%H_%M_%S')}.mp4"
        threading.Thread(target=self._save_event_thread, args=(filename,), daemon=True).start()
        return filename

    def _save_event_thread(self, filename):
        self.event_counter += 1
        path = os.path.join(self.output_dir, filename)

        with self.lock:
            event_frame = self.frame_counter
            before_frames = [frame.copy() for _, _, frame in self.buffer]

        before_count = int(self.seconds_before * self.fps)
        after_count = int(self.seconds_after * self.fps)
        before_frames = before_frames[-before_count:]
        target_frame = event_frame + after_count

        # Zabezpieczenie przed nieskończoną pętlą oczekiwania w przypadku awarii kamery
        timeout_start = time.time()
        while self.running and (time.time() - timeout_start < (self.seconds_after + 5)):
            with self.lock:
                current_frame = self.frame_counter
            if current_frame >= target_frame:
                break
            time.sleep(0.005)

        with self.lock:
            after_frames = [
                frame.copy() for number, _, frame in self.buffer
                if event_frame < number <= target_frame
            ]

        frames = before_frames + after_frames
        expected_frames = before_count + after_count
        if not frames:
            print("Nie udało się zapisać filmu.")
            return

        height, width = frames[0].shape[:2]
        writer = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.fps,
            (width, height)
        )
        for frame in frames:
            writer.write(frame)
        writer.release()

        print("\n================================")
        print("ZAPISANO ZDARZENIE")
        print(f"Plik: {path}")
        print(f"Klatki: {len(frames)} / Oczekiwano: {expected_frames}")
        print("=========================\n")

    def stop(self):
        self.running = False
        self.picam2.stop()


def cleanup(gps, mpu, camera):
    print("\nCzyszczenie sprzętu...")
    if gps:
        gps.close()
    if mpu:
        mpu.close()
    if camera:
        camera.stop()
    GPIO.cleanup()
    sys.exit(0)


if __name__ == "__main__":
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)

    date_str = datetime.now().strftime("%d_%m_%y")
    existing_runs = glob.glob(os.path.join(log_dir, f"AKCELEROMETR_{date_str}_*.csv"))
    run_id = len(existing_runs) + 1

    gps = GPSParser(log_dir=log_dir, run_id=run_id, baudrate=115200)
    mpu = MPUParser(log_dir=log_dir, run_id=run_id)
    camera = CameraBuffer()

    print("Rozpoczęto odczyt z czujników...")
    camera.start()
    threading.Thread(target=wait_for_input, daemon=True).start()

    signal.signal(signal.SIGTERM, lambda s, f: cleanup(gps, mpu, camera))

    try:
        last_gps_time = 0
        gps_has_fix = False
        gps_failures = 0

        while True:
            try:
                # IMU
                data = mpu.get_data()
                if data["event"] or manual_event:
                    manual_event = False
                    video_filename = camera.trigger_save_event()
                    gps_data = gps.get_last_fix()

                    mpu.record_event(
                        vertical_acc=data["vertical_acc"],
                        roll=data["roll"],
                        pitch=data["pitch"],
                        gps=gps_data,
                        video=video_filename
                    )

                # GPS co 1s
                if time.time() - last_gps_time > 1.0:
                    gps_res = gps.get_data()
                    last_gps_time = time.time()

                    if gps_res is not None:
                        gps_failures = 0
                        if not gps_has_fix:
                            gps_has_fix = True
                            print("GPS FIX")
                    else:
                        gps_failures += 1
                        if gps_has_fix and gps_failures >= 5:
                            gps_has_fix = False
                            print("GPS LOST")

                time.sleep(0.01)

            except Exception as e:
                # Ochrona przed sprzętowymi błędami odczytu (np. przerwane połączenie I2C przy wstrząsie)
                err_msg = traceback.format_exc()
                print(
                    f"\n[BŁĄD ZIGNOROWANY] {datetime.now().isoformat()}: Odczyt z czujnika zawiódł. Patrz error_log.txt")
                with open(os.path.join(log_dir, "error_log.txt"), "a") as err_file:
                    err_file.write(f"--- {datetime.now().isoformat()} ---\nMAIN LOOP ERROR:\n{err_msg}\n")
                time.sleep(1)  # Zabezpiecza przed "spamowaniem" logów jeśli sprzęt całkowicie odpadnie

    except KeyboardInterrupt:
        print("\nZatrzymano odczyt danych.")
        cleanup(gps, mpu, camera)