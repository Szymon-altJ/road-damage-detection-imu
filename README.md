# System wizyjny do detekcji uszkodzeń nawierzchni z wykorzystaniem czujników IMU

### Projekt Przejściowy | Praca Inżynierska | Engineering Thesis 2026

## Opis projektu
System mobilny przeznaczony do automatycznej detekcji i lokalizacji uszkodzeń dróg (dziury, pęknięcia). Rozwiązanie integruje analizę obrazu w czasie rzeczywistym z danymi z czujników inercyjnych (IMU), aby zwiększyć skuteczność wykrywania ubytków. Każde zdarzenie jest automatycznie tagowane precyzyjnymi współrzędnymi GPS.

## Specyfikacja sprzętowa (Hardware Stack)
* **Jednostka centralna:** Raspberry Pi 4 Model B
* **Pozycjonowanie:** Moduł Quectel LC76G (Interface: UART)
* **Kamera:** Waveshare RPi Camera (D) - szerokokątna
* **Pamięć systemowa:** Karta microSD Goodram 128GB
* **Czujniki ruchu:** IMU [Wpisz model, np. MPU6050] (Interface: I2C)

## 📂 Struktura repozytorium


## 🚀 Instalacja i uruchomienie
1. **Klonowanie repozytorium:**
   ```bash
   git clone [https://github.com/](https://github.com/)[TWÓJ-USER]/road-damage-detection-imu.git
   cd road-damage-detection-imu
