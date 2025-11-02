import serial
import csv
from datetime import datetime

ser = serial.Serial('COM4', 115200)  # Cambia COM3 según tu puerto
filename = f"captura_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

with open(filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    while True:
        line = ser.readline().decode().strip()
        if line:
            values = line.split('\t')  # o .split(',') si usas coma
            writer.writerow(values)
            print(values)
