#%%
import serial

# Config (must match ESP32 UART settings)
ser = serial.Serial(
    port = "COM5",
    baudrate = 115200,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    timeout = 1
)

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("Distance: "):
        print(line)
    
    if line.startswith("$GNGGA"):
        print(line)
# %%
