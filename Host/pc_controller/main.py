import serial

with serial.Serial("COM7", 115200, timeout=1) as port:
    frame = b"@00000001,FFFF,8000*54\n"
    port.write(frame)

    print("Wysłano ramkę:", len(frame), "bajtów")