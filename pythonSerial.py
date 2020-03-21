import time
import serial

ser = serial.Serial("/dev/ttyUSB1", 2400)
ser.setRTS(False)
ser.setDTR(False)
print "initaized finished"

c = 1
a = 0;
while c:
    ser.setRTS(False)
    ser.setDTR(True)

    time.sleep(0.5)

    ser.setRTS(True)
    ser.setDTR(False)
    time.sleep(0.5)
    a += 1
    print a

ser.close()
