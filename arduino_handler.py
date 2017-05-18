from periphery import Serial


# Open /dev/ttyACM0 with baudrate 9600, and defaults of 8N1, no flow control
serial = Serial("/dev/ttyACM0", 9600)

# Read up to 128 bytes with 500ms timeout
while(true):
    try:
        buf = serial.read(128, 0.5)
        ary = buf.decode().split(',')
        # status, hum, temp
        print("status: {d[0]}, humidity: {d[1]}, temperature: {d[2]}".format(d=ary))
    except KeyboardInterrupt:
        serial.close()
        break
