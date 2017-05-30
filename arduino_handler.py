#!/usr/bin/python
import serial
import random


# Open /dev/ttyACM0 with baudrate 9600, and defaults of 8N1, no flow control
serial_connection = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

if __name__ == "__main__":
    while True:
        try:
            buf = serial_connection.readline()
            white = random.random()
            blue = random.random()
            red = random.random()

            # serial_connection.write("0,{},{},{}\n".format(red, white, blue))
            ary = buf.decode().split(',')
            if ary[0] != "0":
                print("status: {d[0]}, device: {d[1]}, code: {d[2]}, msg: {d[3]}".format(d=ary))
                continue
            # status, hum, temp, co2, water_temp
            print("status: {d[0]}, humidity: {d[1]}, temperature: {d[2]}, co2: {d[3]}, water_temp: {d[4]}".format(d=ary))
        # Short reads will show up as key errors since there are less per array
        except IndexError:
            print("Short read, received part of a message: {}".format(buf))
            continue
        # Occasionally, we get rotten bytes which couldn't decode
        except UnicodeDecodeError:
            print("Received weird bits, ignoring...")
            continue
        # Stop elegantly on ctrl+c
        except KeyboardInterrupt:
            serial_connection.close()
            break
