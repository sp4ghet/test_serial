#!/usr/bin/python
import serial
import random
import time

baud = 115200
# Open /dev/ttyACM0 with baudrate 9600, and defaults of 8N1
serial_connection = serial.Serial("/dev/ttyACM0", baud, timeout=1)

prev_time = time.time()
flush_period = 2000

if __name__ == "__main__":
    while True:

        # print("inWaiting: {}.".format(serial_connection.inWaiting()))
        now_time = time.time()
        # print(now_time - prev_time)
        prev_time = now_time
        try:
            buf = serial_connection.readline()

            pump_1_nutrient_a_1 = 0
            pump_2_nutrient_b_1 = 0
            pump_3_ph_up_1 = False
            pump_4_ph_down_1 = False
            pump_5_water_1 = False
            chiller_fan_1 = False
            chiller_pump_1 = False
            heater_core_2_1 = False
            air_flush_1 = 0
            water_aeration_pump_1 = False
            water_circulation_pump_1 = False
            chamber_fan_1 = True
            blue = random.random()
            white = random.random()
            red = random.random()
            heater_core_1_1 = False
            chiller_compressor_1 = False

            # status, pump1, pump2, pump3, pump4, pump5, chiller_fan, chiller_pump, heater_core2, air_flush, water_aeration, water_circulation, chamber_fan, blue, white, red, heater_core1, chiller_compressor
            message = "0,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                pump_1_nutrient_a_1,
                pump_2_nutrient_b_1,
                pump_3_ph_up_1,
                pump_4_ph_down_1,
                pump_5_water_1,
                chiller_fan_1,
                chiller_pump_1,
                heater_core_2_1,
                air_flush_1,
                water_aeration_pump_1,
                water_circulation_pump_1,
                chamber_fan_1,
                blue,
                white,
                red,
                heater_core_1_1,
                chiller_compressor_1
            ).encode('utf-8')

            serial_connection.write(message)
            serial_connection.flush()

            ary = buf.decode().split(',')
            if ary[0] != "0":
                print("status: {d[0]}, device: {d[1]}, code: {d[2]}".format(d=ary))
                continue
            # status, hum, temp, co2, water_temp, water_level_low, water_level_high, water_potential_hydrogen, water_electrical_conductivity
            print("status: {d[0]}, humidity: {d[1]}, temperature: {d[2]}, co2: {d[3]}, water_temp: {d[4]}, water_low: {d[5]}, water_high: {d[6]}, pH: {d[7]}, EC:{d[8]}".format(d=ary))
        # Short reads will show up as key errors since there are less per array
        except IndexError:
            print("Short read, received part of a message: {}".format(buf.decode()))
            serial_connection.close()
            serial_connection.open()
            continue
        # Occasionally, we get rotten bytes which couldn't decode
        except UnicodeDecodeError:
            print("Received weird bits, ignoring: {}".format(buf))
            serial_connection.close()
            serial_connection.open()
            continue
        # Stop elegantly on ctrl+c
        except KeyboardInterrupt:
            serial_connection.close()
            break
