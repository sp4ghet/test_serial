#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import rospy
from std_msgs.msg import String
from roslib.message import get_message_class
from openag_brain.load_env_var_types import VariableInfo

def expand_unknown_status(status_code):
    return {
        "is_ok": False,
        "message": "Unknown status code {}".format(status_code)
    }

def ros_next(rate_hz):
    prev_time = rospy.get_time()
    timeout = 1 / rate_hz
    def closure():
        curr_time = rospy.get_time()
        if curr_time - prev_time > timeout:
            prev_time = curr_time
            return true
        else:
            return false

    return closure()

# Read the serial message string, and publish to the correct topics
def process_message(line):
    try:
        values = line.decode().split(',')
        pairs = {k: v for k, v in zip(sensor_csv_headers, values)}
        status_code = pairs["status"]
        # Expand status code to status dict
        status = (
            STATUS_CODE_INDEX.get(status_code) or
            expand_unknown_status(status_code)
        )

        # WARN/ERR format: "status_code, device_name, message"
        if not status["is_ok"]:
            error_device = values[1]
            error_message = values[2]
            message = "{}> {}: {}".format(
                status["message"],
                error_device,
                error_message)
            rospy.logerr(message)
            return message
        # status: OK

        # Zip values with the corresponding environmental variable
        variable_values = values[1:]
        pairs = zip(sensor_csv_headers, variable_values)

        for header, value in pairs:
            if VALID_SENSOR_VARIABLES[header].

        return {"ok": pairs}

    except IndexError:
        rospy.logwarn("Short read, received part of a message: {}".format(buf.decode()))
        serial_connection.close()
        serial_connection.open()
        continue
    # Occasionally, we get rotten bytes which couldn't decode
    except UnicodeDecodeError:
        rospy.logwarn("Received weird bits, ignoring: {}".format(buf))
        serial_connection.close()
        serial_connection.open()
        continue

if __name__ == '__main__':
    rospy.init_node('handle_arduino')

    # Read configurable params
    sensor_csv_headers = rospy.get_param("~sensor_csv_headers", (
        "status",
        "air_humidity",
        "air_temperature",
        "air_carbon_dioxide",
        "water_temperature",
        "water_level_low",
        "water_level_high",
        "water_potential_hydrogen",
        "water_electrical_conductivity"
    ))

    actuator_csv_headers = rospy.get_param("~actuator_csv_headers", (
        "status",
        "pump_1_nutrient_a_1",
        "pump_2_nutrient_b_1",
        "pump_3_ph_up_1",
        "pump_4_ph_down_1",
        "pump_5_water_1",
        "chiller_fan_1",
        "chiller_pump_1",
        "heater_core_2_1",
        "air_flush_1",
        "water_aeration_pump_1",
        "water_circulation_pump_1",
        "chamber_fan_1",
        "light_intensity_blue",
        "light_intensity_white",
        "light_intensity_red",
        "heater_core_1_1",
        "chiller_compressor_1"
    ))

    ENVIRONMENTAL_VARIABLES = frozenset(
        VariableInfo.from_dict(d)
        for d in rospy.get_param("/var_types/environment_variables").itervalues())

    VALID_SENSOR_VARIABLES = [v for v in ENVIRONMENTAL_VARIABLES if v.name in sensor_csv_headers]

    PUBLISHERS = {
        variable.name: rospy.Publisher(
            "{}/raw".format(variable.name),
            get_message_class(variable.type),
            queue_size=10)
        for variable in VALID_SENSOR_VARIABLES
    }

    ARDUINO_STATUS_PUBLISHER = rospy.Publisher(
        "/arduino_status",
        String(),
        queue_size=10)

    STATUS_CODE_INDEX = {
        "0": {
            "is_ok": True,
            "message": "OK"
        },
        "1": {
            "is_ok": False,
            "message": "WARN"
        },
        "2": {
            "is_ok": False,
            "message": "ERROR"
        }
    }

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/ttyACM0")
    publisher_rate_hz = rospy.get_param("~publisher_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 115200)

    timeout_s = 1 / publisher_rate_hz
    # Initialize the serial connection
    serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=timeout_s)

    publish_time = ros_next(publisher_rate_hz)
    while not rospy.is_shutdown():
        # Read before writing
        buf = serial_connection.readline()

        # Generate the message for the current state
        # status, pump1, pump2, pump3, pump4, pump5, chiller_fan, chiller_pump, heater_core2, air_flush, water_aeration, water_circulation, chamber_fan, blue, white, red, heater_core1, chiller_compressor
        message = "0,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            state["pump_1_nutrient_a_1"],
            state["pump_2_nutrient_b_1"],
            state["pump_3_ph_up_1"],
            state["pump_4_ph_down_1"],
            state["pump_5_water_1"],
            state["chiller_fan_1"],
            state["chiller_pump_1"],
            state["heater_core_2_1"],
            state["air_flush_1"],
            state["water_aeration_pump_1"],
            state["water_circulation_pump_1"],
            state["chamber_fan_1"],
            state["light_intensity_blue"],
            state["light_intensity_white"],
            state["light_intensity_red"],
            state["heater_core_1_1"],
            state["chiller_compressor_1"]
        ).encode('utf-8')
        serial_connection.write(message)
        serial_connection.flush()

        pairs_or_error = process_message(buf)
        if pairs_or_error.get("ok") is not None:
            pairs = pairs_or_error["ok"]
        else:
            error_message = pairs_or_error
            ARDUINO_STATUS_PUBLISHER.publish(error_message)

        if publish_time() and pairs is not None:
            ARDUINO_STATUS_PUBLISHER.publish("OK")
            for header, value in pairs:
                PUBLISHERS[header].publish(value)


    serial_connection.close()
