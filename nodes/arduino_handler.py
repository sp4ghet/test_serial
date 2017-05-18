#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import rospy
from std_msgs.msg import Float64
import serial

STATUS_CODE_INDEX = {
    "0": {
        "is_ok": True,
        "message": "OK"
    },
    "1": {
        "is_ok": False,
        "message": "Failed to start device"
    }
}

def expand_unknown_status(status_code):
    return {
        "is_ok": False,
        "message": "Unknown status code {}".format(status_code)
    }

if __name__ == '__main__':
    rospy.init_node('handle_arduino')

    # Read configurable params
    csv_headers = rospy.get_param("~csv_headers", (
        "status",
        "air_humidity",
        "air_temperature"
    ))
    serial_port_id = rospy.get_param("~serial_port_id", "/dev/ttyACM0")
    loop_rate_hz = rospy.get_param("~loop_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 9600)
    serial_buffer_max = rospy.get_param("~serial_buffer_max", 128)

    # Open /dev/ttyACM0 with baudrate 9600, and defaults of 8N1, no flow control
    serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=(1/loop_rate_hz))
    r = rospy.Rate(loop_rate_hz)
    # Read up to 128 bytes with 500ms timeout
    while not rospy.is_shutdown():
        timeout_s = 1 / loop_rate_hz
        buf = serial_connection.readline(serial_buffer_max)
        values = buf.decode().split(',')

        # status, hum, temp
        status_code = values[0]

        # Zip values with the corresponding environmental variable
        variable_values = values[1:]
        pairs = zip(csv_headers, variable_values)

        # Expand status code to status dict
        status = (
            STATUS_CODE_INDEX.get(status_code) or
            expand_unknown_status(status_code)
        )

        if status["is_ok"]:
            rospy.loginfo(status["message"])
        else:
            rospy.logerr(status["message"])

        for environmental_variable, value in pairs:
	    parsed_value = float(value)
            pub = rospy.Publisher(environmental_variable, Float64, queue_size=10)
            pub.publish(parsed_value)
	r.sleep()
    serial.close()
