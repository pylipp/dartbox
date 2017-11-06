#! /usr/bin/env python

"""Server running on the Raspi instead of the arduino."""

from rosserial_arduino.srv import Test as GetInput
from rosserial_arduino.srv import TestResponse as GetInputResponse
import rospy


def handle_request(request):
    return GetInputResponse(raw_input(request.input))

if __name__ == "__main__":
    rospy.init_node("get_input_server")
    rospy.Service("/get_input", GetInput, handle_request)
    rospy.spin()
