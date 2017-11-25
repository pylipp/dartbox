#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from rosserial_arduino.srv import Test as GetInput

from .communication import RosCommunicator
from pydarts.game import Game


class DartNode(object):

    def __init__(self):
        print_info_publisher = rospy.Publisher("/print_info", String,
                queue_size=None)

        rospy.wait_for_service("/get_input")
        get_input_proxy = rospy.ServiceProxy("/get_input", GetInput)

        def input_method(prompt):
            """Wrapper function to return the `output` attribute of the response
            object returned from the service proxy.
            """
            output = get_input_proxy(prompt).output
            rospy.loginfo("RosCommunicator: received {}".format(output))
            return output

        communicator = RosCommunicator(
                input_method,
                print_info_publisher.publish,
                )

        self.game = Game(communicator)
