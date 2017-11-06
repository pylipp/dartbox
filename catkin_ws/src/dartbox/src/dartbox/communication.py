import rospy

from pydarts.communication import (CommunicatorBase, SanitizationError,
        sanitized_input, MinLargerMaxError,
        )


class RosCommunicator(CommunicatorBase):
    """Subclass providing communication using ROS via the '/get_input' service
    and the 'print_output' topic.
    """

    def get_input(self, prompt=None, **kwargs):
        """The user reply is a ServiceResponse object that the actual reply
        string has to be passed to sanitized_input().
        """
        while True:
            try:
                user_input = self._input_method(prompt or "").output
                sanitized_input_ = sanitized_input(user_input, **kwargs)
                rospy.loginfo("RosCommunicator: received {}".format(
                    sanitized_input_))
                return sanitized_input_
            except SanitizationError as e:
                self.print_output(str(e))

                if isinstance(e, MinLargerMaxError):
                    # re-raise to avoid infinite loop
                    raise

