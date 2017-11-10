import rospy

from pydarts.communication import (CommunicatorBase, SanitizationError,
        sanitized_input, MinLargerMaxError, ERROR, INFO_VISIT, INFO_FINISH,
        INFO_LEG, finishes
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
                self.print_output(ERROR, error=e)

                if isinstance(e, MinLargerMaxError):
                    # re-raise to avoid infinite loop
                    raise

    def print_output(self, message_type, **data):
        output = None

        if message_type == ERROR:
            output = str(data["error"])
        elif message_type == INFO_VISIT:
            player = data["player"]
            output = "{p.name} - {p.score_left} - {0}".format(
                    player.darts * "|", p=player)
        elif message_type == INFO_FINISH:
            player = data["player"]
            if player.score_left in finishes:
                output = " ".join(finishes[player.score_left])
        elif message_type == INFO_LEG:
            players = data["players"]
            output = ", ".join(
                    ("{}: {:1d}".format(p.name, p.nr_won_legs) for p in players)
                    )

        if output is not None:
            self._output_method(output)
