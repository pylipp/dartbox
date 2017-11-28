import time

from pydarts.communication import (CommunicatorBase, INFO_VISIT, INFO_FINISH,
                                   INFO_LEG, finishes, INPUT_ANOTHER_SESSION)


class RosCommunicator(CommunicatorBase):
    """Subclass providing communication using ROS via the '/get_input' service
    and the 'print_info'/'print_error' topics.
    """

    def __init__(self, input_method, output_info_method,
                 output_error_method=None):
        super(RosCommunicator, self).__init__(
                input_method, output_info_method, output_error_method)
        self._input_prompts[INPUT_ANOTHER_SESSION] = {
            "prompt": "Again? ",
            "kwargs": {"choices": "ABC"},  # our keypad keys...
        }

    def get_input(self, input_mode, *format_args):
        """Custom implementation mapping the keypad options to the y/n/q choices
        when querying for another session.
        """

        if input_mode == INPUT_ANOTHER_SESSION:
            self._output_info_method("%A=yes, B=no, C=quit")
            # minimal delay, otherwise service request gets lost
            time.sleep(0.1)

        user_input = super(RosCommunicator, self).get_input(input_mode,
                *format_args)

        if input_mode == INPUT_ANOTHER_SESSION:
            user_input = {"A": "y", "B": "n", "C": "q"}[user_input]

        return user_input

    def print_info(self, message_type, **data):
        """Custom implementation with shorter messages that fit the 20 character
        width of the LCD. Special chars are used to indicate the LCD row to
        write the message to.
        """

        output = None

        if message_type == INFO_VISIT:
            player = data["player"]
            output = "%" + "{p.score_left} - {0}".format(
                    player.darts * "|", p=player)
        elif message_type == INFO_FINISH:
            player = data["player"]
            score_left = str(player.score_left)
            if score_left in finishes:
                # output only the first option
                output = "?" + " ".join(finishes[score_left][0])
        elif message_type == INFO_LEG:
            players = data["players"]
            output = "!" + ", ".join(
                    ("{}: {:1d}".format(p.name, p.nr_won_legs) for p in players)
                    )

        if output is not None:
            self._output_info_method(output)

    def print_error(self, **data):
        output = "!" + str(data["error"])
        self._output_error_method(output)
