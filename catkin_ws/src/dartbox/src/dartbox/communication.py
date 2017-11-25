from pydarts.communication import (CommunicatorBase, INFO_VISIT, INFO_FINISH,
                                   INFO_LEG, finishes)


class RosCommunicator(CommunicatorBase):
    """Subclass providing communication using ROS via the '/get_input' service
    and the 'print_info'/'print_error' topics.
    """

    def print_info(self, message_type, **data):
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
