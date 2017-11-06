    echo -e "source /opt/ros/kinetic/setup.zsh\n[ -e devel/setup.zsh ] && source devel/setup.zsh\nPYTHONPATH=./../pydarts:\$PYTHONPATH" > .envrc
    direnv allow

    catkin_make
    source devel/setup.zsh

    roslaunch dartbox dartbox.launch
    # in separate shell, obviously
    rosrun dartbox dartbox
