## DARTBOX

DIY-project helping to track dart scores

### Hardware

- Arduino nano
- Raspberry Pi 3 running Ubuntu Mate 16.04
- LCD
- 4x4 keypad

### Software

- ROS for communication
- [pydarts](http://github.com/pylipp/pydarts) package for game logic 

### Installation

    sudo apt-get install ros-kinetic-ros-base ros-kinetic-rosserial ros-kinetic-rosserial-arduino
    sudo rosdep init
    rosdep update

    # development tools, optional
    sudo apt-get install direnv python-virtualenv virtualenvwrapper

    git clone https://github.com/pylipp/dartbox --recursive

Please also see the `README`s in `catkin_ws/src` and `arduino_ws`.

### Run it!

    source catkin_ws/devel/setup.zsh
    roslaunch dartbox dartbox.launch

### TODOs

- [x] make LCD print incoming messages to different rows
- [x] clear '... starting ...' row
- enable clearing of chars with `#`
