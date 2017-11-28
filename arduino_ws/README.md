Setup `platformio` (I prefer installing it to a venv)

    mkvirtualenv --python=$(which python2) platformio
    pip install platformio

Allow `platformio` to access the serial port without using sudo

    wget -qO- https://raw.githubusercontent.com/platformio/platformio-core/develop/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null
    sudo usermod -a -G dialout philipp

Optionally: set up direnv to automatically activate the venv when cd'ing into the directory

    echo layout virtualenvwrapper platformio > .envrc
    direnv allow

Initialize the platformio project (adjust the board name if necessary)

    platformio init -b nanoatmega328

Install libraries (rosserial, Keypad, LiquidCrystal_I2C)

    platformio install lib 1634
    platformio install lib 165
    platformio install lib 576

Compile and upload
    
    cd arduino_ws
    platformio run -t upload
