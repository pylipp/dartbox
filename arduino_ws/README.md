    mkvirtualenv --python=$(which python2) platformio
    pip install platformio
    wget -qO- https://raw.githubusercontent.com/platformio/platformio-core/develop/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null
    sudo usermod -a -G dialout philipp

    echo layout virtualenvwrapper platformio > .envrc
    direnv allow
    platformio init -b nanoatmega328
    platformio install lib 1634
    platformio run -t upload
