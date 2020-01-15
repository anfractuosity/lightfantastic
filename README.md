# lightfantastic

Display patterns on randomly arranged WS2812B lighting strips

* pulse_ids.py - This file is used to pulse out an ID bit pattern for each of your WS2812 LEDs.  You will need
                 to record a video of the LEDs.

* tag_leds.py  - This file is used to process the calibration video and save the position of LEDs along with their ID to a pickled file.

# How it works

See https://www.anfractuosity.com/projects/tree-lighting/ for information on the calibration process and playing 
simple 'videos' on the lights.

See https://www.anfractuosity.com/projects/painting-a-christmas-tree/ for information on the simple webapp to 
'paint a tree'.

# Installation of libraries on your Pi

* Install the following library https://github.com/jgarff/rpi_ws281x

* sudo apt-get install python3-opencv

* sudo apt-get install python3-numpy

# Commercial alternatives

* https://twinkly.com/ - seems to use a mobile phone for calibration, not sure what LEDs they use

