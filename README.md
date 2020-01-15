# lightfantastic

Display patterns on randomly arranged WS2812B lighting strips

* pulse_ids.py - This file is used to pulse out an ID bit pattern for each of your WS2812 LEDs.  You will need
                 to record a video of the LEDs.

* tag_leds.py  - This file is used to process the calibration video and save the position of LEDs along with their ID to a pickled file.

# Installation of libraries on your Pi

* Install the following library https://github.com/jgarff/rpi_ws281x

* sudo apt-get install python3-opencv

* sudo apt-get install python3-numpy
