# lightfantastic

Display patterns on randomly arranged WS2812B lighting strips

* lightfantastic-calibrate - This command is used to pulse out an ID bit pattern for each of your WS2812 LEDs.  You will need
                 to record a video of these LEDs.

* lightfantastic-process - This command is used to process the video of the LED's calibration pattern

# How it works

See https://www.anfractuosity.com/projects/tree-lighting/ for information on the calibration process and playing 
simple 'videos' on the lights.

See https://www.anfractuosity.com/projects/painting-a-christmas-tree/ for information on the simple webapp to 
'paint a tree'.

# Installation of libraries on your Pi

* Install the following library https://github.com/jgarff/rpi_ws281x

* sudo apt-get install python3-opencv

* sudo apt-get install python3-numpy

* pip3 install . --user 

# Use

To generate the calibration pattern on the LEDs:

* lightfantastic-calibrate

To generate a pickled file containing the position of your LEDs, you can run:

* lightfantastic-process --movie lights.MOV --out lights.p

# To Do

* Add webapp code, to paint the LEDs

* Add code to display simple video on LEDs

# Commercial alternatives

* https://twinkly.com/ - seems to use a mobile phone for calibration, not sure what LEDs they use

