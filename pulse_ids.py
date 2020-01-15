#!/usr/bin/python3

#
# lightfantastic
# https://github.com/anfractuosity/lightfantastic
#

import time
from ids import *

from neopixel import *
import random

# LED strip configuration:
LED_COUNT = (60 * 5 * 3) - 7  # Number of LED pixels.
LED_PIN = 18  # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 5  # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 2  # 2     # Set to 0 for darkest and 255 for brightest
LED_INVERT = False  # True to invert the signal (when using NPN transistor level shift)
LEDs = []


def tree(strip, wait_ms=100):
    for i in range(strip.numPixels()):
        v = LEDs[i].get()
        if v == 0:
            col = Color(0, 0, 0)
        else:
            col = Color(255, 255, 255)
        strip.setPixelColor(i, col)
    strip.show()
    time.sleep(wait_ms / 1000.0)


# Main program logic follows
if __name__ == "__main__":

    for i in range(LED_COUNT):
        LEDs.append(IDs(i))

    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(
        LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS
    )

    # Intialize the library (must be called once before other functions).
    strip.begin()

    print("Press Ctrl-C to quit.")

    while True:
        tree(strip)
