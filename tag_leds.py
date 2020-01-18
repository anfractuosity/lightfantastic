#!/usr/bin/python3

#
# lightfantastic
# https://github.com/anfractuosity/lightfantastic
#

import numpy as np
import cv2
import time
import collections
import copy
import math
import time
import pickle
from operator import itemgetter
from collections import defaultdict
from collections.abc import Sequence
from itertools import tee
from math import *
from ids import *
from spatial import *

FPS = 50
ZERO_COLOUR = (0, 0, 255)
TIME_THRESHOLD = 300 # milliseconds
BRUTE = False

# LED representation
class LED(Sequence):
    def __init__(self, x, y, tim, idv):
        self.x = x
        self.y = y
        self.idv = idv
        self.timestamp = collections.deque(maxlen=100000)
        self.timestamp.append(tim)

    def __getitem__(self, i):
        return [self.x, self.y][i]

    def __len__(self):
        return 2

    def __eq__(self, other):
        if other == None:
            return False

        if self == None:
            return False

        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return self.idv

    def addtimestamp(self, tstamp):
        self.timestamp.append(tstamp)

# Detect blobs in frame
def blobdetect(frame):
    lthres = 140
    ret, frame = cv2.threshold(frame, lthres, 255, 0)
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = lthres
    params.maxThreshold = 255
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByColor = True
    params.blobColor = 255
    detector = cv2.SimpleBlobDetector_create(params)
    return detector.detect(frame)

# Find nearest potential LED
def nearesti(allh,search, dist, spatial,rect):

    found = []
    if BRUTE:
        for idv in allh:
            ldv = allh[idv]
            dst = math.sqrt((search.x - ldv.x) ** 2 + (search.y - ldv.y) ** 2)
            if dst < dist:
                found.append((dst, ldv))
        return sorted(found, key=itemgetter(0))
    else:
        for idv in spatial.potential_collisions(rect,search):
            found.append((None,idv))
            break
        return found

# Sliding window
def window(iterable, size):
    iters = tee(iterable, size)
    for i in range(1, size):
        for each in iters[i:]:
            next(each, None)
    return zip(*iters)

def binary(arr):
    m = 0
    s = 0
    for o in arr:
        s = s + (o * (2 ** m))
        m = m + 1
    return s

# Manchester decode
def mancdec(arr):
    out = []
    for i in range(0, len(arr), 2):
        if arr[i : i + 2] == [0, 1]:
            out.append(0)
        elif arr[i : i + 2] == [1, 0]:
            out.append(1)
    return out

# Extract bits from frame differences, this then needs to be decoded
# using the mancdec function
def getbits(framediffs):

    nout = 0
    bits = []
    f = ((1 / FPS) * 2) * 1000

    for framediff in framediffs:
        if framediff < f:
            nout = nout + 1
        else:
            if nout*((1/FPS)*1000) <= TIME_THRESHOLD:
                bits.append(1)
            else:
                bits.extend([1, 1])

            if framediff <= TIME_THRESHOLD:
                bits.append(0)
            else:
                bits.extend([0, 0])

            nout = 1

    return bits


# Calculate CRC checksum
def crc(val):
    s = ""
    for b in val:
        s = s + str(b)
    return binascii.crc32(s.encode("ascii")) % 256


# Get blobs from video frames
def getblobs(videofile):
    spatial = SpatialHash()
    cap = cv2.VideoCapture(videofile)
    potentialleds = {}
    blobframes = []
    fcount = 0
    fps = 0
    frameno = 0
    old = -1
    uid = 0

    while True:

        # Calculate framerate
        timestamp = int(cap.get(cv2.CAP_PROP_POS_MSEC))
        fcount = fcount + 1
        if fcount > FPS:
            frameno = frameno + 1
            if timestamp > old + 1000 and frameno > 1:
                print("FPS ", frameno)
                fps = frameno
                frameno = 0
                old = timestamp

        ret, frame = cap.read()
        if type(frame) != np.ndarray:
            break

        # Find blobs, and draw them on image
        lastf = frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blobs = blobdetect(frame)
        im = cv2.drawKeypoints(
            frame,
            blobs,
            np.array([]),
            ZERO_COLOUR,
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        cv2.imshow("", im)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        cur = []
        if fps > 20:

            for k in blobs:
                x = k.pt[0]
                y = k.pt[1]
                cur.append((x, y, k.size))

            # Process blobs in current frame
            for l in cur:

                uid = uid + 1
                search_led = LED(l[0], l[1], timestamp, uid)
                rect = Rect(x1=l[0]-1,y1=l[1]-1,x2=l[0]+1,y2=l[1]+1)
                nearest = nearesti(potentialleds, search_led, 8,spatial,rect)

                # If no previous potential LED, add this LED to potential LEDs
                if len(nearest) == 0:
                    potentialleds[search_led.idv] = search_led
                    spatial.add_rect(rect,search_led)
                    continue

                # If we found a potential previous LED, add this timestamp to it
                add = True
                for allv in nearest:
                    if allv[1].timestamp[-1] != timestamp:
                        o = allv[1]
                        o.addtimestamp(timestamp)
                        o.x = search_led.x
                        o.y = search_led.y
                        potentialleds[o.idv] = o
                        add = False
                        break

                if add:
                    potentialleds[search_led.idv] = search_led
                    spatial.add_rect(rect,search_led)

    cap.release()
    return lastf, potentialleds

if __name__ == '__main__':

    print("Analysing")

    frame = 0
    mp = IDs(0).manchester(IDs(0).preamble)
    tagged_leds = {}
    idd = 0

    lastf, potentialleds = getblobs("lights.MOV")

    for key in potentialleds:
        t = potentialleds[key].timestamp
        led = potentialleds[key]

        if len(t) > 20:
            t2 = [t[i] for i in range(1, len(t))]
            old = t2[0]
            out = []

            for v in t2[1:]:
                out.append(v - old)
                old = v

            bits = getbits(out)
            found = False

            most = defaultdict(int)
            for x in window(bits, 68):
                n2 = [z for z in x]
                pre = n2[:32]

                if pre == mp:
                    m = mancdec(n2)
                    check = binary(m[-8:])
                    bbb = binary(m[16: 16 + 10])

                    if crc(m[0: 16 + 10]) == check:
                        most[bbb] = most[bbb] + 1
                        found = True
                        break

            last = 0
            idv = -1
            tup = (led.x, led.y)
            for k in most:
                if most[k] > last:
                    idv = k
                    last = most[k]

            if found:
                tagged_leds[idv] = tup

    # Display tagged LEDs on last processed video frame
    numleds = 0
    for led_id in tagged_leds:
        numleds += 1
        led_pos = tagged_leds[led_id]
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            lastf,
            str(led_id),
            (int(led_pos[0]), int(led_pos[1])),
            font,
            0.3,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    print("found ", numleds)

    pickle.dump(tagged_leds, open("save.p", "wb"))
    cv2.imshow("Tagged image", lastf)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
