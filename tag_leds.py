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
from math import *
from ids import *
import math
import time
import pickle

WIDTH = 1280
HEIGHT = 720
FPS = 50
RECORD = False

if RECORD:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
else:
    cap = cv2.VideoCapture("lights.MOV")

ZERO_COLOUR = (0, 0, 255)
ONE_COLOUR = (0, 255, 0)

from collections.abc import Sequence


class rect(object):
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


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

    def addtimestamp(self, tstamp):
        self.timestamp.append(tstamp)


def blobDet(val, frame):

    lthres = 140

    ret, frame = cv2.threshold(frame, lthres, 255, 0)

    # cv2.imshow("",frame)

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = lthres
    params.maxThreshold = 255

    # Filter by Area.
    params.filterByArea = False
    # params.minArea = 1500

    # Filter by Circularity
    params.filterByCircularity = False
    # params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    # params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = False
    # params.minInertiaRatio = 0.01
    params.maxThreshold = 255
    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(frame)

    return keypoints


old = -1
f = 0
fcount = 0
counter = 0
seq = []
cc = 0

allleds = []

x = 10
y = 10
k = 10
idcount = 0
fps = 1


if RECORD:
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    out = cv2.VideoWriter("output.avi", fourcc, float(FPS), (WIDTH, HEIGHT))


fcount = 1

lastf = None

while True:

    if RECORD:
        ret, frame = cap.read()
        out.write(frame)
        fcount = fcount + 1
        if fcount == FPS * 60:
            out.release()
            break
        continue

    timestamp = int(cap.get(cv2.CAP_PROP_POS_MSEC))
    # print(timestamp)

    fcount = fcount + 1
    if fcount > FPS:
        f = f + 1
        if timestamp > old + 1000:
            if f > 1:
                print("FPS ", f)

                fps = f
                f = 0
                old = timestamp

    ret, frame = cap.read()

    if type(frame) != np.ndarray:
        break
    else:
        lastf = frame

    try:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except cv2.error:
        break

    z = blobDet(0, frame)
    newz = z

    # print("BLOBS",len(newz))

    im = cv2.drawKeypoints(
        frame,
        newz,
        np.array([]),
        ZERO_COLOUR,
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    cv2.imshow("", im)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    cur = []

    if fps > 20:
        for k in newz:
            x = k.pt[0]
            y = k.pt[1]
            cur.append((x, y, k.size))

        allleds.append({"timestamp": timestamp, "points": cur})


if RECORD:
    quit()

print("Analysing")

uid = 0
vall = {}
added = 0
allh = {}


def getKey(item):
    return item[0]


def additem(led):
    allh[led.idv] = led


def nearesti(search, dist):

    found = []

    for idv in allh:

        ldv = allh[idv]
        dst = math.sqrt((search.x - ldv.x) ** 2 + (search.y - ldv.y) ** 2)

        if dst < dist:
            found.append((dst, ldv))

    return sorted(found, key=getKey)


frame = 0
for i in allleds:
    frame = frame + 1

    if frame % 10:
        print("Complete ", (frame / len(allleds)) * 100, " --- len ", len(allh.keys()))

    for l in i["points"]:
        uid = uid + 1
        ld = LED(l[0], l[1], i["timestamp"], uid)

        nearest = nearesti(ld, 8)

        if len(nearest) == 0:
            additem(ld)
            continue

        add = True
        for allv in nearest:

            if (
                not allv[1].timestamp[-1] == i["timestamp"]
            ):  # and i["timestamp"] - allv[1].timestamp[-1] < 450 :

                old = allv[1]

                old.addtimestamp(i["timestamp"])
                old.x = ld.x
                old.y = ld.y
                additem(old)
                add = False
                break

        if add == True:
            additem(ld)

from itertools import tee


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


def mancdec(arr):
    out = []
    for i in range(0, len(arr), 2):
        if arr[i : i + 2] == [0, 1]:
            out.append(0)
        elif arr[i : i + 2] == [1, 0]:
            out.append(1)
    return out


def getbits(p2):

    nout = 0
    bits = []
    for v in p2:

        if v < 35:
            nout = nout + 1
        else:
            # on = nout *FPS
            if nout <= 16:
                bits.append(1)
            else:
                bits.append(1)
                bits.append(1)

            off = v

            if off <= 300:
                bits.append(0)
            else:
                bits.append(0)
                bits.append(0)
            nout = 1
    return bits


mp = IDs(0).manchester(IDs(0).preamble)
tups = {}


def crc(val):
    s = ""
    for b in val:
        s = s + str(b)
    return binascii.crc32(s.encode("ascii")) % 256


idd = 0


for key in allh:
    t = allh[key].timestamp
    led = allh[key]

    if len(t) > 20:

        t2 = [t[i] for i in range(1, len(t))]

        old = t2[0]
        out = []

        for v in t2[1:]:
            out.append(v - old)
            old = v

        print("key ", key)
        print(out)

        bits = getbits(out)

        print(bits)
        print("---------------------")
        marker = 0
        found = False

        most = {}
        for x in window(bits, 68):
            n2 = [z for z in x]
            pre = n2[:32]

            if pre == mp:
                m = mancdec(n2)
                check = binary(m[-8:])
                bbb = binary(m[16 : 16 + 10])

                if crc(m[0 : 16 + 10]) == check:

                    try:
                        most[bbb] = most[bbb] + 1
                    except KeyError:
                        most[bbb] = 1

                    print("woooo", binary(m[-10:]), " x: ", led.x, " y: ", led.y)
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
            tups[idv] = tup

        marker = marker + 1

        if found:
            print("--------------------------")


ct = 0
for k in tups:
    ct += 1
    idv = tups[k]
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(
        lastf,
        str(k),
        (int(idv[0]), int(idv[1])),
        font,
        0.3,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )

print("found ", ct)
pickle.dump(tups, open("save.p", "wb"))

cv2.imshow("Tagged image", lastf)
cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()
