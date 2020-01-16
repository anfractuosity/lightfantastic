#!/usr/bin/python3

#
# lightfantastic
# https://github.com/anfractuosity/lightfantastic
#

import numpy as np
import cv2
import time
import collections
from collections.abc import Sequence
from itertools import tee
import copy
from math import *
from ids import *
import math
import time
import pickle

WIDTH = 1280
HEIGHT = 720
FPS = 50
ZERO_COLOUR = (0, 0, 255)
ONE_COLOUR = (0, 255, 0)

old = -1
frameno = 0
seq = []
allleds = []

x = 10
y = 10
k = 10

idcount = 0
fps = 1
fcount = 1
lastf = None
uid = 0

allh = {}
frame = 0

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
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = lthres
    params.maxThreshold = 255
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    params.maxThreshold = 255
    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(frame)
    return keypoints

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

def crc(val):
    s = ""
    for b in val:
        s = s + str(b)
    return binascii.crc32(s.encode("ascii")) % 256

cap = cv2.VideoCapture("lights.MOV")

while True:
    timestamp = int(cap.get(cv2.CAP_PROP_POS_MSEC))

    fcount = fcount + 1
    if fcount > FPS:
        frameno = frameno + 1
        if timestamp > old + 1000:
            if frameno > 1:
                print("FPS ", frameno)
                fps = frameno
                frameno = 0
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

print("Analysing")
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
            ):
                old = allv[1]
                old.addtimestamp(i["timestamp"])
                old.x = ld.x
                old.y = ld.y
                additem(old)
                add = False
                break

        if add == True:
            additem(ld)

mp = IDs(0).manchester(IDs(0).preamble)
tups = {}
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

        bits = getbits(out)
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
