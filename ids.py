#
# lightfantastic
# https://github.com/anfractuosity/lightfantastic
#

import binascii

class IDs:
    def manchester(self, val):
        out = []

        for bit in val:
            if bit == 0:
                out.append(0)
                out.append(1)
            else:
                out.append(1)
                out.append(0)
        return out

    def get(self):
        v = self.idbitsO[self.counter]

        self.counter = self.counter + 1

        if self.counter == len(self.idbitsO):
            self.counter = 0

        return v

    def bitlist(self, val):

        tmp = []
        if val == 0:
            tmp.append(0)

        while val:
            if val & 1 == 1:
                tmp.append(1)
            else:
                tmp.append(0)

            val //= 2

        while not len(tmp) % 8 == 0:
            tmp.append(0)

        return tmp

    def crc(self, val):
        s = ""
        for b in val:
            s = s + str(b)

        return self.bitlist(binascii.crc32(s.encode("ascii")) % 256)

    def __init__(self, idv):
        self.idv = idv
        self.idbits = []
        self.idbitsO = []
        self.counter = 0
        self.preamble = [1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0]

        tmp = []
        if idv == 0:
            self.idbits.append(0)
        while idv:

            if idv & 1 == 1:
                self.idbits.append(1)
            else:
                self.idbits.append(0)

            idv = int(idv / 2)

        while not len(self.idbits) % 10 == 0:
            self.idbits.append(0)

        check = self.crc(self.preamble + self.idbits)
        self.idbitsO = self.manchester(self.preamble + self.idbits + check)
