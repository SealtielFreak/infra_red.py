# sony.py Encoder for IR remote control using synchronous code
# Sony SIRC protocol.

# Author: Peter Hinch
# Copyright Peter Hinch 2020 Released under the MIT license

import micropython
import infrared.transmission


class TransmissionSony(infrared.transmission.Transmission):
    def __init__(self, pin, bits, freq, verbose):
        super().__init__(pin, freq, 3 + bits * 2, 30, verbose)
        if bits not in (12, 15, 20):
            raise ValueError('bits must be 12, 15 or 20.')
        self.bits = bits

    def tx(self, addr, data, ext):
        self.append(2400, 600)
        bits = self.bits
        v = data & 0x7f
        if bits == 12:
            v |= (addr & 0x1f) << 7
        elif bits == 15:
            v |= (addr & 0xff) << 7
        else:
            v |= (addr & 0x1f) << 7
            v |= (ext & 0xff) << 12
        for _ in range(bits):
            self.append(1200 if v & 1 else 600, 600)
            v >>= 1


# Sony specifies 40KHz
class TransmissionSony12(TransmissionSony):
    valid = (0x1f, 0x7f, 0)  # Max addr, data, toggle

    def __init__(self, pin, freq=40000, verbose=False):
        super().__init__(pin, 12, freq, verbose)


class TransmissionSony15(TransmissionSony):
    valid = (0xff, 0x7f, 0)  # Max addr, data, toggle

    def __init__(self, pin, freq=40000, verbose=False):
        super().__init__(pin, 15, freq, verbose)


class TransmissionSony20(TransmissionSony):
    valid = (0x1f, 0x7f, 0xff)  # Max addr, data, toggle

    def __init__(self, pin, freq=40000, verbose=False):
        super().__init__(pin, 20, freq, verbose)
