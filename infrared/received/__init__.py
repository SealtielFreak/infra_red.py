# ir_rx __init__.py Decoder for IR remote control using synchronous code
# IR_RX abstract base class for IR receivers.

# Author: Peter Hinch
# Copyright Peter Hinch 2020-2021 Released under the MIT license

import micropython

import abc
import array
import utime
import machine

# Save RAM
# from micropython import alloc_emergency_exception_buf
# alloc_emergency_exception_buf(100)


# On 1st edge start a block timer. While the timer is running, record the time
# of each edge. When the timer times out decode the data. Duration must exceed
# the worst case block transmission time, but be less than the interval between
# a block start and a repeat code start (~108ms depending on protocol)

class Received(abc.ABC):
    # Result/error codes
    # Repeat button code
    REPEAT = micropython.const(-1)
    # Error codes
    BADSTART = micropython.const(-2)
    BADBLOCK = micropython.const(-3)
    BADREP = micropython.const(-4)
    OVERRUN = micropython.const(-5)
    BADDATA = micropython.const(-6)
    BADADDR = micropython.const(-7)

    def __init__(self, pin, nedges, tblock, callback, *args):  # Optional args for callback
        self._times = array.array('i', [0 for _ in range(nedges + 1)])  # +1 for overrun
        self._pin = pin
        self._nedges = nedges
        self._tblock = tblock
        self._errf = lambda _: None

        self.callback = callback
        self.args = args
        self.verbose = False

        self.edge = 0
        self.tim = machine.Timer(-1)  # Sofware timer
        self.cb = self.decode

        pin.irq(handler=self.__cb_pin, trigger=(machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING))

    # Pin interrupt. Save time of each edge for later decode.
    def __cb_pin(self, line) -> None:
        t = utime.ticks_us()
        # On overrun ignore pulses until software timer times out
        if self.edge <= self._nedges:  # Allow 1 extra pulse to record overrun
            if not self.edge:  # First edge received
                self.tim.init(period=self._tblock, mode=machine.Timer.ONE_SHOT, callback=self.cb)
            self._times[self.edge] = t
            self.edge += 1

    def do_callback(self, cmd, addr, ext, thresh=0) -> None:
        self.edge = 0
        if cmd >= thresh:
            self.callback(cmd, addr, ext, *self.args)
        else:
            self._errf(cmd)

    def error_function(self, func) -> None:
        self._errf = func

    def close(self) -> None:
        self._pin.irq(handler=None)
        self.tim.deinit()

    @abc.abstractmethod
    def decode(self, _) -> None:
        ...
