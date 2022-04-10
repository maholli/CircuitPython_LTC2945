"""
`ltc2945`
====================================================

CircuitPython driver for Analog Devices I2C Power Monitor LTC2945

* Author(s): Max Holliday

Implementation Notes
--------------------
    - device addressing see Table 1 in DS
    | ADDR  |   ADR1   |   ADR0   |
    | :---: | :------: | :------: |
    | 0x67  |   High   |   Low    |
    | 0x68  |   Float  |   High   |
    | 0x69  |   High   |   High   |
    | 0x6A  |   Float  |   Float  |
    | 0x6B  |   Float  |   Low    |
    | 0x6C  |   Low    |   High   |
    | 0x6D  |   High   |   Float  |
    | 0x6E  |   Low    |   Float  |
    | 0x6F  |   Low    |   Low    |

https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/libraries/LTC2945/LTC2945.h
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit


_CTRL = const(0x00)
_STATUS = const(0x00)
_PWR = const(0x05)
_DSNSE = const(0x14)
_VIN = const(0x1E)


class LTC2945:
    _BUFFER = bytearray(4)
    DSenseRes=25e-6 # 25uV resolution
    VinRes   =25e-3 # 25mV resolution

    shutdown = RWBit(_CTRL,1)

    def __init__(self, i2c_bus, addr=0x6A, rsense=0.02):
        if addr not in (0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F):
            raise Exception(f'Invalid I2C address for LTC2945 device: {addr}')
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)
        self.sense_resistor=rsense

        # print control register
        if self.read_u8(_CTRL) not in (0x05,0x07):
            raise Exception(f'Invalid POR control register value. Should be 0x5, returned {hex(self.read_u8(_CTRL))}')

    def read_u8(self, address):
        with self.i2c_device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_start=1, in_end=2)
        return self._BUFFER[1]

    def write_u8(self, address, val):
        with self.i2c_device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    def read_bytes(self, address, count):
        with self.i2c_device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_end=count)
        return self._BUFFER[:count]

    def read_vin(self):
        self.read_bytes(address=_VIN,count=2)
        _v = ((self._BUFFER[0]<<8) | self._BUFFER[1]) >> 4 # reduce from 16 bits to 12
        _v *= self.VinRes
        return _v

    def read_current(self):
        self.read_bytes(address=_DSNSE,count=2)
        _i = ((self._BUFFER[0]<<8) | self._BUFFER[1]) >> 4 # reduce from 16 bits to 12
        _i = _i * self.DSenseRes / self.sense_resistor
        return _i

    def read_power(self):
        self.read_bytes(address=_PWR,count=3)
        _p = (self._BUFFER[0]<<8) | self._BUFFER[1]
        _p = (_p<<8) | self._BUFFER[2]
        _p *= self.DSenseRes
        return _p




