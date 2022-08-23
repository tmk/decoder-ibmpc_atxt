##
## IBMPC AT keyboard/mouse
##
## This is based on libsigrokdecode/decoders/ps2/
## Copyright (C) 2022 Jun Wako <wakojun@gmail.com>
##

##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Daniel Schulte <trilader@schroedingers-bit.net>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from collections import namedtuple

class Ann:
    BIT, START, STOP, PARITY_OK, PARITY_ERR, DATA, COMMAND, STATE, ERROR = range(9)

Bit = namedtuple('Bit', 'val ss es')

class Decoder(srd.Decoder):
    api_version = 3
    id = 'ibmpc_at'
    name = 'IBMPC AT'
    longname = 'IBMPC AT'
    desc = 'IBMPC AT keyboard/mouse interface.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['PC']
    channels = (
        {'id': 'clk', 'name': 'Clock', 'desc': 'Clock line'},
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    annotations = (
        ('bit', 'Bit'),
        ('start-bit', 'Start bit'),
        ('stop-bit', 'Stop bit'),
        ('parity-ok', 'Parity OK'),
        ('parity-err', 'Parity Error'),
        ('data-bit', 'Data bit'),
        ('command-bit', 'Command bit'),
        ('state', 'state'),
        ('error', 'error'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('fields', 'Fields', (1, 2, 3, 4, 5, 6, 7, 8)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.bits = []
        self.bitcount = 0
        # state: IDLE[H, H], INHIBIT[L, H], REQUEST[L, L], HOST_TO_DEVICE, DEVICE_TO_HOST, TRANSIENT
        self.state = 'IDLE'

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putb(self, bit, ann_idx):
        b = self.bits[bit]
        self.put(b.ss, b.es, self.out_ann, [ann_idx, [str(b.val)]])

    def putx(self, bit, ann):
        self.put(self.bits[bit].ss, self.bits[bit].es, self.out_ann, ann)

    def handle_bits(self, datapin):
        # Ignore non start condition bits (useful during keyboard init).
        if self.bitcount == 0 and datapin == 1:
            return

        # Store individual bits and their start/end samplenumbers.
        self.bits.append(Bit(datapin, self.samplenum, self.samplenum))

        # Fix up end sample numbers of the bits.
        if self.bitcount > 0:
            b = self.bits[self.bitcount - 1]
            self.bits[self.bitcount - 1] = Bit(b.val, b.ss, self.samplenum)

        # Annotation
        if self.bitcount == 1:
            self.putx(0, [Ann.START, ['Start bit', 'Start', 'S']])

        # bit0:7 and parity
        if self.bitcount > 1 and self.bitcount < 11:
                self.putb(self.bitcount - 1, Ann.BIT)

        if self.bitcount == 9:
            byte = 0
            for i in range(8):
                byte |= (self.bits[i + 1].val << i)

            if self.state == 'DEVICE_TO_HOST':
                self.put(self.bits[1].ss, self.bits[8].es, self.out_ann, [Ann.DATA,
                    ['D->H: %02X' % byte, 'D: %02X' % byte, '%02X' % byte]])
            else:
                self.put(self.bits[1].ss, self.bits[8].es, self.out_ann, [Ann.COMMAND,
                    ['H->D: %02X' % byte, 'H: %02X' % byte, '%02X' % byte]])

        if self.bitcount == 10:
            byte = 0
            for i in range(8):
                byte |= (self.bits[i + 1].val << i)

            parity_ok = (bin(byte).count('1') + self.bits[9].val) % 2 == 1
            if parity_ok:
                self.putx(9, [Ann.PARITY_OK, ['Parity OK', 'Par OK', 'P']])
            else:
                self.putx(9, [Ann.PARITY_ERR, ['Parity error', 'Par err', 'PE']])

            if self.state == 'DEVICE_TO_HOST':
                # stop bit width determined
                width = self.bits[2].es - self.bits[1].es
                b = self.bits[-1]
                self.bits[-1] = Bit(b.val, b.ss, b.es + width)
                #self.wait({'skip': width})

                self.putx(10, [Ann.STOP, ['Stop bit', 'Stop', 'St', 'T']])
                self.bits, self.bitcount = [], 0
                return

        if self.bitcount == 11:
            self.putx(10, [Ann.STOP, ['Stop bit', 'Stop', 'St', 'T']])

        # Find all 11 bits. Start + 8 data + odd parity + stop.
        if self.bitcount < 11:
            self.bitcount += 1
            return

        self.bits, self.bitcount = [], 0

    def decode(self):
        while True:
            # Sample data bits on the falling clock edge (assume the device
            # is the transmitter). Expect the data byte transmission to end
            # at the rising clock edge. Cope with the absence of host activity.
            if (self.state == 'IDLE'):
                # clock:H, data:H
                clock_pin, data_pin = self.wait([{0: 'f'}, {1: 'f'}])
                if (clock_pin == 0 and data_pin == 1):
                    self.state = 'INHIBIT'
                elif (clock_pin == 1 and data_pin == 0):
                    # start bit
                    self.handle_bits(data_pin)
                    self.state = 'DEVICE_TO_HOST'
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = 'TRANSIENT'
                elif (clock_pin == 1 and data_pin == 1):
                    self.state = 'IDLE'

            elif (self.state == 'DEVICE_TO_HOST'):
                # clock:L, data:X
                _, data_pin = self.wait({0: 'r'})
                # clock:H, data:X
                if bool(self.samplerate) and self.bitcount > 0:
                    # timeout
                    if 100 < (self.samplenum - self.bits[-1].es) / (self.samplerate / 1000000):
                        self.put(self.bits[-1].es, self.samplenum, self.out_ann, [Ann.ERROR, ['Timeout Error/Inhibit', 'TOE',  'E']])
                        self.bits, self.bitcount = [], 0
                        if data_pin == 1:
                            self.state = 'IDLE'
                        else:
                            # Cancel receiving and Host request to send
                            self.state = 'HOST_TO_DEVICE'
                            # start bit
                            self.handle_bits(data_pin)
                        continue

                # read data at falling edge
                clock_pin, data_pin = self.wait({0: 'f'})
                self.handle_bits(data_pin)

                # end
                # clock:L, data:X
                if self.bitcount == 0:
                    self.state = 'TRANSIENT'

            elif (self.state == 'INHIBIT'):
                # clock:L, data:H
                ss = self.samplenum
                clock_pin, data_pin = self.wait([{0: 'r'}, {1: 'e'}])

                self.put(ss, self.samplenum, self.out_ann, [Ann.STATE, ['Inhibit', 'INH',  'I']])

                if (clock_pin == 1 and data_pin == 1):
                    self.state = 'IDLE'
                elif (clock_pin == 1 and data_pin == 0):
                    self.state = 'HOST_TO_DEVICE'
                    # start bit
                    self.handle_bits(data_pin)
                elif (clock_pin == 0 and data_pin == 1):
                    self.state = 'INHIBIT'
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = 'REQUEST'

            elif self.state == 'REQUEST':
                # clock:L, data:L
                ss = self.samplenum
                clock_pin, data_pin = self.wait([{0: 'e'}, {1: 'e'}])

                self.put(ss, self.samplenum, self.out_ann, [Ann.STATE, ['Request', 'REQ',  'R']])

                if (clock_pin == 1 and data_pin == 1):
                    self.state = 'IDLE'
                elif (clock_pin == 1 and data_pin == 0):
                    self.state = 'HOST_TO_DEVICE'
                    # start bit
                    self.handle_bits(data_pin)
                elif (clock_pin == 0 and data_pin == 1):
                    self.state = 'INHIBIT'
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = 'REQUEST'

            elif (self.state == 'HOST_TO_DEVICE'):
                _, data_pin = self.wait({0: 'f'})
                if self.bitcount == 1 + 8 + 1 + 1:
                    _, data_pin = self.wait({0: 'r'})
                    self.handle_bits(data_pin)
                    # TODO: check signal
                    self.state = 'TRANSIENT'
                    continue
                elif bool(self.samplerate) and self.bitcount > 1:
                    # timeout
                    # start bit(bitount == 1) can be long and is not checked
                    if 100 < (self.samplenum - self.bits[-1].es) / (self.samplerate / 1000000):
                        self.put(self.bits[-1].es, self.samplenum, self.out_ann, [Ann.ERROR, ['H->D Timeout Error', 'TOE',  'E']])
                        self.bits, self.bitcount = [], 0
                        if data_pin == 1:
                            # clock:L, data:H
                            self.state = 'INHIBIT'
                        else:
                            # clock:L, data:L
                            self.state = 'REQUEST'
                        continue

                # read data at rising edge
                _, data_pin = self.wait({0: 'r'})
                self.handle_bits(data_pin)

            elif self.state == 'TRANSIENT':
                clock, data = self.wait([{0: 'e'}, {1: 'e'}])

                # wait for stable state
                if (clock == 1 and data == 1):
                    self.state = 'IDLE'
                elif (clock == 1 and data == 0):
                    self.state = 'TRANSIENT'
                elif (clock == 0 and data == 1):
                    self.state = 'INHIBIT'
                elif (clock == 0 and data == 0):
                    self.state = 'TRANSIENT'
