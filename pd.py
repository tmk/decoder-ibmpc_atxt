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
from enum import IntEnum

class Ann(IntEnum):
    BIT = 0
    START = 1
    STOP = 2
    PARITY_OK = 3
    PARITY_ERR = 4
    DATA = 5
    COMMAND = 6
    STATE = 7
    ERROR = 8

class State(IntEnum):
    IDLE = 0
    INHIBIT = 1
    REQUEST = 2
    HOST_TO_DEVICE = 3
    DEVICE_TO_HOST = 4
    TRANSIENT = 5

Bit = namedtuple('Bit', 'val ss es')

class Decoder(srd.Decoder):
    api_version = 3
    id = 'ibmpc_atxt'
    name = 'AT/XT'
    longname = 'IBM PC AT/XT'
    desc = 'IBM PC AT/XT keyboard/mouse interface.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['PC']
    channels = (
        {'id': 'clk', 'name': 'Clock', 'desc': 'Clock line'},
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    options = (
            {'id': 'protocol', 'desc': 'Signal protocol', 'default': 'AT', 'values': ('AT', 'XT')},
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
        self.state = State.IDLE

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
        # bits[]    AT:D->H         AT:H->D         XT:D->H
        # -----------------------------------------------------
        # 0         start bit(0)    start bit(0)    start bit(1)
        # 1         bit0            bit0            bit0
        # 8         bit7            bit7            bit7
        # 9         parity bit      parity bit      end
        # 10        stop bit        stop bit        -
        # 11        -               ack/nak         -

        # Store individual bits and their start/end samplenumbers.
        self.bits.append(Bit(datapin, self.samplenum, self.samplenum))

        # Fix up end sample numbers of the last bit
        if self.bitcount > 0:
            b = self.bits[self.bitcount - 1]
            self.bits[self.bitcount - 1] = Bit(b.val, b.ss, self.samplenum)

        if bool(self.samplerate) and self.bitcount > 0:
            # timeout(us)
            to = 100 if self.options['protocol'] == 'AT' else 500
            # start bit of AT:H->D can be long and is not be checked
            bc = 1 if (self.options['protocol'] == 'AT' and self.state == State.HOST_TO_DEVICE) else 0
            # the last bit
            b = self.bits[self.bitcount - 1]

            if to < (b.es - b.ss) / (self.samplerate / 1000000) and self.bitcount > bc:
                self.put(b.ss, b.es, self.out_ann, [Ann.ERROR, ['Timeout Error', 'TOE',  'E']])

                self.bits, self.bitcount = [], 0
                return

        # start bit annotation
        if self.bitcount == 1:
            self.putx(0, [Ann.START, ['Start bit', 'Start', 'S']])

        # bit0:7 and parity
        if self.bitcount > 1 and self.bitcount < 11:
                self.putb(self.bitcount - 1, Ann.BIT)

        # data byte
        if self.bitcount == 9:
            byte = 0
            for i in range(8):
                byte |= (self.bits[i + 1].val << i)

            if self.state == State.DEVICE_TO_HOST:
                self.put(self.bits[1].ss, self.bits[8].es, self.out_ann, [Ann.DATA,
                    ['D->H: %02X' % byte, 'D: %02X' % byte, '%02X' % byte]])
            else:
                self.put(self.bits[1].ss, self.bits[8].es, self.out_ann, [Ann.COMMAND,
                    ['H->D: %02X' % byte, 'H: %02X' % byte, '%02X' % byte]])

            # XT ends here
            if self.options['protocol'] == 'XT':
                self.bits, self.bitcount = [], 0
                return

        if self.bitcount == 10:
            byte = 0
            for i in range(8):
                byte |= (self.bits[i + 1].val << i)

            parity_ok = (bin(byte).count('1') + self.bits[9].val) % 2 == 1
            if parity_ok:
                self.putx(9, [Ann.PARITY_OK, ['Parity OK', 'Par OK', 'P']])
            else:
                self.putx(9, [Ann.PARITY_ERR, ['Parity Error', 'Par Err', 'PE']])

            if self.state == State.DEVICE_TO_HOST:
                # max width of stop bit determined from parity bit
                width = self.bits[9].es - self.bits[9].ss
                self.wait([{0: 'r'}, {'skip': width}])

                b = self.bits[-1]
                self.bits[-1] = Bit(b.val, b.ss, self.samplenum)

                self.putx(10, [Ann.STOP, ['Stop bit', 'Stop', 'St', 'T']])
                self.bits, self.bitcount = [], 0
                return

        if self.bitcount == 11:
            # HOST_TO_DEVICE ack/nak
            if datapin == 0:
                # data:H
                self.wait({1: 'r'})
                b = self.bits[10]
                self.bits[10] = Bit(b.val, b.ss, self.samplenum)
                self.putx(10, [Ann.STOP, ['Stop bit/ACK', 'Stop', 'St', 'T']])
            else:
                self.putx(10, [Ann.STOP, ['Stop bit/NAK', 'Stop', 'St', 'T']])

            self.bits, self.bitcount = [], 0
            return

        # Find all 11 bits. Start + 8 data + odd parity + stop.
        if self.bitcount < 11:
            self.bitcount += 1
            return

        self.bits, self.bitcount = [], 0

    def decode(self):
        if self.options['protocol'] == 'AT':
            self.decode_at()
        else:
            self.decode_xt()

    def decode_at(self):
        while True:
            # Sample data bits on the falling clock edge (assume the device
            # is the transmitter). Expect the data byte transmission to end
            # at the rising clock edge. Cope with the absence of host activity.
            if (self.state == State.IDLE):
                # clock:H, data:H
                clock_pin, data_pin = self.wait([{0: 'f'}, {1: 'f'}])

                if (clock_pin == 0 and data_pin == 1):
                    self.state = State.INHIBIT
                elif (clock_pin == 1 and data_pin == 0):
                    # start bit
                    self.handle_bits(data_pin)
                    self.state = State.DEVICE_TO_HOST
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = State.TRANSIENT
                elif (clock_pin == 1 and data_pin == 1):
                    self.state = State.IDLE

            elif (self.state == State.DEVICE_TO_HOST):
                _, data_pin = self.wait({0: 'r'})

                # read data at falling edge
                clock_pin, data_pin = self.wait({0: 'f'})

                self.handle_bits(data_pin)

                # stop bit or timeout
                if self.bitcount == 0:
                    self.state = State.TRANSIENT
                    continue

            elif (self.state == State.INHIBIT):
                # clock:L, data:H
                ss = self.samplenum
                clock_pin, data_pin = self.wait([{0: 'r'}, {1: 'e'}])

                self.put(ss, self.samplenum, self.out_ann, [Ann.STATE, ['Inhibit', 'INH',  'I']])

                if (clock_pin == 1 and data_pin == 1):
                    self.state = State.IDLE
                elif (clock_pin == 1 and data_pin == 0):
                    self.state = State.HOST_TO_DEVICE
                    # start bit
                    self.handle_bits(data_pin)
                elif (clock_pin == 0 and data_pin == 1):
                    self.state = State.INHIBIT
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = State.REQUEST

            elif self.state == State.REQUEST:
                # clock:L, data:L
                ss = self.samplenum
                clock_pin, data_pin = self.wait([{0: 'e'}, {1: 'e'}])

                self.put(ss, self.samplenum, self.out_ann, [Ann.STATE, ['Request', 'REQ',  'R']])

                if (clock_pin == 1 and data_pin == 1):
                    self.state = State.IDLE
                elif (clock_pin == 1 and data_pin == 0):
                    self.state = State.HOST_TO_DEVICE
                    # start bit
                    self.handle_bits(data_pin)
                elif (clock_pin == 0 and data_pin == 1):
                    self.state = State.INHIBIT
                elif (clock_pin == 0 and data_pin == 0):
                    self.state = State.REQUEST

            elif self.state == State.HOST_TO_DEVICE:
                _, data_pin = self.wait({0: 'f'})

                # read data at rising edge
                _, data_pin = self.wait({0: 'r'})

                self.handle_bits(data_pin)

                # ack/nak or timeout
                if self.bitcount == 0:
                    self.state = State.TRANSIENT
                    continue

            elif self.state == State.TRANSIENT:
                clock, data = self.wait({'skip': 0})

                if (clock == 1 and data == 1):
                    self.state = State.IDLE
                elif (clock == 1 and data == 0):
                    self.state = State.TRANSIENT
                elif (clock == 0 and data == 1):
                    self.state = State.INHIBIT
                elif (clock == 0 and data == 0):
                    self.state = State.TRANSIENT

                # skip to next state
                if self.state == State.TRANSIENT:
                    self.wait([{0: 'e'}, {1: 'e'}])

    def decode_xt(self):
        while True:
            if (self.state == State.IDLE):
                # clock:H, data:H
                clock_pin, data_pin = self.wait([{0: 'f'}, {1: 'f'}])

                if (clock_pin == 1 and data_pin == 1):
                    self.state = State.IDLE
                elif (clock_pin == 1 and data_pin == 0):
                    # inhibit
                    self.state = State.TRANSIENT
                elif (clock_pin == 0 and data_pin == 1):
                    # start bit
                    self.handle_bits(data_pin)
                    self.state = State.DEVICE_TO_HOST
                elif (clock_pin == 0 and data_pin == 0):
                    # pseudo start bit
                    self.state = State.TRANSIENT

            elif (self.state == State.DEVICE_TO_HOST):
                # XT protocol has virtually no timeout
                #if bool(self.samplerate) and self.bitcount > 1:
                #    # timeout: 500us
                #    if 500 < (self.bits[-2].es - self.bits[-2].ss) / (self.samplerate / 1000000):
                #        self.put(self.bits[-2].ss, self.bits[-2].es, self.out_ann, [Ann.ERROR, ['D->H Timeout Error/Inhibit', 'TOE',  'E']])
                #        # reset bitcount
                #        self.bits, self.bitcount = [], 0
                #        self.state = State.TRANSIENT
                #        continue

                # clock:L, data:X
                _, data_pin = self.wait({0: 'r'})

                if self.bitcount == 9:
                    # end at rising edge
                    self.handle_bits(data_pin)
                else:
                    # read data at falling edge
                    clock_pin, data_pin = self.wait({0: 'f'})
                    self.handle_bits(data_pin)

                # end or timeout
                if self.bitcount == 0:
                    self.state = State.TRANSIENT
                    continue

            elif self.state == State.TRANSIENT:
                clock, data = self.wait([{0: 'e'}, {1: 'e'}])

                # wait for stable state
                if (clock == 1 and data == 1):
                    self.state = State.IDLE
                elif (clock == 1 and data == 0):
                    # inhibit
                    self.state = State.TRANSIENT
                elif (clock == 0 and data == 1):
                    # reset / start bit
                    self.state = State.TRANSIENT
                elif (clock == 0 and data == 0):
                    # pseudo start bit
                    self.state = State.TRANSIENT
