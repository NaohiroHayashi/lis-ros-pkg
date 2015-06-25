#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# ver1.30622
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# (C) 2013 Hiroaki Matsuda

import serial
import time

class Mx(object):

        def __init__(self):
                self.myserial = serial.Serial()
                print('Generated the serial object')
                
        def open_port(self, port = 'COM1', baudrate = 115200, timeout = 1):
                self.myserial.port = port
                self.myserial.baudrate = baudrate
                self.myserial.timeout = timeout
                self.myserial.parity = serial.PARITY_NONE
                try:
                        self.myserial.open()
                except IOError:
                        print('Failed to open port, check the device and port number')
                else:
                        print('Succeede to open port: ' + port)

        def close_port(self):
                self.myserial.close()

        def set_port(self, baudrate = 115200, timeout = 1):
                self.myserial.baudrate = baudrate
                self.myserial.timeout = timeout
                self.myserial._reconfigurePort()
                print('Succeede to set baudrate:%d, timeout:%d' %(baudrate, timeout))

        def read_data(self, id, address, length):
                self._check_range(id  , 0, 254, 'id')
                
                send = [0xFF, 0xFF, id, 0x04, 0x02, address, length]

                send.append(self._calc_checksum(send[2:]))

                self._write_serial(send)

                return self._read_data(length)

        def write_data(self, id, address, parameters):
                self._check_range(id  , 0, 254, 'id')
                parameters_length = len(parameters)
                
                send = [0xFF, 0xFF, id, parameters_length + 3, 0x03, address]
                send += parameters
                send.append(self._calc_checksum(send[2:]))

                self._write_serial(send)

                return self._read_data(parameters_length)

        def reg_write_data(self, id, address, parameters):
                self._check_range(id  , 0, 254, 'id')
                parameters_length = len(parameters)
                
                send = [0xFF, 0xFF, id, parameters_length + 3, 0x04, address]
                send += parameters
                send.append(self._calc_checksum(send[2:]))

                self._write_serial(send)

                return self._read_data(parameters_length)

        def action(self, id):
                self._check_range(id  , 0, 254, 'id')
                send = [0xFF, 0xFF, id, 0x02, 0x05]
                send.append(self._calc_checksum(send[2:]))

                self._write_serial(send)
                
        def move(self, id, position):
                position = int(position / 0.088)

                return self.write_data(id,
                                       0x1E,
                                       [ position & 0x00FF,
                                        (position & 0xFF00) >> 8])

        def speed(self, id, speed):
                speed = int(speed / 0.114)

                return self.write_data(id,
                                       0x20,
                                       [ speed & 0x00FF,
                                        (speed & 0xFF00) >> 8])

        def servo(self, id, enable):
                return self.write_data(id,
                                       0x18,
                                       [ enable])

        def reg_move(self, id_list, position_list):
                for i in range(len(id_list)):
                        position = int(position_list[i] / 0.088)

                        self.reg_write_data(id_list[i],
                                            0x1E,
                                            [ position & 0x00FF,
                                             (position & 0xFF00) >> 8])

        def reg_servo(self, id_list, enable_list):
                for i in range(len(id_list)):
                        self.reg_write_data(id_list[i],
                                            0x18,
                                            [ enable_list[i]])
                
# The following functions are provided for use in Mx class
        def _calc_checksum(self, send):
                checksum = 0
                
                for temp in send:
                        checksum += temp
                checksum = ~checksum & 0x00FF
                
                return checksum

        def _check_range(self, value, lower_range, upper_range, name = 'value'):
                if value < lower_range or value > upper_range:
                        raise ValueError(name + ' must be set in the range from '
                                         + str(lower_range) + ' to ' + str(upper_range))

        def _read_data(self, parameter_length):
                receive = self.myserial.read(6 + parameter_length)
                
                return [ord(data) for data in receive]

        def _check_ack(self, id):
                receive = self.myserial.read(1)
                length = len(receive)
                
                if length == 1:
                        ack = ord(receive)
                        if ack == 0x07:
                                return id, 'ACK'
                        elif ack == 0x08:
                                return id, 'NACK'
                        else:
                                return id, 'unKnown'
                elif length != 1:
                        return id, 'unReadable'

        def _write_serial(self, send):
                self.myserial.flushOutput()
                self.myserial.flushInput()
                self.myserial.write("".join(map(chr, send)))                

if __name__ == '__main__':
        import time
        import pymx
        
        mx = pymx.Mx()  
        mx.open_port('COM5', 57600, 1 )

        #print mx.read_data(1, 0x03, 1)
        #print mx.write_data(2, 0x18, [0x01])
        mx.speed(4, 10)
        mx.move(4, 180)
         
        #mx.close_port()
