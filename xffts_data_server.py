#!/usr/bin/env python

import sys
import rospy
import numpy
import socket
import struct
import signal
import threading

from necst.msg import XFFTS_msg
from necst.msg import XFFTS_pm_msg


class data_server(object):
    header_size = 64
    BE_num_Max = 16
    _sock = None
    _stop_loop = False

    def __init__(self, host='localhost', port=25144):
        self.connect(host, port)
        pass

    def connect(self, host, port):
        print('Create New Socket: host={}, port={}'.format(host, port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        self._sock = sock
        return

    def data_relaying_loop(self):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function relays XFFTS data from XFFTS to FAC by ROS method.
        When you stop the loop, call stop_loop function or press Ctrl-C.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing but send data listed below.
        1. XFFTS_SPEC : Send spectrum data
            1. timestamp : XFFTS-format timestamp.
                Type     : str
                fmt      : '2017-10-20T09:34:13.9193PC  '
            2. BE_num    : Number of Back End Board.
                Number   : 1 - 16
                Type     : int
            3. SPEC_BE1-16 : The spectrum of each BE(1-16).
                Type       : float list
        2. XFFTS_PM : Send total power data(=continuum data).
            1. timestamp : Same as above.
            2. BE_num    : Same as above.
            3. POWER_BE1-16 : The total counts of each BE(1-16).
        """
        # Print Welcome Massage
        # ---------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Start : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')

        # ROS setting
        # -----------
        rospy.init_node('XFFTS_data_server')
        pub = rospy.Publisher('XFFTS_SPEC', XFFTS_msg, queue_size=10)
        pub2 = rospy.Publisher('XFFTS_PM', XFFTS_pm_msg, queue_size=10)             # PM = Power Meter
        XFFTS_SPEC = XFFTS_msg()
        XFFTS_PM = XFFTS_pm_msg()

        # Signal handler
        # --------------
        def signal_handler(num, flame):
            self.stop_loop()
            sys.exit()
        signal.signal(signal.SIGINT, signal_handler)

        # data relaying loop
        # ------------------
        while True:
            # get data
            # --------
            if self._stop_loop: break
            header = self.recv_header()
            rawdata = self.recv_data(header.data_size)
            timestamp = header.timestamp.decode('utf-8')
            BE_num = header.BE_num                                                  # BE_num = BE_num_temp
            print(header.timestamp, header.BE_num)

            # binary to float conversion
            # --------------------------
            spec = []
            pow = []
            for i in range(self.BE_num_Max):
                # For Available BE
                if i+1 <= header.BE_num:
                    start = header.data_size*i
                    fin = header.data_size*(i+1)
                    BE_num_temp, ch_num = struct.unpack('2I', rawdata[start:start+8])
                    data_temp = list(struct.unpack('{}f'.format(ch_num), rawdata[start+8:fin]))
                    pow_temp = numpy.sum(data_temp)
                    spec.append(data_temp)
                    pow.append(pow_temp)
                # For Unavailable BE
                elif header.BE_num <= i+1:
                    spec.append([0])
                    pow.append([0])

            # ROS Data Trans
            # --------------
             # Spectrum
            XFFTS_SPEC.timestamp = timestamp
            XFFTS_SPEC.BE_num = BE_num
            XFFTS_SPEC.SPEC_BE1 = spec[0]
            XFFTS_SPEC.SPEC_BE2 = spec[1]
            XFFTS_SPEC.SPEC_BE3 = spec[2]
            XFFTS_SPEC.SPEC_BE4 = spec[3]
            XFFTS_SPEC.SPEC_BE5 = spec[4]
            XFFTS_SPEC.SPEC_BE6 = spec[5]
            XFFTS_SPEC.SPEC_BE7 = spec[6]
            XFFTS_SPEC.SPEC_BE8 = spec[7]
            XFFTS_SPEC.SPEC_BE9 = spec[8]
            XFFTS_SPEC.SPEC_BE10 = spec[9]
            XFFTS_SPEC.SPEC_BE11 = spec[10]
            XFFTS_SPEC.SPEC_BE12 = spec[11]
            XFFTS_SPEC.SPEC_BE13 = spec[12]
            XFFTS_SPEC.SPEC_BE14 = spec[13]
            XFFTS_SPEC.SPEC_BE15 = spec[14]
            XFFTS_SPEC.SPEC_BE16 = spec[15]
            pub.publish(XFFTS_SPEC)

             # total power
            XFFTS_PM.timestamp = timestamp
            XFFTS_PM.BE_num = BE_num
            XFFTS_PM.POWER_BE1 = pow[0]
            XFFTS_PM.POWER_BE2 = pow[1]
            XFFTS_PM.POWER_BE3 = pow[2]
            XFFTS_PM.POWER_BE4 = pow[3]
            XFFTS_PM.POWER_BE5 = pow[4]
            XFFTS_PM.POWER_BE6 = pow[5]
            XFFTS_PM.POWER_BE7 = pow[6]
            XFFTS_PM.POWER_BE8 = pow[7]
            XFFTS_PM.POWER_BE9 = pow[8]
            XFFTS_PM.POWER_BE10 = pow[9]
            XFFTS_PM.POWER_BE11 = pow[10]
            XFFTS_PM.POWER_BE12 = pow[11]
            XFFTS_PM.POWER_BE13 = pow[12]
            XFFTS_PM.POWER_BE14 = pow[13]
            XFFTS_PM.POWER_BE15 = pow[14]
            XFFTS_PM.POWER_BE16 = pow[15]
            pub2.publish(XFFTS_PM)

        # Print Shut Down Massage
        # -----------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Shut Down : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')
        return

    def recv_header(self):
        """
        DESCRIPTION
        ===========
        This function receives and returns XFFTS data header.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        See "data_header" class.
        """
        header = self._sock.recv(self.header_size)
        return data_header(header)

    def recv_data(self, data_size):
        """
        DESCRIPTION
        ===========
        This function receives and returns XFFTS data.

        ARGUMENTS
        =========
        1. data_size  : Data size of XFFTS output. Maybe changing as BE_num changes.
             Type     : int
             Default  : Nothing.
             example  : header.data_size

        RETURNS
        =======
        1. data : The XFFTS spectrum in binary format.
            Type : binary
        """
        data = self._sock.recv(data_size, socket.MSG_WAITALL)
        return data

    def stop_loop(self):
        """
        DESCRIPTION
        ===========
        This function stops data relaying loop function.
        This function is also called when Ctrl-C is pressed.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing.
        """
        self._stop_loop = True
        return

    def start_thread(self):
        th = threading.Thread(target=self.data_relaying_loop)
        th.setDaemon(True)
        th.start()
        return

    def grave(self):
        """
        # define publisher
        # ----------------
        BE_array = [XFFTS_SPEC.SPEC_BE1, XFFTS_SPEC.SPEC_BE2, XFFTS_SPEC.SPEC_BE3, XFFTS_SPEC.SPEC_BE4,
                    XFFTS_SPEC.SPEC_BE5, XFFTS_SPEC.SPEC_BE6, XFFTS_SPEC.SPEC_BE7, XFFTS_SPEC.SPEC_BE8,
                    XFFTS_SPEC.SPEC_BE9, XFFTS_SPEC.SPEC_BE10, XFFTS_SPEC.SPEC_BE11, XFFTS_SPEC.SPEC_BE12,
                    XFFTS_SPEC.SPEC_BE13, XFFTS_SPEC.SPEC_BE14, XFFTS_SPEC.SPEC_BE15, XFFTS_SPEC.SPEC_BE16]

        Pow_array = [XFFTS_PM.POWER_BE1, XFFTS_PM.POWER_BE2, XFFTS_PM.POWER_BE3, XFFTS_PM.POWER_BE4,
                     XFFTS_PM.POWER_BE5, XFFTS_PM.POWER_BE6, XFFTS_PM.POWER_BE7, XFFTS_PM.POWER_BE8,
                     XFFTS_PM.POWER_BE9, XFFTS_PM.POWER_BE10, XFFTS_PM.POWER_BE11, XFFTS_PM.POWER_BE12,
                     XFFTS_PM.POWER_BE13, XFFTS_PM.POWER_BE14, XFFTS_PM.POWER_BE15, XFFTS_PM.POWER_BE16]

        # data relaying loop
        # ------------------
        while True:
            # get data
            # --------
            if self._stop_loop: break
            header = self.recv_header()
            rawdata = self.recv_data(header.data_size)
            print(header.timestamp, header.BE_num)


            # binary to float conversion
            # --------------------------
            for i in range(header.BE_num):
                start = header.data_size*i
                fin = header.data_size*(i+1)
                BE_num_temp, ch_num = struct.unpack('2I', rawdata[start:start+8])
                data_temp = list(struct.unpack('{}f'.format(ch_num), rawdata[start+8:fin]))
                pow_temp = numpy.sum(data_temp)
                BE_array[i] = data_temp
                Pow_array[i] = pow_temp

            # Unavailable BE procedure
            # ------------------------
            for i in range(header.BE_num, self.BE_num_Max+1):
                BE_array[i] = 0
                Pow_array[i] = 0

            # binary to float conversion
            # --------------------------
            for i in range(self.BE_num_Max):
                # For Available BE
                if i+1 <= header.BE_num:
                    start = header.data_size*i
                    fin = header.data_size*(i+1)
                    BE_num_temp, ch_num = struct.unpack('2I', rawdata[start:start+8])
                    data_temp = list(struct.unpack('{}f'.format(ch_num), rawdata[start+8:fin]))
                    pow_temp = numpy.sum(data_temp)
                    BE_array[i] = data_temp
                    Pow_array[i] = pow_temp
                # For Unavailable BE
                elif header.BE_num <= i+1:
                    BE_array[i] = [0]
                    Pow_array[i] = [0]
        """


class data_header(object):
    header_size = 64

    def __init__(self, header):
        self.ieee = struct.unpack('<4s', header[0:4])[0]
        self.data_format = struct.unpack('4s', header[4:8])[0]
        self.package_length = struct.unpack('I', header[8:12])[0]
        self.BE_name = struct.unpack('8s', header[12:20])[0]
        self.timestamp = struct.unpack('28s', header[20:48])[0]
        self.integration_time = struct.unpack('I', header[48:52])[0]
        self.phase_number = struct.unpack('I', header[52:56])[0]
        self.BE_num = struct.unpack('I', header[56:60])[0]
        self.blocking = struct.unpack('I', header[60:64])[0]
        self.data_size = self.package_length - self.header_size
        return


# History
# -------
# written by T.Inaba
