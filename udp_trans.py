#! /usr/bin/env python
# _*_ coding: UTF-8 _*_


import socket

class udp_client(object):
    """
    DESCRIPTION
    ===========

    ARGUMENTS
    =========
    """
    def __init__(self, host='localhost', port=16210, print=True, bufsize=16*1024):
        self.host = host
        self.port = port
        self.print = print
        self.bufsize = bufsize
        pass

    # Base func -----
    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return self.sock  ## これreturnする必要ある??

    def close(self):
        self.sock.close()
        self.sock = None
        return self.sock

    def send(self, msg):
        if self.print:
            print('SEND> {0}'.format(msg))
        msg += '\n'  ## これがないと語尾1文字が認識されない??
        ret = self.sock.sendto(bytes(msg, 'utf8'), (self.host, self.port))
        return ret

    def recv(self, byte=16*1024):
        ret = self.sock.recv(byte)
        if self.print:
            print('RECV> {0}'.format(ret))
        return ret

    # XFFTS Methods -----
    def start(self):
        self.send('RPG:XFFTS:START')
        return self.recv(self.bufsize)

    def stop(self):
        self.send('RPG:XFFTS:STOP')
        return self.recv(self.bufsize)

    def abort(self):
        self.send('RPG:XFFTS:ABORT')
        return self.recv(self.bufsize)

    def configure(self):
        self.send('RPG:XFFTS:CONFIGURE')
        return self.recv(self.bufsize)

    def initialize(self):
        self.send('RPG:XFFTS:INITSYNTHESIZER')
        return self.recv(self.bufsize)

    # XFFTS Properties -----
    def query_state(self):
        self.send('RPG:XFFTS:STATE')
        return self.recv(self.bufsize)
    """
    #TODO::
    def set_state_enabled(self):
        self.send('RPG:XFFTS:STATE ENABLED')
        return self.recv(self.bufsize)

    #TODO::
    def set_state_disabled(self):
        self.send('RPG:XFFTS:STATE DISABLED')
        return self.recv(self.bufsize)
    """

    def query_blanktime(self):
        self.send('RPG:XFFTS:BLANKTIME')
        ret = self.recv(self.bufsize)
        return int(ret.split()[1])

    def set_blanktime(self, usec):
        if usec < 1000:
            print('ERROR:MINIMUM BLANKTIME IS 1000 [us]')
            return self.query_blanktime()
        self.send('RPG:XFFTS:CMDBLANKTIME {0}'.format(usec))
        ret = self.recv(self.bufsize)
        blanktime = int(ret.split()[1])
        return blanktime

    def query_synctime(self):
        self.send('RPG:XFFTS:SYNCTIME')
        ret = self.recv(self.bufsize)
        return int(ret.split()[1])

    def set_synctime(self, usec):
        if usec < 100000:
            print('ERROR:MINIMUM SYNCTIME IS 100,000 [us]')
            return self.query_synctime()
        self.send('RPG:XFFTS:CMDSYNCTIME {0}'.format(usec))
        ret = self.recv(self.bufsize)
        blanktime = int(ret.split()[1])
        return blanktime

    def query_numphases(self):
        self.send('RPG:XFFTS:NUMPHASES')
        ret = self.recv(self.bufsize)
        result = int(ret.split()[1])
        return result

    def set_numphases(self, phase):
        """
        phase = [1,4]
        """
        self.send('RPG:XFFTS:CMDNUMPHASES {0}'.format(phase))
        ret = self.recv(self.bufsize)
        result = int(ret.split()[1])
        return result

    def query_mode(self):
        self.send('RPG:XFFTS:MODE')
        ret = self.recv(self.bufsize)
        result = ret.split()[1]
        return result

    def set_mode(self, mode):
        """
        mode = INTERNAL | EXTERNAL
        """
        self.send('RPG:XFFTS:CMDMODE %s'.format(mode))
        return self.recv(self.bufsize)

    def query_usedsections(self):
        self.send('RPG:XFFTS:USEDSECTIONS')
        ret = self.recv(self.bufsize)
        ret = ret.split()[1:-1]
        ret_int = list(map(int, ret))
        return ret_int

    def set_usedsections(self, section):
        """
        section :: list
        """
        #TODO:
        if len(section)!=32:
            section += [0] * (32-len(section))
            pass
        self._used = section
        section_str = ' '.join(map(str, section))
        self.send('RPG:XFFTS:CMDUSEDSECTIONS {0}'.format(section_str))
        return self.recv(self.bufsize)

    def release_date(self):
        self.send('RPG:XFFTS:RELEASE')
        ret = self.recv(self.bufsize)
        return ret.split()[1]

    def caladc(self):
        self.send('RPG:XFFTS:CALADC')
        return self.recv(self.bufsize)


    def info(self, n):
        """
        DESCRIPTION
        ===========
        Show board information on "FFTS display".
        (Maybe) The information is not returned to client.

        ARGUMENT
        ===========
        boardnum::
        """
        self.send('RPG:XFFTS:INFO {0}'.format(n))
        ret = self.recv(self.bufsize)
        return int(ret.split()[1])

    def dump(self, n):
        """
        boardnum::
        """
        self.send('RPG:XFFTS:DUMP {0}'.format(n))
        return self.recv(self.bufsize)

    def saveadcdelays(self):
        #self.send('RPG:XFFTS:SAVEADCDELAYS')
        #return self.recv(self.bufsize)
        pass

    def loadadcdelays(self):
        #self.send('RPG:XFFTS:LOADADCDELAYS')
        #return self.recv(self.bufsize)
        pass

    # Band dependent command -----
    def query_board_numspecchan(self, n):
        self.send('RPG:XFFTS:BAND{0}:NUMSPECCHAN'.format(n))
        ret = self.recv(self.bufsize)
        numspecchan = int(ret.split()[1])
        return numspecchan

    def set_board_numspecchan(self, n, chan):
        self.send('RPG:XFFTS:BAND{0}:CMDNUMSPECCHAN {1}'.format(n, chan))
        ret = self.recv(self.bufsize)
        numspecchan = int(ret.split()[1])
        return numspecchan

    def query_board_bandwidth(self, n):
        self.send('RPG:XFFTS:BAND{0}:BANDWIDTH'.format(n))
        ret = self.recv(self.bufsize)
        return float(ret.split()[1])

    def set_board_bandwidth(self, n, width):
        """
        ARGUMENTS
        ===========
        1. n:
        2. width:
        """
        #self.send('RPG:XFFTS:BAND{0}:CMDBANDWIDTH %.2f'.format(n, width))  ## なんで %.2f なの??
        self.send('RPG:XFFTS:BAND{0}:CMDBANDWIDTH {1}'.format(n, width))
        ret = self.recv(self.bufsize)
        bandwidth = ret.split()[1]
        if bandwidth == 'ERROR':
            return 'ERROR:INVALID_BAND MONEY!!'
        else:
            return float(bandwidth)

    def query_board_mirrorspectra(self, n):
        self.send('RPG:XFFTS:BAND{0}:MIRROSPECTRA'.format(n))
        ret = self.recv(self.bufsize)
        if ret.split()[1] == '0':
            result = 'MIRROR OFF'
            pass
        elif ret.split()[1] == '1':
            result = 'MIRROR ON'
            pass
        else:
            result = 'ERROR:INVALID_BAND'
            pass
        return result

    def set_board_mirrorspectra(self, n, switch):
        self.send('RPG:XFFTS:BAND{0}:CMDMIRROSPECTRA {1}'.format(n, switch))
        return self.recv(self.bufsize)

    def board_caladc(self, n):
        self.send('RPG:XFFTS:BAND{0}:CALADC'.format(n))
        return self.recv(self.bufsize)

    def board_adcdelay(self, n):
        self.send('RPG:XFFTS:BAND{0}:ADCDELAY'.format(n))
        ret = self.recv(self.bufsize)
        delay = float(ret.split()[1])
        return delay

    def query_board_time(self, n):
        self.send('RPG:XFFTS:BAND{0}:TIME'.format(n))
        ret = self.recv(self.bufsize)
        time = ret.split()[1]
        return time

    def query_board_temperature(self, n):
        self.send('RPG:XFFTS:BAND{0}:TEMPERATURE'.format(n))
        ret = self.recv(self.bufsize)
        temp = list(map(float, [ret.split()[1], ret.split()[2], ret.split()[3]]))
        return temp

    def query_all_temperature(self):
        all_temp = []
        for i, used in enumerate(self.query_usedsections()):
            if used == 1:
                ret = self.query_board_temperature(i+1)
                all_temp.append(ret)
            else:
                all_temp.append(None)
                pass
            continue
        return all_temp

    def board_specfilter(self, n):
        self.send('RPG:XFFTS:BAND{0}:SPECFILTER'.format(n))
        ret = self.recv(self.bufsize)
        return int(ret.split()[1])

















