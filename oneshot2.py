#! /usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
import sys
import numpy
import threading

sys.path.append('/home/amigos/ros/src/NASCORX')
from NASCORX_XFFTS import data_client2

dir = '/home/amigos/ros/src/NASCORX/NASCORX_XFFTS/data2/'

class Oneshot(object):
    spec = None
    conti = None
    btemp = None

    def __init__(self, synctime=0.1):
        self.client = data_client2.data_client(synctime=synctime)
        pass

    def get_spectrum(self, integtime, repeat):
        data = self.client.oneshot(integtime=integtime, repeat=repeat)
        self.spec = data
        return

    def get_continuum(self, integtime, repeat):
        data = self.client.conti_oneshot(integtime=integtime, repeat=repeat)
        self.conti = data
        return

    def get_btemp(self, integtime, repeat):
        sec = int(integtime * repeat)
        data = self.client.btemp_oneshot(sec=sec)
        self.btemp = data
        return

    def measure(self, integtime, repeat):
        th1 = threading.Thread(target=self.get_spectrum, args=(integtime, repeat))
        th2 = threading.Thread(target=self.get_continuum, args=(integtime, repeat))
        th3 = threading.Thread(target=self.get_btemp, args=(integtime, repeat))
        th1.setDaemon(True)
        th2.setDaemon(True)
        th3.setDaemon(True)
        th1.start()
        th2.start()
        th3.start()

        th1.join()
        th2.join()
        th3.join()

        print('\n'
              'End Measurement'
              '\n')
        return self.spec, self.conti, self.btemp

def run(integtime, repeat, num, synctime=0.1):
    c = Oneshot(synctime=synctime)
    spec, conti, btemp = c.measure(integtime=integtime, repeat=repeat)
    unixtime = spec[1]
    spectrum = spec[2]
    spec1 = numpy.array(spectrum)
    continuum = conti[2]

    for i in range(numpy.shape(spec1)[1]):
        numpy.savetxt(dir+'spec_{}-{}_exp{}_BE{}.csv'.format(integtime, repeat, num, i+1), spec1[:, i, :], delimiter=',')
        pass
    numpy.savetxt(dir+'unix_{}-{}.csv'.format(integtime, repeat), unixtime, delimiter=',')

if __name__ == 'main':
    integtime = int(input('integ ?:  '))
    repeat = int(input('repeat ?:  '))
    synctime = float('synctime ?:  ')
    run(integtime=integtime, repeat=repeat, synctime=synctime)




