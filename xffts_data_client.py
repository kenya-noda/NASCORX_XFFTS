#!/usr/bin/env python

import time
import rospy
import numpy
import calendar
import datetime
import threading

from necst.msg import XFFTS_msg
from necst.msg import XFFTS_pm_msg


class data_client(object):
    synctime = 0.1

    def __init__(self):
        rospy.init_node('XFFTS_data_Subscriber')
        pass

    def oneshot(self, integtime, repeat, start=None):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function returns integrated XFFTS spectrum.
        You can choose integration time, Number of repetition and start time.

        ARGUMENTS
        =========
        1. integtime [sec] : integration time of each observation.
             Type          : float
             Default       : Nothing
        2. repeat    : How many times to repeat observation.
             Type    : int
             Default : Nothing
        3. start [Unix time] : When to start observation in Unix-time.
             Type            : float (time.time like value)
             Default         : None
             Example         : time.time()+5
                             : 1508495163.9506361

        RETURNS
        =======
        1. timelist : The timestamp of the first data of each integrations.
             Type   : Str list
             Length : Same as "repeat"
        2. unixlist : The Unix-time version of timelist.
             Type   : float list
             Length : Same as "repeat"
        3. spectrum : The integrated spectrum.
             Type   : float array
             dim    : 3
             shape  : [repeat, BE_num, ch_num]

        USAGE
        =====
        ret = data_client.oneshot(integtime=0.5, repeat=5)
        ts, uts, spec = data_client.oneshot(0.5, 5, start=time.time()+5)
        """

        # define data list
        # ----------------
        self.timestamp = []
        self.unixlist = []
        self.data = []

        if start is None: waittime = 0
        else: waittime = start - time.time()

        # subscribe data
        # --------------
        self.data_subscriber(integtime=integtime, repeat=repeat, waittime=waittime)

        # data integration
        # ----------------
        spectrum = []
        timelist = []
        unixlist = []
        init_index = self.index_search(start)
        for i in range(repeat):
            start = init_index + int(integtime / self.synctime * i)
            fin = init_index + int(integtime / self.synctime * (i+1))
            spectrum.append(numpy.average(self.data[start:fin], axis=0))
            timelist.append(self.timestamp[start])
            unixlist.append(self.unixlist[start])
        return timelist, unixlist, spectrum

    def data_subscriber(self, integtime, repeat, waittime):
        """
        DESCRIPTION
        ===========
        This function receives values from data-server(GIGABYTE PC) and append it to list.

        ARGUMENTS
        =========
        1. integtime [sec] : Same as that of oneshot.
        2. repeat          : Same as that of oneshot.
        3. waiting [sec]   : How long to wait until the first observation starts.
             Type            : float
             Default         : Nothing.
        """
        sub = rospy.Subscriber('XFFTS_SPEC', XFFTS_msg, self.append)
        time.sleep(waittime + integtime * repeat + 0.5)
        sub.unregister()
        return

    def append(self, req):
        """
        DESCRIPTION
        ===========
        This function appends received-data to self list.

        ARGUMENTS
        =========
        1. req : ROS format argument.

        RETURNS
        =======
        Nothing but append to self-list.
        1. timestamp : The timestamp of XFFTS(UTC, str).
             where   : self.timestamp
        2. unix_ret  : The unix-timestamp(UTC, float) of format "xxxxxxxxxx.x" .
             where   : self.unixlist
        3. data_temp : The spectrum of each sync-time. Only for available Back-End.
             where   : self.data
        """
        self.data_temp = []

        # Calculate UNIX-time
        # -------------------
        unixtime = self.timestamp_to_unixtime(req.timestamp)
        unix_ret = round(unixtime, 1)                                               # using xx.x [sec] format

        # append data to temporary list
        # -----------------------------
        reqlist = [req.SPEC_BE1, req.SPEC_BE2, req.SPEC_BE3, req.SPEC_BE4,
                   req.SPEC_BE5, req.SPEC_BE6, req.SPEC_BE7, req.SPEC_BE8,
                   req.SPEC_BE9, req.SPEC_BE10, req.SPEC_BE11, req.SPEC_BE12,
                   req.SPEC_BE13, req.SPEC_BE14, req.SPEC_BE15, req.SPEC_BE16]
        for i in range(req.BE_num):
            self.data_temp.append(reqlist[i])

        # append return value
        # -------------------
        self.timestamp.append(req.timestamp)
        self.unixlist.append(unix_ret)
        self.data.append(self.data_temp)
        return

    def timestamp_to_unixtime(self, timestamp):
        """
        DESCRIPTION
        ===========
        This function converts "XFFTS-timestamp(UTC, string)" to "UNIX-time(UTC, float)".

        ARGUMENTS
        =========
        1. timestamp : The timestamp of XFFTS(UTC, str).
             Type    : Str
             format  : '2017-10-20T09:34:13.9193PC  '

        RETURNS
        =======
        1. unixtime : The unix-timestamp(UTC, float) of input XFFTS-timestamp(UTC, str).
             Type   : float
        """
        t = datetime.datetime.strptime(timestamp, '%Y-%m-%dT%H:%M:%S.%fPC  ')
        unixtime = calendar.timegm(t.timetuple()) + t.microsecond / 1e6
        return unixtime

    def index_search(self, start):
        """
        DESCRIPTION
        ===========
        This function returns the index of the first data to use.
        Used to take into account of start time(waiting time).

        ARGUMENTS
        =========
        1. start : Same as that of oneshot.

        RETURNS
        =======
        1. index  : The index of the first data to use.
             Type : int
        """
        if start is None: index = 0
        else: index = self.unixlist.index(round(start, 1))
        return index

    # For Non-"start-time" function
    def oneshot_light(self, integtime, repeat):
        """
        DESCRIPTION
        ===========
        -- COMING SOON --

        :param integtime:
        :param repeat:
        :return:
        """

        # define data list
        # ----------------
        self.timestamp = []
        self.data = []

        # subscribe data
        # --------------
        self.data_subscriber_light(integtime, repeat)

        # data integration
        # ----------------
        spectrum = []
        timelist = []
        for i in range(repeat):
            start = int(integtime / self.synctime * i)
            fin = int(integtime / self.synctime * (i+1))
            spectrum.append(numpy.average(self.data[start:fin], axis=0))
            timelist.append(self.timestamp[start])
        return timelist, spectrum

    def data_subscriber_light(self, integtime, repeat):
        """
        DESCRIPTION
        ===========
        timestamp is not taken into consideration.

        -- COMING SOON --
        """
        data_num = int(integtime * repeat / self.synctime)
        sub = rospy.Subscriber('XFFTS_SPEC', XFFTS_msg, self.append)
        time.sleep(integtime*repeat+0.5)
        self.data = self.data[0:data_num]
        return

    def append_light(self, req):
        self.data_temp = []
        reqlist = [req.SPEC_BE1, req.SPEC_BE2, req.SPEC_BE3, req.SPEC_BE4,
                   req.SPEC_BE5, req.SPEC_BE6, req.SPEC_BE7, req.SPEC_BE8,
                   req.SPEC_BE9, req.SPEC_BE10, req.SPEC_BE11, req.SPEC_BE12,
                   req.SPEC_BE13, req.SPEC_BE14, req.SPEC_BE15, req.SPEC_BE16]
        for i in range(req.BE_num):
            self.data_temp.append(reqlist[i])
        self.timestamp.append(req.timestamp)
        self.data.append(self.data_temp)
        return


# History
# -------
# written by T.Inaba
