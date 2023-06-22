#!/usr/bin/env python

'''
split log by system ID
'''
from __future__ import print_function

import os
import struct
import re

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--types", default=None, help="list of types to include")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil

def process(filename):
    '''process one logfile'''
    print("Processing %s" % filename)

    types = args.types
    if types is not None:
        types = types.split(',')

    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    base, ext = os.path.splitext(filename)
    output = None
    count = 1
    dirname = os.path.dirname(filename)
    extension = "tlog"

    file_header = bytearray()

    messages = []
    takeoff = False
    start_time = 0.0
    last_curr_up = 0
    last_curr = 0

    # dictionary of outputs by sysid
    output = {}

    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break

        if args.condition and not mavutil.evaluate_condition(args.condition, mlog.messages):
            continue

        m = mlog.recv_match(type=['SYS_STATUS','STATUSTEXT'], condition=args.condition)
        if groundspeed > args.groundspeed and not in_air:
            print("In air at %s (percent %.0f%% groundspeed %.1f)" % (time.asctime(t), mlog.percent, groundspeed))
            in_air = True
            start_time = time.mktime(t)
        elif groundspeed < args.groundspeed and in_air:
            print("On ground at %s (percent %.1f%% groundspeed %.1f  time=%.1f seconds)" % (
                time.asctime(t), mlog.percent, groundspeed, time.mktime(t) - start_time))
            in_air = False
            total_time += time.mktime(t) - start_time
        
        # power consumption
        if takeoff == 0 and m.get_type() == "SYS_STATUS":
            if int(m.current_battery)*0.01 > 10: last_curr_up += 1
            else: last_curr_up = 0
                
        # High power consumption for 5 sec, presume takeoff
        if takeoff == 0 and last_curr_up >= 5:
            last_curr_up = 0
            takeoff = 1
            
        # Takeoff message
        if takeoff == 0 and m.get_type() == "STATUSTEXT" and re.match(r".*takeoff.*", str(m.Text), re.I):
            takeoff = 1

        # Skip pre
        if takeoff == 0: continue
            
        # power consumption
        if takeoff == 1 and m.get_type() == "SYS_STATUS":
            if int(m.current_battery)*0.01 < 5: last_curr += 1
            else: last_curr = 0
                
        # No power consumption for 5 sec, presume landing
        if takeoff == 1 and last_curr >= 5:				
            last_curr = 0
            takeoff = 0
            continue

        sysid = m.get_srcSystem()
        if not sysid in output:
            fname = "%s-%u.%s" % (base, sysid, extension)
            print("Creating %s" % fname)
            output[sysid] = open(fname, mode='wb')

        if output[sysid] and m.get_type() != 'BAD_DATA':
            timestamp = getattr(m, '_timestamp', None)
            output[sysid].write(struct.pack('>Q', int(timestamp*1.0e6)))
            output[sysid].write(m.get_msgbuf())

for filename in args.logs:
    process(filename)
