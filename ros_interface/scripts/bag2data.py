#!/usr/bin/env python

import rosbag
import sys

SCAN_TOPIC = '/scan'
ENCODERS_TOPIC = '/encoders'

def process(input, output):
  bag = rosbag.Bag(input)
  
  with open(output, 'w') as outfile:
    for topic, msg, t in bag.read_messages(topics=[SCAN_TOPIC, ENCODERS_TOPIC]):
      if topic==SCAN_TOPIC:
        #S <STAMP> <READINGSx720>\n
        outfile.write("S %s %s\n" % (msg.header.stamp, " ".join(str(r) for r in msg.ranges)))
      elif topic==ENCODERS_TOPIC:
        #E <STAMP> <TIME> <L_TICKS> <R_TICKS> <L_VEL> <R_VEL>\n
        outfile.write("E %s %s %d %d %f %f\n" % (msg.stamp, msg.time, msg.leftTicks, msg.rightTicks, msg.leftVel, msg.rightVel))
  bag.close()

if __name__=="__main__":
  if len(sys.argv) < 3:
    print("usage: %s <bagfile> <outputfile>")
  else:
    process(sys.argv[1], sys.argv[2])